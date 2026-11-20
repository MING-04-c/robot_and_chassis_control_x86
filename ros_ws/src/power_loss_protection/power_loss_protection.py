#!/usr/bin/env python3
"""Independent MIDUPS guard. No third-party Python dependencies, no UPS writes."""
import argparse
import fcntl
import json
import logging
import math
import os
from pathlib import Path
import pwd
import re
import signal
import subprocess
import sys
import time

from modbus import ProtocolError, display_telemetry, read_registers, telemetry

HERE = Path(__file__).resolve().parent
LOG = logging.getLogger("power_loss_protection")


class Decision:
    """Monotonic, latched decisions; errors interrupt consecutive confirmations."""
    def __init__(self, started, confirm_count, loss_seconds):
        self.last_good = started
        self.confirm_count, self.loss_seconds = confirm_count, loss_seconds
        self.backups = 0
        self.reason = None

    def update(self, now, sample=None):
        if self.reason:
            return self.reason
        # A late good frame must not erase an already-expired communication gap.
        if now-self.last_good >= self.loss_seconds:
            self.reason = "UPS communication unavailable"
        elif sample is None:
            self.backups = 0
        else:
            self.last_good = now
            self.backups = self.backups+1 if sample["backup"] else 0
            if self.backups >= self.confirm_count:
                self.reason = "UPS BACKUP confirmed"
            elif not sample["backup"] and not sample["main"]:
                self.reason = "UPS output state unsafe"
            elif sample["status"] & 0x10:
                self.reason = "UPS fault state"
        return self.reason


def load_config(path):
    config = json.loads(Path(path).read_text())
    if config["mode"] not in ("observe", "armed"):
        raise ValueError("mode must be observe or armed")
    if config.get("computer_power_source", "ups") not in ("external", "ups"):
        raise ValueError("computer_power_source must be external or ups")
    if config["transport"] not in ("rtu_over_tcp", "modbus_tcp"):
        raise ValueError("Explicit transport required")
    if not isinstance(config["host"], str) or not re.fullmatch(r"[0-9.]+", config["host"]):
        raise ValueError("Use a numeric IPv4 gateway address (no blocking DNS)")
    import ipaddress
    ipaddress.IPv4Address(config["host"])
    for key, low, high in (("port", 1, 65535), ("slave_id", 1, 247),
                           ("confirm_count", 1, 20), ("ros_domain_id", 0, 232)):
        if type(config[key]) is not int or not low <= config[key] <= high:
            raise ValueError(f"Invalid {key}")
    durations = ("poll_seconds", "io_timeout_seconds", "communication_loss_seconds",
                 "ros_stop_seconds", "process_stop_seconds", "os_shutdown_budget_seconds",
                 "reserve_seconds")
    for key in durations:
        if type(config[key]) not in (int, float) or not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"{key} must be finite and positive")
    detection = max(config["communication_loss_seconds"]+config["io_timeout_seconds"],
                    config["confirm_count"]*(config["poll_seconds"]+config["io_timeout_seconds"]))
    if config["communication_loss_seconds"] <= config["io_timeout_seconds"]:
        raise ValueError("Communication loss timeout must exceed one I/O timeout")
    if not re.fullmatch(r"[a-zA-Z0-9_.@-]+\.service", config["robot_service"]):
        raise ValueError("Invalid robot service name")
    if config["robot_service"] == "power-loss-protection.service":
        raise ValueError("Guard cannot stop itself")
    for key in ("workspace", "latch_file", "ready_file"):
        if not Path(config[key]).is_absolute():
            raise ValueError(f"{key} must be absolute")
    if not isinstance(config["launch_command"], list) or not config["launch_command"] or not all(
            isinstance(item, str) and item for item in config["launch_command"]):
        raise ValueError("launch_command must be a nonempty argument list")
    if config["mode"] == "armed" and config.get("computer_power_source", "ups") == "ups":
        measured = config["measured_min_backup_seconds"]
        required = detection + 3. + sum(config[key] for key in (
            "ros_stop_seconds", "process_stop_seconds", "os_shutdown_budget_seconds", "reserve_seconds"))
        if type(measured) not in (int, float) or not math.isfinite(measured) or measured <= required:
            raise ValueError(f"Measure backup first: it must exceed configured worst-case budget {required:.1f}s")
    return config


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    with temporary.open("w") as stream:
        json.dump(value, stream, ensure_ascii=False)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
    fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def read_sample(c, transport=None):
    return telemetry(read_registers(c["host"], c["port"], c["slave_id"],
                                   transport or c["transport"], c["io_timeout_seconds"]))


def ros_environment(c):
    return {"ROS_DOMAIN_ID": str(c["ros_domain_id"]),
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"}


def ros_command(c, argv):
    # Shell text is fixed; paths and launch arguments are positional, never code.
    return ["/usr/bin/bash", "-c",
            'source "$1/scripts/robot_env.bash" || exit; shift; exec "$@"',
            "power-loss-protection", c["workspace"], *argv]


def run_bounded(argv, seconds):
    """Bound the process group as well as its direct child."""
    process = subprocess.Popen(argv, start_new_session=True)
    try:
        return process.wait(timeout=seconds)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()
        raise TimeoutError(f"Command deadline: {argv[0]}") from None


def shutdown(c, reason):
    """Best effort stages, with an unconditional normal OS shutdown request."""
    if c["mode"] != "armed":
        LOG.warning("OBSERVE ONLY: would stop ROS and power off: %s", reason)
        return
    LOG.critical("SHUTDOWN COMMITTED: %s", reason)
    try:
        atomic_json(c["latch_file"], dict(reason=reason, time=time.time(),
                                        boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip()))
    except OSError:
        LOG.exception("Cannot persist shutdown latch; continuing shutdown")
    try:
        Path(c["ready_file"]).unlink(missing_ok=True)
    except OSError:
        LOG.exception("Cannot clear readiness; continuing shutdown")
    try:
        user = pwd.getpwnam(c["robot_user"])
        env_args = [f"{key}={value}" for key, value in ros_environment(c).items()]
        cmd = ["/usr/sbin/runuser", "-u", user.pw_name, "--", "/usr/bin/env", *env_args,
               *ros_command(c, ["/usr/bin/python3", str(HERE/"ros_stop.py"),
                                "--seconds", str(max(.1, c["ros_stop_seconds"]-.2))])]
        result = run_bounded(cmd, c["ros_stop_seconds"])
        if result:
            LOG.error("ROS stop helper returned %s; proceeding with service stop", result)
    except Exception:
        LOG.exception("ROS stop stage failed; continuing")
    try:
        # systemd sends SIGINT to the launch supervisor and owns all descendants.
        result = run_bounded(["/usr/bin/systemctl", "stop", c["robot_service"]], c["process_stop_seconds"])
        if result:
            LOG.error("Robot service stop returned %s", result)
    except Exception:
        LOG.exception("ROS process stop deadline/failure; OS shutdown will stop remaining services")
    # Do not call UPS output-off registers: both published manuals disagree.
    # Do not run blocking sync here; systemd performs filesystem shutdown.
    for attempt in range(3):
        try:
            if run_bounded(["/usr/bin/systemctl", "--no-block", "poweroff"], 1.) == 0:
                return
        except Exception:
            LOG.exception("OS poweroff request failed (attempt %s)", attempt+1)
    raise RuntimeError("OS poweroff was not accepted")


def monitor(c):
    if c["mode"] == "armed" and os.geteuid() != 0:
        raise PermissionError("Armed monitor requires system service/root")
    if c["mode"] == "armed":
        LOG.warning("PROTECTION ACTIVE: confirmed power loss, UPS unsafe state or sustained "
                    "communication loss will stop local ROS and power off THIS computer")
        if c.get("computer_power_source", "ups") == "external":
            LOG.info("Computer has independent power; UPS backup runtime check is not applicable. "
                     "Set computer_power_source=ups when the computer is connected to UPS output.")
    # Never let two guards race on socket requests, readiness or shutdown.
    lock = open("/run/power-loss-protection.lock" if os.geteuid() == 0
                else f"/tmp/power-loss-protection-{os.getuid()}.lock", "a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if c["mode"] == "armed":
        Path(c["ready_file"]).unlink(missing_ok=True)
        try:
            latch = json.loads(Path(c["latch_file"]).read_text())
            boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
            if latch.get("boot_id") == boot_id:
                shutdown(c, "Shutdown committed earlier in this boot")
                return
            LOG.warning("Previous-boot shutdown latch retained; robot launch remains blocked")
        except FileNotFoundError:
            pass
        except (OSError, ValueError, AttributeError):
            LOG.exception("Unreadable latch retained; robot launch remains blocked")
    decision = Decision(time.monotonic(), c["confirm_count"], c["communication_loss_seconds"])
    previous = None
    reported = False
    while True:
        started = time.monotonic()
        sample = None
        try:
            sample = read_sample(c)
            summary = (sample["status"], sample["fault"])
            if summary != previous:
                LOG.info("UPS %s", json.dumps(display_telemetry(sample)))
                previous = summary
        except (OSError, ProtocolError) as error:
            LOG.warning("UPS read failed: %s", error)
        reason = decision.update(time.monotonic(), sample)
        if c["mode"] == "armed":
            if sample and sample["main"] and not reason:
                atomic_json(c["ready_file"], dict(monotonic=time.monotonic(), main=True))
            else:
                Path(c["ready_file"]).unlink(missing_ok=True)
        if reason and not reported:
            shutdown(c, reason)
            # Observe mode continues reading after recording the latched event.
            if c["mode"] == "armed":
                return
            reported = True
        time.sleep(max(0., c["poll_seconds"]-(time.monotonic()-started)))


def can_start(c):
    if c["mode"] != "armed" or Path(c["latch_file"]).exists():
        return False
    try:
        state = json.loads(Path(c["ready_file"]).read_text())
        age = time.monotonic()-state["monotonic"]
        return state["main"] is True and 0 <= age < c["communication_loss_seconds"]
    except (OSError, ValueError, KeyError, TypeError):
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["check", "probe", "monitor", "run", "can-start", "launch"])
    parser.add_argument("--config", default=str(HERE/"config.json"))
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    c = load_config(args.config)
    if args.command == "check":
        print(f"Valid configuration; mode={c['mode']}; transport={c['transport']}")
    elif args.command == "probe":
        for transport in ("rtu_over_tcp", "modbus_tcp"):
            try:
                print(json.dumps(dict(transport=transport,
                                      telemetry=display_telemetry(read_sample(c, transport))),
                                 ensure_ascii=False))
            except (OSError, ProtocolError) as error:
                print(f"{transport}: {error}")
    elif args.command == "can-start":
        return 0 if can_start(c) else 1
    elif args.command == "launch":
        if not can_start(c):
            raise RuntimeError("UPS readiness missing or shutdown latched; launch blocked")
        env = os.environ.copy()
        env.update(ros_environment(c))
        os.execve("/usr/bin/bash", ros_command(c, c["launch_command"]), env)
    else:
        if args.command == "run" and c["mode"] != "armed":
            raise ValueError("Formal run requires mode=armed; use monitor with observe config for read-only monitoring")
        monitor(c)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
    except Exception:
        LOG.exception("Power loss protection failed")
        sys.exit(1)
