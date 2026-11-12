"""Build a remote-session recipe from the existing production motion recipe."""
from copy import deepcopy
import ipaddress
from pathlib import Path
import yaml

from robot_task_control.task_config import load_task_config


def load_oline_config(path):
    path = Path(path).expanduser().resolve(strict=True)
    values = yaml.safe_load(path.read_text())
    if not isinstance(values, dict) or set(values) != {'base_config_file', 'receiver_ip', 'hk_bind_ip'}:
        raise ValueError('oline config requires base_config_file, receiver_ip, hk_bind_ip')
    for name in ('receiver_ip', 'hk_bind_ip'):
        ipaddress.IPv4Address(values[name])
    base = Path(values['base_config_file']).expanduser()
    if not base.is_absolute():
        base = path.parent/base
    _, document = load_task_config(base)
    config = deepcopy(document)
    config['system']['auto_start'] = False
    config['tcp'].update(enabled=True, enable_status_monitor=True)
    config['workflow'].update(name='robot_task_oline', remote_detection_cycles=True)
    config['camera_detection_task']['ros__parameters'].update(target_source='tcp', auto_start=False,
                                                              capture_local=True)
    config['capture'].update(local_only=False, receiver_ip=values['receiver_ip'], hk_bind_ip=values['hk_bind_ip'])
    return config
