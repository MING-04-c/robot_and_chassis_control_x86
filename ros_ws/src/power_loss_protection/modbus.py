"""Read-only Modbus client: transparent RTU over TCP or native Modbus TCP."""
import socket
import struct
import time


class ProtocolError(ValueError):
    pass


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return crc


def recv_exact(sock, count, deadline):
    result = bytearray()
    while len(result) < count:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("Modbus response deadline exceeded")
        sock.settimeout(remaining)
        chunk = sock.recv(count - len(result))
        if not chunk:
            raise ConnectionError("Gateway closed connection")
        result.extend(chunk)
    return bytes(result)


def read_registers(host, port, slave, transport, timeout, start=0, count=9):
    """One fresh connection per request; old/partial replies cannot be reused."""
    if not 1 <= slave <= 247 or not 1 <= count <= 125 or not 0 <= start <= 65535-count+1:
        raise ValueError("Invalid read address/count/slave")
    if transport not in ("rtu_over_tcp", "modbus_tcp"):
        raise ValueError("Choose rtu_over_tcp or modbus_tcp explicitly")
    deadline = time.monotonic() + timeout
    pdu = struct.pack(">BHH", 3, start, count)
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(max(0.001, deadline-time.monotonic()))
        if transport == "rtu_over_tcp":
            request = bytes([slave]) + pdu
            sock.sendall(request + struct.pack("<H", crc16(request)))
            head = recv_exact(sock, 3, deadline)
            if head[0] != slave:
                raise ProtocolError("Wrong slave address")
            if head[1] & 0x80:
                frame = head + recv_exact(sock, 2, deadline)
                if crc16(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
                    raise ProtocolError("Bad exception CRC")
                raise ProtocolError(f"Modbus exception {head[2]}")
            if head[1:] != bytes([3, count*2]):
                raise ProtocolError("Wrong function or byte count")
            frame = head + recv_exact(sock, count*2+2, deadline)
            if crc16(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
                raise ProtocolError("Bad CRC")
            data = frame[3:-2]
        else:
            # One transaction per connection; transaction ID must still match.
            sock.sendall(struct.pack(">HHHB", 1, 0, 6, slave) + pdu)
            tid, protocol, length, unit = struct.unpack(">HHHB", recv_exact(sock, 7, deadline))
            if (tid, protocol, unit) != (1, 0, slave) or not 2 <= length <= 254:
                raise ProtocolError("Invalid MBAP header")
            response = recv_exact(sock, length-1, deadline)
            if response[0] & 0x80:
                raise ProtocolError(f"Modbus exception {response[1]}")
            if response[:2] != bytes([3, count*2]) or len(response) != count*2+2:
                raise ProtocolError("Wrong function or byte count")
            data = response[2:]
        return list(struct.unpack(">" + "H"*count, data))


def telemetry(registers):
    if len(registers) < 9:
        raise ProtocolError("Missing telemetry registers")
    status = registers[6]
    # The operating state must be unambiguous. Fault/ESTOP bits are independent.
    state = status & 0xf
    if state not in (1, 2, 4, 8) or registers[5] > 100:
        raise ProtocolError(f"Unusable telemetry/status: {status:#06x}")
    return dict(vmain_mv=registers[0], vcap_mv=registers[1],
                vout_mv=registers[2], iout_ma=registers[4], soc=registers[5],
                status=status, fault=registers[7], firmware=f"0x{registers[8]:04x}",
                main=state == 1 and not bool(status & 0x10), backup=state == 2)


def display_telemetry(sample):
    """Convert register units for output without changing internal telemetry."""
    units = {"vmain_mv": "vmain_v", "vcap_mv": "vcap_v",
             "vout_mv": "vout_v", "iout_ma": "iout_a"}
    return {units.get(key, key): value / 1000.0 if key in units else value
            for key, value in sample.items()}
