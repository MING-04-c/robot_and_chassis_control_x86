#!/usr/bin/env python3
"""麒麟端共享配置和TCP完整性校验请求；只使用 Python 标准库。"""
import argparse
import json
import math
from pathlib import Path
import socket
import time
from uuid import uuid4

from industrial_pc_protocol import authenticate, envelope, load_key, read_message, write_message, validate_heartbeat, perform_handshake


def parser(description):
    result = argparse.ArgumentParser(description=description)
    result.add_argument('--config', default=str(Path(__file__).with_name('config.json')))
    result.add_argument('--host', help='覆盖树莓派实际IP')
    return result


def configuration(args):
    """从配置文件读取地址；密钥和输出目录的相对路径均相对于配置文件。"""
    path = Path(args.config).expanduser().resolve()
    config = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(config, dict):
        raise ValueError('config.json 必须为JSON对象')
    if args.host:
        config['robot_host'] = args.host
    if not isinstance(config.get('robot_host'), str) or not config['robot_host'].strip():
        raise ValueError('请在 config.json 填写树莓派实际 robot_host')
    for name in ('control_port', 'hk_port', 'rgbd_port'):
        if type(config.get(name)) is not int or not 1 <= config[name] <= 65535:
            raise ValueError(name+' 必须为1～65535整数')
    config.setdefault('use_shared_key', False)
    if type(config['use_shared_key']) is not bool:
        raise ValueError('use_shared_key 必须为bool')
    for name in (('output_dir', 'key_file') if config['use_shared_key'] else ('output_dir',)):
        if not isinstance(config.get(name), str) or not config[name].strip():
            raise ValueError(name+' 必须为非空路径')
        value = Path(config[name]).expanduser()
        config[name] = str(value if value.is_absolute() else path.parent / value)
    for name, default in (('timeout', 5), ('status_interval', 2)):
        value = config.get(name, default)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(name+' 必须为有限正数')
        config[name] = value
    config.setdefault('rgbd_bind', '0.0.0.0')
    config.setdefault('heartbeat_enabled', True)
    config.setdefault('handshake_enabled', True)
    if type(config['handshake_enabled']) is not bool:
        raise ValueError('handshake_enabled 必须为bool')
    if type(config['heartbeat_enabled']) is not bool:
        raise ValueError('heartbeat_enabled 必须为bool')
    value = config.setdefault('heartbeat_timeout', 10.)
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError('heartbeat_timeout 必须为有限正数')
    if config['heartbeat_enabled'] and value <= config['timeout']+config['status_interval']:
        raise ValueError('heartbeat_timeout 必须大于 timeout + status_interval')
    return config


def request(config, payload, retries=2):
    """同一请求内保持相同ID重试；拒绝回执不会换ID重复发命令。"""
    key = load_key(config['key_file']) if config.get('use_shared_key', False) else None
    packet = envelope(payload, key)
    for attempt in range(retries+1):
        try:
            with socket.create_connection((config['robot_host'], config['control_port']),
                                          timeout=config['timeout']) as connection:
                connection.settimeout(config['timeout'])
                # 心跳 ping/pong 本身就是一次只读应用握手，不重复发 hello。
                if config.get('handshake_enabled', False) and payload.get('schema') != 'paint_robot_heartbeat_v1':
                    perform_handshake(connection, key)
                write_message(connection, packet)
                response = authenticate(read_message(connection), key)
            if (response.get('schema') != 'paint_v54_pi_result_ack_v1' or
                    response.get('message_id') != payload['message_id']):
                raise ValueError('回执类型或message_id不匹配')
            return response
        except (OSError, EOFError):
            if attempt == retries:
                raise
            time.sleep(.5)


def status(config):
    return request(config, {'schema': 'paint_robot_status_request_v1',
                            'message_id': uuid4().hex}, retries=0)


def heartbeat(config, session_id, sequence):
    payload = dict(schema='paint_robot_heartbeat_v1', message_id=uuid4().hex,
                   session_id=session_id, sequence=sequence)
    validate_heartbeat(payload)
    response = request(config, payload, retries=0)
    echo = response.get('heartbeat')
    if (response.get('ok') is not True or not isinstance(echo, dict) or
            echo.get('session_id') != session_id or type(echo.get('sequence')) is not int or
            echo['sequence'] != sequence or not isinstance(echo.get('server_id'), str) or
            not echo['server_id']):
        raise ValueError('心跳应答无效；请同步更新两端，旧机器人可配置 heartbeat_enabled=false')
    return response
