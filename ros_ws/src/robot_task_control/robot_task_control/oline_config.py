"""Build a remote-session recipe from the existing production motion recipe."""
import ipaddress
from pathlib import Path
import yaml

from robot_task_control.task_config import load_task_config
from robot_task_control.task_modes import mode_config


def load_oline_config(path):
    path = Path(path).expanduser().resolve(strict=True)
    values = yaml.safe_load(path.read_text())
    required = {'base_config_file', 'receiver_ip', 'hk_bind_ip'}
    if not isinstance(values, dict) or not required <= set(values) or set(values)-required-{'station_return_enabled'}:
        raise ValueError('oline config requires base_config_file, receiver_ip, hk_bind_ip')
    if type(values.get('station_return_enabled', False)) is not bool:
        raise ValueError('station_return_enabled must be boolean')
    for name in ('receiver_ip', 'hk_bind_ip'):
        ipaddress.IPv4Address(values[name])
    base = Path(values['base_config_file']).expanduser()
    if not base.is_absolute():
        base = path.parent/base
    _, document = load_task_config(base)
    config = mode_config(document, 'online')
    config['workflow'].update(name='robot_task_oline', remote_detection_cycles=True)
    config['camera_detection_task']['ros__parameters'].update(target_source='tcp', auto_start=False,
                                                              capture_local=True)
    config['capture'].update(local_only=False, receiver_ip=values['receiver_ip'], hk_bind_ip=values['hk_bind_ip'])
    if values.get('station_return_enabled'):
        config['camera_detection_task']['ros__parameters']['station_return_enabled'] = True
        for step in config['workflow']['steps']:
            if step['action'] == 'line_follow' and step.get('drive_direction') == 'forward':
                step.update(finish_at_first_marker=False, skip_intermediate_markers=True)
    return config
