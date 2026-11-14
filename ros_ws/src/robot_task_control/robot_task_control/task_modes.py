"""Select local/online operation without duplicating motion and safety recipes."""
from copy import deepcopy


def mode_config(document, mode='local', workflow='full', auto_start=None):
    if mode not in ('local', 'online') or workflow not in ('full', 'line', 'photo'):
        raise ValueError('mode requires local/online; workflow requires full/line/photo')
    if auto_start is not None and type(auto_start) is not bool:
        raise ValueError('auto_start must be boolean')
    config = deepcopy(document)
    system, flow = config['system'], config['workflow']
    flow.update(task_mode=mode, task_profile=workflow,
                remote_detection_cycles=mode == 'online' and workflow != 'line')
    flow['name'] = 'robot_task_'+mode+'_'+workflow
    if mode == 'online':
        system['auto_start'] = False
        config['tcp'].update(enabled=True, enable_status_monitor=True)
        config['camera_detection_task']['ros__parameters'].update(
            target_source='tcp', auto_start=False, capture_local=True)
        config['capture']['local_only'] = False
    else:
        config['camera_detection_task']['ros__parameters']['target_source'] = 'local'
        config['capture']['local_only'] = True
    if auto_start is not None:
        system['auto_start'] = auto_start
    # Full and line profiles return along the standard route to the charging dock.
    system['line_charging_mode'] = mode == 'online' and workflow != 'photo'
    camera = config['camera_detection_task']['ros__parameters']
    camera['low_battery_return_enabled'] = mode == 'online' and workflow == 'full'
    camera['low_battery_threshold'] = 10.0
    if workflow == 'photo':
        flow['steps'] = [s for s in flow['steps'] if s['action'] == 'camera_detection']
    elif workflow == 'line':
        flow['steps'] = [s for s in flow['steps']
                         if s['action'] in ('wait_chassis_stopped', 'line_follow')]
    if not flow['steps']:
        raise ValueError('Selected workflow contains no steps')
    return config
