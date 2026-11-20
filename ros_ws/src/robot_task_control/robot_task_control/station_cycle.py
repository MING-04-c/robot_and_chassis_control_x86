"""复用当前配方的机械臂往返点位，生成终点-中途拍照-终点的任务段。"""
from copy import deepcopy


def station_cycle_steps(original):
    camera = next(i for i, s in enumerate(original) if s['action'] == 'camera_detection')
    forward = next(i for i, s in enumerate(original) if s['action'] == 'line_follow' and s.get('drive_direction') == 'forward')
    reverse = next(i for i, s in enumerate(original) if i > camera and s['action'] == 'line_follow' and s.get('drive_direction') == 'reverse')
    outward = deepcopy(original[forward+1:camera])
    inward = deepcopy(original[camera+1:reverse])
    if not outward or not inward or inward[-1].get('posture') != 'travel':
        raise ValueError('中途往返必须有完整的去程、回程以及行车关节位')
    to_middle = deepcopy(original[reverse])
    to_middle.update(name='reverse_to_photo_station', stop_at_intermediate=True,
                     skip_intermediate_markers=False, route_segment='station')
    to_final = deepcopy(original[forward])
    to_final.update(name='forward_to_final_station', finish_at_first_marker=False,
                    skip_intermediate_markers=True, route_segment='forward')
    home = deepcopy(original[reverse])
    home.update(name='middle_to_charging_dock', route_segment='home', skip_intermediate_markers=True)
    return dict(inward=inward, outward=outward, to_middle=to_middle, to_final=to_final, home=home)


def excursion_steps(original):
    parts = station_cycle_steps(original)
    return [*parts['inward'], parts['to_middle'], *parts['outward'],
            dict(action='camera_detection', station_role='mid_photo', step_timeout_sec=1200.),
            *deepcopy(parts['inward']), parts['to_final'], *deepcopy(parts['outward']),
            dict(action='camera_detection', station_role='auto_terminal', step_timeout_sec=1200.),
            *deepcopy(parts['inward']), deepcopy(next(s for s in original if s['action']=='line_follow' and s.get('drive_direction')=='reverse')),
            dict(action='wait_chassis_stopped', stable_seconds=1., step_timeout_sec=15.)]


def middle_home_steps(original):
    parts = station_cycle_steps(original)
    return [*parts['inward'], parts['home'], dict(action='wait_chassis_stopped', stable_seconds=1., step_timeout_sec=15.)]
