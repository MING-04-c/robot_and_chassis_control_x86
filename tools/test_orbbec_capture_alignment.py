"""Offline checks for the ROS capture recipe; never imports the Windows SDK."""
import ast
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
ROS = ROOT/'ros_ws/src'


def windows_constants():
    tree = ast.parse((ROOT/'Orbbec_Portable_TEST/capture_orbbec_v54_windows.py').read_text())
    constants = {}
    for statement in tree.body:
        if not isinstance(statement, ast.Assign):
            continue
        try:
            value = ast.literal_eval(statement.value)
        except (ValueError, TypeError):
            continue
        for target in statement.targets:
            if isinstance(target, ast.Name):
                constants[target.id] = value
            elif isinstance(target, ast.Tuple):
                constants.update((name.id, item) for name, item in zip(target.elts, value))
    return constants


def test_ros_stream_specifications_match_portable_windows():
    expected = windows_constants()
    config = yaml.safe_load((ROS/'geminipro_camer_calibration/config/gemini_pro_capture.yaml').read_text())
    params = config['/**']['ros__parameters']
    assert (params['color_width'], params['color_height'], params['color_fps']) == (
        expected['COLOR_W'], expected['COLOR_H'], expected['FPS'])
    assert (params['depth_width'], params['depth_height'], params['depth_fps']) == (
        expected['DEPTH_NATIVE_W'], expected['DEPTH_NATIVE_H'], expected['FPS'])
    assert params['color_format'] == 'MJPG'
    assert params['depth_format'] == 'Y11'
    assert params['depth_registration'] is True
    assert params['align_mode'] == 'SW'
    assert params['align_target_stream'] == 'COLOR'
    assert params['frame_aggregate_mode'] == 'full_frame'
    assert params['enable_frame_sync'] is False
    assert params['enable_v54_capture'] is True
    assert params['color_flip'] is params['depth_flip'] is False
    assert params['color_rotation'] == params['depth_rotation'] == -1


def test_imaging_defaults_are_explicit_not_invented_manual_values():
    config = yaml.safe_load((ROS/'geminipro_camer_calibration/config/gemini_pro_capture.yaml').read_text())
    params = config['/**']['ros__parameters']
    assert params['enable_color_auto_exposure'] is True
    assert params['enable_color_auto_white_balance'] is True
    assert params['enable_color_auto_exposure_priority'] is False
    assert params['color_exposure'] == params['color_gain'] == params['color_white_balance'] == -1


def test_same_frameset_capture_retains_warmup_and_strict_checks():
    expected = windows_constants()
    capture = (ROS/'gemini_pro_ros2/orbbec_camera/src/v54_capture.cpp').read_text()
    assert f"warmup_ = {expected['WARMUP_FRAMESETS']};" in capture
    assert f"std::chrono::milliseconds({int(expected['CAPTURE_TIMEOUT_SEC']*1000)})" in capture
    assert 'frames->colorFrame()' in capture and 'frames->depthFrame()' in capture
    assert 'delta > 100 || nowMs() > ct + 1000 || nowMs() > dt + 1000' in capture
    assert 'ct <= previous_color_' in capture and 'dt <= previous_depth_' in capture
    assert 'depth->getValueScale()' in capture
    assert 'response.depth_raw.encoding = "16UC1";' in capture
