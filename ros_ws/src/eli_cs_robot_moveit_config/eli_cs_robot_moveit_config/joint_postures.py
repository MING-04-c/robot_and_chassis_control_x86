"""共用关节姿态：输入度制YAML，输出ROS弧度目标及MoveIt约束。"""
import math
from pathlib import Path
import yaml
from moveit_msgs.msg import Constraints, JointConstraint

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def load_postures(path):
    """读取行车/过渡位及可选过渡位2；不取模，保留指定关节分支。"""
    values = yaml.safe_load(Path(path).expanduser().read_text())
    output = {}
    keys = ('travel', 'detection_ready') + (('detection_ready_2',) if 'detection_ready_2' in values else ())
    for key in keys:
        angles = values[key]
        if (not isinstance(angles, list) or len(angles) != 6 or
                any(isinstance(x, bool) or not isinstance(x, (int, float)) or
                    not math.isfinite(x) for x in angles)):
            raise ValueError(key+'必须是六个有限角度，单位度')
        output[key] = [math.radians(x) for x in angles]
    return output


def joint_constraints(positions, tolerance=.005):
    """输入六轴弧度值，输出纯关节目标，不添加笛卡尔路径约束。"""
    if len(positions) != 6 or not all(math.isfinite(x) for x in positions):
        raise ValueError('关节目标必须包含六个有限弧度值')
    return Constraints(joint_constraints=[JointConstraint(
        joint_name=name, position=float(value), tolerance_above=tolerance,
        tolerance_below=tolerance, weight=1.) for name, value in zip(JOINTS, positions)])
