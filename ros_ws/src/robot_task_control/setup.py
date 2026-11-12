from setuptools import find_packages, setup
from pathlib import Path

package_name = "robot_task_control"

# 项目说明的唯一源在工作区 docs，安装时直接从该目录复制，不在包内维护副本。
documentation_root = Path('../../docs')
documentation_files = [
    (str(Path('share')/package_name/'docs'/directory.relative_to(documentation_root)),
     [str(path) for path in sorted(directory.iterdir())
      if path.is_file() and path.suffix in ('.md', '.rst')])
    for directory in [documentation_root, *sorted(p for p in documentation_root.rglob('*') if p.is_dir())]
]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/robot_task.launch.py", "launch/robot_task_oline.launch.py"]),
        ("share/" + package_name + "/launch/internal", ["launch/internal/robot_system.launch.py"]),
        ("share/" + package_name + "/config", ["config/robot_task.yaml", "config/robot_task_oline.yaml"]),
        ("share/" + package_name + "/config/diagnostics", [
            "config/diagnostics/arrival_detect.yaml", "config/diagnostics/capture_detect.yaml",
        ]),
        ("share/" + package_name + "/simulation", [
            "simulation/path_preview.launch.py", "simulation/path_preview.yaml",
            "simulation/path_preview.rviz",
        ]),
    ] + documentation_files,
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    scripts=["simulation/path_preview.py"],
    maintainer="robot",
    maintainer_email="robot@example.com",
    description="Execute local chassis and Elite robot YAML workflows.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "velocity_mux = robot_task_control.velocity_mux:main",
            "task_controller = robot_task_control.workflow_controller:main",
            "arm_joint_postures = robot_task_control.joint_posture_control:service_main",
            "camera_detection_task = robot_task_control.camera_detection_task:main",
            "camera_detection_oline = robot_task_control.camera_detection_oline:main",
        ],
    },
)
