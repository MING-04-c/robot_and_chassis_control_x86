from setuptools import find_packages, setup

package_name = "robot_task_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/robot_task.launch.py"]),
        ("share/" + package_name + "/launch/internal", ["launch/internal/robot_system.launch.py"]),
        ("share/" + package_name + "/config", ["config/robot_task.yaml"]),
        ("share/" + package_name + "/config/diagnostics", ["config/diagnostics/arrival_detect.yaml"]),
        ("share/" + package_name + "/simulation", [
            "simulation/path_preview.launch.py", "simulation/path_preview.yaml",
            "simulation/path_preview.rviz", "simulation/README.md",
        ]),
    ],
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
        ],
    },
)
