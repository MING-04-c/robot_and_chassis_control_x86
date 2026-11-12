# 当前代码导航

入口和参数说明见[任务指南](tasks/ROBOT_TASK_GUIDE.md)，这里仅列维护位置。
旧版代码地图见[归档](archive/CODE_MAP_CN.md)，不要照其中已退休的 launch 启动。

| 职责 | 唯一维护位置（相对 ros_ws） |
|---|---|
| 自动单轮启动、公共设备栈 | `src/robot_task_control/launch/robot_task.launch.py`、`launch/internal/robot_system.launch.py` |
| oline 启动/主配置覆盖 | `src/robot_task_control/launch/robot_task_oline.launch.py`、`robot_task_control/oline_config.py` |
| 去回程点位、力和相机参数 | `src/robot_task_control/config/robot_task.yaml`，诊断覆盖在 `config/diagnostics/` |
| 全流程顺序/互锁 | `src/robot_task_control/robot_task_control/workflow_controller.py` |
| 采图取点、坐标批次、检测调度 | `src/robot_task_control/robot_task_control/camera_detection_task.py` |
| 远程多轮等待/检测/回家阶段 | `src/robot_task_control/robot_task_control/camera_detection_oline.py` |
| 稳定采图和图像发送 | `src/robot_task_control/robot_task_control/final_capture.py` |
| 真机限力接触、测厚、撤离回位 | `src/detect_control/scripts/detect_controller.py` |
| 仿真路径/局部倾角试验 | `src/robot_task_control/simulation/path_preview.py`、`path_preview.yaml` |
| TCP 协议与机器人唯一桥 | `src/eli_cs_robot_serial/device/industrial_pc/src/` |
| 麒麟工具唯一代码源 | `src/eli_cs_robot_serial/device/industrial_pc/host_tools/` |
| 麒麟导出工具 | 仓库根目录 `tools/export_industrial_pc.py` |
| 相机采集与坐标变换 | `src/geminipro_camer_calibration/` |
| 模型与末端安装 TF | `src/eli_cs_robot_description/urdf/` |
| 底盘循迹/驱动 | `src/masc_chassis_user_control/`、`src/masc_chassis_ros_driver/` |

先改工作流/参数表达的内容；只有新增行为才改执行器。不要为 oline 另复制真机测厚、
TCP、CAN 或相机驱动。文档源统一在本目录，麒麟发布副本用导出工具生成。
