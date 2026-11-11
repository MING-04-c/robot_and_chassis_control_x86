# 任务控制包

正式联动任务只保留一个入口 `launch/robot_task.launch.py`，唯一任务配置为
`config/robot_task.yaml`。任务顺序、延时、点位、速度、关节姿态、拍照保存和检测参数均从该文件读取。
修改后重新启动任务生效，不支持运行中热改参数。

## 启动与检查

```bash
cd ~/project/robot_and_chassis_control-main/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# 仅检查配置和路线，不启动任何节点或设备：
ros2 launch robot_task_control robot_task.launch.py validate_only:=true
# 实机执行，先确保运动区域安全、设备连接正常：
ros2 launch robot_task_control robot_task.launch.py
```

两条命令分别执行。实机启动前关闭独立相机预览和其他占用机械臂、底盘、相机或 TCP 9010 的任务。
也可以通过 `config_file:=/绝对路径/robot_task.yaml` 指定同格式文件。

底盘相机画面默认不显示。需要查看时将 YAML 的 `system.line_show_window` 改为
`true` 后重新启动任务；`false` 只关闭预览，不影响相机采集和循迹。

### 到站拍照测厚诊断

点位文件为 `config/diagnostics/arrival_detect.yaml`，复用同一执行器，不是另一套正式联动任务。
顺序：当前位到过渡位、拍照位、拍照并触碰测厚回拍照原位、过渡位、行车位。
不启动底盘驱动、速度选择器或循迹相机，不执行小车往返。先关闭原任务及键盘遥控，
固定小车并确认运动区域安全；不允许与正式任务并行启动。

```bash
ros2 launch robot_task_control robot_task.launch.py station_workflow_file:=$(ros2 pkg prefix robot_task_control)/share/robot_task_control/config/diagnostics/arrival_detect.yaml
```

该模式默认等待手动开始。在另一个已加载相同 ROS 环境的终端，确认安全恢复完成后执行：

```bash
ros2 service call /task_controller/start std_srvs/srv/Trigger "{}"
```

在启动命令末尾加 `validate_only:=true` 可仅检查配置，不启动设备。
诊断 YAML 中 `travel`、`detection_ready` 是六轴角度（度）；`target_pose.position` 是
`base_link` 下 `tool0` 的米制目标，`orientation` 是 RPY 度；`camera_points_mm` 是相机坐标系
的毫米点，不是机械臂基座坐标。默认 `[10, 10, 140]` 只是原任务测试点，必须核对实物再触碰。
只排查规划时可暂时删掉 `capture_and_thickness` 整个步骤。

拍照目录、测厚串口、限力阈值及相机配置仍来自 `config_file` 指定的正式 YAML。
照片默认只存桌面。诊断保留外控、控制器就绪、地面与碰撞检查、保护锁及真实限力测厚；
任何一步失败都会停止，不跳过失败去强行回位或测量。查看 `/task_controller/status` 的
`step_index`（从0开始）、`phase`、`failure_reason` 可定位失败阶段。

## 当前工作流

1. 设备就绪、底盘停稳，机械臂到行车位。
2. 起步视觉对齐，沿原标准路线前进；忽略起点横线，在下一条有效横线停车。
3. 底盘停稳，机械臂到中转位，再到拍照位。
4. 拍照位：base_link 下 XYZ `[0.375, 0.360, 0.590]` 米，RPY `[-175, 0, -90]` 度，到位后等 0.5 秒。
5. 本地拍照保存，使用相机坐标 `[10, 10, 140]` 毫米作为当前测试点；转换、补偿探针偏置后限力触碰，读取串口测厚，撤离并回拍照原位。
6. 回中转位，再到行车位。
7. 沿标准路线倒车，途中站自动继续，到原出发点完成视觉定位、停稳结束；不搜索充电。

前进 0.1 m/s，倒车上限 0.05 m/s。重放标准文件为
`~/.ros/line_follow/standards/route_01/standard_route.json`；本任务不录制、不覆盖它。
路线基准仍为 URDF 修改前的 107 点路线，不因本次整理重新选线。

## 修改 YAML

| 配置位置 | 修改内容 |
|---|---|
| `workflow.steps` | 按顺序执行的动作、每步超时；可增删或调整步骤 |
| `system` | IP、循迹相机设备、画面显示、速度、处理帧率、线程和启动稳定等待 |
| `travel`、`detection_ready` | 六轴关节角，单位度，顺序为基座/肩/肘/手腕1/手腕2/手腕3 |
| `arm_joint_postures.ros__parameters` | 关节运动速度缩放、首次 MoveIt 就绪等待 |
| `camera_detection_task.ros__parameters` | 本地毫米点或 TCP 点源、检测就绪等待、采集和结果超时 |
| `detect_controller.ros__parameters` | 限力接近、保持、撤离、读数和检测超时 |
| `capture` | 桌面保存目录、预热、稳定帧数、本地保存或 TCP 传输 |
| `thickness_detector` | 串口设备、波特率、数据目录 |
| `tcp` | 公共 TCP 桥、状态监视开关 |

例如固定等待用 `action: delay` 和 `seconds`，笛卡尔位姿用 `action: arm_move`，
拍照测厚用 `action: camera_detection`。删除拍照测厚相关步骤即可得到不检测的往返流程，
不再复制一份 Python 或 YAML 任务版本。

`workflow.arm_postures_before_chassis: true` 会在每次底盘运动前自动插入行车姿态确认，
同时保留底盘/机械臂互锁；正常使用应保持开启。每个动作完成后才进入下一步，失败保持停车。
配置格式、工作流、关节姿态、拍照参数、路线结构会先校验，再创建驱动启动动作。
这不是可达性、碰撞自由或实机安全的证明，实际规划和硬件检查仍由运行时执行。

### 延时和超时不是同一件事

| 字段 | 当前值 | 含义 |
|---|---|---|
| `system.arm_control_start_delay` | 0.5 秒 | 外控连接稳定后的启动等待 |
| `arm_joint_postures.ros__parameters.initial_settle_sec` | 5 秒 | 本节点首次执行时等待 MoveIt 客户端发现；只等一次 |
| `arm_move.after_arrival_delay_sec` | 0.5 秒 | 到拍照位后的固定等待 |
| `capture.robot_settle_sec` | 0.5 秒 | 开始采集前的固定等待 |
| `capture.camera_warmup_sec` | 3 秒 | 相机预热，然后检查画面稳定 |
| `camera_detection_task.ros__parameters.ready_stable_sec` | 2 秒 | 检测条件持续就绪时间 |
| `detect_controller.ros__parameters.hold_seconds` | 5 秒 | 实际接触测厚保持时间，不是行驶等待 |
| `line_follow.marker_confirm_timeout_sec` | 5 秒 | 停止线确认超时上限，不是每次固定等 5 秒 |
| `boundary_wait_sec`、`progress_timeout_sec` | 各 5 秒 | 边界确认/跟踪进度的超时上限 |
| `step_timeout_sec` | 按步骤 | 整个动作最长耗时上限 |

其他工作流等待期限集中在 `workflow` 下，包括设备检查、就绪持续时间、相机启停服务和恢复画面。
底层驱动、通信、控制周期和安全监控仍由各功能包维护，不把它们改成工作流动作。

## 目录与职责

- `launch/robot_task.launch.py`：读取并分发同一个 YAML，启动共用设备和控制节点。
- `robot_task_control/task_config.py`：正式任务配置检查。
- `robot_task_control/workflow_controller.py`：唯一工作流执行器，负责顺序、完成确认、失败/取消和安全互锁。
- `robot_task_control/camera_detection_task.py`：采图、匹配本次点位、坐标转换及触发检测。
- `robot_task_control/final_capture.py`：复用相机驱动和通讯实现，稳定采图后保存或发送。
- `robot_task_control/joint_posture_control.py`：共用关节姿态执行服务。
- `robot_task_control/velocity_mux.py`：速度仲裁，禁止过期命令持续驱动。
- `launch/internal/`：共用驱动控制栈，不是另一套正式任务。
- `simulation/`：独立域、仅规划的 RViz 路径预览；不连接真实设备，详见该目录 README。
- `test/`：当前实现的回归测试及隔离测试夹具，不安装到 ROS 包，不连接实机运动硬件。

旧任务归档、独立调试入口、像素仿真程序及配套配置/RViz 文件已删除。
正式任务只保留 `robot_task.launch.py` 一个入口、内部设备栈和一份正式任务配置。
`config/diagnostics/arrival_detect.yaml` 仅用于到站点位排查，不是另一套联动任务版本。
主机 TCP 唯一实现在 `eli_cs_robot_serial/device/industrial_pc/`，本包不复制通讯代码。

安全停机、真实硬件审计、地面/碰撞检查、力数据新鲜度和人工恢复要求保留。
正常工作流由控制器自动调度，不需要手工调用逐步服务；真正保护停机后必须检查并按安全流程恢复。
旧任务名称已退休，请用上面的统一启动命令，历史任务文档中的旧命令不再作为正式入口。

## 仅预览规划路径

```bash
ros2 launch robot_task_control path_preview.launch.py
```

该仿真入口自动读取到站排查 YAML，显示行车位、过渡位、拍照位及返回路径。
独立 ROS 域 190 / LOCALHOST，仅做运动学规划，不执行真机运动；显示名义检测位置，但不仿真拍照、力触碰或厚度结果。
点位在 `config/diagnostics/arrival_detect.yaml`，预览参数和 RViz 配置集中在 `simulation/`。
