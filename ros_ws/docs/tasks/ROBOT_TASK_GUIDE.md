# 任务操作指南

所有命令在 `ros_ws` 目录执行。真实运动前确认工作区域、工具/相机安装和外控安全状态，
关闭其他占用机械臂、底盘、相机或 TCP 9010 的任务。下列模式不能并行运行。

## 启动与配置

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select eli_cs_robot_serial robot_task_control --executor sequential
source install/setup.bash
# 仅校验，不启动设备、不运动：
ros2 launch robot_task_control robot_task.launch.py validate_only:=true
```

上述增量构建假设完整工作区及依赖已经构建；新机见[迁移说明](../MIGRATION_JAZZY_CN.md)。

| 模式 | 启动方式和行为 |
|---|---|
| 自动单轮真机 | `ros2 launch robot_task_control robot_task.launch.py`，当前主配置 `auto_start: true` |
| 麒麟分步、多轮真机 | `ros2 launch robot_task_control robot_task_oline.launch.py`，等待麒麟 start，见 [oline](../tcp/OLINE流程与协议.md) |
| 到站机械臂诊断 | 用下面的 `arrival_detect.yaml`；固定底盘，不启动底盘驱动/循迹，设备就绪后自动运动 |
| 仅最后拍照测厚 | 用下面的 `capture_detect.yaml`；启动前已在拍照位，设备就绪后自动运动触碰 |
| 只预览规划 | `ros2 launch robot_task_control path_preview.launch.py`，独立域 190，无硬件运动 |

```bash
# 以下仍为仅校验；确认现场安全后去掉 validate_only:=true 才执行诊断。
ros2 launch robot_task_control robot_task.launch.py station_workflow_file:="$PWD/src/robot_task_control/config/diagnostics/arrival_detect.yaml" validate_only:=true
ros2 launch robot_task_control robot_task.launch.py station_workflow_file:="$PWD/src/robot_task_control/config/diagnostics/capture_detect.yaml" validate_only:=true
```

主配置：[robot_task.yaml](../../src/robot_task_control/config/robot_task.yaml)。
远程覆盖：[robot_task_oline.yaml](../../src/robot_task_control/config/robot_task_oline.yaml)，
它复用主配置的运动点位及保护参数，不复制另一套路径。
诊断同样继承主配置的设备、相机、限力参数，仅覆盖自己的工作流和测试点。
可用 `config_file:=/绝对路径/robot_task.yaml` 指定主配置；安装目录文件需重新构建才更新。

## 中间点、拍照位及返回

下表均为 **base_link 下 tool0 的目标**，不是镜头光心或探针尖端。
XYZ 为米，RPY 为固定轴欧拉角、单位度。

| 点位 | YAML 名称 | XYZ（米） | RPY（度） |
|---|---|---|---|
| 中间点一 | `to_camera_via` / `camera_via` 锚点 | `[0.150, 0.530, 0.810]` | `[75, 20, 170]` |
| 中间点二 | `to_camera_intermediate` / `camera_intermediate` 锚点 | `[0.300, 0.410, 0.795]` | `[-90, 45, -45]` |
| 拍照位 | 第三个 `arm_move`；诊断名 `to_photo_pose` | `[0.375, 0.360, 0.590]` | `[-175, 0, -90]` |

另有两个六轴关节姿态，顺序为基座/肩/肘/手腕1/手腕2/手腕3，单位度：

- 行车位 `travel`：`[-45, 15, -71, 4.4, 148, 75]`。
- 过渡位 `detection_ready`：`[-45, 0, 0, 0, 90, 0]`，不是上表的笛卡尔中间点。

去程：行车位 → 过渡关节位 → 中间点一 → 中间点二 → 拍照位。
每个检测点结束先回实际记录的拍照关节原位。
离站：拍照原位 → 中间点二 → 中间点一 → 过渡关节位 → 行车位 → 底盘倒车返程。
回程通过 YAML 锚点复用去程位置和朝向，每段重新规划、检查碰撞，不逐帧反放去程轨迹。
主任务在底盘运动前自动插入行车位确认；到站诊断显式返回行车位，但不驱动小车。

oline 每轮结束停在拍照位等待，不立即离站；只有允许阶段收到 `go_home` 才执行上述离站过程。
仅最后阶段的 `capture_detect.yaml` 没有中间点和过渡动作，测完只回当前记录的拍照原位。
故障时不承诺强行走完返回路径。

## 相机与检测坐标

系统有三类图像来源，不要混用：

- `system.line_camera_device`：底盘 USB 循迹相机；默认关闭预览但继续采集/识别。
- `capture.hk_image_topic`：海康 HK，采集 JPEG；端口 9002。
- 深度相机：RGB 及原始 uint16 深度，端口 9030；配对、内参与深度比例按本次采集清单读取。

`camera_frame: terminal_link_1` 是当前采用的模型相机安装坐标，
`probe_frame: terminal_link_2` 是探针尖端，运动控制点仍为 `tool0`。
实际镜头位姿由 URDF/TF 安装变换决定，不等于上表拍照位 XYZ。
**模型安装坐标不自动等于相机 optical frame**；真实光学点必须经已验证的安装外参转到配置坐标，
不能仅凭接口中的 `CAMERA_PINHOLE_PRELIMINARY` 标签认定已经标定。
末端安装模型见[模块说明](../modules/TERMINAL_MODEL.md)。

`camera_points_mm` 为相机坐标的毫米点，多个点按 `[x1,y1,z1,x2,y2,z2]` 填：
主任务当前是 `[-10,-10,130]`；两个诊断当前均为 `[0,0,130]`。
这些是测试占位值，不是实物可达/碰撞安全或标定正确的证明。
只改正在使用的配置；修改诊断点不会自动改变主任务。
oline 覆盖为 `target_source: tcp`，不使用这些本地测试点，必须等本批照片对应的真实坐标。

拍照保存由 `capture` 配置；主任务当前 `local_only: true`，oline 覆盖为 `false`。
`receiver_ip` 填远端麒麟 IP；`hk_bind_ip` 是机器人本机监听地址，oline 默认 `0.0.0.0`。
不要把远端地址当成本机绑定地址。主配置保留的 HK 绑定值在本地保存模式下不用于发送，
如手工启用远传必须一并核对，优先使用 oline 覆盖配置。

## 参数、等待与排错

动作与超时在 `workflow.steps`；关节速度/启动等待在 `arm_joint_postures`；
本地/TCP 点源在 `camera_detection_task`；接触速度和限力在 `detect_controller`；
采图保存/发送在 `capture`；测厚串口在 `thickness_detector`；公共状态桥在 `tcp`。
修改后重启生效，不支持运行中热改。

到位等待 0.5 秒、采图前停稳 0.5 秒、相机预热 3 秒是固定等待；
`step_timeout_sec`、结果/通信超时是最长允许时间，不是固定停留。
接触保持 5 秒是测厚窗口，不是恒力控制。详细流程见[漆膜检测](PAINT_DETECTION.md)。

查看 `/task_controller/status` 的 `step_index`（从 0 起）、`phase`、`failure_reason`，
以及 `/camera_detection_task/status`、`/detect_controller/status`。
`stop` 是取消并保持停止，不等于正常 `go_home`；TCP 应答成功也不代表运动已经完成。
保护锁、外控断开、力数据断流或碰撞失败时先处理原因，不绕过保护强行回家。
