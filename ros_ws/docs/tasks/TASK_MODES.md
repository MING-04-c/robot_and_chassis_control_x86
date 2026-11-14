# local / online 任务入口

所有入口共用 `robot_task_control/config/robot_task.yaml` 中的点位、速度、导纳和力保护。
同一时间只启动一个任务 launch。模式通过启动参数选择，不复制维护整套 YAML。

```bash
cd /home/robot/project/robot_and_chassis_control-main/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

local（本地点位）——完整流程：

```bash
ros2 launch robot_task_control robot_task.launch.py \
  mode:=local workflow:=full
```

local（本地点位）——仅循迹往返：

```bash
ros2 launch robot_task_control robot_task.launch.py \
  mode:=local workflow:=line
```

local（本地点位）——从当前拍照位开始检测：

```bash
ros2 launch robot_task_control robot_task.launch.py \
  mode:=local workflow:=photo
```

online（电脑坐标）——完整流程：

```bash
ros2 launch robot_task_control robot_task.launch.py \
  mode:=online workflow:=full
```

online（远程模式）——仅循迹往返：

```bash
ros2 launch robot_task_control robot_task.launch.py \
  mode:=online workflow:=line
```

online（电脑坐标）——从当前拍照位开始检测：

```bash
ros2 launch robot_task_control robot_task.launch.py \
  mode:=online workflow:=photo
```

`full` 包含行车位确认、正向循迹、机械臂到拍照位、检测、收臂、反向循迹。
`line` 只执行循迹往返，运动前仍确认机械臂处于行车位，不去拍照位、不检测。
`photo` 从当前准备好的拍照位开始，不启动底盘或循迹相机，不执行过渡点或底盘返程。
photo 模式的 go_home 仅结束检测会话、保持拍照位，不执行回桩充电。

## 启动来源

local 默认按主配置自动启动；online 默认等待 start。两种模式都可加
`auto_start:=false`，改为等待本机 ROS 服务或远程 TCP start，二者使用同一任务入口。
两种模式也都支持 `auto_start:=true`；online 自动启动只推进到拍照位，仍需 detect 才采图。

本机启动整个已加载流程：

```bash
ros2 service call /task_controller/start std_srvs/srv/Trigger '{}'
```

远端启动整个流程：

```bash
python3 start_task.py --action start
```

先用 `validate_only:=true` 可仅校验配置、不启动设备。
full/line 需要已验收的标准路线；photo 不依赖路线文件。

## local

使用主配置 `camera_detection_task.ros__parameters.camera_points_mm` 的本地毫米坐标。
到拍照位后按现有流程采图保存本机、检测一轮、每点回拍照原位；full 随后收臂并循迹回起点。
本地点位须由操作员按实际现场填写。local 的返程沿用普通终点停车，不启动充电搜索。

## online

远端地址使用主配置 `capture.receiver_ip`；本机监听使用 `capture.hk_bind_ip`。
旧的 `robot_task_oline.launch.py` 与 `robot_task_oline.yaml` 保留兼容，并使用其中的地址覆盖。

1. start 启动流程；full 先循迹到站，再经两个关节过渡位、两个中间点到拍照位。
2. 就绪及静止检查通过后进入 WAIT_DETECT，等待 detect；此时不自动拍照。
3. 收到 detect 后生成新批次，重新冻结实际相机/探针位姿，采集并传输 HK、RGB、深度图。
4. 等待电脑确认本批三图校验保存（WAIT_SAVED），然后等待本批照片对应的相机毫米坐标（WAIT_RESULT）。
5. 执行真实导纳接近和测厚，每点安全撤离、回拍照原位，并回传本次读数和回位确认。
6. 本轮结束进入 WAIT_NEXT。再次 detect 从第3步重新采图；go_home 结束会话并放行返程。
7. full/line 的返程沿标准路线回起点，再复用现有末段视觉回中和充电桩对接；收到连续有效充电反馈才完成。

完整协议和照片校验规则见 [远程协议](../tcp/OLINE流程与协议.md)。本机也可推进 online：

```bash
ros2 service call /camera_detection_task/detect std_srvs/srv/Trigger '{}'
ros2 service call /camera_detection_task/go_home std_srvs/srv/Trigger '{}'
```

## 低电量返航

online full 使用有效 BMS 电量 **严格低于 10%** 触发自动回桩，10% 本身不触发。
电量与通信反馈均须在3秒内，且带有效源时间戳；未知、断流或无效电量不当作0%。
状态中提供 `battery_percent`、`low_battery_return_pending` 和 `return_reason`。

返航请求一旦锁定，不再接受新 detect。尚未检测时结束采集/坐标等待并返回；
若检测动作已启动，先让当前整批检测按原保护逻辑完成撤离、回拍照原位，再收臂返航。
若检测或安全状态故障，则保持停止，不绕过故障强行返航。
若前进途中已收到低电量，请求会保留，到拍照阶段不启动检测而直接按既有返程路径返回。
online line 本来就是循迹往返并回桩，不安排到站检测等待；photo 不启用自动底盘返航。

未完成真机联调前，离线测试只证明软件状态和协议行为，不能代替路线、机械臂及充电桩现场验证。
