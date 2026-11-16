> 本文唯一源为 ros_ws/docs/tcp，麒麟交付包中的说明由导出工具生成。参见同目录 [工具说明](README.md)。

# oline 多轮漆膜检测

远程模式由麒麟发指令推进。统一入口为 `robot_task.launch.py mode:=online`；
旧入口 `robot_task_oline.launch.py` 保留兼容。local、online 的 full/line/photo 选择见
[任务模式](../tasks/TASK_MODES.md)。各入口不能同时启动。

## 启动

机器人端（ros_ws 内）先构建并加载环境：

```bash
colcon build --packages-select eli_cs_robot_serial robot_task_control --executor sequential
source install/setup.bash
ros2 launch robot_task_control robot_task_oline.launch.py validate_only:=true
# 校验通过且现场准备就绪后，启动设备；工作流等待麒麟start：
ros2 launch robot_task_control robot_task_oline.launch.py
```

配置为 src/robot_task_control/config/robot_task_oline.yaml，receiver_ip 必须填麒麟实际 IP。
当前示例沿用正式配置的 169.254.199.250；hk_bind_ip=0.0.0.0 是机器人本机监听地址。
该文件引用 robot_task.yaml 中的去程、回程点位、速度、力保护与测厚参数，不另复制运动点。
麒麟 config.json 的 robot_host 填机器人 IP，output_dir 是麒麟收图保存目录。

麒麟端先保持一个监听终端：

```bash
python3 listen_robot.py
```

监听器使用 Python 标准库，连接9010查询状态、接收9002的HK、监听9030接收RGB-D。
三图按同一采集编号保存并校验，自动向机器人发送“本批HK、RGB、深度图已经保存”的确认。
新入口必须同步更新监听器；旧监听器没有保存确认，会停在WAIT_SAVED并最终报超时。

## 完整执行顺序

1. 启动launch后不自动执行工作流，等待start。设备启动和驱动准备仍按共用启动栈执行。
2. 麒麟发start；机器人确认设备、底盘停稳和行车姿态，前进到站。
3. 机械臂经过渡位1、过渡位2、中间点一、中间点二，到拍照位。
4. 机器人在拍照位完成就绪和静止检查，进入WAIT_DETECT；等待detect，此时不自动采图。
5. 麒麟发detect，机器人重新稳定采集本批HK、RGB、深度图，发给麒麟保存，进入WAIT_SAVED。
6. 麒麟确认三图SHA256与本机采集一致后进入WAIT_RESULT；再发与本批照片清单哈希匹配的相机毫米坐标。
   仅发坐标不能越过检测指令。收到本批有效点后执行真实导纳限力漆膜检测。
7. 每一点均接近、接触停稳、保持测厚、撤离、返回同一拍照关节原位，回传带读数的逐点回位结果。
8. 本轮全部成功后进入WAIT_NEXT，继续停在拍照位，不自动返回或倒车。
9. 再发detect，机器人重新拍照和发送新三图，确认麒麟保存后进入WAIT_RESULT；
   麒麟必须针对新照片发新坐标，然后重复第7、8步。可多轮重复，同一个point_id允许跨轮使用。
10. 麒麟在WAIT_DETECT或WAIT_NEXT发go_home，机器人结束远程检测会话，
    依次返回中间点二、中间点一、过渡位2、过渡位1、行车位；小车沿标准路线倒车回起点充电桩，
    末段视觉对齐并有限距离对接，连续有效充电反馈确认后停车结束。

online full收到有效新鲜电量低于10%时自动锁定返回充电；新detect会被拒绝。
未开始检测时结束照片/坐标等待并返航；检测已启动时先完成当前整批检测并确认回拍照原位。
故障、外控失联或保护锁定时不放行返程。电量未知或过期不当作0%，10%本身不触发。

接近/接触运动中发go_home会被拒绝，须等待本轮回位后再发。
stop 是立即请求中止、保持停止的命令，不自动回家，也不保证命令回执时硬件已经停止。
故障/保护锁定期间不强行返回，按现场恢复流程处理。
本入口仍用正式检测朝向和真实力保护，未启用仿真的倾角搜索。

## 麒麟命令

在另一个终端依次使用：

```bash
python3 start_task.py --action start
# 状态WAIT_DETECT后：
python3 start_task.py --action detect
# 查询状态确认WAIT_RESULT，再发送当前照片的结果文件：
python3 send_points.py --file result.json
# 本轮结束，状态WAIT_NEXT后，可重新拍照并再检测：
python3 start_task.py --action detect
# 等新的三图已保存、状态WAIT_RESULT后：
python3 send_points.py --file next_result.json
# 本轮回位、状态WAIT_NEXT后结束任务：
python3 start_task.py --action go_home
python3 start_task.py --action status
# 需要中止时：
python3 start_task.py --action stop
```

start_task.py 的ACK表示受理，进度/完成需查询状态。每个新操作使用新message_id；
网络重发同一操作必须沿用原ID。脚本一次调用内自动复用ID，--message-id可显式指定。
oline中禁止独立start_capture.py/capture抢占相机；拍照统一由会话管理。

## TCP报文

仍使用四字节大端长度＋JSON，以及原protocol/payload/payload_sha256封装和可选HMAC。
这里只展示封装内payload；应使用随包的envelope、write_message和request函数。

```json
{"schema":"paint_robot_task_request_v1","message_id":"unique-start-001","action":"start"}
```

action可为start、detect、go_home、stop（resume仅用于旧途中站流程）。
每次detect都重新采图，包括首轮；到拍照位时没有预先采集的照片，不能复用上一轮坐标。

```json
{
  "schema": "paint_v54_pi_remote_result_v1",
  "message_id": "unique-points-001",
  "status": "OK_3D_POINTS",
  "capture_manifest_sha256": "这里必须是当前照片真实的64位小写SHA256",
  "coordinate_frame": "CAMERA_PINHOLE_PRELIMINARY",
  "depth_source": "MEASURED",
  "robot_motion_authorized": false,
  "robot_base_coordinate_calibrated": false,
  "robot_command": null,
  "points_xyz_camera_mm": [{"point_id":"point-1","xyz_camera_mm":[0.0,0.0,130.0]}]
}
```

上例仅说明字段，不能直接执行：SHA256占位无效，点也是示例。
真实hash从本批.capture_index.json的sessions[采集编号].rgbd.capture_manifest_sha256获取，
也会出现在robot_status.camera_detection.data.capture_manifest_sha256。
坐标须来自本批真实图像/深度，单位毫米，不把旧点改个hash继续发。
当前机器人配置使用模型相机安装 frame terminal_link_1，它不自动等于 optical frame；
必须校验光学点到配置 frame 的外参，不能凭 PRELIMINARY 标签认定已完成手眼标定。
旧照片、DEMO、错误阶段的点会拒绝；重复的已存结果只返回duplicate，不再次执行。

保存确认由新版listen_robot.py自动发送：

```text
schema: paint_robot_capture_saved_v1
message_id: saved-<capture_set_id>
capture_set_id: 当前拍照run_id
capture_manifest_sha256: 当前采集清单SHA256
files_sha256: {"hk.jpg": "...", "rgb.png": "...", "depth.png": "..."}
```

只有三图真实落盘且文件校验一致才能发送。自研麒麟接收程序也要实现这一步。
HK9002流本身仍无落盘ACK，新增确认通过9010同时覆盖HK和RGB-D，不改变两条图像通道格式。

## 状态、电池和异常

listen_robot.py默认约每2秒通过心跳/状态请求获取并保存当前状态。
无需另开运动控制节点，不额外占用CAN口。

| 字段 | 含义 |
|---|---|
| robot_status.task.data | 整个工作流的步骤、阶段、失败原因和是否结束 |
| robot_status.camera_detection.data | oline阶段、当前run_id、照片hash、已完成轮数 |
| robot_status.camera_detection.data.low_battery_return_pending / return_reason | 低电量返航锁定；返回原因low_battery或go_home |
| robot_status.chassis_power.battery_percent | 电量百分比；无有效新反馈为null，不伪造为0 |
| robot_status.chassis_power.charging | 是否充电，未知为null |
| robot_status.chassis_power.voltage_v / current_a | 电池电压、电流 |
| robot_status.abnormal_status.has_fault | 当前是否存在故障 |
| robot_status.abnormal_status.active_faults | 错误码、来源、原因和设备原始码 |
| robot_status.detection_results | 最近逐点测厚结果，包含本次读数和回位确认 |

新增检测错误码：E5001检测故障、E5002已回位但本轮需重试、E5003检测状态断流。
原有机器人保护锁、规划/执行失败、底盘/CAN异常和ROS错误日志继续回传。
照片/坐标/检测执行各有超时；WAIT_DETECT和WAIT_NEXT可长期等待主机，
但仍监控真实外控、检测状态与安全条件，不因为等待而忽略状态断流。
通信恢复后不会自动补发start/detect/go_home，操作者需核对当前状态。
