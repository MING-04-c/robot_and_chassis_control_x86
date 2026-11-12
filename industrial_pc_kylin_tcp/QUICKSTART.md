# 麒麟 oline 快速启动

机器人端在已构建的 ros_ws 加载 ROS 和工作区环境；先修改
`src/robot_task_control/config/robot_task_oline.yaml` 的麒麟实际 IP，重新构建并加载安装环境。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select eli_cs_robot_serial robot_task_control --executor sequential
source install/setup.bash
ros2 launch robot_task_control robot_task_oline.launch.py validate_only:=true
# 校验不是运动安全证明，确认现场就绪后再启动：
ros2 launch robot_task_control robot_task_oline.launch.py
```

麒麟解压新版 `industrial_pc_kylin_tcp.zip`，进入包目录，核对 `config.json` 中的机器人 IP 和保存目录。
开放/连通 9010、9002、9030，先启动一个常驻终端：

```bash
python3 listen_robot.py
```

另一终端依次操作：

```bash
python3 start_task.py --action start
# 到站三图保存确认后 WAIT_DETECT：
python3 start_task.py --action detect
# WAIT_RESULT：发送本批真实三维点，result.json 不是包内 DEMO 文件。
python3 send_points.py --file result.json
# 本轮测完回拍照原位 WAIT_NEXT，可再发 detect：
python3 start_task.py --action detect
# 等新三图确认、WAIT_RESULT 后，发送新结果：
python3 send_points.py --file next_result.json
# WAIT_NEXT 时回家；也允许首次 WAIT_DETECT 时不测直接回家：
python3 start_task.py --action go_home
```

`python3 start_task.py --action status` 查询阶段；`--action stop` 请求中止，不自动回家。
运动中 go_home 被拒绝；重发同一指令复用 message_id，新操作使用新 ID。
不要把上述注释当固定延时，需要观察实际阶段。
详细条件、坐标报文、三图保存确认及故障字段见 [OLINE流程与协议](OLINE流程与协议.md)。
