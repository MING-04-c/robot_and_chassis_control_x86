# 电池功率监控

订阅 `masc_chassis_msgs/msg/BmsStatus` 类型的 `/masc_chassis/bms_status`，
用 `abs(bat_voltage × bat_current)` 计算功率大小，并发布 `std_msgs/msg/Float64` 到
`/power/watts`。消息里的电压、电流已经是 V、A，不需要再乘 CAN 协议的 0.1。

功率单位 W，曲线和 `/power/watts` 均显示非负功率大小。
例如 48.9 V、-1.5 A 对应 73.35 W。标题中的电流保留原始正负号，
图中不根据符号直接标记充电或放电。

## 启动

先启动已有的底盘驱动或机器人全流程，确认 BMS 话题正在发布，再在图形桌面终端运行：

```bash
cd ~/project/robot_and_chassis_control-main/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select eli_cs_robot_serial
source install/setup.bash
ros2 launch eli_cs_robot_serial power_plot.launch.py
```

折线图默认显示最近120秒，窗口标题实时显示 V、A、W。关闭窗口或 Ctrl+C 退出。
本 launch 只启动功率监控，不重复启动底盘驱动。

```bash
# 改为最近5分钟
ros2 launch eli_cs_robot_serial power_plot.launch.py window_seconds:=300.0
# 无桌面，仅计算和发布功率
ros2 launch eli_cs_robot_serial power_plot.launch.py show_plot:=false
ros2 topic echo /power/watts
```

BMS 标记超时、消息采样时间超过2秒、电压无效或数值非有限时，不发布功率。
曲线断开并显示数据失效，不把无效数据画成0 W。历史样本最多保留10000条。
