# 路径规划仿真预览

```bash
cd ~/project/robot_and_chassis_control-main/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_task_control path_preview.launch.py
```

启动后自动规划，通过 MoveIt 标准轨迹话题在 RViz 连续循环播放，不需要调用任务启动服务。
使用原 MoveIt RViz 配置中的 `0.05 s` 逐点显示时间；隐藏场景中重复的静止机器人，只保留轨迹动画和碰撞场景。
取消逐段自动停留；规划失败时只循环播放已成功的前缀，不跳过失败段。
仅启动 MoveIt、模型 TF、虚拟关节状态、地面场景、预览节点和 RViz。
固定独立 ROS_DOMAIN_ID=190 / LOCALHOST；不启动硬件驱动、底盘、相机、串口、
保护恢复或轨迹控制器，MoveIt 禁止执行轨迹。不要把域 190 用于真实设备。
这是运动学规划预览，不是物理、接触力或底盘动力学仿真。

## 点位和显示

- `path_preview.yaml` 设置规划时间、速度缩放、初始姿态及点位文件路径。
- 默认直接读取 `config/diagnostics/arrival_detect.yaml`，关节角和 RPY 单位为度，XYZ 为米。
- 从 YAML 的 `travel` 开始，逐段接续规划终点：过渡位、中间位、拍照位、检测预接近、名义检测点、撤离、返回拍照位、过渡位、行车位。
- 中间位使用 base_link / tool0：XYZ `[0.300, 0.410, 0.795]` 米，RPY `[-90, 45, -45]` 度。
- 使用现有 LS65 URDF/SRDF、完整末端及侧装盒碰撞几何、Z=0.08m 地面，碰撞检查不豁免运动部件。
- 默认恢复为原来的 OMPL RRTConnect，直接规划带碰撞检查的绕障路径，不先尝试 PTP。
  完成规划后才发布预览；增加中间位不保证可达，失败时仍停止后续规划。
- 位姿目标恢复为直接交给 OMPL 求解，不再预选并锁定某个 IK 关节解；不保证全局最短。
- RViz 只显示机器人、地面和连续的 `Planned Motion` 动画，不叠加彩线或标签。
- 检测点复用正式任务的相机/探针变换和探头安装补偿，接近/撤离做带碰撞检查的直线规划。
  相机毫米点仍是 YAML 中的占位测试点；这里只显示名义检测位置，不模拟力触碰、不做额外探入，不生成照片或厚度结果。
  测厚点若无碰撞逆解/直线路径不完整，会明确失败，不能把预览当作实际接触验证。
- 规划失败即停止后续段，保留成功的前缀轨迹；终端和 JSON 报告显示失败点、错误码或碰撞对。
- 报告默认 `/tmp/robot_task_path_preview.json`。随机采样规划每次可能不同；仿真结果不等于真机可直接执行。
- 保留原始分段轨迹，通过标准 `/display_planned_path` 话题每 3 秒补发。
  RViz 设置 `Interrupt Display: false`，补发不打断正在播放的轨迹，连续循环由 RViz 控制。
- 在启动终端按 Ctrl+C 退出；关闭 RViz 窗口也会关闭这组仿真，子进程退出等待有上限。

指定另一份同格式的机械臂点位 YAML：

```bash
ros2 launch robot_task_control path_preview.launch.py workflow_file:=/绝对路径/点位.yaml
```

只计算规划、不打开窗口：`rviz:=false`。不支持的步骤/相对目标会报错，不会冒充已验证。
修改源码配置后需重新 `colcon build --packages-select robot_task_control --executor sequential`
并重新 source；也可以用 `config_file:=/绝对路径/path_preview.yaml` 直接读取源码配置。

规划库参考：[MoveIt OMPL](https://moveit.picknik.ai/main/doc/examples/ompl_interface/ompl_interface_tutorial.html)。正式任务的规划配置未修改。
