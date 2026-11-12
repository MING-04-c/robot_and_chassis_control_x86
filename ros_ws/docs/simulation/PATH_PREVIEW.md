> 项目自写模块说明已集中到 docs；原位置：ros_ws/src/robot_task_control/simulation/README.md。文中相对命令路径仍按原模块执行，整机启动以[统一文档入口](../README.md)为准。

# 路径规划仿真预览

```bash
cd ~/project/robot_and_chassis_control-main/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_task_control path_preview.launch.py
```

启动后自动规划，通过 MoveIt 标准轨迹话题在 RViz 连续播放一次，不需要调用任务启动服务。
使用每帧 `0.1 s` 的固定间隔播放，方便观察；该预览播放速度不代表真机执行速度。隐藏场景中重复的静止机器人，只保留轨迹动画和碰撞场景。
打开原生 `Planned Motion - Trajectory Slider` 面板，使单次播放结束后机器人仍显示在最后一帧，而不是被 MoveIt 隐藏。
规划失败时只预览成功部分一次，停在最后有效点，不跳过失败段、不自动循环。
仅启动 MoveIt、模型 TF、虚拟关节状态、地面场景、预览节点和 RViz。
固定独立 ROS_DOMAIN_ID=190 / LOCALHOST；不启动硬件驱动、底盘、相机、串口、
保护恢复或轨迹控制器，MoveIt 禁止执行轨迹。不要把域 190 用于真实设备。
这是运动学规划预览，不是物理、接触力或底盘动力学仿真。

## 点位和显示

- `path_preview.yaml` 设置规划时间、速度缩放、初始姿态及点位文件路径。
- 默认直接读取 `config/diagnostics/arrival_detect.yaml`，关节角和 RPY 单位为度，XYZ 为米。
- 从 YAML 的 `travel` 开始，逐段接续规划终点：过渡位、中间点一、中间点二、拍照位、检测预接近、名义检测点、撤离、返回拍照位、中间点二、中间点一、过渡位、行车位。
  回程复用去程点的位置和朝向，每段重新规划并检查碰撞，不直接反放原轨迹。
- 中间位使用 base_link / tool0：XYZ `[0.300, 0.410, 0.795]` 米，RPY `[-90, 45, -45]` 度。
- 使用现有 LS65 URDF/SRDF、完整末端及侧装盒碰撞几何、Z=0.08m 地面，碰撞检查不豁免运动部件。
- 默认恢复为原来的 OMPL RRTConnect，直接规划带碰撞检查的绕障路径，不先尝试 PTP。
  本轮规划结束后才发布预览；增加中间位不保证可达，失败时仍停止后续规划。
- 普通位姿目标直接交给 OMPL 求解，不预选并锁定某个 IK 关节解；不保证全局最短。
- 检测预接近与真机采用相同的探针长度姿态容差预算；检测段使用主配置的速度/加速度缩放和笛卡尔步长。
  真机配置的两种候选姿态选择仍关闭；预览不支持直接启用这些真机开关。
- RViz 只显示机器人、地面和连续的 `Planned Motion` 动画，不叠加彩线或标签。
- 检测点复用正式任务的相机/探针变换和探头安装补偿，接近/撤离做带碰撞检查的直线规划。
  相机毫米点仍是 YAML 中的占位测试点；这里只显示名义检测位置，不模拟力触碰、不做额外探入，不生成照片或厚度结果。
  测厚点若无碰撞逆解/直线路径不完整，会明确失败，不能把预览当作实际接触验证。
- 规划失败即停止后续段，成功部分只播放一次并停住；JSON 保持 `FAILED`，显示状态为 `PARTIAL_STOP`，不冒充完整往返。
  终端和 JSON 报告保留成功段和失败点、错误码或碰撞对。
- 报告默认 `/tmp/robot_task_path_preview.json`。随机采样规划每次可能不同；仿真结果不等于真机可直接执行。
- 检查分段边界并拼成一条时间连续的显示轨迹，不改规划路径、不抹去实际停顿。
  最终动画独立使用 `/path_preview/display_planned_path`，不接收 MoveIt 自动发布的原始分段轨迹。
  RViz 订阅后等待 2 秒再发布一次；不再每 3 秒补发，不自动循环，避免未闭合路径跳回起点。
  这只修复播放时序，不改变真机规划、速度或 GPU 设置；不能消除软件渲染本身的帧率限制。
- 在启动终端按 Ctrl+C 退出；关闭 RViz 窗口也会关闭这组仿真，子进程退出等待有上限。

指定另一份同格式的机械臂点位 YAML：

```bash
ros2 launch robot_task_control path_preview.launch.py workflow_file:=/绝对路径/点位.yaml
```

只计算规划、不打开窗口：`rviz:=false`。不支持的步骤/相对目标会报错，不会冒充已验证。
修改源码配置后需重新 `colcon build --packages-select robot_task_control --executor sequential`
并重新 source；也可以用 `config_file:=/绝对路径/path_preview.yaml` 直接读取源码配置。

## 仿真专用检测朝向试验

`simulation/path_preview.yaml` 的 `detection_tilt_limit_deg: 15.0` 开启仅仿真的局部 X/Y 倾斜搜索。
先尝试拍照原朝向，再以 5° 为间隔、八个方向尝试倾角，最大 15°；设为 `0.0` 恢复原朝向规划。
每个候选都固定拍照时换算出的**同一个基座探针接触 XYZ**，重新计算 tool0 安装补偿和沿新工具 +Z 退后 20mm 的预接近点。
这会改变接触方向，不会修改真实任务 YAML、检测控制器或原相机毫米点。

只有候选的预接近 OMPL 路径、名义接触直线、撤离直线和返回拍照关节原位全部规划成功后，才加入动画。
候选路径一直保留碰撞、地面及关节检查，接近和撤离的笛卡尔完成比例要求至少 0.999。
失败候选不会进入动画；候选全部失败则停止后续段，保留拍照位之前的成功轨迹。
报告的 `detection_orientation_trials` 保存每次角度、接触点、接近轴和拒绝原因。
此试验只预览名义接触点，不验证实际表面法向、接触力或额外 50mm 探入行程，不能直接当作真机执行验证。

规划库参考：[MoveIt OMPL](https://moveit.picknik.ai/main/doc/examples/ompl_interface/ompl_interface_tutorial.html)。预览不执行真机运动。
