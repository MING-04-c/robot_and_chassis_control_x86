# Ubuntu 24.04 / ROS 2 Jazzy 部署

当前迁移目标为本机实测的 Ubuntu 24.04（Noble）、x86_64、ROS 2 Jazzy。
统一脚本也保留 ARM64 构建支持；本次 x86_64 验证不能替代 ARM64 的重新验证。
Ubuntu 22.04 不是本次验证环境。旧 `src/README_CN.md` 的 Ubuntu 22.04 / Humble 要求属于上游驱动的历史说明。

## 安装与构建

在新的 Bash 终端中执行，以下相对路径均以 `ros_ws` 为工作目录：

```bash
cd /home/robot/project/robot_and_chassis_control-main/ros_ws
bash scripts/install_robot_dependencies.sh
ROBOT_BUILD_JOBS=4 bash scripts/build_robot.sh
source scripts/robot_env.bash
```

依赖安装会通过 sudo 请求密码；不要使用 sudo 编译工作空间。
安装脚本按与构建相同的包集合调用 rosdep，包含 MoveIt Servo 等运行依赖。
Elite SDK 的 Boost、libssh、Eigen、yaml-cpp 及 Pillow ImageTk 由 apt 显式安装；
当前 rosdep 数据库无法解析的 `ament_python` 和 `python3-pil.imagetk` 键单独处理。

SDK 编译安装到当前工作空间的 `local/elite_sdk`，不依赖原树莓派的二进制文件。
使用固定的 `lib` 安装路径，运行环境脚本据此加载动态库。
`local/` 和 `build_elite_sdk/` 保留 `COLCON_IGNORE`，防止普通 `colcon build`
把 SDK 安装目录中的 `package.xml` 当成源码；构建脚本会在清理后重建这些标记。
首次使用统一脚本完成 SDK 引导后，可在 `ros_ws` 加载 `source scripts/robot_env.bash`
并直接执行 `colcon build` 做增量编译。环境脚本同时导出 SDK 的 CMake 查找路径和动态库路径。
相机 SDK 根据当前架构选择仓库内的 amd64/x64 或 arm64 库。
`--cmake-force-configure` 确保补装依赖后重新发现头文件和库。

构建入口通过 `--packages-up-to` 解析 `robot_task_control`、机械臂校准包和 Orbbec 模型包的完整本地依赖，
包含当前 USB 循迹包 `ruier_camer_diver`、深度相机和检测控制包。
旧 `eli_cs_robot_simulation_gz` 依赖 `ign_ros2_control`，不属于此次真机控制栈的构建范围；
需要 Gazebo 时应另外适配 Jazzy 的 `gz_ros2_control`，不能将本次假硬件检查等同于 Gazebo 验证。

旧 `build_raspberry_pi.sh`、`raspberry_pi_env.bash` 转发到统一入口，已有启动习惯仍可沿用。
脚本要求 Jazzy，并在已有其他 ROS 发行版环境时提示使用新终端。

## 无实体硬件验证

完成构建并加载 `scripts/robot_env.bash` 后：

```bash
python3 src/robot_task_control/test/test_arm_jazzy_startup.py
```

此检查在独立 ROS 域使用 mock 硬件，验证四个控制器激活、MoveIt 规划初始化、
IK 服务及末端模型加载，不发送实体机械臂运动命令。ROS 通信需要本机 socket 权限。
它验证的是启动就绪，不能证明各子进程退出正常；本机测试中的 MoveIt 退出异常见下方验证记录。

已有纯算法测试：

```bash
python3 -m pytest -q \
  src/masc_chassis_user_control/test/test_line_follow_control.py \
  src/masc_chassis_user_control/test/test_forward_navigation.py \
  src/masc_chassis_user_control/test/test_exposure_control.py
```

## 本次适配与验证记录（2026-11-09）

- 已安装并检查本地控制栈所需的系统依赖，包括单独的 `ros-jazzy-moveit-servo`。
- 机械臂 SDK 及 21 个 ROS 包完成 x86_64 / Jazzy 编译。
- 六轴和五轴 mock 模型为速度缩放状态设置标准 `initial_value=1.0`。
  原自定义 GPIO 参数不能替代 GenericSystem 的状态初值。
- 缩放轨迹控制器对 NaN、无穷、负数及大于 1 的反馈暂停虚拟时钟；
  有效反馈恢复后继续推进，避免无效浮点数进入纳秒时间计算。
  新增暂停及恢复回归用例。
- 四个必要控制器激活、MoveIt IK 服务及末端模型的假硬件启动检查通过。
- 循迹、路线和曝光纯算法测试 149 项通过；控制器、轨迹时间、图像处理及驻车 C++ 用例 61 项通过；
  另有两组 CAN SDK 协议／运行时测试及三项假力传感器测试通过。
- 机械臂、海康及 Orbbec 驱动动态库均未发现缺失的链接依赖，两个常用任务入口可解析启动参数。
- 历史集成测试已同步当前启动行为：工作流测试写入 `original_steps`；
  预览测试显式启用按需预览；倒车巡航检查等待当前加速度所需的爬升时间。
  这些调整不修改生产控制的超时、速度或设备检查。
- 假相机／底盘测试覆盖丢线停车、急停、断联、横线等待／继续、方向切换、路线倒车、
  视觉停车以及普通／重放、在桩／离桩四种起步对齐场景，均已通过对应检查。
  任务控制 10 项用例中，9 项在套件运行中通过；一项采图联动曾因并行测试的调度延迟
  触发 150ms 里程计超时，停止并行负载后单独复查通过。未放宽生产超时参数。

仍存在的验证限制：

- `eli_cs_robot_usr` 原有 `flake8`、`uncrustify` 检查未通过，主要是已有源码格式；
  本次没有整包重排格式，因此不能宣称完整 `colcon test` 全绿。
- 本机安装的 MoveIt 2.12.4 在启动检查完成、收到 SIGINT 后退出时，
  曾在 `TrajectoryExecutionManager` / `rclcpp::CallbackGroup` 析构路径出现段错误。
  启动和 IK 就绪已验证，但正常退出尚未通过验证。当前 apt 源没有更新的候选版本；
  该问题没有通过改变退出信号或屏蔽日志来绕过，仍需后续定位。
- 尚未连接实体机械臂、CAN 底盘、相机或串口设备执行任务。

## 实物接入参数

软件编译和 mock 测试不代表 CAN、相机、串口及实体机器人已完成联调。
本次迁移不自动修改原来的设备参数：

| 项目 | 目前配置与位置 |
|---|---|
| 机械臂 IP | 联合入口 `169.254.199.150`，通过 `robot_ip` 覆盖 |
| 本机网络地址 | 联合入口 `local_ip` 默认 `0.0.0.0`，按实物网络选择 |
| CAN 接口 | `masc_chassis_ros_driver/config/ros2/driver_params.yaml`：`can0`、500000 bit/s |
| USB 循迹相机 | `ruier_camer_diver/config/camera.yaml`：`/dev/video0` |
| 力传感器／测厚串口 | 各自配置默认 `/dev/ttyCH341USB0`，同时接入时需分别指定实际设备 |

CAN 配置、设备权限及正式任务命令参见工作空间已有的《机械臂与底盘ROS控制命令》。
先核对实际设备，再按原有流程逐个联调，最后验证联合任务。
