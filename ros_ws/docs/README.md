# 项目文档总入口

当前工作区是 `ros_ws`；`ros_ws2` 是旧版参考，不参与本次修改。
自写操作说明集中在本目录；原来的散落说明和跳转占位文件已移除。

## 当前使用

| 要查什么 | 文档 |
|---|---|
| 启动、点位、相机、诊断、返回顺序 | [任务操作指南](tasks/ROBOT_TASK_GUIDE.md) |
| 真机漆膜测厚、朝向、限力、失败处理 | [漆膜检测逻辑](tasks/PAINT_DETECTION.md) |
| 麒麟 start / detect / go_home、报文、电池与异常 | [oline 流程与协议](tcp/OLINE流程与协议.md) |
| 麒麟工具部署及配置 | [TCP 目录](tcp/README.md)、[快速启动](tcp/QUICKSTART.md) |
| 仅规划的 RViz 仿真 | [路径预览](simulation/PATH_PREVIEW.md) |
| 从哪里改代码 | [代码导航](CODE_MAP_CN.md) |
| 新机部署、Jazzy 构建 | [迁移说明](MIGRATION_JAZZY_CN.md) |

当前仿真只保留 axis_only：预接近终点保持拍照工具 Z 方向，绕自身 Z 可自由选角；途中不持续锁姿态，最后沿工具 Z 接近 5 cm、撤离。
已移除自由朝向、全程完整朝向直线、旧版姿态对照、就近数值 IK 和求解时间试验；直接使用 MoveIt 规划并保留碰撞检查。
真机仍使用原有限角度候选，未修改真机接触控制方式；仿真不等于实物接触验证。
仿真拍照位目前单独覆盖为 tool0 XYZ `(240,390,580) mm`、RPY `(-180,0,-90)°`；真机流程文件未同步修改。
仿真中间点一/二目前分别为 `(110,450,600) / (180,0,-25)`、`(-435,254,720) / (-90,0,45)`（mm/度）；回程复用相同点。
原过渡位后新增仿真关节过渡位2 `[-45,-90,180,0,0,90]°`，回程倒序经过。
仅仿真的肘关节有效范围按要求设为±250°；真机共享模型及其他关节不改，不代表硬件允许该范围。
真机主任务、oline与到站诊断已同步过渡位2及倒序返回；真机关节规划如被限位裁剪会在执行前拒绝，180°目标尚需实机限位确认。

## 模块与专题资料

- 相机：[深度相机采集/坐标转换](modules/GEMINI_CAMERA.md)、[循迹相机](modules/LINE_CAMERA.md)。
- 外设：[串口汇总](modules/SERIAL_DEVICES.md)、[力传感器](modules/FORCE_SENSOR.md)、[测厚仪](modules/THICKNESS_SERIAL.md)。
- 模型：[新增末端装配](modules/TERMINAL_MODEL.md)。
- 路线：[标准路线](tasks/STANDARD_ROUTE.md)、[底盘导航逻辑](tasks/CHASSIS_NAVIGATION_LOGIC.md)、[资源配置](tasks/RESOURCE_PROFILE_CN.md)。
- 底盘整合笔记：[chassis 目录](chassis/README.md)。模块/专题中的旧任务入口不能覆盖上方现行指南。
- 协议底稿：[TCP 原协议说明](tcp/TCP通讯协议_麒麟工控机.md)。oline 新增阶段和保存确认以上方现行协议为准。

## 文档维护与归档

旧命令、旧 profile、旧代码地图及已退休 launch 的说明保存在 [archive](archive/README.md)，
保留原信息供追溯，不当作当前操作步骤。整理没有删除历史技术内容。

ELI、MASC 官方文档保留在各自源码包，未移动也未改内容；本目录集中的是项目自写说明。
厂商相机、Elite SDK 文档也保持原样。

麒麟交付目录及 ZIP 内的 README/协议是发布副本，唯一编辑源是本目录。
修改后在仓库根目录运行 `python3 tools/export_industrial_pc.py` 重新生成；
该工具保留已有交付目录中的现场 `config.json`，导出后仍须核对两端实际 IP。
不要在生成包里单独维护第二份说明。

修改配置后重新构建、加载环境并重启任务。仅注释/文档整理不改变运动参数；
配置校验、离线测试和规划预览都不能替代真机安全检查。
