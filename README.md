# RMUC 2027 · PX4 x500 + D430 · EGO Planner ROS 2

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic。飞行动力学与传感器由 Gazebo 计算，PX4 SITL 执行 Offboard 位置控制，EGO Planner 使用实时深度图进行局部建图与避障。

## 数据流

```text
Gazebo rmuc_2027 + x500_d430_0
  ├─ ground-truth odometry → ros_gz_bridge → sim_adapter → /sim/odom → EGO
  └─ D430 depth + CameraInfo → ros_gz_bridge → sim_adapter
       └─ 同时间戳的深度图 + 插值光学位姿 → EGO 栅格地图
EGO B-spline → traj_server → /position_cmd → offboard (ENU → NED)
  → px4_msgs / MicroXRCEAgent → PX4 SITL → Gazebo 电机
```

规划器的里程计和相机位姿来自 Gazebo 真值。`sim_adapter` 同时将真值转成 NED/FRD，发布到 PX4 外部视觉里程计，EKF2 融合位置、高度、速度和偏航。Offboard 使用固定世界 NED 坐标，起飞前检查估计值已与真值对齐。EGO 直接使用 Gazebo 真值，不订阅 PX4 EKF 里程计。

飞行采用 **DART** 物理后端与标准 x500 电机插件，保留原始场地三角网格碰撞。当前安装版本已通过中央结构落体碰撞检查。起飞点增加 1.5×1.5 m 的平整台面，避免起落架与原场地网格穿插。

虚拟 D430 使用 Harmonic 原生 `depth_camera`：640×480、30 Hz、87° 水平视场角、0.1–10 m 裁剪、`32FC1` 米制深度。它模拟几何深度，不模拟真实双目匹配误差、红外纹理或 USB 设备。realsense-ros 已纳入编译，仿真启动时不运行需要 USB 相机的驱动，也不使用 Gazebo Classic 插件。

## 依赖与构建

依赖版本固定在 [dependencies.repos](dependencies.repos)，EGO 基于 `ros2_version`，补丁保存在 [patches/ego-jazzy-sim-time.patch](patches/ego-jazzy-sim-time.patch)。补丁包括 Jazzy 的 cv_bridge 头文件、仿真时钟、图像缓冲大小、场地地图原点和手动目标高度。

本机复用的安装：

- `/opt/ros/jazzy`
- `~/Projects/PX4-Autopilot/build/px4_sitl_default/bin/px4`
- `/usr/local/bin/MicroXRCEAgent`
- `/usr/local` 下 librealsense 2.59.1
- 系统 Gazebo Harmonic 8.15.0

```bash
cd ~/src/ego-gazebo-ros2-sim
export PX4_DIR=~/Projects/PX4-Autopilot
bash scripts/fetch_dependencies.sh
```

小型系统依赖从 ROS Jazzy 软件仓库下载并解包到 `.deps`。需要已安装 ROS Jazzy 的开发环境、PCL、OpenCV、Eigen、Gazebo Harmonic 开发库、colcon 和 librealsense。PX4 消息从本机 PX4 的普通及 versioned uORB 定义同步；PX4 提交改变时准备脚本会拒绝继续，避免消息结构不匹配。

## 启动与飞行

```bash
cd ~/src/ego-gazebo-ros2-sim
source scripts/env.sh
ros2 launch ego_gazebo sim.launch.py
```

默认启动 Gazebo GUI、RViz、MicroXRCEAgent、PX4 SITL、桥接、EGO 和 C++ 节点。初始位置 `(2, 2, 0.8)`，先自然落在起飞台上，起飞目标高度为世界坐标 `z=1.8 m`。每次启动创建独立的 `.runtime/px4/run.*` 参数与日志目录，原 PX4 源码不被改写。

另开终端执行：

```bash
cd ~/src/ego-gazebo-ros2-sim
source scripts/env.sh
ros2 service call /sim/start std_srvs/srv/Trigger '{}'
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 5.0, y: 2.0, z: 1.8}, orientation: {w: 1.0}}}'
```

也可以在 RViz 的 **2D Goal Pose** 指定 XY，零高度自动替换为起飞高度。目标会等待起飞稳定后才交给 EGO。所有目标必须在 `world` 坐标系，范围 `x=[0.5,27.65]、y=[0.5,15.57]、z=[0.6,4.2] m`。

降落：

```bash
ros2 service call /sim/land std_srvs/srv/Trigger '{}'
```

默认不自动解锁；`auto_arm:=true` 可自动起飞。`headless:=true rviz:=false` 用于无 GUI 运行，但深度渲染仍需 EGL/OpenGL。如果已有 Agent 使用 UDP 8888，传入 `start_agent:=false`。ROS domain 默认 27，Gazebo partition 默认 `ego_rmuc_2027`；启动脚本同步设置 PX4 DDS domain。一次启动一架 SITL。

启动脚本设置 `UXRCE_DDS_SYNCT=0`，让 PX4 lockstep 与 ROS `/clock` 共用仿真时间，避免 Agent 将时间戳转换为 Unix 系统时间；控制节点会拒绝时间域不一致的飞控输入。

Offboard 以 50 Hz 发送心跳和设定值，持续预发送至少两秒才请求进入模式并解锁。深度图或轨迹中断时悬停；里程计/飞控状态中断时停止发送，让 PX4 Offboard 丢失保护接管。模式退出、降落或时钟回跳后不自动重新解锁，需要重新启动本次仿真。

## 观察与验证

| Topic | 含义 |
|---|---|
| `/clock` | Gazebo 仿真时间 |
| `/gz/odom` | 原始 Gazebo 里程计，body-frame twist |
| `/sim/odom` | EGO 输入，世界 ENU 位姿与世界系线速度 |
| `/camera/depth/image_rect_raw` | D430 米制深度图 |
| `/camera/depth/camera_info` | Gazebo 相机标定 |
| `/sim/camera_pose` | 深度采集时刻的世界系光学位姿 |
| `/fmu/in/vehicle_visual_odometry` | 向 PX4 提供仿真真值，NED 位置与 FRD 姿态 |
| `/grid_map/occupancy_inflate` | 深度融合后的膨胀障碍地图 |
| `/planning/bspline` | EGO 规划轨迹 |
| `/position_cmd` | 轨迹采样位置、速度、加速度与偏航 |
| `/sim/offboard_state` | 控制状态 |

`/sim/odom` 的线速度按 EGO 输入约定转成世界系；需要标准 body-frame twist 的消费者应订阅 `/gz/odom`。光学坐标为右、下、前；相机安装偏移为机体 FLU `(0.12,0,0.02) m`。

CMake 测试包含 `frames`（坐标与光学位姿）、`config_contracts`（内参/地图/桥接一致性）、`sitl_flight`（完整起飞与短程目标到达）、`sitl_obstacle_route`（穿越中央障碍区域并检查绕行）、`map_collision`（DART 与真实场地网格接触）。在 CMake Tools 中运行测试；飞行测试前应关闭其他本工程仿真。测试日志位于 `log/validation/`。

2026-10-25 已实测：13 个 ROS 包编译通过；短程目标 `(5,2,1.8)` 到达；中央障碍路线目标 `(16,8,1.8)` 到达，测试结束位置约 `(15.79,7.79,1.80)`，相对直线路径最大偏离约 1.11 m，1,417 帧深度与光学位姿配对，输出 2,756 条规划指令。场地碰撞检查中，0.2 m 立方体停在中央结构顶部 `z≈3.216 m`。

已有仿真运行时可单独观察链路：

```bash
source scripts/env.sh
ros2 run ego_gazebo verify_sim --ros-args -p use_sim_time:=true
```

附加 `-p run_flight:=true` 会请求起飞并发送测试目标，成功后打印 JSON。此验证器检查有限深度、深度/位姿时间戳配对、相机内参、非空占据地图、PX4 Offboard 状态、轨迹指令以及目标位置持续到达。
