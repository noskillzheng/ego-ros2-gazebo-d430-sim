# RoboMaster 官方场地仿真资源汇总

调研时间：2026-10-29。重点是 RMUC（超级对抗赛）2026 / 2027 赛季，同时收录 RMUL 和历史赛季资源。

星数、更新时间均为调研时的数据。大部分社区仓库**没有声明许可证**；场地 CAD 的版权归 DJI / RoboMaster 所有，复用和再分发前需自行确认。

---

## 1. 官方场地 STEP 原件

DJI 在 RoboMaster 论坛（<https://bbs.robomaster.com>）发布了官方场地 CAD。下表链接整理自 [hxyulin/rm-map-tools `source/SOURCES.json`](https://github.com/hxyulin/rm-map-tools/blob/main/source/SOURCES.json)，发布时间根据链接中的时间戳推算。

| 文件 | 大小 | 约发布时间 | 下载 |
|---|---:|---|---|
| `RMUC2026.stp`（首版） | 649 MB | 2025-10 | [链接](https://bbs-web-static.robomaster.com/3abb26b20cd946509523df5d9f42bd631761107118068/RMUC2026.stp) |
| `RMUC2026_V1.2.0.step` | 993 MB | 2025-12 | [链接](https://bbs-web-static.robomaster.com/29ce9515248f4a45b64387c4a83206c81767087556146/RMUC2026_V1.2.0.step) |
| `RMUC2026_V2.0.0.stp` | 1.25 GB | 2026-06 | [链接](https://bbs-web-static.robomaster.com/f637e13eadfc4602a76c7a05c54f71271782803452461/RMUC2026_V2.0.0.stp) |
| `RMUL2026.stp` | 1.7 MB | 2025-10 | [链接](https://bbs-web-static.robomaster.com/390f5a969de14811948cc0e102fc699f1761107116899/RMUL2026.stp) |
| `能量单元.stp`（2026） | 0.8 MB | 2025-10 | [链接](https://bbs-web-static.robomaster.com/21cbbecaaceb47729bc477cd12f8de5b1761107116144/%E8%83%BD%E9%87%8F%E5%8D%95%E5%85%83.stp) |
| `RMUC2027_V1.0.0.stp` | 653 MB | 2026-10-14 | [链接](https://hz-rm-bbs-web-prod.oss-cn-hangzhou.aliyuncs.com/080ae7230f6c469c9bec9d2c892797b61791994813953/RMUC2027_V1.0.0.stp) |
| `RMUL2027_3V3_V1.0.0.stp` | 2.0 MB | 2026-10-14 | [链接](https://hz-rm-bbs-web-prod.oss-cn-hangzhou.aliyuncs.com/12d6225c83984f00a0cc27628ecea74b1791994867597/RMUL2027_3V3_V1.0.0.stp) |
| `RMUL2027_1V1_V1.0.0.stp` | 1.1 MB | 2026-10-14 | [链接](https://hz-rm-bbs-web-prod.oss-cn-hangzhou.aliyuncs.com/cc52e8820e32454e8d34bcddc0b8ddfa1791994866414/RMUL2027_1V1_V1.0.0.stp) |
| `能量单元2027.stp` | 3.1 MB | 2026-10-14 | [链接](https://hz-rm-bbs-web-prod.oss-cn-hangzhou.aliyuncs.com/2c5546d12d7b4f5492998eef274d2f341791994875037/%E8%83%BD%E9%87%8F%E5%8D%95%E5%85%83.stp) |

注意：

- 2026 赛季官方 STEP 改过多个版本。社区 Gazebo 世界大多没写基于哪一版，复用前要核对。
- 整场 STEP 直接转出的网格对 Gazebo 太大，需要减面，最好把视觉网格和碰撞网格分开。在

### STEP 转换工具 / 素材包

- [hxyulin/rm-map-tools](https://github.com/hxyulin/rm-map-tools)（Apache-2.0 / MIT）：把官方 STEP 拆成独立部件，导出 GLB / SDF / URDF / USD / MJCF，视觉和碰撞网格分开。
  - [rm2027-assets-20261015](https://github.com/hxyulin/rm-map-tools/releases/tag/rm2027-assets-20261015)：RMUC 2027、RMUL 2027 3v3/1v1、能量单元的 CAD / GLB / SDF / URDF / USD。
  - [rmuc2027-simplified-glb-20261018](https://github.com/hxyulin/rm-map-tools/releases/tag/rmuc2027-simplified-glb-20261018)：RMUC 2027 简化 GLB，约 10 MB。
  - [rmuc2027-cad-stage1-20261017](https://github.com/hxyulin/rm-map-tools/releases/tag/rmuc2027-cad-stage1-20261017)：RMUC 2027 分层 CAD。
  - [rmuc2026-assets-20260911](https://github.com/hxyulin/rm-map-tools/releases/tag/rmuc2026-assets-20260911)：RMUC 2026，基于 V1.2.0 并嫁接 V2.0.0 道路。整场为 GLB（带贴图），另有 5 个可动机构的 SDF / URDF / USD。
  - [rmuc2026-simulator-field-20260913](https://github.com/hxyulin/rm-map-tools/releases/tag/rmuc2026-simulator-field-20260913)：RMUC 2026 运行时场地包。
- [keilking66/RMUC2027-map](https://github.com/keilking66/RMUC2027-map)：RMUC 2027 官方 STEP 原件、Isaac Sim USD、644 个分件清单（含颜色），以及规则手册 PDF。

---

## 2. RMUC（超级对抗赛）2027

规则 V1.0.0 发布于 2026-10-15，下列项目均为发布后两周内建立。

### Gazebo 可直接使用

| 仓库 | Gazebo 版本 | 说明 |
|---|---|---|
| [truue1/ego-ros2-gazebo-d430-sim](https://github.com/truue1/ego-ros2-gazebo-d430-sim) → [`assets/rmuc_2027_gazebo`](https://github.com/truue1/ego-ros2-gazebo-d430-sim/tree/main/assets/rmuc_2027_gazebo) | Harmonic (gz-sim 8) | 视觉网格 38 万面，碰撞网格 3.8 万面，两者分开；场地 28.15 × 16.07 m，米制，Z 向上。仓库本身是 PX4 无人机 + ego-planner 仿真。网格是 STL，没有颜色和贴图 |
| [ECUST-Origin-Sentry-Team/gazebo_simulator_origin](https://github.com/ECUST-Origin-Sentry-Team/gazebo_simulator_origin) | Fortress | 基于 `rmu_gazebo_simulator`，加入 `rmuc_2027` 世界；单个 STL 同时做视觉和碰撞 |
| [tungchiahui/ysu2026_rm_sentry](https://github.com/tungchiahui/ysu2026_rm_sentry) | Fortress | 同上，另有 `rmul_2027` |
| [SuisDokoXD/pfa-nav-own](https://github.com/SuisDokoXD/pfa-nav-own) | Fortress | 同上，附 CAD 转网格脚本 `tools/sim/cad_to_field_mesh.py` |
| [movoQAQ/hxy](https://github.com/movoQAQ/hxy) | Fortress | 同上，附导航地图 `rmuc_2027.yaml` |
| [shiuhou/scene](https://github.com/shiuhou/scene) → `gazebo/rmuc2027_sentry_field` | Fortress / Classic | OBJ + MTL 视觉网格（低精度 LOD），碰撞体是为哨兵导航简化过的近似形状，几何上不是 1:1 |
| [mikiuvu/drone-lab](https://github.com/mikiuvu/drone-lab) | Gazebo + PX4 | RoboMaster 无人机算法实验台，含 `simulation/worlds/rmuc_2027.sdf` 和导入脚本 `import_rmuc2027.py` |

### 其他仿真器

- [diaofenyuan/RMUC_2027map_unity](https://github.com/diaofenyuan/RMUC_2027map_unity)：Unity 三维模型 + 面向雷达的二维地图包（10 星）
- [CosmosMount/rmuc2027_mjarena](https://github.com/CosmosMount/rmuc2027_mjarena)、[CosmosMount/rm_mjcf](https://github.com/CosmosMount/rm_mjcf)：MuJoCo
- [liukong1220/ats_mujoco_sim](https://github.com/liukong1220/ats_mujoco_sim)：MuJoCo，含 RMUC 2026 / 2027
- [ling37luo/rmuc-mujoco](https://github.com/ling37luo/rmuc-mujoco)：MuJoCo / Isaac，含 2026 / 2027 赛季
- [trident-rm/rm-simulator](https://github.com/trident-rm/rm-simulator)：Rust + Bevy 第一人称模拟器，加载 RMUC 2027 CAD，可选 ROS 2 适配器；2026 版本保留在 tag `rm2026-final`
- [RM-Arena/Arena-1](https://github.com/RM-Arena/Arena-1)
- [a3203space/rmuc2027](https://github.com/a3203space/rmuc2027)、[a3203space/rmuc2027_rl](https://github.com/a3203space/rmuc2027_rl)：强化学习方向

---

## 3. RMUC（超级对抗赛）2026

| 仓库 | 仿真器 | 说明 |
|---|---|---|
| [jiedong01/26rmu_gazebo_simulator](https://github.com/jiedong01/26rmu_gazebo_simulator) | Fortress | 在 PolarBear 版本上加入 `rmuc_2026` / `rmul_2026`，Apache-2.0；场地为 6 MB STL，未说明网格来源和对应的 STEP 版本 |
| [Walker152/nav-simulation](https://github.com/Walker152/nav-simulation) | Fortress | 原始 RMUC2026.stl（755 万面）减到 50 万面，用 ICP 与实测点云对齐，RMSE 4.2 cm；来源记录最完整，但原始 STL 的作者和许可证不明 |
| [movoQAQ/bit-nav](https://github.com/movoQAQ/bit-nav) | Fortress | 与 Walker152 同源 |
| [loong55/sentry_nav](https://github.com/loong55/sentry_nav) | Fortress | 含 `rmuc_2026`、`rmuc_2026fix`、`rmuc2026_in` 三个世界 |
| [LemperorD/guganav](https://github.com/LemperorD/guganav) | Fortress | 北航 Transistor 战队哨兵工作空间，含 `rmuc_2026` / `rmul_2026` |
| [grow-up-happily/pfa-nav](https://github.com/grow-up-happily/pfa-nav) | Fortress | 厦门理工导航仓库，含 `rmuc_2026` |
| [Boombroke/Sentry26](https://github.com/Boombroke/Sentry26) | Fortress | 26 赛季哨兵导航框架 |
| [meichuan951-maker/rmuc_map](https://github.com/meichuan951-maker/rmuc_map) | Gazebo | `world/rmuc_2026.sdf`，另有窄通道、高障碍等测试场景 |
| [mikiuvu/drone-lab](https://github.com/mikiuvu/drone-lab) | Gazebo + PX4 | `simulation/worlds/rmuc_2026_sentry26.sdf` |
| [adoreATRI/nav_simulation](https://github.com/adoreATRI/nav_simulation) | Gazebo | `gazebo_simulation/worlds/rmuc2026/world.sdf` |
| [renquan87/radar-simulation](https://github.com/renquan87/radar-simulation) | Gazebo (rmoss) | 雷达站仿真，`rmuc2026_competition.sdf` |
| [Xsxin1/radar_simulator](https://github.com/Xsxin1/radar_simulator) | Gazebo | 雷达站仿真 |
| [Z-WM/ros2_nav_simulation](https://github.com/Z-WM/ros2_nav_simulation) | Gazebo Classic | `RMUC2026.world` |

其他仿真器：

- [CosmosMount/rmuc2026_battlefield_mjcf](https://github.com/CosmosMount/rmuc2026_battlefield_mjcf)：MuJoCo，带碰撞
- [liu31415926535/SimulatorX-RMUC2026](https://github.com/liu31415926535/SimulatorX-RMUC2026)
- [CityWithoutMe/rmuc-2026-simulator](https://github.com/CityWithoutMe/rmuc-2026-simulator)：Unity
- [SnowStain/Artinx_Simulator_for_RM26UC](https://github.com/SnowStain/Artinx_Simulator_for_RM26UC)：pygame 2D 战术模拟
- [xiaoqi371317/pcd_rmuc2026](https://github.com/xiaoqi371317/pcd_rmuc2026)：RMUC 2026 点云

---

## 4. RMUL（高校联盟赛）2026 / 2027

### RMUL 2027

- [ZhuoruiYUN/Sentry-Navigation](https://github.com/ZhuoruiYUN/Sentry-Navigation) → `src/rm2027_field`：由官方 `RMUL2027_3V3_V1.0.0.stp` 转换，保留 9 组表面颜色，完整网格做碰撞；附 STEP 原件、转换脚本 `import_official_step.py`（依赖 gmsh）和导航地图。Humble
- [ZQU-Foray/foray_simulation](https://github.com/ZQU-Foray/foray_simulation)：Humble + Fortress，分件 DAE + 贴图，含官方 PGM、STEP 复核参数、PCD 先验点云，回归测试要求与官方地图 IoU ≥ 0.90
- [tungchiahui/ysu2026_rm_sentry](https://github.com/tungchiahui/ysu2026_rm_sentry)：`rmul_2027` 世界（单 STL）
- [wunai821/xxu_rm_26](https://github.com/wunai821/xxu_rm_26)：`rmul_2027` 模型
- [Cranyozen/rm_vision_sim_godot](https://github.com/Cranyozen/rm_vision_sim_godot)：Godot，RMUL 2027 1v1 / 3v3 GLB
- rm-map-tools [rm2027-assets-20261015](https://github.com/hxyulin/rm-map-tools/releases/tag/rm2027-assets-20261015)：含 RMUL 2027 3v3 / 1v1

### RMUL 2026

- [jiedong01/26rmu_gazebo_simulator](https://github.com/jiedong01/26rmu_gazebo_simulator)：`rmul_2026`，Fortress，Apache-2.0
- [rm-controls/rm_control](https://github.com/rm-controls/rm_control) → `rm_gazebo/worlds/rmul2026.world`：Gazebo Classic，ROS 1，单个 STL，BSD-3-Clause（308 星）
- [laohao78/ROS2_RM_Navigation](https://github.com/laohao78/ROS2_RM_Navigation)：Humble + Gazebo Classic，`RMUL2026_world`，含动态障碍物版本，MIT（42 星）
- [ZhuoruiYUN/Sentry-Navigation](https://github.com/ZhuoruiYUN/Sentry-Navigation) → `src/rm2026_field`：Gazebo Classic，按规格参数生成，不是 STEP 转换
- [NYUSH-Robotics-Club/nyush_rm_sentry](https://github.com/NYUSH-Robotics-Club/nyush_rm_sentry)、[YD07-xyt/NavX](https://github.com/YD07-xyt/NavX)：`RMUL2026_world`

---

## 5. 通用基础 / 历史赛季

- [SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator)：Gazebo Fortress + ROS 2 Humble，内置 `rmul_2024`、`rmuc_2024`、`rmul_2025`、`rmuc_2025`；上表多数 2026 / 2027 世界都是在它上面扩展的。Apache-2.0（107 星）
  - 配套导航：[pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav)
- [robomaster-oss](https://github.com/robomaster-oss)（RoboMaster OSS）
  - [rmoss_gz_resources](https://github.com/robomaster-oss/rmoss_gz_resources)：Gazebo 公共资源
  - [rmoss_gazebo](https://github.com/robomaster-oss/rmoss_gazebo)：Gazebo 插件和模型
  - [rmuc21_ignition_simulator](https://github.com/robomaster-oss/rmuc21_ignition_simulator)：RMUC 2021
  - [rmua19_gazebo_simulator](https://github.com/robomaster-oss/rmua19_gazebo_simulator)：RMUA 2019
  - [awesome-robomaster](https://github.com/robomaster-oss/awesome-robomaster)：资源清单
- [66Lau/sentry_sim](https://github.com/66Lau/sentry_sim)：RM 2023 / 2024 哨兵仿真，ROS Noetic
- [nusrobomaster](https://github.com/orgs/nusrobomaster/repositories)：NUS 的 RMUC 哨兵 Gazebo 仿真
- [SCAU-RM-NAV](https://github.com/SCAU-RM-NAV)：`rm2022_auto_infantry_ws` 自动步兵 Gazebo 环境
- [Ruinwalker7/rm_decision](https://github.com/Ruinwalker7/rm_decision)：BehaviorTree.cpp + Gazebo
- [XDUZero2Hero/zero2hero_rmua2022_simulation](https://github.com/XDUZero2Hero/zero2hero_rmua2022_simulation)：RMUA 2022
- [yizhengzhang1/RoboMaster-Simulator](https://github.com/yizhengzhang1/RoboMaster-Simulator)：ICRA AI Challenge 2020
- [Team-RoboTO/Robomaster-Simulator-3D](https://github.com/Team-RoboTO/Robomaster-Simulator-3D)：RMUA 2019
- [sunduoze/ICRA2018_DJI_RM_AI_Challenge_NJUST](https://github.com/sunduoze/ICRA2018_DJI_RM_AI_Challenge_NJUST)：ICRA 2018

---

## 6. 用于无人机仿真时的光影要求

“Gazebo 中最好光影都一样，用于无人机仿真”的意思是：**仿真里场地的光照、阴影和材质外观，要尽量和真实赛场一致。**

原因：无人机主要靠相机（RGB、深度相机、视觉里程计、目标识别）感知环境，图像像不像真实场地，直接决定算法能不能从仿真迁移到实机。地面机器人导航主要靠激光雷达，只要几何正确就够用；无人机的视觉算法对外观敏感，几何对了但颜色、亮度、阴影不对，识别和定位结果仍会和实机差很多。

落到实现上要注意：

1. **视觉网格必须带材质或贴图。** STL 不带颜色。上面多数社区世界用单个 STL，渲染出来是统一的灰色，不适合视觉仿真。应选用带颜色或贴图的格式：
   - rm-map-tools 的 GLB（带贴图和颜色）
   - keilking66 的 USD（带分件颜色）
   - DAE / OBJ + MTL（例如 ZQU-Foray 的分件 DAE）
2. **注意 Gazebo 版本对格式的支持。** Fortress 能加载 DAE / OBJ / STL。glTF / GLB 需要 Garden / Harmonic 及以后的版本（gz-common 5 的 assimp 加载器）。truue1 的 RMUC 2027 资产用的是 Harmonic，但网格是 STL，没有颜色，要换成带材质的视觉网格。
3. **按真实赛场布置光源。** 室内赛场以顶灯照明为主，可以用多个 spot / point 光源模拟顶灯阵列，打开 `<cast_shadows>`，渲染引擎用 `ogre2`。不要只用默认的单个方向光（太阳光）。
4. **光照要固定，便于复现。** 同一测试场景每次运行的光照应保持一致，方便对比算法版本；需要时再额外做光照随机化，测试鲁棒性。
5. **主动发光的部件要单独处理。** 装甲板灯条、能量机关、基地 LED 是视觉识别的目标，需要用 emissive 材质模拟发光。
6. **Gazebo 的真实感有上限。** ogre2 没有完整的全局光照，反光和间接光照的效果有限。如果视觉算法对画面真实感要求很高，可以考虑 Isaac Sim（keilking66 的 USD）或 Unity（diaofenyuan 的模型），Gazebo 只承担动力学和 PX4 SITL 部分。
