# RMUC 2027 场地 · Gazebo Harmonic (gz-sim 8) 场景

## 打开

```bash
./run.sh
```

`run.sh` 会设置 `GZ_SIM_RESOURCE_PATH` 并启动 `gz sim`。手动启动：

```bash
export GZ_SIM_RESOURCE_PATH=$PWD/models
gz sim -r --physics-engine gz-physics-dartsim-plugin worlds/rmuc_2027.sdf
```

- 默认用 DART。gz-physics7 的 Bullet 插件在 `CollisionInfo` 里先析构 `btTriangleMesh`、后析构引用它的 `btGImpactMeshShape`，带网格碰撞的场景退出时会在 `btGImpactMeshShapePart::~btGImpactMeshShapePart()` 段错误。要用 Bullet：`RM_PHYSICS_ENGINE=bullet ./run.sh`。
- 检测到 NVIDIA 独显时，`run.sh` 用 PRIME offload 渲染（Intel 核显的 Mesa 驱动曾在 GUI 渲染线程里崩溃）。不想用独显：`RM_USE_DGPU=0 ./run.sh`。

## 坐标

- 单位：米，Z 向上
- 场地范围：x ∈ [0, 28.15]，y ∈ [0, 16.07]，z ∈ [0, 3.82]
- 模型放在世界原点，因此场地中心在 (14.07, 8.03)
- 地面平面在 z=0，场地地面在 z≈0.019

## 内容

```
models/rmuc_2027/                        静态场地
  meshes/rmuc_2027_visual.glb            视觉，828,057 面，65 MB（不含前哨站转子）
  meshes/rmuc_2027_collision.stl         碰撞，135,867 面，由视觉 GLB 逐部件减面生成（不含前哨站转子）
models/rmuc_2027_outpost_rotor_red/      红方前哨站转子
models/rmuc_2027_outpost_rotor_blue/     蓝方前哨站转子（灯条颜色不同，其余相同）
  meshes/rotor_visual.glb                三块装甲模块 + 支架 + 装甲贴纸
  model.sdf                              关节、控制器
worlds/rmuc_2027.sdf                     run.sh 使用（默认用 DART 运行）
tools/build_from_glb.py                  从 rm-map-tools GLB 重新生成上述网格和转子模型（GLB 是唯一的几何输入）
tools/stickers/*.png                     官方贴纸图案（构建输入）
tools/build_report.json                  转子位姿、换色统计、贴纸位置等校验数据
```

### 视觉网格

来自 [rm-map-tools](https://github.com/hxyulin/rm-map-tools) 的
`rmuc2027-simplified-glb-20261018` release（`field.glb`，SHA-256 `ef40412b…`）。
颜色和基地贴图都在 GLB 里，所以 `model.sdf` 的视觉部分**不能加 `<material>`**，否则会覆盖 GLB 的材质。

转换方法：
- 取 GLB 的 `rmuc-2027` 节点（CAD 坐标，Z 向上），平移 (14.1491, 6.4410, 1.9608)，让场地一角落在原点。这个平移当初按旧版 STL 场地拟合（偏差中位数 0 mm，p90 < 0.5 mm），保留下来是为了让 world 里已有的坐标继续有效。原 GLB 根节点的 Y-up 旋转不使用。
- 5,527 个摆放的图元按材质合并成 307 个，减少渲染批次。几何没有改动。

### 碰撞网格

碰撞网格和视觉用同一份几何：GLB 的静态场地，去掉前哨站转子和支架，不含贴纸。脚本按 GLB 的网格节点逐个处理：

- 包围盒对角线小于 5 cm 的零件（螺钉、小灯等，1,490 个）直接丢掉。
- 其余零件从 1% 面数开始逐级尝试减面（1%、2%、4% … 32%），取第一个满足精度的结果：在原表面上采 400 个点，98% 的点到减面结果的距离 < 3 cm，且全部 < 12 cm。
- 达不到精度的零件保留原网格；其中小于 0.3 m 的用凸包代替。

不对整个场地一次减面，因为那样会先把公路这类本来面数就很少的大板子的顶面减没，细节却还没去掉。

和视觉 GLB 对比（0.1 m 网格，45,241 条竖直射线测顶面高度）：高度差 > 10 cm 的点占 0.07%，> 2 cm 的点占 0.23%。旧的 STL 碰撞（63,712 面，缺两个飞镖站和科技核心机械臂，靠 `collision_extra` 补）对应是 0.61% 和 0.74%。

### 红蓝颜色修正

CAD 里红蓝两份相同的设备共用一套网格和颜色，所以蓝方的基地、前哨站、资源区、堡垒、机库平台停机坪都带着红色灯条，红方的前哨站和飞镖站上也有蓝色件。地面、公路、高地的标线本来就分了红蓝，不受影响。

构建脚本按设备归属换色：名字带 `red-`/`blue-` 的按名字，堡垒、机库平台停机坪和科技核心按位置（x > 14.15 为红方）。换色表参照地面标线已有的红蓝配对：

| 红 | 蓝 |
|---|---|
| `1.0, 0.0271, 0.0271` | `0.066, 0.004, 0.593` |
| `1.0, 0, 0` | `0, 0, 1.0` |
| `0.44, 0.14, 0.14` | `0.14, 0.14, 0.44`（新增） |
| — | `0.1333, 0.2902, 0.7451`（飞镖站状态灯，红方改为第一行红色） |

前哨站上的灯条大多是 `1.0, 0.0271, 0.0271`，按上表换成深蓝后几乎看不见。CAD 里前哨站自带的蓝色灯就是 `0, 0, 1`，所以蓝方前哨站的这一色改用 `0, 0, 1`。

飞镖检测模块的装甲板两侧各有一条灯条（规则图 4-13 里是红色），CAD 给的是不发光时的颜色（基地为灰色，前哨站为白色导光条）。脚本把这两条灯条（每个模块约 380 个三角形）改成队伍颜色：红方 `1, 0, 0`，蓝方 `0, 0, 1`。

科技核心红蓝各一个（规则 5.4.3）。比赛中它的第二、三组灯（装配柱两侧的灯板）常亮队伍颜色（表 5-13），所以按归属换色；第一组灯是白色条纹，不换。能量机关本身就分红蓝两面，没有换色。

### 贴纸

图案取自《RoboMaster 2027 机甲大师高校系列赛机器人制作规范手册 V1.0.0》附录 2（印刷页 88–89），放在 `tools/stickers/`：

| 文件 | 位置 | 数量 | 尺寸 |
|---|---|---|---|
| `outpost_armor.png` | 前哨站转子装甲模块 | 每方 3 | 115 × 100 mm |
| `outpost_dart.png` | 前哨站飞镖检测模块 | 每方 1 | 约 140 × 129 mm |
| `base_armor.png` | 基地顶部装甲模块 | 每方 3 | 109 × 95 mm |
| `base_dart.png` | 基地飞镖检测模块 | 每方 1 | 135 × 124 mm |

- 脚本在 GLB 里自动找装甲板面（按材质和尺寸），取最外层的平面，贴纸贴在它前方 0.8 mm，保持原图比例，尽量铺满板面。
- 贴纸按图案的外轮廓（带缺角）三角化，不依赖透明通道。
- 换图案：替换 `tools/stickers/` 里的同名 PNG，再运行 `build_from_glb.py --visual-only`。
- 规则写基地有 6 块装甲模块。CAD 闭合状态下只有顶部 3 块露在外面，另外 3 块被基地护甲挡住，没有贴。

### 机库

2027 规则取消了停机坪，改为战场中央的红蓝机库平台（规则 4.3.2.2）。场地里两个带 “H” 的平台（GLB 节点 `drone-landing-pad`）就是机库平台，已经导入，颜色也已按红蓝修正。机库本身属于参赛队的空中系统，规则里没有标准尺寸，官方 CAD 也没有它，需要自己建模放上去。

### 前哨站转子

- 红蓝各一个转子模型（`rmuc_2027_outpost_rotor_red` / `_blue`），用 `revolute` 关节连到 world。
- `JointController` 速度模式，转速 0.8π = 2.513 rad/s（规则 5.6.1）。启动即转，没有 5 s 加速，也不会按比赛进程停转。
- 转轴取自 rm-map-tools `rules/source-motion-rmuc2027.json` 的 `outpost.rotor.spin`。CAD 里整个前哨站倾斜 1.5°，转轴也跟着倾斜，所以 world 里的 pose 带 0.02618 rad 的 roll。三块装甲绕这条轴转 120° 后与相邻装甲重合，误差中位数 0.1 mm。
- 支架在 GLB 里和固定的上部结构是同一个网格。脚本按连通块拆分，最高点低于装甲顶部的块归转子（5,074 面），不裁剪三角形。
- 碰撞：每块装甲一个包围盒，`collide_bitmask` 为 0x02。场地碰撞为 0x01，所以转子不会和场地互相碰撞；机器人用默认的 0xffff，两者都能撞到。
- 质量和惯量是占位值（2 kg 圆环），CAD 里没有实测数据。

运行时改转速或停转：

```bash
gz topic -t /model/red_outpost_rotor/joint/rotor_joint/cmd_vel -m gz.msgs.Double -p 'data: 0'
```

### 重新生成

```bash
uv venv .venv --python 3.12
uv pip install --python .venv/bin/python numpy scipy trimesh pygltflib rtree networkx \
    fast_simplification shapely mapbox_earcut contourpy pillow
.venv/bin/python tools/build_from_glb.py /path/to/field.glb                 # 约 30 秒
.venv/bin/python tools/build_from_glb.py --visual-only /path/to/field.glb   # 只改颜色/贴纸时，约 10 秒
```

源文件 `field.glb` 在仓库根目录的 `.cache/rmuc2027/rmuc2027-simplified-glb-20261018/`（被 `.gitignore` 忽略，不在 git 里）。

脚本会先校验 GLB 的 SHA-256，因为节点的选择方式只针对这个版本核对过。它会覆盖上面列出的生成文件和两个转子模型。world 里两个转子的 pose 要按 `tools/build_report.json` 的 `model_pose_xyz_rpy` 手动更新。

## 已验证

- 碰撞（改为从 GLB 生成之后测的）：在 8 个位置（中央结构顶部、红方飞镖站、公路、高地）各丢一个 0.2 m 立方体，仿真 9 s。DART 下 8 个全部停在视觉顶面 +0.1 m 处，误差 ≤ 1 mm，实时因子 1.0。飞镖站那一点用旧碰撞网格会掉到 z≈0.244 m 的地面。
- Bullet 下有 2 个立方体停的位置不对（飞镖站上的掉到地面，一处公路上的陷下去 18 cm），其余偏高约 1 cm。换成旧碰撞网格也有 3 个不对，所以问题出在 Bullet 的网格碰撞，与新网格无关。用 Bullet 时注意这一点。
- 以下检查是在精简仓库之前做的，对应的测试脚本和 x500 world 已删除：
- 转子：DART 和 Bullet 下，红蓝两个转子的关节速度都是 2.5133 rad/s，2 s 内的角度增量与速度一致，日志里没有 `[Err]`。
- 相机渲染对比：GLB 与原 STL 在基地、前哨站、全场视角下的轮廓一致；连续帧可以看到转子带着装甲和支架转动。
- 红蓝颜色：对每个红/蓝色三角形，检查它关于场地中心对称位置的颜色。原 GLB 有 35,055 个三角形在对称位置是同色（说明一方颜色错了）；生成的场地 GLB 只剩 1,967 个，全部在 z > 2 m 的能量机关上（能量机关两面本来就一红一蓝，扇叶转角不同，对称位置对不上）。
- 贴纸：相机正对 16 张贴纸逐一渲染，全部可见，没有被板面遮住或闪烁。

## 说明

- 这是一个静态场地，不含机器人；直接在 world 里加机器人模型即可
- 地面上的 “ROBOMASTER 2027” 字样从 y=0 一侧看是倒着的，CAD 原本如此，不是镜像
- 碰撞用减面网格是近似，轮子/底盘的精细接触建议另外加碰撞体
- 如果机器人打滑，可调 `model.sdf` 里的 `mu` / `mu2`
- 能量机关、基地护甲、飞镖闸门仍然是静止的
- CAD 几何和外观归 DJI / RoboMaster 所有，见 `NOTICE.md`（从 rm-map-tools release 复制）
