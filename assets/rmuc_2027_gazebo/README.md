# RMUC 2027 场地 · Gazebo Harmonic (gz-sim 8) 场景

## 打开

```bash
./run.sh
```

`run.sh` 会设置 `GZ_SIM_RESOURCE_PATH` 并启动 `gz sim`。手动启动：

```bash
export GZ_SIM_RESOURCE_PATH=$PWD/models
gz sim -r --physics-engine gz-physics-bullet-plugin worlds/rmuc_2027.sdf
```

## 坐标

- 单位：米，Z 向上
- 场地范围：x ∈ [0, 28.15]，y ∈ [0, 16.07]，z ∈ [0, 3.82]
- 模型放在世界原点，因此场地中心在 (14.07, 8.03)
- 地面平面在 z=0，场地地面在 z≈0.019

## 内容

```
models/rmuc_2027/
  model.config
  model.sdf
  meshes/rmuc_2027_visual.stl      # 379,082 tris, 19 MB
  meshes/rmuc_2027_collision.stl   # 37,908 tris, 1.9 MB
worlds/rmuc_2027.sdf
run.sh
```

- 视觉网格：完整场地（含两个小停机坪），STL 无贴图，用 SDF 材质着色
- 碰撞网格：二次误差减面到 10%，静态 trimesh
- 静态模型（`static=true`），不会掉落
- 物理：bullet（`gz-physics-bullet-plugin`），1 ms 步长
  - `run.sh` 保留地图展示时的 Bullet 配置。无人机集成使用 DART；在当前 Harmonic 8.15.0 安装中，DART 已通过场地三角网格落体碰撞测试，不需要替换成简化方盒。

## 说明

- 这是一个静态场地，不含机器人；直接在 world 里加机器人模型即可
- STL 是三角网格、非水密；碰撞用减面网格是近似，轮子/底盘的精细接触建议另外加碰撞体
- 如果机器人打滑，可调 `model.sdf` 里的 `mu` / `mu2`

## 已验证

- `gz sim -s -r --iterations 100` 正常启动，无网格/物理报错
- 在中央结构上方 5 m 处丢一个 0.2 m 立方体，最终停在 z=3.216 m，
  说明 37.9k 面的 trimesh 碰撞确实生效（不是只落在 z=0 的地面上）
- 视觉网格与碰撞网格均从修改后的 `rmuc_2027.stl` 生成
