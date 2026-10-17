# MuJoCo 基础 V3 资源

本目录只服务 `urdf/tiangong3.urdf` 的基础 V3 全身模型，不包含 BrainCo 或 Inspire 灵巧手。

## 内容

- `meshes_convex/`：由根目录 `meshes/` 的 40 个 STL 生成的凸包网格，用于碰撞（`ankle_roll_l_link` / `ankle_roll_r_link` 的脚底除外，见「脚底碰撞」）。原始 `meshes/` 保持不变。
- `meshes_simplify/`：由 `script/prime_to_simplify.py` 生成的 10% 减面网格，仅用于视觉查看。
- `tiangong3_torq.xml`：由 `script/make_mjcf_torq.py` 生成的 MJCF，引用 `meshes_convex/`。**实际使用请用此文件。**
- `tiangong3_simplify.xml`：同一脚本以 `--mesh-subdir meshes_simplify` 生成的变体。除脚底碰撞外，其视觉与碰撞 geom 都指向减面网格，**仅用于视觉查看，不适用于接触仿真**。

## 脚底碰撞

`ankle_roll_l_link` / `ankle_roll_r_link` 的碰撞体为 7 个 `cylinder` geom（`ankle_roll_{l,r}_link_collision_0..6`），不再引用 `meshes_convex/ankle_roll_*.STL`；对应的 `visual` geom 仍引用该网格。参数与 URDF 侧一致，见根目录 `README.md` 的「脚底碰撞体」。

7 个圆柱的 `size` 为 `半径 半长`，最低点为 `-0.058 m`；两份 XML 的 `floor` 高度仍按原网格最低点自动计算，默认姿态下脚底有约 1.23 mm（`tiangong3_torq.xml`）/ 0.6 mm（`tiangong3_simplify.xml`）的初始穿透。

各文件的面数对比与选用依据见 `script/README.md` 的「MJCF 选用说明」与「凸包面数的原则」两节。

已于 2026-09-22 验证：40 个凸包网格与源网格同名、包围盒不变，总面数由 2,903,682 降至 83,714。MuJoCo 3.10.0 可加载并步进 `tiangong3_torq.xml`（40 bodies、32 joints、31 actuators、39 个被引用网格）。

## 生成与校验

在包根目录执行：

```bash
/Users/loong/miniconda3/bin/conda run -n xSIM python script/prime_to_convex.py
/Users/loong/miniconda3/bin/conda run -n xSIM python script/make_mjcf_torq.py
/Users/loong/miniconda3/bin/conda run -n xSIM python -c \
  "import mujoco; m=mujoco.MjModel.from_xml_path('mujoco/tiangong3_torq.xml'); print('OK', m.nbody, m.njnt, m.nu, m.nmesh)"
```

## 灵巧手边界

当前 XML 不引用 `BrainCo_revo2/` 或 `Inspire_RH5DG2/`。如需在 MuJoCo 中使用手部，须单独处理：为对应 URDF 建立资源路径转换，并根据任务决定保留原始视觉网格或另行生成凸包碰撞网格。手部 STL 不应直接放入本目录，也不应改写本基础 V3 XML。
