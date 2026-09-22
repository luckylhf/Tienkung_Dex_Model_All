# MuJoCo 基础 V3 资源

本目录仅服务 `urdf/tiangong3.urdf` 的基础 V3 全身模型，不包含 BrainCo 或 Inspire 灵巧手。

## 内容

- `meshes_convex/`：由根目录 `meshes/` 的 40 个 STL 生成的凸包网格。它用于当前 MJCF 的网格加载和碰撞，保留原始 `meshes/` 不变。
- `tiangong3_torq.xml`：由 `script/make_mjcf_torq.py` 生成的 MJCF，引用本目录的 `meshes_convex/`。

已于 2026-09-22 验证：40 个凸包网格与源网格同名、包围盒不变；总面数从 2,903,682 降至 83,714。MuJoCo 3.10.0 可加载并步进 `tiangong3_torq.xml`（40 bodies、32 joints、31 actuators、39 个被引用网格）。

## 生成与校验

在包根目录执行：

```bash
/Users/loong/miniconda3/bin/conda run -n xSIM python script/prime_to_convex.py
/Users/loong/miniconda3/bin/conda run -n xSIM python script/make_mjcf_torq.py
/Users/loong/miniconda3/bin/conda run -n xSIM python -c \
  "import mujoco; m=mujoco.MjModel.from_xml_path('mujoco/tiangong3_torq.xml'); print('OK', m.nbody, m.njnt, m.nu, m.nmesh)"
```

## 灵巧手边界

当前 XML 不引用 `BrainCo_revo2/` 或 `Inspire_RH5DG2/`。如需在 MuJoCo 使用手部，使用者须单独处理：为对应 URDF 建立资源路径转换，并根据任务决定保留原始视觉网格还是生成独立的凸包碰撞网格。不要把手部 STL 直接放入本目录或改写本基础 V3 XML。
