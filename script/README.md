# 天工行者Dex URDF 脚本说明

本目录的脚本只服务于上一级 `tiangong3_urdf` 包，默认输入均按脚本自身位置解析，因此可以从任意工作目录调用。

## 环境

本机已在 Miniconda 的 `xSIM` 环境中验证：

- Python 3.11.15
- MuJoCo 3.10.0
- SciPy 1.17.1
- `urdf2mjcf==0.2.39`
- `trimesh==5.1.0`

如果需要重新安装后两个依赖：

```bash
/Users/loong/miniconda3/bin/conda run -n xSIM \
  python -m pip install 'urdf2mjcf==0.2.39' 'trimesh==5.1.0'
```

`prime_to_simplify.py` 另外需要 `fast-simplification`（导入名是 `fast_simplification`）：

```bash
/Users/loong/miniconda3/bin/conda run -n xSIM \
  python -m pip install 'fast-simplification>=0.2.0'
```

下文命令均在 `Tienkung3Dex_URDF_V3` 目录执行。

## 脚本用途

| 脚本 | 用途 | 默认输入 | 输出或影响 |
| --- | --- | --- | --- |
| `check_link_and_update.py` | 检查或回写 link 的质量、质心和惯性参数 | `关键参数/URDF关键参数表 - 天工行者Dex(V3).csv`、`urdf/tiangong3.urdf` | `--check` 只报告；`--update` 修改 URDF |
| `check_joint_and_update.py` | 检查或回写 joint 的类型、轴向和限位参数 | 同上 | `--check` 只报告；`--update` 修改 URDF |
| `prime_to_convex.py` | 将 STL 转为凸包，降低 MuJoCo 加载网格的复杂度 | `meshes/` | 写入 `mujoco/meshes_convex/` |
| `make_mjcf_torq.py` | 按 URDF 的 effort 等数据生成带执行器参数的 MJCF，网格目录可选 | `urdf/tiangong3.urdf`、`mujoco/meshes_convex/` 或 `mujoco/meshes_simplify/` | 生成 MJCF（默认覆盖 `mujoco/tiangong3_torq.xml`） |
| `prime_to_simplify.py` | 按比例（默认 10%）做 QEM 边坍缩减面，打印面数/包围盒/体积偏差；可用 `--reference` 与参考结果做回归对照 | 由 `-i` 指定（默认 `meshes/`） | 默认只读；给出 `-o` 时才写出减面网格 |

## 参数速查

### `make_mjcf_torq.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--urdf` | `urdf/tiangong3.urdf` | 输入 URDF |
| `--mjcf` | `mujoco/tiangong3_torq.xml` | 输出 MJCF。更换文件名即可生成变体，不会覆盖默认文件 |
| `--mesh-subdir` | `meshes_convex` | MJCF 引用的网格子目录名（相对 MJCF 所在目录）。改成 `meshes_simplify` 即生成用减面网格的版本 |
| `--floor-z` | 自动计算 | 地面平面的高度。默认加载刚生成的模型，取默认位姿下**全部网格 geom 的最低点**，让默认姿态刚好踩在地面上 |

补充说明：

- `--mjcf` 传相对路径时按**当前工作目录**解析，建议在包根目录执行；默认值是包内绝对路径。
- 自动计算地面高度需要 `mujoco`；不可用时退回 `z=0` 并打印告警。
- `--mesh-subdir` 指向的目录不存在不会报错，MJCF 照样生成，但 MuJoCo 加载时会找不到网格。

常用组合：

```bash
# 默认：凸包网格 + mujoco/tiangong3_torq.xml
/Users/loong/miniconda3/bin/conda run -n xSIM \
  python script/make_mjcf_torq.py

# 减面网格 + 另存为 mujoco/tiangong3_simplify.xml
/Users/loong/miniconda3/bin/conda run -n xSIM \
  python script/make_mjcf_torq.py \
    --mjcf mujoco/tiangong3_simplify.xml --mesh-subdir meshes_simplify

# 手动指定地面高度（不自动计算时）
/Users/loong/miniconda3/bin/conda run -n xSIM \
  python script/make_mjcf_torq.py --floor-z -0.0574
```

### `prime_to_simplify.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `-i` / `--input` | `meshes/` | 输入网格目录；只处理该目录下的 STL，不递归子目录 |
| `-o` / `--output` | 无 | 输出目录。**不给出时只打印报告，不写任何文件**；与输入目录相同会直接报错 |
| `--ratio` | `0.1` | 目标面数比例，目标面数 = `round(ratio × 源面数)` |
| `--merge-tolerance` | `1e-8` | 焊接顶点容差（米）。默认只合并坐标完全相同的重复顶点，不要随意放大 |
| `--agg` | `7` | `fast_simplification` 的激进程度参数 |
| `--skip` | 空 | 按文件名子串跳过，可给多个 |
| `--min-faces` | `0` | 源面数低于该值的文件跳过 |
| `--reference` | 无 | 参考结果目录（例如 Blender 减面产物），逐文件对照面数/包围盒/体积 |
| `--deviation` | 关 | 额外计算减面结果相对源网格的表面偏差（需要 scipy，较慢） |
| `--deviation-samples` | `2000` | 表面偏差的采样点数 |
| `--tol-faces` | `2` | 与目标面数的允许偏差（面） |
| `--tol-bbox` | `0.003` m | 包围盒每轴允许偏差 |
| `--tol-volume` | `5.0` % | 体积相对偏差上限；源网格非水密时该值不参与判定 |
| `--strict` | 关 | 出现超差时以退出码 1 结束 |

### `prime_to_convex.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `-i` / `--input` | `meshes/` | 输入网格目录 |
| `-o` / `--output` | `mujoco/meshes_convex/` | 输出目录；与输入目录相同会直接报错 |

### `check_link_and_update.py` / `check_joint_and_update.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--check` | 默认行为 | 只读比对并打印不一致项，不修改任何文件 |
| `--update` | 关 | 按 CSV 回写 `urdf/tiangong3.urdf`，会直接覆盖 |

## 推荐执行顺序

### 1. 只读检查 URDF 参数

不带参数与显式传入 `--check` 等效，均不会修改 URDF：

```bash
/Users/loong/miniconda3/bin/conda run -n xSIM \
  python script/check_link_and_update.py --check

/Users/loong/miniconda3/bin/conda run -n xSIM \
  python script/check_joint_and_update.py --check
```

检查不一致时以终端打印内容为准；当前脚本不会仅因参数不一致而返回非零退出码。

### 2. 按需回写 URDF

只有确认 CSV 是本次更新的参数源时才执行：

```bash
/Users/loong/miniconda3/bin/conda run -n xSIM \
  python script/check_link_and_update.py --update

/Users/loong/miniconda3/bin/conda run -n xSIM \
  python script/check_joint_and_update.py --update
```

`--update` 会直接改写 `urdf/tiangong3.urdf`，执行前应自行保留版本或备份。

### 3. 重新生成凸包 STL

仅在 `meshes/` 发生变化或需要重建凸包时执行：

```bash
/Users/loong/miniconda3/bin/conda run -n xSIM \
  python script/prime_to_convex.py
```

默认将 `meshes/` 中所有 STL 的凸包写到 `mujoco/meshes_convex/`，同名文件会被覆盖。也可指定临时目录进行无覆盖测试：

```bash
/Users/loong/miniconda3/bin/conda run -n xSIM \
  python script/prime_to_convex.py -i meshes -o /tmp/tiangong3_meshes_convex
```

脚本禁止输入目录与输出目录相同，以免覆盖原始网格。

### 4. 生成 MJCF

确认 `mujoco/meshes_convex/` 已准备好后执行：

```bash
/Users/loong/miniconda3/bin/conda run -n xSIM \
  python script/make_mjcf_torq.py
```

该命令会覆盖 `mujoco/tiangong3_torq.xml`（用 `--mjcf` 可换输出文件）。生成脚本会：

- 为 31 个非固定关节创建执行器元数据；
- 根据 URDF 的 `effort` 设置力矩范围，并按现有规则设置阻尼和摩擦；
- 将网格路径切换到 `mujoco/<--mesh-subdir>/`（相对于 MJCF 文件为 `./<子目录>/`）；
- 修正 URDF 空 material 名称产生的无效 MJCF 名称；
- 在 MJCF 的 `worldbody` 中加入地面，并按默认位姿下全部网格 geom 的最低点自动定位地面高度。

### 5. 减面网格（可选，独立于上述流程）

`prime_to_simplify.py` 与步骤 1-4 相互独立：它只对网格做 QEM 边坍缩减面并输出偏差报告，
不改动 URDF 或凸包。其产物可作为步骤 4 中 `--mesh-subdir` 的输入：

```bash
/Users/loong/miniconda3/bin/conda run -n xSIM \
  python script/prime_to_simplify.py -o mujoco/meshes_simplify
```

不带 `-o` 时只扫描并打印报告，不写任何文件；输入与输出目录相同会直接报错。
默认目标面数为 `round(0.1 × 源面数)`、顶点焊接容差 1e-8。容差不宜放大：实测
放大到 1e-4 会把邻近的独立顶点一并合并，`shoulder_pitch_l_link` 这类网格因此坍缩
受阻，面数只能降至目标的约 2.5 倍。

与参考结果做逐文件回归对照（例如和工程师用 Blender 减面得到的 `meshes_new`）：

```bash
/Users/loong/miniconda3/bin/conda run -n xSIM \
  python script/prime_to_simplify.py -i /tmp/test/meshes -o /tmp/decimate_out \
    --reference /tmp/test/meshes_new
```

报告列：面数、包围盒偏差、体积偏差、与参考的面数差；`--strict` 时出现超差返回
退出码 1。体积指标只对水密源网格参与判定，其余只打印数值并注明「体积仅参考」。

## MJCF 选用说明

**结论：实际使用请使用 `mujoco/tiangong3_torq.xml`；`mujoco/tiangong3_simplify.xml` 仅用于视觉查看。**

| 文件 | 网格来源 | 网格合计面数 | 碰撞是否可靠 | 用途 |
| --- | --- | --- | --- | --- |
| **`mujoco/tiangong3_torq.xml`** | `meshes_convex/`（凸包） | 83,714 | **可靠** | **实际使用**：接触仿真、站立、抓取、需要真实接触力的场景、RL 训练 |
| `mujoco/tiangong3_simplify.xml` | `meshes_simplify/`（10% 减面） | 293,815 | **不可靠** | **仅视觉查看**：查看外观、轻量化预览、动作回放，不得用于任何需要接触的场景 |

两者的区别：

- **凸包**：MuJoCo 的 mesh 碰撞按凸体处理，因此凸包是唯一可直接用于碰撞的网格形式；
  代价是外观呈多面体状，明显凹陷的零件（如带开口的 TCP、线束通道）形状会失真。
- **减面网格**：外观接近原始网格（原始 2,903,682 面降至 293,815 面），但**仍然是非凸的**，
  以其作为碰撞体会得到错误接触，只能作为视觉网格使用。
- 原始 `meshes/`：既不能直接用于碰撞（非凸），其中 `pelvis`（253,940 面）和
  `waist_pitch_link`（323,344 面）还超过 MuJoCo 单网格 20 万面的上限，连加载都会失败。

### 实测：MuJoCo 对非凸网格按凸包计算

用 36 面的凹槽（底板 + 两侧壁、顶部敞开，包围盒 z∈[0, 0.080]）与小球做判定实验：

| 判定 | 小球应停在 | 实测 |
| --- | --- | --- |
| 按三角形（保留凹陷） | z ≈ 0.050（槽底） | 未发生（不符） |
| 按凸包（忽略凹陷） | z ≈ 0.110（凸包顶面） | **0.1096**（符合） |

凹陷被完全忽略；MuJoCo 加载时不产生任何告警，完全以输入网格为准。

由此可得：**以减面网格作为碰撞体，等价于以"减面后网格的凸包"作为碰撞体**，其精度低于直接
使用凸包。用 V3 的 40 个零件实测，减面后凸包相对原始凸包：

| 对比项 | 结果 |
| --- | --- |
| 包围盒差 | 中位 1.647 mm，最大 14.719 mm，超 1 mm 的有 28/40 |
| 体积差 | 中位 1.15%，最大 97.63% |

碰撞实际使用的是凸包，而该凸包由少一个数量级的点集算出，精度反而下降；减面带来的外观改善
对碰撞没有作用。

### simplify 版本为何只适用于视觉查看

`tiangong3_simplify.xml` 是通过 `--mesh-subdir meshes_simplify` 整体替换路径生成的，
其 `*_visual` 与 `*_collision` **都指向减面网格**，因而不适合接触仿真。
它的用途限于在 MuJoCo viewer 中查看外观、确认装配关系与轻量预览。

需要接触的场景一律使用 `tiangong3_torq.xml`。若既要保留外观、又要有可靠的碰撞，需要将两者
组合（视觉 geom 用减面、碰撞 geom 用凸包）——`make_mjcf_torq.py` 当前不支持这种拆分，
如需该功能请先确认。

两份 XML 的地面高度都按各自网格自动计算（见「参数速查」的 `--floor-z`）：凸包版本为
`z=-0.056774`，减面版本为 `z=-0.057388`——减面后足部网格最低点变化 0.614 mm，不能沿用原值。

## 凸包面数的原则

**不应假定"凸包一定很小"。** 凸包面数由落在凸包上的极值面复杂度决定，与原始网格面数没有联动关系。
实测反例：

| 形状 | 原面数 | 凸包面数 | 比例 |
| --- | --- | --- | --- |
| 球 subdivisions=6 | 81,920 | 81,920 | 1.0000 |
| 球 subdivisions=4 | 5,120 | 5,120 | 1.0000 |
| 圆柱 256 段 | 1,024 | 1,020 | 0.9961 |

也就是说，一个 20 万面的球状件，其凸包仍约为 20 万面，同样会触及 MuJoCo 单网格 20 万面的上限。
MuJoCo 的面数限制按每个 mesh 资产计算，凸包同样是 mesh 资产，限制同样适用。

本包这套网格余量充足，原因在于它是机械 CAD 网格：凹陷、内腔、走线槽、螺纹孔占去了绝大部分
三角面，而这些**不进凸包**。

| 文件 | 原网格面数 | 凸包面数 | 比例 |
| --- | --- | --- | --- |
| `hand_flange_link.STL` | 52,294 | **15,780** | 0.3018（本仓库最大） |
| `waist_pitch_link.STL` | 323,344 | 5,774 | 0.0179 |
| `pelvis.STL` | 253,940 | 5,452 | 0.0215 |
| 合计 | 2,903,682 | 83,714 | 0.0288 |

最大凸包 15,780 面，距 20 万上限约 12 倍余量。生成后如需确认余量，查看最大凸包面数即可
（二进制 STL 面数 = `(字节数 - 84) ÷ 50`）：

```bash
cd Tienkung3Dex_URDF_V3/mujoco/meshes_convex
for f in *.STL; do echo "$(( ( $(stat -f%z "$f") - 84 ) / 50 )) $f"; done | sort -rn | head -5
```

### 触及上限时的处置

**不建议将"先 10% 减面再求凸包"作为常规手段。** 实测（V3 的 40 个零件，先减面 10% 再求凸包 vs 直接求凸包）：

| 文件 | 凸包(直接) | 凸包(先减面) | 包围盒Δ | 体积Δ |
| --- | --- | --- | --- | --- |
| `imu_waist_link` | 84 | 38 | 14.719 mm | 97.63% |
| `elbow_pitch_l_link` | 910 | 102 | 10.581 mm | 20.72% |
| `wrist_pitch_r_link` | 1,164 | 128 | 7.827 mm | 38.04% |
| 合计/统计 | 83,714 | 10,074 | 中位 1.647 mm，超 1 mm 的有 28/40 | 中位 1.15%，最大 97.63% |

面数可压缩至 1/8，但误差分布极不均匀：决定凸包极值点的正是细长凸起与薄壁，而减面会优先消除
这些特征。

按以下优先级处理：

1. **primitive 近似**（`sphere` / `capsule` / `cylinder` / `box`）：面数为 0，接触更稳定，适用于盒体类
   零件（`imu_head_link`、`imu_waist_link`、`radar_head_link`）；
2. **拆分为多个凸块**：每块都在限内，既可保形又能碰撞；
3. **先轻度减面再求凸包**：比例取 0.3~0.5，不宜直接取 0.1。误差随减面强度放大，仅对"本身接近凸"
   的密集曲面件合算——该情况下误差是可控的弦高误差；
4. **不建议对凸包结果再做 QEM 减面**：会破坏严格凸性，而 MuJoCo 的 mesh 碰撞按凸体处理，非凸结果不可靠。

## MuJoCo 加载检查

生成后可用下面的命令做结构和网格加载检查：

```bash
/Users/loong/miniconda3/bin/conda run -n xSIM python -c \
  "import mujoco; m=mujoco.MjModel.from_xml_path('mujoco/tiangong3_torq.xml'); print('OK', m.nbody, m.njnt, m.nu, m.nmesh)"
```

当前实测结果为：

```text
OK 40 32 31 39
```

## 已知注意事项

- `make_mjcf_torq.py` 转换时会提示忽略 URDF 惯性矩阵的非对角项。这不阻塞 MuJoCo 加载，但可能影响动力学精度，需要单独确认是否符合仿真要求。
- 原始 `meshes/` 中部分 STL 超过 MuJoCo 的单网格面数限制，因此生成的 MJCF 使用 `mujoco/meshes_convex/`，不能只生成 XML 而缺少凸包文件。
- `--update`、凸包默认输出和 MJCF 生成都会覆盖现有文件；日常核对优先使用两个检查脚本的默认 `--check` 模式。
- `make_mjcf_torq.py` 现在会自动计算地面高度。此前 `mujoco/tiangong3_torq.xml` 里的 `pos="0 0 -0.056774"` 是人工改的，直接重新生成会丢失；改为自动计算后，用 `meshes_convex` 重新生成仍得到 `-0.056774`（已实测一致），只是注释文字不同。
- `prime_to_simplify.py` 与 Blender Decimate 不等价，只是同族的 QEM 边坍缩减面。以 `tmp/test` 那 39 对文件实测：33 个文件的面数能对齐到 ±2 面，其余 6 个因两边停止规则不同而不一致——`imu_head_link`/`imu_waist_link` Blender 只减到 50%（非流形边导致提前停止），`camera_body_front_link`/`camera_head_link`/`radar_head_link` 本脚本减不到 10%，`waist_pitch_link` 参考结果未做减面。
- 凸包的面数**不受原始网格面数保护**：球、圆柱这类接近凸的密集曲面件，凸包几乎不缩，同样会触及 20 万面上限。判断余量与触及上限时的处置优先级见「凸包面数的原则」一节。
