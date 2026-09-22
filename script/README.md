# 天工 3 URDF 脚本说明

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

下文命令均在 `Tienkung3Dex_URDF_V3` 目录执行。

## 脚本用途

| 脚本 | 用途 | 默认输入 | 输出或影响 |
| --- | --- | --- | --- |
| `check_link_and_update.py` | 检查或回写 link 的质量、质心和惯性参数 | `关键参数/URDF关键参数表 - 天工3.0(V3).csv`、`urdf/tiangong3.urdf` | `--check` 只报告；`--update` 修改 URDF |
| `check_joint_and_update.py` | 检查或回写 joint 的类型、轴向和限位参数 | 同上 | `--check` 只报告；`--update` 修改 URDF |
| `prime_to_convex.py` | 将 STL 转为凸包，降低 MuJoCo 加载网格的复杂度 | `meshes/` | 写入 `mujoco/meshes_convex/` |
| `make_mjcf_torq.py` | 按 URDF 的 effort 等数据生成带执行器参数的 MJCF | `urdf/tiangong3.urdf`、`mujoco/meshes_convex/` | 覆盖生成 `mujoco/tiangong3_torq.xml` |

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

该命令会覆盖 `mujoco/tiangong3_torq.xml`。生成脚本会：

- 为 31 个非固定关节创建执行器元数据；
- 根据 URDF 的 `effort` 设置力矩范围，并按现有规则设置阻尼和摩擦；
- 将网格路径切换到 `mujoco/meshes_convex/`（相对于 MJCF 文件为 `./meshes_convex/`）；
- 修正 URDF 空 material 名称产生的无效 MJCF 名称；
- 在 MJCF 的 `worldbody` 中加入地面。

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
