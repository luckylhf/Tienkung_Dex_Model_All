# 天工行者Dex 模型包

天工行者Dex人形机器人的 ROS 2 描述包，提供 URDF 模型、网格资源、RViz 显示配置、MuJoCo 仿真资源，以及配套的生成与校验脚本。

## 命名说明

文档与文件名中出现的几种名称，指的都是同一台机器人：

| 中文 | 英文 | 说明 |
| --- | --- | --- |
| 天工行者Dex | Walker Tienkung Dex | 机器人名称。「行者」对应 `Walker`，`Dex` 是机器人指定型号 |
| 天工、天工3、天工3.0 | Tiangong、TIANGONG 3.0 | 早期写法，仍保留在部分技术标识中 |

**`tiangong` 与 `tienkung` 是同一名称的两种拼写**：早期标识沿用 `tiangong`，后续统一为 `tienkung`。
本包的显示名称已按新写法更新，但为免牵动 launch 文件、`package.xml` 依赖与 `package://` 资源路径，
下列技术标识**保持旧拼写不变**：

- ROS 包名 `tiangong3_urdf`，它同时是资源前缀 `package://tiangong3_urdf/`
- `urdf/tiangong3.urdf` 中的 `<robot name="tiangong3_urdf">`，以及由它生成的两份 MJCF 的 `<mujoco model="tiangong3_urdf">`
- 各文件名中的 `tiangong3`（如 `urdf/tiangong3.urdf`、`mujoco/tiangong3_torq.xml`）

因此文档里的「天工行者Dex / Walker Tienkung Dex」与包名、文件名里的 `tiangong3_urdf` 指同一套模型。
新拼写 `tienkung` 另见目录名 `Tienkung3Dex_URDF_V3`、上游仓库 `Open-X-Humanoid/TienKung_URDF`
与手部候选包 `tienkung3_urdf`。

上游本身也是这种混用：`Open-X-Humanoid/TienKung_URDF` 的仓库名用 `TienKung`，其内存放模型的目录
仍叫 `tiangong3_urdf`，与本地情况一致。

## 目录结构

| 路径 | 内容 |
| --- | --- |
| `urdf/` | 三份 URDF，见「模型文件」 |
| `meshes/` | 40 个 STL；`tiangong3.urdf` 用其中 39 个，`hand_flange_link.STL`（BrainCo 转接件）由 `tiangong3_brainco.urdf` 引用 |
| `launch/display.launch.py` | RViz2 显示启动文件 |
| `config/display.rviz` | 随启动文件使用的 RViz2 配置 |
| `mujoco/` | MuJoCo 资源：凸包网格、减面网格与两份 MJCF，详见 `mujoco/README.md` |
| `script/` | 生成与校验脚本，详见 `script/README.md` |
| `关键参数/` | V3 关键参数表（CSV）与参考图，供校验脚本比对 |
| `BrainCo_revo2/`、`Inspire_RH5DG2/`、`USD候选资产包/` | 随包交付的独立 ROS 包，见「随包的独立包」 |
| `LICENSE` | 许可证正文 |

## 模型文件

| 文件 | links | joints | 说明 |
| --- | --- | --- | --- |
| `urdf/tiangong3.urdf` | 39 | 38 | 基础 V3 全身模型。根 link 为 `pelvis`；31 个旋转关节 + 7 个固定关节；双臂末端为 `left_tcp_link` / `right_tcp_link` |
| `urdf/tiangong3_brainco.urdf` | 83 | 82 | 集成 BrainCo Revo2 灵巧手的派生模型。腕部经 `*_hand_flange_link` 转接件连接到手基座 |
| `urdf/tiangong3_inspire.urdf` | 101 | 100 | 集成 Inspire RH5DG2 灵巧手的派生模型。腕部直接连接到 `*_hand_base` |

两份派生模型通过 `package://` 引用独立手包的网格，需要先构建对应包。

## 依赖

构建依赖：`ament_cmake`。

运行 `launch/display.launch.py` 需要：`launch`、`launch_ros`、`xacro`、`robot_state_publisher`、`joint_state_publisher_gui`、`rviz2`。

`tiangong3_brainco.urdf` 与 `tiangong3_inspire.urdf` 另外需要 `revo2_description`、`RH5DG2_L`、`RH5DG2_R`（见「随包的独立包」）。

## 构建与显示

```bash
colcon build
source install/setup.bash
ros2 launch tiangong3_urdf display.launch.py
```

启动文件提供两个参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `model` | `<share>/urdf/tiangong3.urdf` | 要显示的 URDF 绝对路径；经 `xacro` 解析后送入 `robot_state_publisher` |
| `rvizconfig` | `<share>/config/display.rviz` | RViz2 配置文件绝对路径 |

显示带灵巧手的派生模型（需已构建对应手包）：

```bash
ros2 launch tiangong3_urdf display.launch.py \
  model:=$(ros2 pkg prefix tiangong3_urdf)/share/tiangong3_urdf/urdf/tiangong3_brainco.urdf
```

## 随包的独立包

本包内嵌套了若干独立 ROS 包。`colcon build --base-paths <本包目录>` 只会发现根包，需要显式列出这些路径，或将其作为独立源目录加入工作空间。

| 路径 | ROS 包名 | 许可证 |
| --- | --- | --- |
| `BrainCo_revo2/revo2_description/` | `revo2_description` | Apache License 2.0 |
| `Inspire_RH5DG2/RH5DG2_L/` | `RH5DG2_L` | BSD |
| `Inspire_RH5DG2/RH5DG2_R/` | `RH5DG2_R` | BSD |
| `USD候选资产包/tienkung3_urdf/` | `tienkung3_urdf` | OpenAtom Open Hardware License 1.0 |

`USD候选资产包/` 是一套独立的候选模型与 USD 资产，不是当前 V3 模型的替换来源，详见该目录的 `README.md`。

## MuJoCo 资源

`mujoco/` 提供两份 MJCF：

| 文件 | 网格来源 | 用途 |
| --- | --- | --- |
| `tiangong3_torq.xml` | `meshes_convex/`（凸包） | 实际使用：接触仿真 |
| `tiangong3_simplify.xml` | `meshes_simplify/`（10% 减面网格） | 仅用于视觉查看 |

凸包与减面网格均由 `script/` 下的脚本生成；参数说明与选型依据见 `script/README.md`。

## 脚本

`script/` 下 5 个脚本，均在包根目录执行：

| 脚本 | 用途 |
| --- | --- |
| `check_link_and_update.py` | 检查或回写 link 的质量、质心与惯性参数 |
| `check_joint_and_update.py` | 检查或回写 joint 的类型、轴向与限位参数 |
| `prime_to_convex.py` | 由 `meshes/` 生成碰撞用凸包网格 |
| `prime_to_simplify.py` | 由 `meshes/` 生成视觉用减面网格 |
| `make_mjcf_torq.py` | 由 URDF 生成 MJCF |

用法与注意事项见 `script/README.md`。

## 许可证

本包采用 OpenAtom Open Hardware License 1.0（开放原子开放硬件许可证 第一版），正文见根目录 `LICENSE`。随包的独立包各自持有许可证，见「随包的独立包」。
