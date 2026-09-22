# USD 候选资产包

本目录从 [Open-X-Humanoid/x-humanoid-vla-simulation-benchmark 的 `usds/tiangong3`](https://github.com/Open-X-Humanoid/x-humanoid-vla-simulation-benchmark/tree/main/usds/tiangong3) 迁入，它用于保留一套独立的候选模型；不是 `Tienkung3Dex_URDF_V3` 当前 V3 模型的替换来源。

## 内容与可用边界

- `tienkung3_urdf/`：带 BrainCo Revo2 灵巧手的自包含 ROS 候选模型。它有 81 links、80 joints；URDF 引用的 120 个网格在其 `meshes_new/` 内均存在。
- `tiangong3_dex_brainco2.usd` 与 `tiangong3_dex_inspire.usd`：已取回实际 USD 二进制（USD Crate 0.9.0，`PXR-USDC`），大小分别为 83,673,544 与 76,883,763 bytes，SHA-256 与 LFS 指针中的 oid 一致。
- ROS 包名、CMake 项目名和 URDF 资源 URI 已统一为 `tienkung3_urdf`；CMake 仅安装实际存在的 `urdf/` 与 `meshes_new/`，其 120 个 URDF 网格引用均解析到本包的 `meshes_new/`。

## 与当前 V3 BrainCo 模型的差异

- 候选模型的腕部直接连接到手基座；当前 V3 BrainCo 变体为“手腕 → 独立转接法兰 → 手基座”。因此两者不能混用腕部安装位姿。
- 已由用户确认：候选模型的 `meshes_new/wrist_roll_l_link.STL` 与 `meshes_new/wrist_roll_r_link.STL` **自带法兰**，与最终真实版本的腕部结构不同。不得用这两个候选网格覆盖当前 V3 的同名网格，也不得把它们当作最终腕部-手部转接尺寸依据。
- 37 个同名机身网格虽然挂在相同 link 名称下，但相对当前 V3 的包围盒尺寸差为 0–38.499 mm；38 个同名 BrainCo 手网格中只有 11 个二进制相同。

结论：需要使用时，应将本目录作为一套完整候选模型处理；不要抽取其中 STL 直接替换当前 V3 的 `meshes/`、法兰或手部资产。
