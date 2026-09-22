#!/usr/bin/env python3
"""按比例减面（QEM 边坍缩），生成用于 MuJoCo 的视觉网格并输出偏差报告。

=============================================================================
前置条件
=============================================================================
Python 依赖（本机已在 Miniconda 的 `xSIM` 环境验证过版本）：

    fast_simplification >= 0.2.0   # 减面核心：QEM（二次误差）边坍缩，纯 C++ 实现
    trimesh            >= 5.1.0    # 读写 STL、统计包围盒/体积
    numpy

安装（注意 pip 包名是 `fast-simplification`，导入名是 `fast_simplification`）：

    /Users/loong/miniconda3/bin/conda run -n xSIM \
        python -m pip install 'fast-simplification>=0.2.0' 'trimesh>=5.1.0'

可选依赖：`--deviation` 需要 scipy（本机 xSIM 已有 1.17.1）。

调用方式（脚本按自身位置解析默认目录，可从任意工作目录运行）：

    /Users/loong/miniconda3/bin/conda run -n xSIM python script/prime_to_simplify.py

=============================================================================
用途与边界
=============================================================================
做：
  - 把 `meshes/` 中每个 STL 焊接顶点后按目标面数做 QEM 边坍缩减面；
  - 打印每个文件的面数、包围盒偏差、体积偏差；
  - 可选 `--reference` 与一套参考结果（例如工程师用 Blender 得到的 meshes_new）
    逐文件对照面数，用于回归校验。

不做：
  - 不能复现 Blender Decimate 的逐字节结果。二者同属 QEM 边坍缩算法族，但代价
    函数、边界权重、停止规则不同；即使面数相同，几何也可差到厘米级。本脚本定位
    是"等效的批量减面工具"，不是"Blender 产物复刻器"。

已知行为差异（实测 tmp/test 下那 39 对文件）：
  - Blender 在非流形网格上会提前停止（imu_head_link 只减到 50%），本脚本会减到目标；
  - Blender 可能对已经是简化件的网格原样输出（waist_pitch_link 15000 → 15000），
    本脚本会继续减面，需要时用 --skip 排除；
  - Blender 默认保留边界/极值，本脚本只有 `--agg` 一个激进程度参数，薄板件更容易
    被塌过头（实测 imu_head_link 的包围盒会明显变化）。

额外注意：
  - STL 里每个三角形存独立顶点，必须先焊接；本脚本默认容差 1e-8，只合并坐标完全相同
    的重复顶点。实测把容差放大到 1e-4 会把邻近的独立顶点一起并掉，
    `shoulder_pitch_l_link` 这类网格因此坍缩受阻、面数只能降到目标的约 2.5 倍，
    所以不要随意放大 --merge-tolerance。

=============================================================================
用法
=============================================================================
默认只读：只扫描 `meshes/` 打印报告，不写任何文件

    python script/prime_to_simplify.py

落盘一套减面网格（目录不存在会创建，同名文件会被覆盖）

    python script/prime_to_simplify.py -o mujoco/meshes_vis

与参考结果做回归对照

    python script/prime_to_simplify.py \
        -i /tmp/test/meshes -o /tmp/out --reference /tmp/test/meshes_new

自定义比例、跳过指定文件、附带表面偏差

    python script/prime_to_simplify.py --ratio 0.2 --skip waist_pitch_link.STL --deviation

=============================================================================
报告判定阈值（均可用参数覆盖）
=============================================================================
  --tol-faces    与目标面数的允许偏差，默认 2 面
  --tol-bbox     包围盒每轴允许偏差（米），默认 0.003（3 mm）
  --tol-volume   体积相对偏差允许上限，默认 5%
阈值都是本仓库 39 个零件实测分布上的经验值。注意体积指标只对水密网格有意义：
源网格非水密时体积由散度定理算出、数值不可靠（实测 imu_head_link 会给出 +98% 的
假偏差），此时只打印数值、不参与判定，报告里会注"体积仅参考"。
超过阈值的行标记 WARN 并在末尾汇总；加 --strict 时以退出码 1 结束。
"""

from __future__ import annotations

import argparse
import math
import sys
import unicodedata
from pathlib import Path

import numpy as np
import trimesh

try:
    import fast_simplification
except ImportError:  # 依赖缺失时给出安装指引，而不是抛裸栈
    raise SystemExit(
        "缺少依赖 fast_simplification。请先安装：\n"
        "  /Users/loong/miniconda3/bin/conda run -n xSIM python -m pip install "
        "'fast-simplification>=0.2.0' 'trimesh>=5.1.0'"
    )


# ---------------------------------------------------------------------------
# 终端输出对齐（中英文混排时按显示宽度补齐）
# ---------------------------------------------------------------------------
def _disp_width(text: str) -> int:
    """按终端显示宽度计算长度：中日韩全角字符算 2 列。"""
    return sum(2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1 for ch in text)


def fmt_row(cells, widths, aligns=None) -> str:
    aligns = aligns if aligns is not None else ["left"] * len(cells)
    parts = []
    for cell, width, align in zip(cells, widths, aligns):
        text = str(cell)
        gap = " " * max(0, width - _disp_width(text))
        parts.append(gap + text if align == "right" else text + gap)
    return "  ".join(parts).rstrip()


# ---------------------------------------------------------------------------
# 网格读写与焊接
# ---------------------------------------------------------------------------
def load_mesh(path: Path) -> trimesh.Trimesh:
    """读取网格文件。

    STL 里每个三角形存的是独立顶点，不焊接的话边坍缩无从下手（一个面都减不掉），
    所以这里先用 process=False 拿原始数据，再由 weld_vertices 显式焊接。
    """
    loaded = trimesh.load(str(path), process=False)
    if isinstance(loaded, trimesh.Scene):
        geoms = [g.copy() for g in loaded.geometry.values()
                 if isinstance(g, trimesh.Trimesh)]
        if not geoms:
            raise ValueError("文件中没有三角网格")
        loaded = trimesh.util.concatenate(tuple(geoms))
    if not isinstance(loaded, trimesh.Trimesh) or len(loaded.faces) == 0:
        raise ValueError("无法解析出三角网格")
    return loaded


def weld_vertices(mesh: trimesh.Trimesh, tolerance: float) -> trimesh.Trimesh:
    """按容差焊接顶点（STL 原始数据每个三角形的顶点都是独立的，必须先焊接）。

    tolerance 单位为米，默认 1e-8，只合并坐标完全相同的重复顶点。实测容差放大到 1e-4
    会把邻近的独立顶点也并掉，导致部分网格坍缩受阻（面数只能降到目标的约 2.5 倍）。
    """
    mesh = mesh.copy()
    digits = max(0, int(round(-math.log10(tolerance)))) if tolerance > 0 else 8
    try:
        mesh.merge_vertices(merge_tex=True, merge_norm=True, digits_vertex=digits)
    except TypeError:  # 兼容 merge_vertices 签名不同的 trimesh 版本
        mesh.merge_vertices()
    return mesh


# ---------------------------------------------------------------------------
# 减面
# ---------------------------------------------------------------------------
def run_simplify(mesh: trimesh.Trimesh, target_count: int, agg):
    """调用 fast_simplification 做 QEM 边坍缩，返回新网格。"""
    kwargs = {"target_count": int(target_count)}
    if agg is not None:
        kwargs["agg"] = int(agg)
    try:
        vertices, faces = fast_simplification.simplify(mesh.vertices, mesh.faces, **kwargs)
    except TypeError:
        # 当前版本不认识 agg 时退一步重试
        kwargs.pop("agg", None)
        vertices, faces = fast_simplification.simplify(mesh.vertices, mesh.faces, **kwargs)
    return trimesh.Trimesh(
        vertices=np.asarray(vertices, dtype=np.float64),
        faces=np.asarray(faces),
        process=False,
    )


# ---------------------------------------------------------------------------
# 指标统计
# ---------------------------------------------------------------------------
def mesh_stats(mesh: trimesh.Trimesh) -> dict:
    stats = {"faces": int(len(mesh.faces)), "bounds": None,
             "volume": float("nan"), "watertight": False}
    if len(mesh.faces):
        stats["bounds"] = np.asarray(mesh.bounds, dtype=np.float64)
    try:
        stats["volume"] = abs(float(mesh.volume))
    except Exception:
        pass  # 非水密/退化网格取不到体积时保持 nan
    try:
        stats["watertight"] = bool(mesh.is_watertight)
    except Exception:
        stats["watertight"] = False
    return stats


def bbox_delta_mm(bounds_a, bounds_b) -> float:
    if bounds_a is None or bounds_b is None:
        return float("nan")
    return float(np.abs(bounds_a - bounds_b).max()) * 1000.0


def volume_delta_pct(volume_a: float, volume_b: float) -> float:
    if not (np.isfinite(volume_a) and np.isfinite(volume_b)) or volume_b == 0:
        return float("nan")
    return abs(volume_a - volume_b) / abs(volume_b) * 100.0


def surface_deviation_mm(source: trimesh.Trimesh, result: trimesh.Trimesh, samples: int):
    """减面结果相对源网格的表面偏差（近似）。

    实现上把源网格顶点建成 KD 树，取减面后表面采样点的最近距离；因此是"到最近源
    顶点"的距离，会略微高估真实到表面的距离，仅用于量级比较。
    """
    from scipy.spatial import cKDTree

    points, _ = trimesh.sample.sample_surface(result, samples)
    distances, _ = cKDTree(source.vertices).query(points)
    return float(distances.max()) * 1000.0, float(distances.mean()) * 1000.0


def fmt_mm(value: float) -> str:
    return "-" if value is None or not np.isfinite(value) else f"{value:.3f}"


def fmt_pct(value: float) -> str:
    return "-" if value is None or not np.isfinite(value) else f"{value:+.3f}"


# ---------------------------------------------------------------------------
# 单文件处理
# ---------------------------------------------------------------------------
def process_one(path: Path, args, reference_dir: Path | None) -> dict:
    record = {
        "name": path.name,
        "src_faces": 0,
        "target": 0,
        "out_faces": 0,
        "bbox_mm": float("nan"),
        "vol_pct": float("nan"),
        "dev_max": float("nan"),
        "dev_mean": float("nan"),
        "ref_faces": None,
        "d_faces": None,
        "bbox_mm_ref": float("nan"),
        "vol_pct_ref": float("nan"),
        "info": [],   # 提示性信息，不计入超差汇总
        "notes": [],  # 超过阈值的问题，计入超差汇总
    }

    mesh = weld_vertices(load_mesh(path), args.merge_tolerance)
    src_stats = mesh_stats(mesh)
    record["src_faces"] = src_stats["faces"]

    target = max(4, int(round(src_stats["faces"] * args.ratio)))
    record["target"] = target

    result = run_simplify(mesh, target, args.agg)
    out_stats = mesh_stats(result)
    record["out_faces"] = out_stats["faces"]
    record["bbox_mm"] = bbox_delta_mm(out_stats["bounds"], src_stats["bounds"])
    record["vol_pct"] = volume_delta_pct(out_stats["volume"], src_stats["volume"])

    if args.deviation:
        record["dev_max"], record["dev_mean"] = surface_deviation_mm(
            mesh, result, args.deviation_samples
        )

    if args.output is not None:
        out_path = args.output / path.name
        if out_path.exists():
            record["info"].append("覆盖已有文件")
        result.export(str(out_path))

    # 与参考结果对照
    if reference_dir is not None:
        ref_path = reference_dir / path.name
        if not ref_path.exists():
            record["notes"].append("参考缺失")
        else:
            ref_stats = mesh_stats(load_mesh(ref_path))
            record["ref_faces"] = ref_stats["faces"]
            record["d_faces"] = out_stats["faces"] - ref_stats["faces"]
            record["bbox_mm_ref"] = bbox_delta_mm(out_stats["bounds"], ref_stats["bounds"])
            record["vol_pct_ref"] = volume_delta_pct(out_stats["volume"], ref_stats["volume"])
            if ref_stats["faces"] == src_stats["faces"]:
                record["info"].append("参考未减面")

    # 判定
    if abs(record["out_faces"] - target) > args.tol_faces:
        record["notes"].append("面数未达目标")
    if np.isfinite(record["bbox_mm"]) and record["bbox_mm"] > args.tol_bbox * 1000.0:
        record["notes"].append("包围盒超差")
    if not src_stats["watertight"]:
        record["info"].append("体积仅参考")
    elif np.isfinite(record["vol_pct"]) and record["vol_pct"] > args.tol_volume:
        record["notes"].append("体积超差")
    if record["d_faces"] is not None and abs(record["d_faces"]) > args.tol_faces:
        record["notes"].append("与参考面数不一致")

    return record


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def parse_args(argv=None):
    package_dir = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(
        description="按比例减面（QEM 边坍缩）并输出面数/包围盒/体积偏差报告",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-i", "--input", type=Path, default=package_dir / "meshes",
                        help="输入文件夹（递归不处理子目录，只处理其中的网格文件）")
    parser.add_argument("-o", "--output", type=Path, default=None,
                        help="输出文件夹；不给出时只打印报告，不写任何文件")
    parser.add_argument("--reference", type=Path, default=None,
                        help="参考结果文件夹（例如 Blender 减面产物），用于逐文件回归对照")
    parser.add_argument("--ratio", type=float, default=0.1,
                        help="目标面数比例，目标面数 = round(ratio × 源面数)")
    parser.add_argument("--merge-tolerance", type=float, default=1e-8,
                        help="焊接顶点容差（米）。默认 1e-8，只合并坐标相同的重复顶点；"
                             "实测放大到 1e-4 会让部分网格坍缩受阻、达不到目标面数")
    parser.add_argument("--agg", type=int, default=7,
                        help="fast_simplification 激进程度参数，越大减面越激进")
    parser.add_argument("--skip", nargs="*", default=[],
                        help="按文件名子串跳过，可给多个，例如 --skip waist_pitch_link.STL")
    parser.add_argument("--min-faces", type=int, default=0,
                        help="源面数小于该值的文件直接跳过")
    parser.add_argument("--deviation", action="store_true",
                        help="额外计算减面结果相对源网格的表面偏差（需要 scipy，较慢）")
    parser.add_argument("--deviation-samples", type=int, default=2000,
                        help="表面偏差采样的点数")
    parser.add_argument("--tol-faces", type=int, default=2,
                        help="与目标面数的允许偏差（面）")
    parser.add_argument("--tol-bbox", type=float, default=3e-3,
                        help="包围盒每轴允许偏差（米），默认 3 mm")
    parser.add_argument("--tol-volume", type=float, default=5.0,
                        help="体积相对偏差允许上限（%%）；源网格非水密时该值不参与判定")
    parser.add_argument("--strict", action="store_true",
                        help="出现 WARN 时以退出码 1 结束")
    return parser.parse_args(argv)


def collect_meshes(directory: Path, skip_tokens, min_faces: int):
    files = sorted(
        f for f in directory.iterdir()
        if f.is_file() and f.suffix.upper() == ".STL" and not f.name.startswith(".")
    )
    kept, skipped = [], []
    for path in files:
        if any(token and token in path.name for token in skip_tokens):
            skipped.append((path.name, "按 --skip 跳过"))
            continue
        if path.stat().st_size <= 84:
            skipped.append((path.name, "文件为空或损毁"))
            continue
        face_count = (path.stat().st_size - 84) // 50  # 二进制 STL：84 + 50×面数
        if face_count < min_faces:
            skipped.append((path.name, f"源面数 {face_count} < --min-faces {min_faces}"))
            continue
        kept.append(path)
    return kept, skipped


def main(argv=None) -> int:
    args = parse_args(argv)

    in_dir = args.input.resolve()
    if not in_dir.is_dir():
        raise SystemExit(f"输入文件夹不存在: {in_dir}")
    out_dir = args.output.resolve() if args.output is not None else None
    if out_dir is not None:
        if out_dir == in_dir:
            raise SystemExit("输入和输出文件夹不能相同，避免覆盖原始网格")
        out_dir.mkdir(parents=True, exist_ok=True)
    reference_dir = args.reference.resolve() if args.reference is not None else None
    if reference_dir is not None and not reference_dir.is_dir():
        raise SystemExit(f"参考文件夹不存在: {reference_dir}")

    meshes, skipped = collect_meshes(in_dir, args.skip, args.min_faces)

    print(f"输入目录: {in_dir}")
    print(f"输出目录: {out_dir if out_dir is not None else '（未指定，只读模式，不写文件）'}")
    if reference_dir is not None:
        print(f"参考目录: {reference_dir}")
    print(f"目标比例: {args.ratio}   焊接容差: {args.merge_tolerance} m   agg: {args.agg}")
    print(f"待处理文件: {len(meshes)} 个" + (f"，跳过 {len(skipped)} 个" if skipped else ""))
    print()

    if not meshes:
        print("没有可处理的网格文件。")
        return 0

    with_ref = reference_dir is not None
    with_dev = args.deviation

    # 表头
    headers = ["文件", "源面数", "目标面数", "减面后", "实际比例", "包围盒Δ(mm)"]
    aligns = ["left", "right", "right", "right", "right", "right"]
    if with_dev:
        headers += ["偏差max(mm)", "偏差mean(mm)"]
        aligns += ["right", "right"]
    headers += ["体积Δ(%)"]
    aligns += ["right"]
    if with_ref:
        headers += ["参考面数", "面数Δ", "参考bboxΔ(mm)"]
        aligns += ["right", "right", "right"]
    headers += ["备注"]
    aligns += ["left"]
    widths = [_disp_width(h) for h in headers]

    records = []
    for path in meshes:
        try:
            record = process_one(path, args, reference_dir)
        except Exception as exc:  # 单个文件失败不影响整批
            records.append({"name": path.name, "src_faces": 0, "target": 0, "out_faces": 0,
                            "bbox_mm": float("nan"), "vol_pct": float("nan"),
                            "dev_max": float("nan"), "dev_mean": float("nan"),
                            "ref_faces": None, "d_faces": None,
                            "bbox_mm_ref": float("nan"), "vol_pct_ref": float("nan"),
                            "info": [], "notes": [f"处理失败: {exc}"]})
            continue
        records.append(record)
        print(f"  已处理 {record['name']}: {record['src_faces']} → {record['out_faces']} 面")

    print()
    print(fmt_row(headers, widths, aligns))
    print("-" * (sum(widths) + 2 * (len(widths) - 1)))

    warn_names = []
    total_src = total_out = total_ref = 0
    for record in records:
        ratio_actual = (record["out_faces"] / record["src_faces"]
                        if record["src_faces"] else float("nan"))
        cells = [record["name"], f"{record['src_faces']:,}", f"{record['target']:,}",
                 f"{record['out_faces']:,}", f"{ratio_actual:.4f}",
                 fmt_mm(record["bbox_mm"])]
        if with_dev:
            cells += [fmt_mm(record["dev_max"]), fmt_mm(record["dev_mean"])]
        cells += [fmt_pct(record["vol_pct"])]
        if with_ref:
            ref_faces = record["ref_faces"]
            cells += ["-" if ref_faces is None else f"{ref_faces:,}",
                      "-" if record["d_faces"] is None else f"{record['d_faces']:+,}",
                      fmt_mm(record["bbox_mm_ref"])]
        remarks = record["info"] + record["notes"]
        cells += ["；".join(remarks) if remarks else "OK"]
        print(fmt_row(cells, widths, aligns))

        total_src += record["src_faces"]
        total_out += record["out_faces"]
        if record["ref_faces"] is not None:
            total_ref += record["ref_faces"]
        if record["notes"]:
            warn_names.append(record["name"])

    print("-" * (sum(widths) + 2 * (len(widths) - 1)))
    totals = ["合计", f"{total_src:,}", "", f"{total_out:,}",
              f"{total_out / total_src:.4f}" if total_src else "-", ""]
    if with_dev:
        totals += ["", ""]
    totals += [""]
    if with_ref:
        totals += [f"{total_ref:,}" if total_ref else "-", "", ""]
    totals += [f"超差 {len(warn_names)}/{len(records)}" if warn_names else "全部在阈值内"]
    print(fmt_row(totals, widths, aligns))

    if skipped:
        print()
        print("跳过的文件:")
        for name, reason in skipped:
            print(f"  {name}: {reason}")

    print()
    if warn_names:
        print(f"需要关注 {len(warn_names)} 个文件（阈值: 面数±{args.tol_faces}，"
              f"包围盒{args.tol_bbox * 1000:.3f} mm，体积{args.tol_volume}%）:")
        for name in warn_names:
            notes = next(r["notes"] for r in records if r["name"] == name)
            print(f"  {name}: {'；'.join(notes)}")
    else:
        print("全部文件均在阈值内。")

    print()
    print("提示: 本脚本是等效减面工具，几何结果不等同于 Blender Decimate 的产物；")
    print("      与参考目录对照时请以面数、包围盒、体积等指标为准，不要期望逐字节一致。")

    if args.strict and warn_names:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
