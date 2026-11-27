import argparse
import math
import re
import stat
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from urdf2mjcf.convert import convert_urdf_to_mjcf
from urdf2mjcf.model import ActuatorMetadata, JointMetadata

FOOT_CAPSULE_COMMENT = "<!-- Foot collision cylinders encode capsule straight sections. -->"

def extract_joint_limits(urdf_path):
    """从 URDF 文件中提取关节名称和对应的限位信息（effort, lower, upper）"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    joint_info = {}
    for joint in root.findall('.//joint'):
        joint_name = joint.get('name')
        if joint_name is None:
            continue
        
        limit = joint.find('limit')
        if limit is not None:
            info = {}
            if limit.get('effort') is not None:
                info['effort'] = float(limit.get('effort'))
            if limit.get('lower') is not None:
                info['lower'] = float(limit.get('lower'))  # 弧度
            if limit.get('upper') is not None:
                info['upper'] = float(limit.get('upper'))  # 弧度
            if info:
                joint_info[joint_name] = info
    
    return joint_info

def create_metadata(urdf_path):
    """创建关节和执行器的元数据"""
    # 提取关节限位信息（effort, lower, upper）
    joint_info = extract_joint_limits(urdf_path)
    print(f"Extracted limit information for {len(joint_info)} joints from URDF")
    
    # 解析 URDF 获取所有关节
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    # 创建关节元数据
    joint_metadata = {}
    actuator_metadata = {}
    actuator_type_to_joints = {}  # 用于跟踪每个 actuator_type 对应的关节
    
    joint_idx = 0
    for joint in root.findall('.//joint'):
        joint_name = joint.get('name')
        if joint_name is None:
            continue
        
        # 跳过 fixed 关节
        if joint.get('type') == 'fixed':
            continue
        
        info = joint_info.get(joint_name, {})
        effort = info.get('effort')
        lower_rad = info.get('lower')
        upper_rad = info.get('upper')
        
        # 为每个不同的 effort 值创建唯一的 actuator_type
        if effort is not None:
            # 创建 actuator_type 名称（基于 effort 值，使用整数部分避免浮点精度问题）
            # 例如：332.0 -> motor_332, 6.3 -> motor_6_3
            if effort == int(effort):
                actuator_type = f"motor_{int(effort)}"
            else:
                actuator_type = f"motor_{effort}".replace('.', '_')
            
            # 根据 effort 值设置阻尼和摩擦力
            # damping: 速度阻尼系数 (N·m·s/rad)，与速度成正比
            # frictionloss: 库伦摩擦损失 (N·m)，恒定摩擦力
            if effort >= 200:  # 大功率关节
                damping = 2.0  # 较大的速度阻尼
                frictionloss = 1.0  # 较大的库伦摩擦
            elif effort >= 50:  # 中等功率关节
                damping = 1.0
                frictionloss = 0.5
            else:  # 小功率关节
                damping = 0.5
                frictionloss = 0.1
            
            # 如果这个 actuator_type 还不存在，创建它
            if actuator_type not in actuator_metadata:
                actuator_metadata[actuator_type] = ActuatorMetadata(
                    actuator_type=actuator_type,
                    max_torque=effort,
                    damping=damping,
                    frictionloss=frictionloss
                )
                print(f"Created actuator type '{actuator_type}' with max_torque={effort} N·m, damping={damping}, frictionloss={frictionloss}")
        else:
            # 如果没有 effort 值，使用默认的 motor
            actuator_type = "motor"
            if actuator_type not in actuator_metadata:
                actuator_metadata[actuator_type] = ActuatorMetadata(
                    actuator_type=actuator_type,
                    damping=0.5,  # 默认阻尼
                    frictionloss=0.1  # 默认摩擦力
                )
                print(f"Created default actuator type '{actuator_type}' with damping=0.5, frictionloss=0.1")
        
        # 将弧度转换为度（用于 JointMetadata）
        min_angle_deg = None
        max_angle_deg = None
        if lower_rad is not None:
            min_angle_deg = math.degrees(lower_rad)
        if upper_rad is not None:
            max_angle_deg = math.degrees(upper_rad)
        
        # 创建关节元数据
        joint_metadata[joint_name] = JointMetadata(
            actuator_type=actuator_type,
            id=joint_idx,
            nn_id=joint_idx,
            kp=1.0,
            kd=1.0,
            soft_torque_limit=effort if effort is not None else 1.0,
            min_angle_deg=min_angle_deg,
            max_angle_deg=max_angle_deg,
        )
        
        limit_str = ""
        if lower_rad is not None and upper_rad is not None:
            limit_str = f" range=[{lower_rad:.3f}, {upper_rad:.3f}] rad"
        elif lower_rad is not None:
            limit_str = f" lower={lower_rad:.3f} rad"
        elif upper_rad is not None:
            limit_str = f" upper={upper_rad:.3f} rad"
        if limit_str:
            print(f"  Joint {joint_name}:{limit_str}")
        
        # 跟踪每个 actuator_type 对应的关节
        if actuator_type not in actuator_type_to_joints:
            actuator_type_to_joints[actuator_type] = []
        actuator_type_to_joints[actuator_type].append(joint_name)
        
        joint_idx += 1
    
    # 打印总结
    print(f"\nActuator type summary:")
    for actuator_type, joints in actuator_type_to_joints.items():
        effort_val = actuator_metadata[actuator_type].max_torque
        damping_val = actuator_metadata[actuator_type].damping
        frictionloss_val = actuator_metadata[actuator_type].frictionloss
        effort_str = f" (max_torque={effort_val} N·m)" if effort_val is not None else ""
        damping_str = f", damping={damping_val}" if damping_val is not None else ""
        frictionloss_str = f", frictionloss={frictionloss_val}" if frictionloss_val is not None else ""
        print(f"  {actuator_type}{effort_str}{damping_str}{frictionloss_str}: {len(joints)} joints")
    
    return joint_metadata, actuator_metadata

def restore_urdf_inertials(mjcf_content, urdf_path):
    """按同名 link 恢复转换器丢失的根部惯性和非对角项。"""
    urdf = ET.parse(urdf_path).getroot()
    mjcf = ET.fromstring(mjcf_content, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    compiler = mjcf.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        compiler.tail = mjcf.text
        mjcf.insert(0, compiler)
    compiler.set("inertiafromgeom", "false")

    body_elements = mjcf.findall("worldbody//body")
    bodies = {body.attrib["name"]: body for body in body_elements}
    if len(bodies) != len(body_elements):
        raise ValueError("MJCF contains duplicate body names")
    expected = {}

    for link in urdf.findall("link"):
        name = link.attrib["name"]
        if name in expected:
            raise ValueError(f"Duplicate URDF link {name!r}")
        inertials = link.findall("inertial")
        if len(inertials) != 1:
            raise ValueError(f"URDF link {name!r} must have exactly one <inertial>, found {len(inertials)}")
        inertial = inertials[0]
        body = bodies.get(name)
        if body is None:
            raise ValueError(f"Missing MJCF body for URDF link {name!r}")

        origin = inertial.find("origin")
        pos = "0 0 0" if origin is None else origin.get("xyz", "0 0 0")
        rpy = "0 0 0" if origin is None else origin.get("rpy", "0 0 0")
        mass_element = inertial.find("mass")
        inertia = inertial.find("inertia")
        if mass_element is None or inertia is None:
            raise ValueError(f"URDF link {name!r} must define <mass> and <inertia>")
        try:
            mass = mass_element.attrib["value"]
            mass_value = float(mass)
            com = [float(value) for value in pos.split()]
            angles = [float(value) for value in rpy.split()]
            ixx, iyy, izz, ixy, ixz, iyz = (
                float(inertia.attrib[key]) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
            )
        except (KeyError, ValueError) as exc:
            raise ValueError(f"URDF link {name!r} has incomplete or invalid <inertial> data") from exc
        tensor = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        if mass_value < 0 or len(com) != 3 or len(angles) != 3 or not np.isfinite(
            [mass_value, *com, *angles, *tensor.flat]
        ).all():
            raise ValueError(f"URDF link {name!r} has invalid <inertial> values")
        orientation = Rotation.from_euler("xyz", angles)
        rotation = orientation.as_matrix()
        body_tensor = rotation @ tensor @ rotation.T

        attrs = {"pos": pos}
        if ixy or ixz or iyz:
            # fullinertia 以 body 坐标系表示，MuJoCo 编译时会求主轴。
            indices = ([0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2])
            attrs.update(mass=mass, fullinertia=" ".join(map(str, body_tensor[indices])))
        else:
            attrs.update(
                quat=" ".join(map(str, orientation.as_quat(scalar_first=True))),
                mass=mass,
                diaginertia=f"{ixx} {iyy} {izz}",
            )

        target = body.find("inertial")
        if target is None:
            target = ET.Element("inertial")
            joints = body.findall("joint") + body.findall("freejoint")
            index = len(joints)
            target.tail = body[index - 1].tail if index else body.text
            body.insert(index, target)
        target.attrib.clear()
        target.attrib.update(attrs)
        expected[name] = (mass_value, com, body_tensor)

    unexpected = bodies.keys() - expected.keys()
    if unexpected:
        raise ValueError(f"MJCF bodies have no matching URDF link: {', '.join(sorted(unexpected))}")
    return ET.tostring(mjcf, encoding="unicode"), expected


def validate_compiled_inertials(mujoco, model, expected):
    # ID 0 is the MuJoCo world body.
    compiled_names = {model.body(body_id).name for body_id in range(1, model.nbody)}
    missing = expected.keys() - compiled_names
    extra = compiled_names - expected.keys()
    if missing or extra or model.nbody - 1 != len(expected):
        raise ValueError(
            f"Compiled body coverage differs from URDF (missing: {sorted(missing)}, extra: {sorted(extra)})"
        )

    for name, (mass, com, tensor) in expected.items():
        body = model.body(name)
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, body.iquat)
        rotation = rotation.reshape(3, 3)
        actual_tensor = (rotation * body.inertia) @ rotation.T
        # 主轴分解的数值误差随惯量大小变化，绝对容差按整个张量的尺度确定。
        inertia_tolerance = 1e-12 + 1e-6 * np.abs(tensor).max()
        for field, actual, source, tolerance in (
            ("mass", body.mass.item(), mass, 1e-12),
            ("center of mass", body.ipos, com, 1e-12),
            ("inertia tensor", actual_tensor, tensor, inertia_tolerance),
        ):
            if not np.allclose(actual, source, rtol=0, atol=tolerance):
                raise ValueError(f"{name}: compiled {field} {actual} differs from URDF {source}")

    expected_mass = math.fsum(mass for mass, _, _ in expected.values())
    compiled_mass = mujoco.mj_getTotalmass(model)
    if not math.isclose(compiled_mass, expected_mass, rel_tol=1e-12, abs_tol=1e-10):
        raise ValueError(f"Compiled total mass {compiled_mass:.12g} kg differs from URDF {expected_mass:.12g} kg")
    print(f"Validated compiled mass, center of mass and inertia for all {len(expected)} bodies ({compiled_mass:.6f} kg)")


def compute_floor_z(mujoco, model):
    """按默认位姿算出所有网格 geom 最低点的 z，用于自动定位地面高度。

    旧版 tiangong3_torq.xml 里的 -0.056774 是人工量出来的，脚本重新生成就会丢失；
    这里改成生成后自动计算，换一套网格（例如 meshes_simplify）也能得到贴合的高度。
    没有网格时返回 None，调用方保持地面 z=0。
    """
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    lowest = None
    for geom_id in range(model.ngeom):
        if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh_id = model.geom_dataid[geom_id]
        start = model.mesh_vertadr[mesh_id]
        stop = start + model.mesh_vertnum[mesh_id]
        vertices = model.mesh_vert[start:stop]
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        # 世界坐标 z = 旋转矩阵第三行 · v + 平移 z
        geom_lowest = float((vertices @ rotation[2] + data.geom_xpos[geom_id][2]).min())
        lowest = geom_lowest if lowest is None else min(lowest, geom_lowest)
    return lowest


def generate_mjcf(urdf_path, mjcf_file, mesh_subdir, floor_z, *, skip_mujoco_validation=False):
    # 创建元数据
    joint_metadata, actuator_metadata = create_metadata(urdf_path)
    
    print(f"\nConverting URDF to MJCF with {len(joint_metadata)} joints and {len(actuator_metadata)} actuator types")
    
    convert_urdf_to_mjcf(
        urdf_path=urdf_path,
        mjcf_path=mjcf_file,
        joint_metadata=joint_metadata,
        actuator_metadata=actuator_metadata
    )
    
    with open(mjcf_file, "r", encoding="utf-8") as f:
        mjcf_content = f.read()

    mjcf_content, expected_inertials = restore_urdf_inertials(mjcf_content, urdf_path)

    # URDF 1.0 uses cylinders for the capsule cores. The radius and half-length
    # produced by urdf2mjcf already have the same meaning in an MJCF capsule.
    if FOOT_CAPSULE_COMMENT in urdf_path.read_text(encoding="utf-8"):
        mjcf_content, count = re.subn(
            r'(<geom name="ankle_roll_[lr]_link_collision_[0-6]"[^>]*\btype=")cylinder(")',
            r'\1capsule\2',
            mjcf_content,
        )
        if count != 14:
            raise ValueError(f"Expected 14 Dex foot cylinders to convert to capsules, found {count}")
    
    # 替换 package:// 路径，并把网格目录换成本次要用的那一套。
    # 默认 meshes_convex：prime_to_convex.py 生成的凸包。
    mjcf_content = re.sub(r'package://[^/]+/', './', mjcf_content)
    mjcf_content = mjcf_content.replace('./meshes/', f'./{mesh_subdir}/')

    # URDF 中的空 material 名会被转换成 MuJoCo 不接受的空名称；统一补成有效名称。
    mjcf_content = re.sub(
        r'(<material\s+)name=""',
        r'\1name="urdf_visual_material"',
        mjcf_content,
    )
    mjcf_content = mjcf_content.replace(
        'material=""', 'material="urdf_visual_material"'
    )
    mjcf_content = mjcf_content.replace(
        'material="visualgeom"', 'material="urdf_visual_material"'
    )
    
    # 添加地面到 worldbody
    ground_geom = '    <!-- 地面 -->\n    <geom name="floor" type="plane" size="10 10 0.1" pos="0 0 0" quat="1 0 0 0" rgba="0.8 0.8 0.8 1" friction="1 0.005 0.0001" condim="3" />\n    \n'
    
    # 在 <worldbody> 标签后添加地面
    if '<worldbody>' in mjcf_content:
        mjcf_content = re.sub(
            r'(<worldbody>\s*\n)',
            r'\1' + ground_geom,
            mjcf_content,
            count=1
        )
        print("Ground added to MJCF file")
    else:
        print("Warning: <worldbody> tag not found, ground not added")
    
    
    # 将 <material name="collision_material" ... /> 的颜色改为灰色
    def replace_collision_color(match):
        return match.group(1) + '0.5 0.5 0.5 0.9' + match.group(2)
    
    mjcf_content = re.sub(
        r'(<material\s+name="collision_material"\s+rgba=")[^"]*(")',
        replace_collision_color,
        mjcf_content
    )
    with open(mjcf_file, "w", encoding="utf-8") as f:
        f.write(mjcf_content)

    # 用暂存文件编译，校验实际生效的惯性参数，并复用模型计算默认地面高度。
    if skip_mujoco_validation:
        print("Warning: 已指定 --skip-mujoco-validation，跳过 MuJoCo 编译校验")
    else:
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError("MuJoCo 不可用，无法编译校验。离线导出请显式指定 --skip-mujoco-validation") from exc
        model = mujoco.MjModel.from_xml_path(str(mjcf_file))
        validate_compiled_inertials(mujoco, model, expected_inertials)
        if floor_z is None:
            floor_z = compute_floor_z(mujoco, model)

    if floor_z is None:
        print("Warning: 地面高度未能确定，保持在 z=0")
    else:
        mjcf_content = mjcf_content.replace(
            '<!-- 地面 -->',
            f'<!-- 地面 z={floor_z:.6f} (make_mjcf_torq.py，默认取零位网格最低点) -->',
            1,
        )
        mjcf_content, replaced = re.subn(
            r'(<geom name="floor"[^>]*?pos=")0 0 0(")',
            lambda match: match.group(1) + f"0 0 {floor_z:.6f}" + match.group(2),
            mjcf_content,
        )
        if replaced:
            with open(mjcf_file, "w", encoding="utf-8") as f:
                f.write(mjcf_content)
            print(f"Floor height set to z={floor_z:.6f} (auto-computed)")
        else:
            print("Warning: 未找到 floor 的 pos 属性，地面保持在 z=0")


def main(urdf_path, mjcf_file, mesh_subdir="meshes_convex", floor_z=None, *, skip_mujoco_validation=False):
    if floor_z is not None and not math.isfinite(floor_z):
        raise ValueError("--floor-z must be finite")
    mjcf_file.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=mjcf_file.parent, suffix=".xml", delete=False) as temp:
        staged_file = Path(temp.name)
    try:
        generate_mjcf(urdf_path, staged_file, mesh_subdir, floor_z, skip_mujoco_validation=skip_mujoco_validation)
        if mjcf_file.exists():
            staged_file.chmod(stat.S_IMODE(mjcf_file.stat().st_mode))
        staged_file.replace(mjcf_file)
    finally:
        staged_file.unlink(missing_ok=True)
    print(f"MJCF file created at {mjcf_file}")


if __name__ == "__main__":
    package_dir = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(
        description="把 urdf/tiangong3.urdf 转成 MJCF，网格目录可选",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--urdf", type=Path,
                        default=package_dir / "urdf" / "tiangong3.urdf",
                        help="输入 URDF")
    parser.add_argument("--mjcf", type=Path,
                        default=package_dir / "mujoco" / "tiangong3_torq.xml",
                        help="输出 MJCF")
    parser.add_argument("--mesh-subdir", default="meshes_convex",
                        help="MJCF 引用的网格子目录名（相对 MJCF 所在目录）")
    parser.add_argument("--floor-z", type=float, default=None,
                        help="地面高度；默认自动计算（默认位姿下全部网格 geom 的最低点）")
    parser.add_argument("--skip-mujoco-validation", action="store_true",
                        help="离线导出 XML，跳过 MuJoCo 编译校验和自动地面计算")
    args = parser.parse_args()

    try:
        main(args.urdf, args.mjcf, args.mesh_subdir, args.floor_z,
             skip_mujoco_validation=args.skip_mujoco_validation)
    except (OSError, RuntimeError, ValueError) as exc:
        parser.exit(1, f"Error: {exc}\n")
