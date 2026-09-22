from urdf2mjcf.convert import convert_urdf_to_mjcf
from urdf2mjcf.model import ActuatorMetadata, JointMetadata
from pathlib import Path
import argparse
import re
import math
import xml.etree.ElementTree as ET

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

def compute_floor_z(mjcf_file):
    """按默认位姿算出所有网格 geom 最低点的 z，用于自动定位地面高度。

    旧版 tiangong3_torq.xml 里的 -0.056774 是人工量出来的，脚本重新生成就会丢失；
    这里改成生成后自动计算，换一套网格（例如 meshes_simplify）也能得到贴合的高度。
    需要 mujoco 才能计算，不可用时返回 None，调用方保持地面 z=0。
    """
    try:
        import mujoco
    except ImportError:
        print("Warning: mujoco 不可用，无法自动计算地面高度")
        return None

    try:
        model = mujoco.MjModel.from_xml_path(str(mjcf_file))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
    except Exception as exc:
        print(f"Warning: 加载 MJCF 计算地面高度失败: {exc}")
        return None

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


def main(urdf_path, mjcf_file, mesh_subdir="meshes_convex", floor_z=None):
    mjcf_file.parent.mkdir(parents=True, exist_ok=True)

    # 创建元数据
    joint_metadata, actuator_metadata = create_metadata(urdf_path)
    
    print(f"\nConverting URDF to MJCF with {len(joint_metadata)} joints and {len(actuator_metadata)} actuator types")
    
    convert_urdf_to_mjcf(
        urdf_path=urdf_path,
        mjcf_path=mjcf_file,
        joint_metadata=joint_metadata,
        actuator_metadata=actuator_metadata
    )
    
    print(f"MJCF file created at {mjcf_file}")

    with open(mjcf_file, "r", encoding="utf-8") as f:
        mjcf_content = f.read()
    
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

    # 地面高度：按默认位姿下所有网格 geom 的最低点自动定位，使默认位姿刚好踩在地面上。
    # 这一步要在文件写盘之后做，因为要先用 mujoco 编译模型才能算出最低点。
    if floor_z is None:
        floor_z = compute_floor_z(mjcf_file)
    if floor_z is None:
        print("Warning: 地面高度未能确定，保持在 z=0")
    else:
        mjcf_content = mjcf_content.replace(
            '<!-- 地面 -->',
            (f'<!-- 地面: 默认位姿下全部网格 geom 的最低点为 z={floor_z:.6f}, '
             f'故平面下移到该高度, 使默认位姿刚好踩在地面上 '
             f'(make_mjcf_torq.py 自动计算) -->'),
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
    args = parser.parse_args()
    

    main(args.urdf, args.mjcf, args.mesh_subdir, args.floor_z)
