"""检查或更新天工行者Dex URDF 的 link 惯性参数。

用法：
  python3 check_link_and_update.py --check   # 只比较并报告，默认模式
  python3 check_link_and_update.py --update  # 从关键参数 CSV 回写 URDF

不带参数时等同于 --check；只有显式传入 --update 才会修改 URDF。
"""

import argparse
import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path

def parse_urdf_links(urdf_path):
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        links = []
        for link in root.findall('link'):
            name = link.get('name')
            if name:
                links.append(name)
        return links
    except Exception as e:
        print(f"Error parsing URDF file: {e}")
        return []



def find_link_name_in_csv(csv_path):
    print(f"读取 {csv_path} 中的 link 命名...")
    link_names = []
    
    with open(csv_path, 'r', encoding='utf-8') as csvfile:
        reader = csv.reader(csvfile)
        
        # 跳过第一行（标题行）
        header = next(reader)
        
        # 读取link部分的数据，直到遇到"joint部分"或空行为止
        for row in reader:
            # 如果遇到"joint部分"行，停止读取
            if len(row) > 0 and row[0] == 'joint部分':
                break
            
            # 跳过空行
            if not row or len(row) < 2:
                continue
            
            # 提取link命名（第二列，索引为1）
            link_name = row[1].strip()
            
            # 只添加非空的link名称
            if link_name and link_name != '':
                link_names.append(link_name)
    
    # print(f"\n关键参数表中找到 {len(link_names)} 个 link:")
    # for name in link_names:
    #     print(f"  - {name}")
    return link_names

def read_link_properties_in_csv(csv_path, link_name):
    # print(f"读取 {csv_path} 中的 link 属性...")
    link_row_num = None
    with open(csv_path, 'r', encoding='utf-8') as csvfile:
        reader = csv.reader(csvfile)
        for idx, row in enumerate(reader):
            if len(row) > 1 and row[1].strip() == link_name:
                link_row_num = idx + 1  # 行号从1开始（加1用于更直观展示）
                # print(f"link_name '{link_name}' 出现在第 {link_row_num} 行: {row}")
                break

    if link_row_num is None:
        raise ValueError(f"CSV 中未找到 link: {link_name}")

    col_names = [
        "mass", "mass_center_x", "mass_center_y", "mass_center_z",
        "inertia_xx", "inertia_xy", "inertia_xz",
        "inertia_yy", "inertia_yz", "inertia_zz"
    ]
    props = {}
    for idx, name in enumerate(col_names, start=2):
        val = row[idx].strip() if len(row) > idx and row[idx] is not None else ''
        if not val:
            raise ValueError(f"link '{link_name}' 的参数 '{name}' 为空")
        props[name] = float(val)
    mass = props["mass"]
    mass_center_x = props["mass_center_x"]
    mass_center_y = props["mass_center_y"]
    mass_center_z = props["mass_center_z"]
    inertia_xx = props["inertia_xx"]
    inertia_xy = props["inertia_xy"]
    inertia_xz = props["inertia_xz"]
    inertia_yy = props["inertia_yy"]
    inertia_yz = props["inertia_yz"]
    inertia_zz = props["inertia_zz"]
    
    link_properties = {
        "mass": mass,
        "mass_center_x": mass_center_x,
        "mass_center_y": mass_center_y,
        "mass_center_z": mass_center_z,
        "inertia_xx": inertia_xx,
        "inertia_xy": inertia_xy,
        "inertia_xz": inertia_xz,
        "inertia_yy": inertia_yy,
        "inertia_yz": inertia_yz,
        "inertia_zz": inertia_zz,
    }
    return link_properties


def check_urdf_links(urdf_file, expected_links, verbose=True):
    """
    检查URDF文件中是否包含所有预期的link名。
    :param urdf_file: URDF文件路径
    :param verbose: 是否打印详细信息
    :return: missing_links列表，如果没有缺失则为空
    """
    links = parse_urdf_links(urdf_file)
    missing_links = [name for name in expected_links if name not in links]
    if verbose:
        if missing_links:
            print("Missing links in URDF:")
            for name in missing_links:
                print(f"  {name}")
        else:
            print("All expected links are present in the URDF.")
    return missing_links

def update_urdf_link_properties(urdf_file, link_name, link_properties):
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    for link in root.findall('link'):
        if link.get('name') == link_name:
            inertial = link.find('inertial')
            if inertial is None:
                continue
            mass_el = inertial.find('mass')
            if mass_el is None:
                # 如果没有mass元素，则创建一个并设置value属性
                mass_el = ET.SubElement(inertial, 'mass')
            # 设置为属性，而不是文本内容
            mass_el.set('value', str(link_properties['mass']))
            # 清空文本，避免写到元素体内
            mass_el.text = None
            # 处理mass center(center of mass)
            origin_el = inertial.find('origin')
            if origin_el is None:
                origin_el = ET.SubElement(inertial, 'origin')

            # 设置质心坐标
            mass_center_x = link_properties.get('mass_center_x', 0)
            mass_center_y = link_properties.get('mass_center_y', 0)
            mass_center_z = link_properties.get('mass_center_z', 0)
            origin_el.set('xyz', f"{mass_center_x} {mass_center_y} {mass_center_z}")
            # 如果原有rpy属性没被指定则保持不变，不覆盖
            if 'rpy' not in origin_el.attrib:
                origin_el.set('rpy', "0 0 0")

            # 处理惯性相关属性
            inertia_el = inertial.find('inertia')
            if inertia_el is None:
                inertia_el = ET.SubElement(inertial, 'inertia')
            inertia_el.set('ixx', str(link_properties['inertia_xx']))
            inertia_el.set('ixy', str(link_properties['inertia_xy']))
            inertia_el.set('ixz', str(link_properties['inertia_xz']))
            inertia_el.set('iyy', str(link_properties['inertia_yy']))
            inertia_el.set('iyz', str(link_properties['inertia_yz']))
            inertia_el.set('izz', str(link_properties['inertia_zz']))
    tree.write(urdf_file, encoding='utf-8', xml_declaration=True)


def compare_urdf_link_properties(urdf_file, link_name, link_properties):
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    for link in root.findall('link'):
        if link.get('name') != link_name:
            continue

        inertial = link.find('inertial')
        if inertial is None:
            return ["URDF 缺少 inertial"]

        differences = []
        expected_values = {
            'mass': link_properties['mass'],
            'mass_center_x': link_properties['mass_center_x'],
            'mass_center_y': link_properties['mass_center_y'],
            'mass_center_z': link_properties['mass_center_z'],
            'inertia_xx': link_properties['inertia_xx'],
            'inertia_xy': link_properties['inertia_xy'],
            'inertia_xz': link_properties['inertia_xz'],
            'inertia_yy': link_properties['inertia_yy'],
            'inertia_yz': link_properties['inertia_yz'],
            'inertia_zz': link_properties['inertia_zz'],
        }
        mass_el = inertial.find('mass')
        origin_el = inertial.find('origin')
        inertia_el = inertial.find('inertia')
        actual_values = {
            'mass': None if mass_el is None else mass_el.get('value'),
            'mass_center_x': None,
            'mass_center_y': None,
            'mass_center_z': None,
            'inertia_xx': None if inertia_el is None else inertia_el.get('ixx'),
            'inertia_xy': None if inertia_el is None else inertia_el.get('ixy'),
            'inertia_xz': None if inertia_el is None else inertia_el.get('ixz'),
            'inertia_yy': None if inertia_el is None else inertia_el.get('iyy'),
            'inertia_yz': None if inertia_el is None else inertia_el.get('iyz'),
            'inertia_zz': None if inertia_el is None else inertia_el.get('izz'),
        }
        if origin_el is not None and origin_el.get('xyz'):
            center = origin_el.get('xyz').split()
            if len(center) == 3:
                actual_values['mass_center_x'] = center[0]
                actual_values['mass_center_y'] = center[1]
                actual_values['mass_center_z'] = center[2]

        for name, expected in expected_values.items():
            actual_text = actual_values[name]
            if actual_text is None or not math.isclose(
                float(actual_text), expected, rel_tol=1e-9, abs_tol=1e-12
            ):
                differences.append(f"{name}: URDF={actual_text}, CSV={expected}")
        return differences

    return ["URDF 中未找到该 link"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='只比较并报告，不修改 URDF（默认）')
    mode.add_argument('--update', action='store_true', help='从 CSV 回写 URDF')
    args = parser.parse_args()

    package_dir = Path(__file__).resolve().parent.parent
    urdf_file = package_dir / "urdf" / "tiangong3.urdf"
    csv_file = package_dir / "关键参数" / "URDF关键参数表 - 天工行者Dex(V3).csv"
    names = find_link_name_in_csv(csv_file)
    missing_links = check_urdf_links(urdf_file, names)
    if missing_links:
        print("以下 link 在 URDF 中未找到：")
        for link in missing_links:
            print(f"  - {link}")
    if args.update:
        for link_name in names:
            if link_name in missing_links:
                continue
            link_properties = read_link_properties_in_csv(csv_file, link_name)
            update_urdf_link_properties(urdf_file, link_name, link_properties)
    else:
        changed_links = []
        for link_name in names:
            if link_name in missing_links:
                continue
            link_properties = read_link_properties_in_csv(csv_file, link_name)
            differences = compare_urdf_link_properties(
                urdf_file, link_name, link_properties
            )
            if differences:
                changed_links.append(link_name)
                print(f"link 参数不一致: {link_name}")
                for difference in differences:
                    print(f"  - {difference}")
        if not missing_links and not changed_links:
            print("检查完成：所有 link 名称和惯性参数均一致，未修改 URDF。")
