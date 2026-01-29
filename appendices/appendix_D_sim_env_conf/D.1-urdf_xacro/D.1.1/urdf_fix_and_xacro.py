#!/usr/bin/env python3
import os
import re
import xml.etree.ElementTree as ET

PKG = "swarm_bringup"

def find_parent_map(root):
    return {c: p for p in root.iter() for c in p}

def mesh_exists(mesh_path: str) -> bool:
    # mesh_path like: package://swarm_bringup/meshes/name.STL
    if not mesh_path.startswith(f"package://{PKG}/"):
        return False
    rel = mesh_path.replace(f"package://{PKG}/", "")
    fs = os.path.join(os.path.dirname(__file__), "..", rel)
    fs = os.path.abspath(fs)
    return os.path.exists(fs)

def swap_visual_collision_meshes(root):
    parent_map = find_parent_map(root)

    for mesh in root.iter("mesh"):
        fn = mesh.get("filename", "")
        if not fn.startswith(f"package://{PKG}/meshes/"):
            continue

        # detect if mesh is under <visual> or <collision>
        p = parent_map.get(mesh)
        while p is not None and p.tag not in ("visual", "collision"):
            p = parent_map.get(p)

        if p is None:
            continue

        base = fn
        # normalize extension case
        m = re.match(r"^(.*)\.(stl|STL)$", fn)
        if not m:
            continue
        stem, ext = m.group(1), m.group(2)

        coll = f"{stem}_coll.{ext}" if not stem.endswith("_coll") else fn
        noncoll = f"{stem.replace('_coll','')}.{ext}"

        if p.tag == "visual":
            # prefer non-coll if exists
            if stem.endswith("_coll"):
                if mesh_exists(noncoll):
                    mesh.set("filename", noncoll)
        elif p.tag == "collision":
            # prefer _coll if exists
            if not stem.endswith("_coll"):
                if mesh_exists(coll):
                    mesh.set("filename", coll)

def rename_joint_name_conflicts(root):
    link_names = {l.get("name") for l in root.findall("link") if l.get("name")}
    for j in root.findall("joint"):
        jn = j.get("name")
        if not jn:
            continue
        if jn in link_names:
            j.set("name", jn + "_joint")

def promote_chassis_to_base(root):
    # if we have empty base_link and a chassis_link fixed to it, move chassis content to base_link
    base = root.find("link[@name='base_link']")
    chassis = root.find("link[@name='chassis_link']")
    j = root.find("joint[@name='chassis_joint']")

    if base is None or chassis is None or j is None:
        return

    # Copy inertial/visual/collision from chassis -> base
    for tag in ("inertial", "visual", "collision"):
        for child in list(chassis.findall(tag)):
            # remove existing base tag(s) if any, then append
            for old in list(base.findall(tag)):
                base.remove(old)
            base.append(child)

    # Update parents that reference chassis_link -> base_link
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        if parent is not None and parent.get("link") == "chassis_link":
            parent.set("link", "base_link")

    # Remove chassis_joint and chassis_link
    root.remove(j)
    root.remove(chassis)

def set_robot_name(root, name="swarm_robot"):
    root.set("name", name)

def write_xml(root, out_path):
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ", level=0)
    tree.write(out_path, encoding="utf-8", xml_declaration=True)

def make_xacro_from_urdf(fixed_urdf_path, out_xacro_path):
    # Read text and inject xacro header and prefixing
    with open(fixed_urdf_path, "r", encoding="utf-8") as f:
        txt = f.read()

    # ensure xmlns:xacro exists
    txt = re.sub(r"<robot([^>]*?)>", r"<robot\1 xmlns:xacro=\"http://www.ros.org/wiki/xacro\">", txt, count=1)

    # inject arg/property after <robot ...>
    txt = re.sub(
        r"(<robot[^>]*?>)",
        r"\1\n  <xacro:arg name=\"prefix\" default=\"\"/>\n  <xacro:property name=\"prefix\" value=\"$(arg prefix)\"/>\n",
        txt,
        count=1
    )

    # Prefix all link/joint names and parent/child link refs
    # link name="X" -> link name="${prefix}X"
    txt = re.sub(r'(<link\s+name=")([^"]+)(")', r'\1${prefix}\2\3', txt)
    # joint name="X" -> joint name="${prefix}X"
    txt = re.sub(r'(<joint\s+name=")([^"]+)(")', r'\1${prefix}\2\3', txt)
    # parent link="X" -> parent link="${prefix}X"
    txt = re.sub(r'(<parent\s+link=")([^"]+)(")', r'\1${prefix}\2\3', txt)
    # child link="X" -> child link="${prefix}X"
    txt = re.sub(r'(<child\s+link=")([^"]+)(")', r'\1${prefix}\2\3', txt)

    with open(out_xacro_path, "w", encoding="utf-8") as f:
        f.write(txt)

def main():
    pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    in_urdf = os.path.join(pkg_dir, "urdf", "robot_raw.urdf")
    out_fixed = os.path.join(pkg_dir, "urdf", "robot_fixed.urdf")
    out_xacro = os.path.join(pkg_dir, "urdf", "robot_fixed.urdf.xacro")

    tree = ET.parse(in_urdf)
    root = tree.getroot()

    set_robot_name(root, "swarm_robot")
    promote_chassis_to_base(root)
    swap_visual_collision_meshes(root)
    rename_joint_name_conflicts(root)

    write_xml(root, out_fixed)
    make_xacro_from_urdf(out_fixed, out_xacro)

    print("✅ Generated:")
    print(" -", out_fixed)
    print(" -", out_xacro)

if __name__ == "__main__":
    main()
