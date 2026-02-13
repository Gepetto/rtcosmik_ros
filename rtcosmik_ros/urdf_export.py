"""URDF export utilities for scaled Pinocchio human models."""

from collections import defaultdict
from pathlib import Path
import os
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin


def _mesh_to_package_uri(mesh_path: str) -> str:
    """Rewrite mesh paths to package-relative URI expected by RViz/robot_state_publisher."""
    name = Path(str(mesh_path)).name
    return f"package://rtcosmik_ros/meshes/{name}"


def _joint_urdf_type(joint):
    """Map Pinocchio joint model shortname to URDF joint type."""
    short = joint.shortname() if hasattr(joint, "shortname") else "Unknown"
    if "JointModel" in short:
        short = short.split("JointModel", 1)[1]

    mapping = {
        "RX": "revolute",
        "RY": "revolute",
        "RZ": "revolute",
        "RevoluteUnaligned": "revolute",
        "PR": "prismatic",
        "Fixed": "fixed",
        # URDF does not support spherical directly; export as fixed for compatibility.
        "SP": "fixed",
        "Spherical": "fixed",
        "SphericalZYX": "fixed",
        "FF": "floating",
        "FreeFlyer": "floating",
    }
    return mapping.get(short, "fixed")


def _joint_axis_in_parent(joint, joint_type):
    """Extract a robust axis vector for revolute/prismatic joints."""
    if joint_type not in {"revolute", "prismatic"}:
        return np.array([0.0, 0.0, 1.0])

    if not hasattr(joint, "createData") or not hasattr(joint, "calc"):
        return np.array([0.0, 0.0, 1.0])

    try:
        joint_data = joint.createData()
        q_local = np.zeros(joint.nq)
        joint.calc(joint_data, q_local)
        S = np.asarray(joint_data.S)
    except Exception:
        return np.array([0.0, 0.0, 1.0])

    if S.ndim == 1:
        S = S.reshape((-1, 1))
    if S.shape[0] < 6 or S.shape[1] < 1:
        return np.array([0.0, 0.0, 1.0])

    motion_axis = S[:, 0]
    ang = motion_axis[:3]
    lin = motion_axis[3:6]

    if joint_type == "revolute":
        axis = ang if np.linalg.norm(ang) > 1e-12 else lin
    else:  # prismatic
        axis = lin if np.linalg.norm(lin) > 1e-12 else ang

    norm = np.linalg.norm(axis)
    if norm <= 1e-12:
        return np.array([0.0, 0.0, 1.0])
    return axis / norm


def save_scaled_urdf(new_model_name, new_model_path, scaled_model, visual_model=None, collision_model=None):
    """
    Save a scaled Pinocchio model as a URDF file.

    Args:
        new_model_name (str): Name of the robot model in the URDF root.
        new_model_path (str | Path): Output URDF path.
        scaled_model (pin.Model): Scaled Pinocchio model.
    """
    output_path = Path(new_model_path)
    urdf = ET.Element("robot", name=str(new_model_name))

    materials = {
        "body_color": "0.2 0.05 0.8 0.3",
        "body_color_R": "0.8 0.05 0.2 0.6",
        "body_color_L": "0.05 0.8 0.2 0.6",
        "Black": "0 0 0 1",
        "marker_color": "1 0 0 1",
    }
    for mat_name, rgba in materials.items():
        material = ET.SubElement(urdf, "material", name=mat_name)
        ET.SubElement(material, "color", rgba=rgba)

    body_frames = [frame for frame in scaled_model.frames if frame.type == pin.FrameType.BODY]
    body_frames_by_joint_id = defaultdict(list)
    for frame in body_frames:
        body_frames_by_joint_id[frame.parentJoint].append(frame)

    # Prefer first non-virtual BODY frame as canonical link for each joint.
    joint_id_to_link_name = {}
    for joint_id, frames in body_frames_by_joint_id.items():
        non_virtual = [f for f in frames if "virtual" not in f.name]
        chosen = non_virtual[0] if non_virtual else frames[0]
        joint_id_to_link_name[joint_id] = chosen.name

    # One inertial carrier link per joint group.
    mass_carrier_by_joint = {}
    for joint_id, frames in body_frames_by_joint_id.items():
        non_virtual = [f for f in frames if "virtual" not in f.name]
        chosen = non_virtual[0] if non_virtual else frames[0]
        mass_carrier_by_joint[joint_id] = chosen.name

    for frame in body_frames:
        link_name = frame.name
        parent_joint_idx = frame.parentJoint
        parent_inertia = scaled_model.inertias[parent_joint_idx]

        if frame.name == mass_carrier_by_joint[parent_joint_idx]:
            mass = float(parent_inertia.mass)
            com = np.asarray(parent_inertia.lever).reshape(3)
            inertia_matrix = np.asarray(parent_inertia.inertia).reshape(3, 3)
        else:
            mass = 0.0
            com = np.zeros(3)
            inertia_matrix = np.zeros((3, 3))

        link_elem = ET.SubElement(urdf, "link", name=link_name)
        inertial_elem = ET.SubElement(link_elem, "inertial")
        ET.SubElement(inertial_elem, "mass", value=f"{mass:.9g}")
        ET.SubElement(
            inertial_elem,
            "origin",
            xyz=f"{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}",
            rpy="0 0 0",
        )
        ET.SubElement(
            inertial_elem,
            "inertia",
            ixx=f"{inertia_matrix[0, 0]:.9g}",
            ixy=f"{inertia_matrix[0, 1]:.9g}",
            ixz=f"{inertia_matrix[0, 2]:.9g}",
            iyy=f"{inertia_matrix[1, 1]:.9g}",
            iyz=f"{inertia_matrix[1, 2]:.9g}",
            izz=f"{inertia_matrix[2, 2]:.9g}",
        )

        # Visual geometry
        if visual_model is not None:
            frame_id = scaled_model.getFrameId(link_name)
            for geom in visual_model.geometryObjects:
                if geom.parentFrame == frame_id:
                    mesh_path = geom.meshPath
                    if not mesh_path:
                        continue
                    visual_elem = ET.SubElement(link_elem, "visual")
                    placement = geom.placement
                    xyz = placement.translation
                    rpy = pin.rpy.matrixToRpy(placement.rotation)
                    ET.SubElement(
                        visual_elem,
                        "origin",
                        xyz=f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}",
                        rpy=f"{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"
                    )
                    geometry_elem = ET.SubElement(visual_elem, "geometry")
                    scale = geom.meshScale
                    ET.SubElement(
                        geometry_elem,
                        "mesh",
                        filename=_mesh_to_package_uri(mesh_path),
                        scale=f"{scale[0]:.6f} {scale[1]:.6f} {scale[2]:.6f}"
                    )
                    material_name = (
                        "body_color_L" if "left_" in link_name
                        else "body_color_R" if "right_" in link_name
                        else "body_color"
                    )
                    ET.SubElement(visual_elem, "material", name=material_name)

        # Collision geometry
        if collision_model is not None:
            frame_id = scaled_model.getFrameId(link_name)
            for geom in collision_model.geometryObjects:
                if geom.parentFrame == frame_id:
                    mesh_path = geom.meshPath
                    if not mesh_path:
                        continue
                    collision_elem = ET.SubElement(link_elem, "collision")
                    placement = geom.placement
                    xyz = placement.translation
                    rpy = pin.rpy.matrixToRpy(placement.rotation)
                    ET.SubElement(
                        collision_elem,
                        "origin",
                        xyz=f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}",
                        rpy=f"{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"
                    )
                    geometry_elem = ET.SubElement(collision_elem, "geometry")
                    scale = geom.meshScale
                    ET.SubElement(
                        geometry_elem,
                        "mesh",
                        filename=_mesh_to_package_uri(mesh_path),
                        scale=f"{scale[0]:.6f} {scale[1]:.6f} {scale[2]:.6f}"
                    )

    # Detect free-flyer to skip it in URDF tree joints.
    has_freeflyer = False
    if scaled_model.njoints > 1:
        shortname_j1 = scaled_model.joints[1].shortname()
        has_freeflyer = ("FreeFlyer" in shortname_j1) or ("FF" in shortname_j1)
    start_idx = 2 if has_freeflyer else 1

    for joint_id in range(start_idx, scaled_model.njoints):
        joint = scaled_model.joints[joint_id]
        joint_name = scaled_model.names[joint_id]
        parent_joint_id = scaled_model.parents[joint_id]

        if parent_joint_id == 0:
            # Universe parent: keep this link as base without explicit URDF joint.
            continue

        parent_link = joint_id_to_link_name.get(parent_joint_id, "middle_pelvis")
        child_link = joint_id_to_link_name.get(joint_id)
        if not child_link:
            continue

        joint_type = _joint_urdf_type(joint)
        joint_elem = ET.SubElement(urdf, "joint", name=joint_name, type=joint_type)
        ET.SubElement(joint_elem, "parent", link=parent_link)
        ET.SubElement(joint_elem, "child", link=child_link)

        placement = scaled_model.jointPlacements[joint_id]
        xyz = placement.translation
        rpy = pin.rpy.matrixToRpy(placement.rotation)
        ET.SubElement(
            joint_elem,
            "origin",
            xyz=f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}",
            rpy=f"{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}",
        )

        if joint_type in {"revolute", "prismatic"}:
            axis = _joint_axis_in_parent(joint, joint_type)
            ET.SubElement(
                joint_elem,
                "axis",
                xyz=f"{axis[0]:.6f} {axis[1]:.6f} {axis[2]:.6f}",
            )

            idx_q = getattr(joint, "idx_q", None)
            idx_v = getattr(joint, "idx_v", None)
            if idx_q is not None and idx_v is not None and idx_q < len(scaled_model.lowerPositionLimit):
                lower = float(scaled_model.lowerPositionLimit[idx_q])
                upper = float(scaled_model.upperPositionLimit[idx_q])
                effort = float(scaled_model.effortLimit[idx_v])
                velocity = float(scaled_model.velocityLimit[idx_v])
                ET.SubElement(
                    joint_elem,
                    "limit",
                    effort=f"{effort:.9g}",
                    velocity=f"{velocity:.9g}",
                    lower=f"{lower:.9g}",
                    upper=f"{upper:.9g}",
                )

    # Add fixed joints for extra BODY frames sharing same parent joint.
    for joint_id, frames in body_frames_by_joint_id.items():
        if len(frames) <= 1:
            continue

        main_link_name = joint_id_to_link_name.get(joint_id, frames[0].name)
        main_frame = next((f for f in frames if f.name == main_link_name), frames[0])
        main_inv = main_frame.placement.inverse()
        for frame in frames:
            if frame.name == main_link_name:
                continue

            rel_placement = main_inv * frame.placement
            xyz = rel_placement.translation
            rpy = pin.rpy.matrixToRpy(rel_placement.rotation)
            joint_name = f"fixed_{main_link_name}_to_{frame.name}"
            joint_elem = ET.SubElement(urdf, "joint", name=joint_name, type="fixed")
            ET.SubElement(joint_elem, "parent", link=main_link_name)
            ET.SubElement(joint_elem, "child", link=frame.name)
            ET.SubElement(
                joint_elem,
                "origin",
                xyz=f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}",
                rpy=f"{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}",
            )

    # Export OP_FRAME markers as fixed child links with tiny visual spheres.
    existing_link_names = {frame.name for frame in body_frames}
    for frame in scaled_model.frames:
        if frame.type != pin.FrameType.OP_FRAME:
            continue
        marker_name = frame.name
        if marker_name in existing_link_names:
            continue

        parent_joint_id = frame.parentJoint
        parent_link = joint_id_to_link_name.get(parent_joint_id, "middle_pelvis")
        placement = frame.placement

        marker_link_elem = ET.SubElement(urdf, "link", name=marker_name)
        visual_elem = ET.SubElement(marker_link_elem, "visual")
        ET.SubElement(visual_elem, "origin", xyz="0 0 0", rpy="0 0 0")
        geometry_elem = ET.SubElement(visual_elem, "geometry")
        ET.SubElement(geometry_elem, "sphere", radius="0.01")
        ET.SubElement(visual_elem, "material", name="marker_color")

        joint_name = f"joint_{marker_name}"
        joint_elem = ET.SubElement(urdf, "joint", name=joint_name, type="fixed")
        ET.SubElement(joint_elem, "parent", link=parent_link)
        ET.SubElement(joint_elem, "child", link=marker_name)
        xyz = placement.translation
        rpy = pin.rpy.matrixToRpy(placement.rotation)
        ET.SubElement(
            joint_elem,
            "origin",
            xyz=f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}",
            rpy=f"{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}",
        )

    tree = ET.ElementTree(urdf)
    ET.indent(tree, space="  ")
    output_dir = output_path.parent
    if str(output_dir):
        os.makedirs(output_dir, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path
