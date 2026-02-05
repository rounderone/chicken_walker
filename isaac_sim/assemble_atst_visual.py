# Assemble AT-ST from Thingiverse STL files
# Run with: D:\Projects\ATST\venv_isaaclab\Scripts\python.exe assemble_atst_visual.py
#
# The STL files are positioned flat for 3D printing.
# This script imports them and positions them to create the assembled AT-ST.

import os
import sys
import math
print("Assembling AT-ST from Thingiverse STL files...", flush=True)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})  # Show window to see assembly

print("Isaac Sim started!", flush=True)

import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf, Usd, UsdShade, Sdf
from stl import mesh as stl_mesh

import omni.usd
import omni.kit.commands
import asyncio

stage = omni.usd.get_context().get_stage()

# =============================================================================
# Configuration
# =============================================================================
THINGIVERSE_BASE = "D:/Projects/ATST/thingiverse"
PART1 = f"{THINGIVERSE_BASE}/AT-ST Highly Detailed and Posable - 6986726 - part 1 of 3/files"
PART2 = f"{THINGIVERSE_BASE}/AT-ST Highly Detailed and Posable - 6986726 - part 2 of 3/files"

# Scale: STL files are in mm, we want meters, and then scale to our target size
# 3D printable model is roughly 1/48 scale (180mm for 8.6m AT-ST)
# We want ~0.8m robot for RL training
# So: mm_to_m (0.001) * scale_up (4.5) = 0.0045 scale factor
MM_TO_METERS = 0.001
TARGET_SCALE = 4.5  # Scale up to get ~0.8m robot
SCALE = MM_TO_METERS * TARGET_SCALE

print(f"Scale factor: {SCALE} (mm to target size)", flush=True)

# Root prim
prim_path = "/ATST"
root = UsdGeom.Xform.Define(stage, prim_path)

# Create visual meshes container (separate from physics)
visual_path = f"{prim_path}/visuals"
UsdGeom.Xform.Define(stage, visual_path)


def sanitize_prim_name(name):
    """Sanitize a name for use as a USD prim path."""
    import re
    # Remove leading dashes/underscores, replace special chars
    name = re.sub(r'^[_\-]+', '', name)  # Remove leading _ or -
    name = re.sub(r'[^a-zA-Z0-9_]', '_', name)  # Replace invalid chars with _
    name = re.sub(r'_+', '_', name)  # Collapse multiple underscores
    name = name.strip('_')  # Remove trailing underscores
    # Ensure it starts with a letter
    if name and not name[0].isalpha():
        name = 'mesh_' + name
    return name if name else 'mesh'


def load_stl_as_mesh(stl_path, prim_path, translate=(0,0,0), rotate=(0,0,0), scale=1.0):
    """Load STL file and create a mesh prim from it."""
    if not os.path.exists(stl_path):
        print(f"  WARNING: File not found: {stl_path}", flush=True)
        return None

    # Read STL data
    stl_data = stl_mesh.Mesh.from_file(stl_path)

    # Get vertices and faces
    vertices = stl_data.vectors.reshape(-1, 3)  # Flatten to (N*3, 3)

    # Remove duplicate vertices and build proper index list
    unique_verts, inverse = np.unique(vertices, axis=0, return_inverse=True)

    # Scale and transform vertices
    # Apply scale
    unique_verts = unique_verts * scale

    # Convert to Gf.Vec3f list
    points = [Gf.Vec3f(float(v[0]), float(v[1]), float(v[2])) for v in unique_verts]

    # Build face indices (triangles)
    face_counts = [3] * (len(inverse) // 3)
    face_indices = inverse.tolist()

    # Create mesh prim
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr(face_counts)
    mesh.CreateFaceVertexIndicesAttr(face_indices)
    mesh.CreateSubdivisionSchemeAttr("none")

    # Apply transform
    xformable = UsdGeom.Xformable(mesh.GetPrim())
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*translate))

    if any(r != 0 for r in rotate):
        xformable.AddRotateXYZOp().Set(Gf.Vec3f(*rotate))

    return prim_path


def load_stl_group(stl_files, group_name, translate=(0,0,0), rotate=(0,0,0)):
    """Load multiple STL files as a group."""
    group_name = sanitize_prim_name(group_name)
    group_path = f"{visual_path}/{group_name}"
    group = UsdGeom.Xform.Define(stage, group_path)

    # Apply group transform
    xformable = UsdGeom.Xformable(group.GetPrim())
    xformable.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if any(r != 0 for r in rotate):
        xformable.AddRotateXYZOp().Set(Gf.Vec3f(*rotate))

    count = 0
    for stl_path in stl_files:
        if os.path.exists(stl_path):
            name = os.path.basename(stl_path).replace(".stl", "").replace(" ", "_")
            name = sanitize_prim_name(name)
            mesh_path = f"{group_path}/{name}"
            load_stl_as_mesh(stl_path, mesh_path, scale=SCALE)
            count += 1

    print(f"  {group_name}: loaded {count} meshes", flush=True)
    return group_path


# =============================================================================
# Assembly Layout
# =============================================================================
# The STL parts are laid flat for printing. We need to position them correctly.
# AT-ST structure (from top to bottom):
#   - Head/cockpit (rotates on neck)
#   - Body (main hull, connects legs)
#   - Hip joints (left and right)
#   - Upper legs (thigh)
#   - Mid legs (shin upper)
#   - Lower legs (shin lower/ankle area)
#   - Feet with toes

# Target heights for our ~0.8m robot (working from ground up):
FOOT_Z = 0.02       # Foot sits on ground
ANKLE_Z = 0.04      # Ankle joint
KNEE_Z = 0.30       # Knee joint
HIP_Z = 0.55        # Hip joint
BODY_Z = 0.65       # Body center
HEAD_Z = 0.75       # Head/cockpit center

# Leg spacing (side to side)
LEG_Y_OFFSET = 0.18  # How far apart legs are

print("\n" + "="*60, flush=True)
print("LOADING AND ASSEMBLING AT-ST", flush=True)
print("="*60, flush=True)

# Since the STL parts are in arbitrary positions for printing,
# we need to assemble them. For a first pass, let's just load
# all parts and let the user see them to understand the structure.

# Load all STLs into their categories
print("\nLoading head/body parts...", flush=True)
head_body_stls = [
    f"{PART1}/AT-ST_Head_Main_a_x1.stl",
    f"{PART1}/AT-ST_Head_Main_b_x1.stl",
    f"{PART1}/AT-ST_Head_Face__x1.stl",
    f"{PART1}/AT-ST_Body_x1.stl",
    f"{PART1}/AT-ST_Head_Neck_x1.stl",
    f"{PART1}/AT-ST_Body_Thingy__x1.stl",
]

print("\nLoading armor parts...", flush=True)
armor_stls = [
    f"{PART1}/AT-ST_Head_Armor_Lg_right_x1.stl",
    f"{PART1}/AT-ST_Head_Armor_Lg_left_x1.stl",
    f"{PART1}/AT-ST_Body_Armor_upper_right__x1.stl",
    f"{PART1}/AT-ST_Body_Armor_upper_left__x1.stl",
    f"{PART2}/AT-ST_Body_Armor_lower_right__x1.stl",
    f"{PART2}/AT-ST_Body_Armor_lower_left__x1.stl",
]

print("\nLoading gun parts...", flush=True)
gun_stls = [
    f"{PART1}/AT-ST_Guns_front_x1.stl",
    f"{PART1}/AT-ST_Guns_right__x1.stl",
    f"{PART1}/AT-ST_Guns_left__x1.stl",
]

print("\nLoading left leg parts...", flush=True)
left_leg_stls = [
    f"{PART2}/AT-ST_Legs_Hip_x2.stl",
    f"{PART2}/AT-ST_Legs_Upper_left_a_x1.stl",
    f"{PART2}/AT-ST_Legs_Upper_left_b_x1.stl",
    f"{PART2}/AT-ST_Legs_Upper_Armor_left_x1.stl",
    f"{PART2}/AT-ST_Legs_Upper_Kneecap_x2.stl",
    f"{PART2}/AT-ST_Legs_Mid_left_a_x1.stl",
    f"{PART2}/AT-ST_Legs_Mid_left_b_x1.stl",
    f"{PART2}/AT-ST_Legs_Mid_LeafSpring_x2.stl",
    f"{PART2}/AT-ST_Legs_Lower_left_a_x1.stl",
    f"{PART2}/AT-ST_Legs_Lower_left_b_x1.stl",
    f"{PART2}/AT-ST_Foot_left__x1.stl",
    f"{PART2}/AT-ST_Foot_Toe_left__x1.stl",
]

print("\nLoading right leg parts...", flush=True)
right_leg_stls = [
    f"{PART2}/AT-ST_Legs_Upper_right_a_x1.stl",
    f"{PART2}/AT-ST_Legs_Upper_right_b_x1.stl",
    f"{PART2}/AT-ST_Legs_Upper_Armor_right_x1.stl",
    f"{PART2}/AT-ST_Legs_Mid_right_a_x1.stl",
    f"{PART2}/AT-ST_Legs_Mid_right_b_x1.stl",
    f"{PART2}/AT-ST_Legs_Lower_right_a_x1.stl",
    f"{PART2}/AT-ST_Legs_Lower_right_b_x1.stl",
    f"{PART2}/AT-ST_Foot_right__x1.stl",
    f"{PART2}/AT-ST_Foot_Toe_right__x1.stl",
]

# Load each group
# Note: Parts will be in their print-bed positions initially
load_stl_group(head_body_stls, "head_body", translate=(0, 0, BODY_Z))
load_stl_group(armor_stls, "armor", translate=(0, 0, BODY_Z))
load_stl_group(gun_stls, "guns", translate=(0, 0, HEAD_Z))
load_stl_group(left_leg_stls, "left_leg", translate=(0, LEG_Y_OFFSET, 0))
load_stl_group(right_leg_stls, "right_leg", translate=(0, -LEG_Y_OFFSET, 0))

# =============================================================================
# Add a ground plane for reference
# =============================================================================
ground = UsdGeom.Mesh.Define(stage, "/World/ground")
ground.CreatePointsAttr([
    Gf.Vec3f(-2, -2, 0),
    Gf.Vec3f(2, -2, 0),
    Gf.Vec3f(2, 2, 0),
    Gf.Vec3f(-2, 2, 0),
])
ground.CreateFaceVertexCountsAttr([4])
ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])

# Add a reference capsule for scale (0.8m tall)
ref_capsule = UsdGeom.Capsule.Define(stage, "/World/reference_height")
ref_capsule.CreateRadiusAttr(0.05)
ref_capsule.CreateHeightAttr(0.7)
ref_capsule.CreateAxisAttr("Z")
xf = UsdGeom.Xformable(ref_capsule.GetPrim())
xf.AddTranslateOp().Set(Gf.Vec3d(1.0, 0, 0.4))

# =============================================================================
# Save and display
# =============================================================================
output_path = "D:/Projects/ATST/usd/atst_assembled.usda"
print(f"\nSaving assembled model to {output_path}...", flush=True)

root_prim = stage.GetPrimAtPath(prim_path)
stage.SetDefaultPrim(root_prim)
stage.Export(output_path)

print(f"\n" + "="*60, flush=True)
print("ASSEMBLY COMPLETE", flush=True)
print("="*60, flush=True)
print(f"Output: {output_path}", flush=True)
print(f"\nThe parts are loaded but may not be positioned correctly yet.", flush=True)
print(f"Open in Isaac Sim to view and manually adjust positions.", flush=True)
print(f"\nKeeping window open for visualization...", flush=True)
print(f"Press Ctrl+C to close.", flush=True)

# Keep running to show the visualization
try:
    while simulation_app.is_running():
        simulation_app.update()
except KeyboardInterrupt:
    pass

simulation_app.close()
print("Done!", flush=True)
