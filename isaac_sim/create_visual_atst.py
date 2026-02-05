# Create AT-ST USD with Thingiverse visual meshes + physics skeleton
# Run with: D:\Projects\ATST\venv_isaaclab\Scripts\python.exe create_visual_atst.py
#
# This script:
# 1. Creates the physics articulation skeleton (capsules/boxes for collision)
# 2. Imports STL meshes from Thingiverse as visual geometry
# 3. Parents visual meshes to the appropriate physics bodies
#
# The result is a walker that LOOKS like the AT-ST but uses simple collision shapes

import sys
import math
import os
print("Creating AT-ST USD with visual meshes...", flush=True)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

print("Isaac Sim started!", flush=True)

import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf, Usd, UsdShade, Sdf

import omni.usd
import omni.kit.commands

stage = omni.usd.get_context().get_stage()

# =============================================================================
# Paths to Thingiverse STL files
# =============================================================================
THINGIVERSE_BASE = "D:/Projects/ATST/thingiverse"
PART1 = f"{THINGIVERSE_BASE}/AT-ST Highly Detailed and Posable - 6986726 - part 1 of 3/files"
PART2 = f"{THINGIVERSE_BASE}/AT-ST Highly Detailed and Posable - 6986726 - part 2 of 3/files"

# Define which STL files belong to which body part
TORSO_STLS = [
    # Head/cockpit
    f"{PART1}/AT-ST_Head_Main_a_x1.stl",
    f"{PART1}/AT-ST_Head_Main_b_x1.stl",
    f"{PART1}/AT-ST_Head_Face__x1.stl",
    f"{PART1}/AT-ST_Head_Power__x1.stl",
    f"{PART1}/AT-ST_Head_Cables_x1.stl",
    f"{PART1}/AT-ST_Head_Neck_x1.stl",
    # Body
    f"{PART1}/AT-ST_Body_x1.stl",
    f"{PART1}/AT-ST_Body_Power__x1.stl",
    f"{PART1}/AT-ST_Body_Exhaust_x1.stl",
    f"{PART1}/AT-ST_Body_Thingy__x1.stl",
    # Armor
    f"{PART1}/AT-ST_Head_Armor_Lg_right_x1.stl",
    f"{PART1}/AT-ST_Head_Armor_Lg_left_x1.stl",
    f"{PART1}/AT-ST_Body_Armor_upper_right__x1.stl",
    f"{PART1}/AT-ST_Body_Armor_upper_left__x1.stl",
    f"{PART2}/AT-ST_Body_Armor_lower_right__x1.stl",
    f"{PART2}/AT-ST_Body_Armor_lower_left__x1.stl",
    # Guns
    f"{PART1}/AT-ST_Guns_front_x1.stl",
    f"{PART1}/AT-ST_Guns_right__x1.stl",
    f"{PART1}/AT-ST_Guns_left__x1.stl",
]

LEFT_HIP_STLS = [
    f"{PART2}/AT-ST_Legs_Hip_x2.stl",  # Will use one instance
]

LEFT_UPPER_LEG_STLS = [
    f"{PART2}/AT-ST_Legs_Upper_left_a_x1.stl",
    f"{PART2}/AT-ST_Legs_Upper_left_b_x1.stl",
    f"{PART2}/AT-ST_Legs_Upper_Armor_left_x1.stl",
    f"{PART2}/AT-ST_Legs_Upper_Kneecap_x2.stl",
]

LEFT_LOWER_LEG_STLS = [
    f"{PART2}/AT-ST_Legs_Mid_left_a_x1.stl",
    f"{PART2}/AT-ST_Legs_Mid_left_b_x1.stl",
    f"{PART2}/AT-ST_Legs_Mid_LeafSpring_x2.stl",
    f"{PART2}/AT-ST_Legs_Lower_left_a_x1.stl",
    f"{PART2}/AT-ST_Legs_Lower_left_b_x1.stl",
]

LEFT_FOOT_STLS = [
    f"{PART2}/AT-ST_Foot_left__x1.stl",
    f"{PART2}/AT-ST_Foot_Toe_left__x1.stl",
]

RIGHT_UPPER_LEG_STLS = [
    f"{PART2}/AT-ST_Legs_Upper_right_a_x1.stl",
    f"{PART2}/AT-ST_Legs_Upper_right_b_x1.stl",
    f"{PART2}/AT-ST_Legs_Upper_Armor_right_x1.stl",
    f"{PART2}/AT-ST_Legs_Upper_Kneecap_x2.stl",
]

RIGHT_LOWER_LEG_STLS = [
    f"{PART2}/AT-ST_Legs_Mid_right_a_x1.stl",
    f"{PART2}/AT-ST_Legs_Mid_right_b_x1.stl",
    f"{PART2}/AT-ST_Legs_Mid_LeafSpring_x2.stl",
    f"{PART2}/AT-ST_Legs_Lower_right_a_x1.stl",
    f"{PART2}/AT-ST_Legs_Lower_right_b_x1.stl",
]

RIGHT_FOOT_STLS = [
    f"{PART2}/AT-ST_Foot_right__x1.stl",
    f"{PART2}/AT-ST_Foot_Toe_right__x1.stl",
]

# =============================================================================
# AT-ST Dimensions - Same as create_atst_usd.py
# =============================================================================
BODY_WIDTH = 0.30
BODY_DEPTH = 0.25
BODY_HEIGHT = 0.25

HIP_CONNECTOR_LENGTH = 0.08
HIP_CONNECTOR_RADIUS = 0.04

THIGH_LENGTH = 0.30
THIGH_RADIUS = 0.035

SHIN_LENGTH = 0.32
SHIN_RADIUS = 0.028
SHIN_ANGLE = 45

FOOT_LENGTH = 0.18
FOOT_WIDTH = 0.12
FOOT_HEIGHT = 0.03

LEG_SPACING = 0.40

# Calculate positions
FOOT_Z = FOOT_HEIGHT / 2
ANKLE_X = -FOOT_LENGTH * 0.3
ANKLE_Z = FOOT_HEIGHT + 0.01

SHIN_ANGLE_RAD = math.radians(SHIN_ANGLE)
SHIN_DX = SHIN_LENGTH * math.sin(SHIN_ANGLE_RAD)
SHIN_DZ = SHIN_LENGTH * math.cos(SHIN_ANGLE_RAD)

KNEE_X = ANKLE_X + SHIN_DX
KNEE_Z = ANKLE_Z + SHIN_DZ

SHIN_CENTER_X = ANKLE_X + SHIN_DX / 2
SHIN_CENTER_Z = ANKLE_Z + SHIN_DZ / 2

THIGH_CENTER_Z = KNEE_Z + THIGH_LENGTH / 2
HIP_Z = KNEE_Z + THIGH_LENGTH
HIP_CONNECTOR_Z = HIP_Z
BODY_Z = HIP_Z + BODY_HEIGHT / 2 + 0.05

print(f"Body at z={BODY_Z:.3f}, Hip at z={HIP_Z:.3f}", flush=True)

prim_path = "/ATST"

# Create root xform with ArticulationRoot
root = UsdGeom.Xform.Define(stage, prim_path)
root_prim = stage.GetPrimAtPath(prim_path)
UsdPhysics.ArticulationRootAPI.Apply(root_prim)


def create_collision_body(name, parent_path, pos, size, mass, shape="box", rotation=None):
    """Create a collision body (invisible in final render, used for physics)."""
    path = f"{parent_path}/{name}"

    # Create an Xform to hold both collision and visual geometry
    xform = UsdGeom.Xform.Define(stage, path)
    prim = stage.GetPrimAtPath(path)

    # Set transform
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))

    if rotation is not None:
        roll, pitch, yaw = [math.radians(r) for r in rotation]
        xf.AddRotateXYZOp().Set(Gf.Vec3f(math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))

    # Add physics to the xform
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(mass)

    # Create collision geometry as a child
    collision_path = f"{path}/collision"
    if shape == "capsule":
        geom = UsdGeom.Capsule.Define(stage, collision_path)
        geom.CreateRadiusAttr(size[0])
        geom.CreateHeightAttr(size[1])
        geom.CreateAxisAttr("Z")
    else:
        geom = UsdGeom.Cube.Define(stage, collision_path)
        geom.CreateSizeAttr(1.0)
        col_prim = stage.GetPrimAtPath(collision_path)
        col_xf = UsdGeom.Xformable(col_prim)
        col_xf.AddScaleOp().Set(Gf.Vec3f(*size))

    col_prim = stage.GetPrimAtPath(collision_path)
    UsdPhysics.CollisionAPI.Apply(col_prim)

    # Make collision geometry invisible (we'll add visual meshes separately)
    geom.CreateVisibilityAttr("invisible")

    return path


def create_revolute_joint(name, parent_body, child_body, anchor_pos, axis="Y",
                          lower_limit=-45, upper_limit=45):
    """Create a revolute joint between two bodies."""
    joint_path = f"{prim_path}/joints/{name}"
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)

    joint.CreateBody0Rel().SetTargets([parent_body])
    joint.CreateBody1Rel().SetTargets([child_body])
    joint.CreateAxisAttr(axis)

    parent_prim = stage.GetPrimAtPath(parent_body)
    child_prim = stage.GetPrimAtPath(child_body)

    parent_xf = UsdGeom.Xformable(parent_prim)
    child_xf = UsdGeom.Xformable(child_prim)

    parent_pos = list(parent_xf.GetOrderedXformOps()[0].Get())
    child_pos = list(child_xf.GetOrderedXformOps()[0].Get())

    local0 = Gf.Vec3f(
        anchor_pos[0] - parent_pos[0],
        anchor_pos[1] - parent_pos[1],
        anchor_pos[2] - parent_pos[2]
    )
    local1 = Gf.Vec3f(
        anchor_pos[0] - child_pos[0],
        anchor_pos[1] - child_pos[1],
        anchor_pos[2] - child_pos[2]
    )

    joint.CreateLocalPos0Attr().Set(local0)
    joint.CreateLocalPos1Attr().Set(local1)

    joint.CreateLowerLimitAttr(lower_limit)
    joint.CreateUpperLimitAttr(upper_limit)

    joint_prim = stage.GetPrimAtPath(joint_path)
    drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(200.0)
    drive.CreateDampingAttr(5.0)
    drive.CreateMaxForceAttr(300.0)
    drive.CreateTargetPositionAttr(0.0)

    return joint_path


# Create joints folder
UsdGeom.Xform.Define(stage, f"{prim_path}/joints")

# =============================================================================
# Create physics bodies (same structure as before)
# =============================================================================
print("Creating physics bodies...", flush=True)

body = create_collision_body("torso_link", prim_path,
                             (0, 0, BODY_Z),
                             (BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT),
                             mass=10.0)

L_Y = LEG_SPACING / 2
R_Y = -LEG_SPACING / 2

# Left leg
l_hip = create_collision_body("left_hip_link", prim_path,
                              (KNEE_X, L_Y, HIP_CONNECTOR_Z),
                              (HIP_CONNECTOR_RADIUS, HIP_CONNECTOR_LENGTH),
                              mass=0.5, shape="capsule")

l_thigh = create_collision_body("left_upper_leg_link", prim_path,
                                (KNEE_X, L_Y, THIGH_CENTER_Z),
                                (THIGH_RADIUS, THIGH_LENGTH - THIGH_RADIUS*2),
                                mass=2.0, shape="capsule")

l_shin = create_collision_body("left_lower_leg_link", prim_path,
                               (SHIN_CENTER_X, L_Y, SHIN_CENTER_Z),
                               (SHIN_RADIUS, SHIN_LENGTH - SHIN_RADIUS*2),
                               mass=1.5, shape="capsule",
                               rotation=(0, -SHIN_ANGLE, 0))

l_foot = create_collision_body("left_ankle_link", prim_path,
                               (0, L_Y, FOOT_Z),
                               (FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT),
                               mass=3.0)

# Right leg
r_hip = create_collision_body("right_hip_link", prim_path,
                              (KNEE_X, R_Y, HIP_CONNECTOR_Z),
                              (HIP_CONNECTOR_RADIUS, HIP_CONNECTOR_LENGTH),
                              mass=0.5, shape="capsule")

r_thigh = create_collision_body("right_upper_leg_link", prim_path,
                                (KNEE_X, R_Y, THIGH_CENTER_Z),
                                (THIGH_RADIUS, THIGH_LENGTH - THIGH_RADIUS*2),
                                mass=2.0, shape="capsule")

r_shin = create_collision_body("right_lower_leg_link", prim_path,
                               (SHIN_CENTER_X, R_Y, SHIN_CENTER_Z),
                               (SHIN_RADIUS, SHIN_LENGTH - SHIN_RADIUS*2),
                               mass=1.5, shape="capsule",
                               rotation=(0, -SHIN_ANGLE, 0))

r_foot = create_collision_body("right_ankle_link", prim_path,
                               (0, R_Y, FOOT_Z),
                               (FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT),
                               mass=3.0)

# =============================================================================
# Create joints
# =============================================================================
print("Creating joints...", flush=True)

create_revolute_joint("left_hip_pitch", body, l_hip,
                      (KNEE_X, L_Y, HIP_Z), "Y", -60, 60)
create_revolute_joint("right_hip_pitch", body, r_hip,
                      (KNEE_X, R_Y, HIP_Z), "Y", -60, 60)

create_revolute_joint("left_hip_roll", l_hip, l_thigh,
                      (KNEE_X, L_Y, HIP_Z), "X", -15, 15)
create_revolute_joint("right_hip_roll", r_hip, r_thigh,
                      (KNEE_X, R_Y, HIP_Z), "X", -15, 15)

create_revolute_joint("left_knee", l_thigh, l_shin,
                      (KNEE_X, L_Y, KNEE_Z), "Y", -120, 30)
create_revolute_joint("right_knee", r_thigh, r_shin,
                      (KNEE_X, R_Y, KNEE_Z), "Y", -120, 30)

create_revolute_joint("left_ankle", l_shin, l_foot,
                      (ANKLE_X, L_Y, ANKLE_Z), "Y", -60, 60)
create_revolute_joint("right_ankle", r_shin, r_foot,
                      (ANKLE_X, R_Y, ANKLE_Z), "Y", -60, 60)

# =============================================================================
# Import STL visual meshes
# =============================================================================
print("\n" + "="*60, flush=True)
print("IMPORTING VISUAL MESHES FROM STL FILES", flush=True)
print("="*60, flush=True)

# First, let's check what STL files actually exist
def check_stl_files(stl_list, name):
    """Check which STL files exist."""
    existing = []
    missing = []
    for stl in stl_list:
        if os.path.exists(stl):
            existing.append(stl)
        else:
            missing.append(stl)
    print(f"{name}: {len(existing)} found, {len(missing)} missing", flush=True)
    if missing:
        for m in missing[:3]:  # Show first 3 missing
            print(f"  Missing: {os.path.basename(m)}", flush=True)
    return existing

torso_files = check_stl_files(TORSO_STLS, "Torso")
left_hip_files = check_stl_files(LEFT_HIP_STLS, "Left Hip")
left_upper_files = check_stl_files(LEFT_UPPER_LEG_STLS, "Left Upper Leg")
left_lower_files = check_stl_files(LEFT_LOWER_LEG_STLS, "Left Lower Leg")
left_foot_files = check_stl_files(LEFT_FOOT_STLS, "Left Foot")
right_upper_files = check_stl_files(RIGHT_UPPER_LEG_STLS, "Right Upper Leg")
right_lower_files = check_stl_files(RIGHT_LOWER_LEG_STLS, "Right Lower Leg")
right_foot_files = check_stl_files(RIGHT_FOOT_STLS, "Right Foot")

# For now, save the physics-only version
# Visual mesh import will be done in a second pass after we analyze the STL sizes/orientations

# =============================================================================
# Set default prim and save
# =============================================================================
root_prim = stage.GetPrimAtPath(prim_path)
stage.SetDefaultPrim(root_prim)

output_path = "D:/Projects/ATST/usd/atst_visual.usda"
print(f"\nSaving physics skeleton to {output_path}...", flush=True)
stage.Export(output_path)

print(f"\n" + "="*60, flush=True)
print("PHYSICS SKELETON CREATED", flush=True)
print("="*60, flush=True)
print(f"Output: {output_path}", flush=True)
print(f"\nNext step: Import and scale STL meshes to fit the skeleton", flush=True)
print(f"The STL files need to be analyzed for size/orientation first.", flush=True)

simulation_app.close()
print("Done!", flush=True)
