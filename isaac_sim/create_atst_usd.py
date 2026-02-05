# Create AT-ST USD file with accurate digitigrade legs for Isaac Lab training
# Run with: D:\Projects\ATST\venv_isaaclab\Scripts\python.exe create_atst_usd.py
#
# AT-ST Leg Structure (from top to bottom):
#   Body (cockpit)
#     └── Hip Connector (short horizontal piece sticking out sideways)
#           └── Thigh (vertical upper leg going down)
#                 └── Shin (angled ~45° backward - the "reverse knee" look)
#                       └── Foot (flat pad)
#
# This creates the classic "chicken walker" digitigrade appearance

import sys
import math
print("Creating AT-ST USD with digitigrade legs...", flush=True)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

print("Isaac Sim started, creating USD...", flush=True)

import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf, Usd, UsdShade, Sdf

import omni.usd
stage = omni.usd.get_context().get_stage()

# =============================================================================
# AT-ST Dimensions - Digitigrade Configuration
# =============================================================================
# Body (cockpit)
BODY_WIDTH = 0.30    # X - front to back
BODY_DEPTH = 0.25    # Y - side to side
BODY_HEIGHT = 0.25   # Z - tall

# Hip connector - short horizontal piece extending from body
HIP_CONNECTOR_LENGTH = 0.08  # How far it sticks out sideways
HIP_CONNECTOR_RADIUS = 0.04

# Thigh - vertical upper leg
THIGH_LENGTH = 0.30
THIGH_RADIUS = 0.035

# Shin - angled backward (the key to digitigrade look!)
SHIN_LENGTH = 0.32
SHIN_RADIUS = 0.028
SHIN_ANGLE = 45  # degrees backward from vertical

# Foot
FOOT_LENGTH = 0.18   # X - front to back (long for stability)
FOOT_WIDTH = 0.12    # Y - side to side
FOOT_HEIGHT = 0.03

# Leg spacing (how far apart the legs are, side to side)
LEG_SPACING = 0.40   # Y distance between leg centers

# Leg radius for capsules
LEG_RADIUS = 0.03

# =============================================================================
# Calculate positions based on geometry
# =============================================================================
# Work from ground up
FOOT_Z = FOOT_HEIGHT / 2  # Foot center height

# Ankle is at the back-top of the foot
ANKLE_X = -FOOT_LENGTH * 0.3  # Slightly back from foot center
ANKLE_Z = FOOT_HEIGHT + 0.01

# Shin goes from ankle up and forward at an angle
SHIN_ANGLE_RAD = math.radians(SHIN_ANGLE)
SHIN_DX = SHIN_LENGTH * math.sin(SHIN_ANGLE_RAD)  # Forward offset
SHIN_DZ = SHIN_LENGTH * math.cos(SHIN_ANGLE_RAD)  # Vertical rise

# Knee position (top of shin)
KNEE_X = ANKLE_X + SHIN_DX  # Forward from ankle
KNEE_Z = ANKLE_Z + SHIN_DZ  # Up from ankle

# Shin center (midpoint of shin)
SHIN_CENTER_X = ANKLE_X + SHIN_DX / 2
SHIN_CENTER_Z = ANKLE_Z + SHIN_DZ / 2

# Thigh goes straight up from knee
THIGH_CENTER_Z = KNEE_Z + THIGH_LENGTH / 2
HIP_Z = KNEE_Z + THIGH_LENGTH

# Hip connector center (sticks out sideways from body)
HIP_CONNECTOR_Z = HIP_Z

# Body sits above the hips
BODY_Z = HIP_Z + BODY_HEIGHT / 2 + 0.05  # Small gap

print(f"Geometry calculated:", flush=True)
print(f"  Foot at z={FOOT_Z:.3f}", flush=True)
print(f"  Ankle at z={ANKLE_Z:.3f}, x={ANKLE_X:.3f}", flush=True)
print(f"  Knee at z={KNEE_Z:.3f}, x={KNEE_X:.3f}", flush=True)
print(f"  Hip at z={HIP_Z:.3f}", flush=True)
print(f"  Body at z={BODY_Z:.3f}", flush=True)
print(f"  Total height: ~{BODY_Z + BODY_HEIGHT/2:.3f}m", flush=True)

prim_path = "/ATST"

# Create root xform with ArticulationRoot
root = UsdGeom.Xform.Define(stage, prim_path)
root_prim = stage.GetPrimAtPath(prim_path)
UsdPhysics.ArticulationRootAPI.Apply(root_prim)


def create_rigid_body(name, parent_path, pos, size, mass, shape="box", rotation=None):
    """Create a rigid body with collision.

    Args:
        name: Name of the body
        parent_path: Parent prim path
        pos: (x, y, z) position
        size: For box: (x, y, z), for capsule: (radius, height)
        mass: Mass in kg
        shape: "box" or "capsule"
        rotation: Optional (roll, pitch, yaw) in degrees for orientation
    """
    path = f"{parent_path}/{name}"

    if shape == "capsule":
        geom = UsdGeom.Capsule.Define(stage, path)
        geom.CreateRadiusAttr(size[0])
        geom.CreateHeightAttr(size[1])
        geom.CreateAxisAttr("Z")  # Default axis, rotation will orient it
    else:
        geom = UsdGeom.Cube.Define(stage, path)
        geom.CreateSizeAttr(1.0)

    prim = stage.GetPrimAtPath(path)

    # Set transform
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))

    # Add rotation if specified (for angled shin)
    if rotation is not None:
        # Convert degrees to radians for quaternion
        roll, pitch, yaw = [math.radians(r) for r in rotation]
        # Create rotation quaternion (pitch around Y axis for shin angle)
        xf.AddRotateXYZOp().Set(Gf.Vec3f(math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))

    if shape == "box":
        xf.AddScaleOp().Set(Gf.Vec3f(*size))

    # Add physics
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(mass)

    return path


def create_revolute_joint(name, parent_body, child_body, anchor_pos, axis="Y",
                          lower_limit=-45, upper_limit=45):
    """Create a revolute joint between two bodies."""
    joint_path = f"{prim_path}/joints/{name}"
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)

    # Set connected bodies
    joint.CreateBody0Rel().SetTargets([parent_body])
    joint.CreateBody1Rel().SetTargets([child_body])
    joint.CreateAxisAttr(axis)

    # Get parent and child positions
    parent_prim = stage.GetPrimAtPath(parent_body)
    child_prim = stage.GetPrimAtPath(child_body)

    parent_xf = UsdGeom.Xformable(parent_prim)
    child_xf = UsdGeom.Xformable(child_prim)

    parent_pos = list(parent_xf.GetOrderedXformOps()[0].Get())
    child_pos = list(child_xf.GetOrderedXformOps()[0].Get())

    # Calculate local positions relative to each body's center
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

    # Set limits (in degrees)
    joint.CreateLowerLimitAttr(lower_limit)
    joint.CreateUpperLimitAttr(upper_limit)

    # Add drive for position control
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
# Create the AT-ST body parts
# =============================================================================

print("Creating body (cockpit)...", flush=True)
body = create_rigid_body("torso_link", prim_path,
                         (0, 0, BODY_Z),
                         (BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT),
                         mass=10.0)

print("Creating left leg...", flush=True)
# Left leg Y position
L_Y = LEG_SPACING / 2

# Left hip connector (short piece extending sideways from body)
l_hip_conn = create_rigid_body("left_hip_link", prim_path,
                               (KNEE_X, L_Y, HIP_CONNECTOR_Z),
                               (HIP_CONNECTOR_RADIUS, HIP_CONNECTOR_LENGTH),
                               mass=0.5, shape="capsule")

# Left thigh (vertical upper leg)
l_thigh = create_rigid_body("left_upper_leg_link", prim_path,
                            (KNEE_X, L_Y, THIGH_CENTER_Z),
                            (THIGH_RADIUS, THIGH_LENGTH - THIGH_RADIUS*2),
                            mass=2.0, shape="capsule")

# Left shin (angled backward - the key feature!)
# Shin is rotated to angle backward
l_shin = create_rigid_body("left_lower_leg_link", prim_path,
                           (SHIN_CENTER_X, L_Y, SHIN_CENTER_Z),
                           (SHIN_RADIUS, SHIN_LENGTH - SHIN_RADIUS*2),
                           mass=1.5, shape="capsule",
                           rotation=(0, -SHIN_ANGLE, 0))  # Pitch backward

# Left foot (flat pad)
l_foot = create_rigid_body("left_ankle_link", prim_path,
                           (0, L_Y, FOOT_Z),
                           (FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT),
                           mass=3.0)  # Heavy feet for stability

print("Creating right leg...", flush=True)
# Right leg Y position
R_Y = -LEG_SPACING / 2

# Right hip connector
r_hip_conn = create_rigid_body("right_hip_link", prim_path,
                               (KNEE_X, R_Y, HIP_CONNECTOR_Z),
                               (HIP_CONNECTOR_RADIUS, HIP_CONNECTOR_LENGTH),
                               mass=0.5, shape="capsule")

# Right thigh
r_thigh = create_rigid_body("right_upper_leg_link", prim_path,
                            (KNEE_X, R_Y, THIGH_CENTER_Z),
                            (THIGH_RADIUS, THIGH_LENGTH - THIGH_RADIUS*2),
                            mass=2.0, shape="capsule")

# Right shin (angled backward)
r_shin = create_rigid_body("right_lower_leg_link", prim_path,
                           (SHIN_CENTER_X, R_Y, SHIN_CENTER_Z),
                           (SHIN_RADIUS, SHIN_LENGTH - SHIN_RADIUS*2),
                           mass=1.5, shape="capsule",
                           rotation=(0, -SHIN_ANGLE, 0))

# Right foot
r_foot = create_rigid_body("right_ankle_link", prim_path,
                           (0, R_Y, FOOT_Z),
                           (FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT),
                           mass=3.0)

# =============================================================================
# Create joints
# =============================================================================
print("Creating joints...", flush=True)

# Hip joints - connect body to hip connectors
# These allow the leg to swing forward/backward (pitch around Y)
create_revolute_joint("left_hip_pitch", body, l_hip_conn,
                      (KNEE_X, L_Y, HIP_Z), "Y", -60, 60)
create_revolute_joint("right_hip_pitch", body, r_hip_conn,
                      (KNEE_X, R_Y, HIP_Z), "Y", -60, 60)

# Upper leg joints - connect hip connector to thigh
# (In this simplified version, hip connector and thigh move together,
#  but having this joint allows future hip yaw if desired)
create_revolute_joint("left_hip_roll", l_hip_conn, l_thigh,
                      (KNEE_X, L_Y, HIP_Z), "X", -15, 15)  # Small roll range
create_revolute_joint("right_hip_roll", r_hip_conn, r_thigh,
                      (KNEE_X, R_Y, HIP_Z), "X", -15, 15)

# Knee joints - connect thigh to shin
# This is the main "reverse knee" joint
create_revolute_joint("left_knee", l_thigh, l_shin,
                      (KNEE_X, L_Y, KNEE_Z), "Y", -120, 30)
create_revolute_joint("right_knee", r_thigh, r_shin,
                      (KNEE_X, R_Y, KNEE_Z), "Y", -120, 30)

# Ankle joints - connect shin to foot
create_revolute_joint("left_ankle", l_shin, l_foot,
                      (ANKLE_X, L_Y, ANKLE_Z), "Y", -60, 60)
create_revolute_joint("right_ankle", r_shin, r_foot,
                      (ANKLE_X, R_Y, ANKLE_Z), "Y", -60, 60)

# =============================================================================
# Finalize
# =============================================================================
# Set the default prim (required for Isaac Lab)
root_prim = stage.GetPrimAtPath(prim_path)
stage.SetDefaultPrim(root_prim)
print(f"Default prim set to: {prim_path}", flush=True)

# Save the USD file
output_path = "D:/Projects/ATST/usd/atst_rl.usda"
print(f"Saving USD to {output_path}...", flush=True)
stage.Export(output_path)

print(f"\nAT-ST USD created successfully!", flush=True)
print(f"Output: {output_path}", flush=True)
print(f"\nJoint structure (8 joints total):", flush=True)
print(f"  Per leg: hip_pitch -> hip_roll -> knee -> ankle", flush=True)
print(f"\nKey feature: Shins angled {SHIN_ANGLE}° backward for digitigrade look", flush=True)

simulation_app.close()
print("Done!", flush=True)
