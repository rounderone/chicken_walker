# AT-ST with simple joints (no articulation)
import socket
import json
import time

def send_command(command_type, params=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(120)
    try:
        sock.connect(("localhost", 8766))
        command = {"type": command_type, "params": params or {}}
        sock.sendall(json.dumps(command).encode('utf-8'))
        response = sock.recv(65536)
        return json.loads(response.decode('utf-8'))
    finally:
        sock.close()

script = '''
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf

stage = omni.usd.get_context().get_stage()

if stage.GetPrimAtPath("/World/ATST"):
    stage.RemovePrim("/World/ATST")

print("Building AT-ST with SIMPLE joints (no articulation)...")

atst_path = "/World/ATST"
UsdGeom.Xform.Define(stage, atst_path)

# Simple measurements - all in world coordinates
BODY_Z = 0.50
LEG_X = 0.08

# Upper leg from Z=0.38 to Z=0.50 (touching body)
UPPER_TOP = 0.50
UPPER_BOTTOM = 0.38
UPPER_CENTER = 0.44

# Lower leg from Z=0.26 to Z=0.38 (touching upper)
LOWER_TOP = 0.38
LOWER_BOTTOM = 0.26
LOWER_CENTER = 0.32

# Foot from Z=0.22 to Z=0.26 (touching lower)
FOOT_TOP = 0.26
FOOT_CENTER = 0.24

STIFF = 5000.0
DAMP = 500.0

def make_part(name, pos, half_size, color, mass, kinematic=False):
    path = atst_path + "/" + name
    cube = UsdGeom.Cube.Define(stage, path)
    prim = stage.GetPrimAtPath(path)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*half_size))
    cube.CreateDisplayColorAttr([color])

    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    if kinematic:
        rb.CreateKinematicEnabledAttr(True)
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    return path

# Body - kinematic (fixed)
body = make_part("body", (0, 0, BODY_Z), (0.06, 0.05, 0.04), (0.9, 0.3, 0.3), 2.0, kinematic=True)

# NO ArticulationRootAPI - just regular rigid bodies with joints

# Left leg
l_up = make_part("l_upper", (-LEG_X, 0, UPPER_CENTER), (0.015, 0.015, 0.06), (0.3, 0.9, 0.3), 0.3)
l_lo = make_part("l_lower", (-LEG_X, 0, LOWER_CENTER), (0.012, 0.012, 0.06), (0.3, 0.3, 0.9), 0.3)
l_ft = make_part("l_foot", (-LEG_X, 0, FOOT_CENTER), (0.03, 0.025, 0.015), (0.9, 0.9, 0.3), 1.0)

# Right leg
r_up = make_part("r_upper", (LEG_X, 0, UPPER_CENTER), (0.015, 0.015, 0.06), (0.3, 0.9, 0.3), 0.3)
r_lo = make_part("r_lower", (LEG_X, 0, LOWER_CENTER), (0.012, 0.012, 0.06), (0.3, 0.3, 0.9), 0.3)
r_ft = make_part("r_foot", (LEG_X, 0, FOOT_CENTER), (0.03, 0.025, 0.015), (0.9, 0.9, 0.3), 1.0)

print("  7 parts (no ArticulationRoot)")

def make_joint(name, parent, child, world_joint_pos, parent_center, child_center):
    """Create joint using world position to calculate local anchors"""
    path = atst_path + "/j_" + name
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateAxisAttr("Y")  # Rotate around Y for forward/back leg swing

    # Calculate local positions from world positions
    local0 = (world_joint_pos[0] - parent_center[0],
              world_joint_pos[1] - parent_center[1],
              world_joint_pos[2] - parent_center[2])
    local1 = (world_joint_pos[0] - child_center[0],
              world_joint_pos[1] - child_center[1],
              world_joint_pos[2] - child_center[2])

    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*local1))

    prim = stage.GetPrimAtPath(path)
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(STIFF)
    drive.CreateDampingAttr(DAMP)
    drive.CreateTargetPositionAttr(0.0)

    print(f"    {name}: local0={local0}, local1={local1}")

# Joints at the interfaces between parts
# Hip joint at Z=0.46 (between body bottom 0.46 and upper top 0.50... wait that doesn't touch)
# Let me recalc: body at Z=0.50, half_size Z=0.04 means it goes from 0.46 to 0.54
# Upper at Z=0.44, half_size Z=0.06 means it goes from 0.38 to 0.50
# So hip joint should be at Z=0.48 (overlap region)

make_joint("hip_l", body, l_up, (-LEG_X, 0, 0.48), (0, 0, BODY_Z), (-LEG_X, 0, UPPER_CENTER))
make_joint("hip_r", body, r_up, (LEG_X, 0, 0.48), (0, 0, BODY_Z), (LEG_X, 0, UPPER_CENTER))

# Knee at Z=0.38
make_joint("knee_l", l_up, l_lo, (-LEG_X, 0, 0.38), (-LEG_X, 0, UPPER_CENTER), (-LEG_X, 0, LOWER_CENTER))
make_joint("knee_r", r_up, r_lo, (LEG_X, 0, 0.38), (LEG_X, 0, UPPER_CENTER), (LEG_X, 0, LOWER_CENTER))

# Ankle at Z=0.26
make_joint("ankle_l", l_lo, l_ft, (-LEG_X, 0, 0.26), (-LEG_X, 0, LOWER_CENTER), (-LEG_X, 0, FOOT_CENTER))
make_joint("ankle_r", r_lo, r_ft, (LEG_X, 0, 0.26), (LEG_X, 0, LOWER_CENTER), (LEG_X, 0, FOOT_CENTER))

print("")
print("Foot bottom at Z = 0.24 - 0.015 = 0.225 (above ground)")
print("Press PLAY!")
'''

print("Building simple AT-ST...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(1)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
