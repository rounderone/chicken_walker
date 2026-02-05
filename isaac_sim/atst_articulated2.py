# AT-ST with articulated joints - proper hierarchy
import socket
import json
import time
import math

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

# Clear first
send_command("execute_script", {"code": '''
from omni.isaac.core.world import World
try:
    World.clear_instance()
except:
    pass
import omni.usd
omni.usd.get_context().new_stage()
import omni.kit.app
omni.kit.app.get_app().update()
'''})
time.sleep(2)

script = '''
import math
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf

from omni.isaac.core.world import World

print("Creating World...")
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

stage = omni.usd.get_context().get_stage()

print("Building articulated AT-ST v2...")

# Dimensions
FOOT_HALF = 0.025
LOWER_HALF = 0.12
UPPER_HALF = 0.12
BODY_HALF = 0.08
LEG_X = 0.15

# Vertical positions
FOOT_Z = FOOT_HALF + 0.01
ANKLE_Z = FOOT_Z + FOOT_HALF
LOWER_Z = ANKLE_Z + LOWER_HALF
KNEE_Z = LOWER_Z + LOWER_HALF
UPPER_Z = KNEE_Z + UPPER_HALF
HIP_Z = UPPER_Z + UPPER_HALF
BODY_Z = HIP_Z + BODY_HALF

# VERY HIGH stiffness
STIFFNESS = 100000.0
DAMPING = 10000.0
MAX_FORCE = 500000.0

# Create articulation root xform
atst_path = "/World/ATST"
atst = UsdGeom.Xform.Define(stage, atst_path)
atst_prim = stage.GetPrimAtPath(atst_path)
UsdPhysics.ArticulationRootAPI.Apply(atst_prim)
print("ArticulationRoot on /World/ATST")

def make_body(name, pos, half_size, color, mass):
    path = atst_path + "/" + name
    cube = UsdGeom.Cube.Define(stage, path)
    prim = stage.GetPrimAtPath(path)

    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*half_size))
    cube.CreateDisplayColorAttr([color])

    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    return path

# All bodies as direct children of articulation root
body = make_body("body", (0, 0, BODY_Z), (0.12, 0.10, BODY_HALF), (0.5, 0.5, 0.6), 10.0)
l_upper = make_body("l_upper", (-LEG_X, 0, UPPER_Z), (0.025, 0.025, UPPER_HALF), (0.4, 0.6, 0.4), 1.5)
l_lower = make_body("l_lower", (-LEG_X, 0, LOWER_Z), (0.02, 0.02, LOWER_HALF), (0.4, 0.4, 0.7), 1.0)
l_foot = make_body("l_foot", (-LEG_X, 0, FOOT_Z), (0.08, 0.06, FOOT_HALF), (0.7, 0.7, 0.3), 3.0)
r_upper = make_body("r_upper", (LEG_X, 0, UPPER_Z), (0.025, 0.025, UPPER_HALF), (0.4, 0.6, 0.4), 1.5)
r_lower = make_body("r_lower", (LEG_X, 0, LOWER_Z), (0.02, 0.02, LOWER_HALF), (0.4, 0.4, 0.7), 1.0)
r_foot = make_body("r_foot", (LEG_X, 0, FOOT_Z), (0.08, 0.06, FOOT_HALF), (0.7, 0.7, 0.3), 3.0)

print("Created 7 rigid bodies")

# Joints as children of articulation root (not in separate scope)
def make_joint(name, parent_path, child_path, world_pos, axis="X"):
    """Create joint at world position"""
    path = atst_path + "/" + name
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([parent_path])
    joint.CreateBody1Rel().SetTargets([child_path])
    joint.CreateAxisAttr(axis)

    # Calculate local positions from world position
    # Parent local pos = world_pos - parent_center
    # Child local pos = world_pos - child_center
    parent_prim = stage.GetPrimAtPath(parent_path)
    child_prim = stage.GetPrimAtPath(child_path)

    parent_xf = UsdGeom.Xformable(parent_prim)
    child_xf = UsdGeom.Xformable(child_prim)

    # Get translations
    parent_pos = None
    child_pos = None
    for op in parent_xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            parent_pos = op.Get()
            break
    for op in child_xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            child_pos = op.Get()
            break

    local0 = (world_pos[0] - parent_pos[0], world_pos[1] - parent_pos[1], world_pos[2] - parent_pos[2])
    local1 = (world_pos[0] - child_pos[0], world_pos[1] - child_pos[1], world_pos[2] - child_pos[2])

    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*local1))

    # Joint limits
    joint.CreateLowerLimitAttr(-30.0)
    joint.CreateUpperLimitAttr(30.0)

    # Drive
    prim = stage.GetPrimAtPath(path)
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(STIFFNESS)
    drive.CreateDampingAttr(DAMPING)
    drive.CreateMaxForceAttr(MAX_FORCE)
    drive.CreateTargetPositionAttr(0.0)

    print(f"  {name} at Z={world_pos[2]:.3f}")
    return path

# Joint positions (where bodies connect)
make_joint("hip_L", body, l_upper, (-LEG_X, 0, HIP_Z))
make_joint("hip_R", body, r_upper, (LEG_X, 0, HIP_Z))
make_joint("knee_L", l_upper, l_lower, (-LEG_X, 0, KNEE_Z))
make_joint("knee_R", r_upper, r_lower, (LEG_X, 0, KNEE_Z))
make_joint("ankle_L", l_lower, l_foot, (-LEG_X, 0, ANKLE_Z))
make_joint("ankle_R", r_lower, r_foot, (LEG_X, 0, ANKLE_Z))

print(f"Created 6 joints (stiffness={STIFFNESS})")

# Camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util
viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(1.5, -1.5, 0.8), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 0.4), True)

print("")
print("=== ARTICULATED AT-ST v2 ===")
print("Press PLAY!")
'''

print("Building articulated AT-ST v2...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken - press PLAY!")
