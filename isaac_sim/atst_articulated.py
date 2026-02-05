# AT-ST with articulated joints - legs can move!
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

print("Building articulated AT-ST...")

# Dimensions (same as working single body)
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

# Joint drive settings - high stiffness to maintain pose
STIFFNESS = 10000.0
DAMPING = 1000.0
MAX_FORCE = 50000.0

# Create root xform
atst = UsdGeom.Xform.Define(stage, "/World/ATST")

def make_body(name, pos, half_size, color, mass, is_root=False):
    path = "/World/ATST/" + name
    cube = UsdGeom.Cube.Define(stage, path)
    prim = stage.GetPrimAtPath(path)

    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*half_size))
    cube.CreateDisplayColorAttr([color])

    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)

    if is_root:
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        print(f"  ArticulationRoot on {name}")

    return path

# Body (articulation root) - heavy for stability
body = make_body("body", (0, 0, BODY_Z), (0.12, 0.10, BODY_HALF), (0.5, 0.5, 0.6), 10.0, is_root=True)

# Left leg
l_upper = make_body("l_upper", (-LEG_X, 0, UPPER_Z), (0.025, 0.025, UPPER_HALF), (0.4, 0.6, 0.4), 1.5)
l_lower = make_body("l_lower", (-LEG_X, 0, LOWER_Z), (0.02, 0.02, LOWER_HALF), (0.4, 0.4, 0.7), 1.0)
l_foot = make_body("l_foot", (-LEG_X, 0, FOOT_Z), (0.08, 0.06, FOOT_HALF), (0.7, 0.7, 0.3), 3.0)

# Right leg
r_upper = make_body("r_upper", (LEG_X, 0, UPPER_Z), (0.025, 0.025, UPPER_HALF), (0.4, 0.6, 0.4), 1.5)
r_lower = make_body("r_lower", (LEG_X, 0, LOWER_Z), (0.02, 0.02, LOWER_HALF), (0.4, 0.4, 0.7), 1.0)
r_foot = make_body("r_foot", (LEG_X, 0, FOOT_Z), (0.08, 0.06, FOOT_HALF), (0.7, 0.7, 0.3), 3.0)

print("Created 7 rigid bodies")

# Create joints scope
joints = UsdGeom.Scope.Define(stage, "/World/ATST/joints")

def make_revolute_joint(name, parent_path, child_path, parent_offset, child_offset, axis="X", target_deg=0.0):
    """Create revolute joint with drive at target position"""
    path = "/World/ATST/joints/" + name
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([parent_path])
    joint.CreateBody1Rel().SetTargets([child_path])
    joint.CreateAxisAttr(axis)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*parent_offset))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*child_offset))

    # Set joint limits (prevent over-rotation)
    joint.CreateLowerLimitAttr(-45.0)
    joint.CreateUpperLimitAttr(45.0)

    # Add drive with high stiffness
    prim = stage.GetPrimAtPath(path)
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(STIFFNESS)
    drive.CreateDampingAttr(DAMPING)
    drive.CreateMaxForceAttr(MAX_FORCE)
    drive.CreateTargetPositionAttr(math.radians(target_deg))

    print(f"  {name}: target={target_deg} deg")
    return path

# Hip joints (body to upper leg) - rotate around X (pitch)
make_revolute_joint("hip_L", body, l_upper, (-LEG_X, 0, -BODY_HALF), (0, 0, UPPER_HALF), "X", 0)
make_revolute_joint("hip_R", body, r_upper, (LEG_X, 0, -BODY_HALF), (0, 0, UPPER_HALF), "X", 0)

# Knee joints (upper to lower leg) - rotate around X (pitch)
make_revolute_joint("knee_L", l_upper, l_lower, (0, 0, -UPPER_HALF), (0, 0, LOWER_HALF), "X", 0)
make_revolute_joint("knee_R", r_upper, r_lower, (0, 0, -UPPER_HALF), (0, 0, LOWER_HALF), "X", 0)

# Ankle joints (lower leg to foot) - rotate around X (pitch)
make_revolute_joint("ankle_L", l_lower, l_foot, (0, 0, -LOWER_HALF), (0, 0, FOOT_HALF), "X", 0)
make_revolute_joint("ankle_R", r_lower, r_foot, (0, 0, -LOWER_HALF), (0, 0, FOOT_HALF), "X", 0)

print("Created 6 revolute joints with drives")

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
print("=== ARTICULATED AT-ST ===")
print(f"Joint drives: stiffness={STIFFNESS}, damping={DAMPING}")
print("Press PLAY - legs should hold position!")
'''

print("Building articulated AT-ST...")
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
