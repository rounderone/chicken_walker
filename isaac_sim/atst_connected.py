# AT-ST with properly connected geometry
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
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf

from omni.isaac.core.world import World

print("Creating World...")
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

stage = omni.usd.get_context().get_stage()

print("Building AT-ST with connected geometry...")

# Calculate positions so parts TOUCH:
# Start from ground up
FOOT_HALF = 0.025    # Foot half-height
LOWER_HALF = 0.12    # Lower leg half-height
UPPER_HALF = 0.12    # Upper leg half-height
BODY_HALF = 0.08     # Body half-height
LEG_X = 0.15         # Leg spread

# Vertical positions (parts stacked touching each other)
FOOT_Z = FOOT_HALF + 0.01  # Small gap above ground
ANKLE_Z = FOOT_Z + FOOT_HALF  # Where foot top meets lower leg bottom
LOWER_Z = ANKLE_Z + LOWER_HALF
KNEE_Z = LOWER_Z + LOWER_HALF  # Where lower leg top meets upper leg bottom
UPPER_Z = KNEE_Z + UPPER_HALF
HIP_Z = UPPER_Z + UPPER_HALF  # Where upper leg top meets body bottom
BODY_Z = HIP_Z + BODY_HALF

print(f"  Foot center: {FOOT_Z:.3f}, top: {ANKLE_Z:.3f}")
print(f"  Lower leg center: {LOWER_Z:.3f}, top: {KNEE_Z:.3f}")
print(f"  Upper leg center: {UPPER_Z:.3f}, top: {HIP_Z:.3f}")
print(f"  Body center: {BODY_Z:.3f}")

# Create root xform
atst = UsdGeom.Xform.Define(stage, "/World/ATST")

def make_part(name, pos, half_size, color, mass, is_root=False):
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
    return path

# Body (heavy, articulation root)
body = make_part("body", (0, 0, BODY_Z), (0.12, 0.10, BODY_HALF), (0.5, 0.5, 0.6), 8.0, is_root=True)

# Left leg
l_upper = make_part("l_upper", (-LEG_X, 0, UPPER_Z), (0.025, 0.025, UPPER_HALF), (0.4, 0.6, 0.4), 1.5)
l_lower = make_part("l_lower", (-LEG_X, 0, LOWER_Z), (0.02, 0.02, LOWER_HALF), (0.4, 0.4, 0.7), 1.0)
l_foot = make_part("l_foot", (-LEG_X, 0, FOOT_Z), (0.08, 0.06, FOOT_HALF), (0.7, 0.7, 0.3), 3.0)

# Right leg
r_upper = make_part("r_upper", (LEG_X, 0, UPPER_Z), (0.025, 0.025, UPPER_HALF), (0.4, 0.6, 0.4), 1.5)
r_lower = make_part("r_lower", (LEG_X, 0, LOWER_Z), (0.02, 0.02, LOWER_HALF), (0.4, 0.4, 0.7), 1.0)
r_foot = make_part("r_foot", (LEG_X, 0, FOOT_Z), (0.08, 0.06, FOOT_HALF), (0.7, 0.7, 0.3), 3.0)

print("Created 7 parts")

# Create FIXED joints at connection points
joints = UsdGeom.Scope.Define(stage, "/World/ATST/joints")

def make_fixed_joint(name, parent_path, child_path, parent_offset, child_offset):
    path = "/World/ATST/joints/" + name
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([parent_path])
    joint.CreateBody1Rel().SetTargets([child_path])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*parent_offset))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*child_offset))
    return path

# Hip joints: body bottom to upper leg top
make_fixed_joint("hip_L", body, l_upper, (-LEG_X, 0, -BODY_HALF), (0, 0, UPPER_HALF))
make_fixed_joint("hip_R", body, r_upper, (LEG_X, 0, -BODY_HALF), (0, 0, UPPER_HALF))

# Knee joints: upper leg bottom to lower leg top
make_fixed_joint("knee_L", l_upper, l_lower, (0, 0, -UPPER_HALF), (0, 0, LOWER_HALF))
make_fixed_joint("knee_R", r_upper, r_lower, (0, 0, -UPPER_HALF), (0, 0, LOWER_HALF))

# Ankle joints: lower leg bottom to foot top
make_fixed_joint("ankle_L", l_lower, l_foot, (0, 0, -LOWER_HALF), (0, 0, FOOT_HALF))
make_fixed_joint("ankle_R", r_lower, r_foot, (0, 0, -LOWER_HALF), (0, 0, FOOT_HALF))

print("Created 6 fixed joints")

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
print("=== AT-ST with FIXED joints ===")
print(f"Body at Z={BODY_Z:.2f}, feet bottom at Z={FOOT_Z-FOOT_HALF:.3f}")
print("All parts are connected - press PLAY!")
'''

print("Building AT-ST with connected geometry...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken - press PLAY to test!")
