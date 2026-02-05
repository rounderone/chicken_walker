# AT-ST as a SINGLE rigid body with compound collider
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

print("Building AT-ST as SINGLE rigid body...")

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

# Create ONE rigid body parent
atst_path = "/World/ATST"
atst = UsdGeom.Xform.Define(stage, atst_path)
atst_prim = stage.GetPrimAtPath(atst_path)

# Apply physics to the parent - this makes it ONE rigid body
UsdPhysics.RigidBodyAPI.Apply(atst_prim)
UsdPhysics.MassAPI.Apply(atst_prim).CreateMassAttr(15.0)  # Total mass

# Helper to add collision shape as child (no separate rigid body)
def add_collision_shape(name, pos, half_size, color):
    path = atst_path + "/" + name
    cube = UsdGeom.Cube.Define(stage, path)
    prim = stage.GetPrimAtPath(path)

    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*half_size))
    cube.CreateDisplayColorAttr([color])

    # Only collision - NO RigidBodyAPI (parent has it)
    UsdPhysics.CollisionAPI.Apply(prim)
    return path

# Add all parts as collision shapes under the single rigid body
# Body
add_collision_shape("body", (0, 0, BODY_Z), (0.12, 0.10, BODY_HALF), (0.5, 0.5, 0.6))

# Left leg
add_collision_shape("l_upper", (-LEG_X, 0, UPPER_Z), (0.025, 0.025, UPPER_HALF), (0.4, 0.6, 0.4))
add_collision_shape("l_lower", (-LEG_X, 0, LOWER_Z), (0.02, 0.02, LOWER_HALF), (0.4, 0.4, 0.7))
add_collision_shape("l_foot", (-LEG_X, 0, FOOT_Z), (0.08, 0.06, FOOT_HALF), (0.7, 0.7, 0.3))

# Right leg
add_collision_shape("r_upper", (LEG_X, 0, UPPER_Z), (0.025, 0.025, UPPER_HALF), (0.4, 0.6, 0.4))
add_collision_shape("r_lower", (LEG_X, 0, LOWER_Z), (0.02, 0.02, LOWER_HALF), (0.4, 0.4, 0.7))
add_collision_shape("r_foot", (LEG_X, 0, FOOT_Z), (0.08, 0.06, FOOT_HALF), (0.7, 0.7, 0.3))

print("Created 7 collision shapes under 1 rigid body")

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
print("=== AT-ST as SINGLE RIGID BODY ===")
print("All parts are one object - cannot fall apart!")
print(f"Body at Z={BODY_Z:.2f}, feet at Z={FOOT_Z:.3f}")
print("Press PLAY to test!")
'''

print("Building AT-ST as single rigid body...")
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
