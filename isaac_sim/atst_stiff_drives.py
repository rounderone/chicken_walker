# AT-ST with very stiff joint drives (like Cassie attempts)
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
import numpy as np
import math
from pxr import Gf, UsdPhysics, UsdGeom

from omni.isaac.core.world import World

print("Creating World...")
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

import omni.usd
stage = omni.usd.get_context().get_stage()

print("Building AT-ST...")

# Create AT-ST root
atst_path = "/World/ATST"
atst = UsdGeom.Xform.Define(stage, atst_path)

# Dimensions - make feet bigger for stability
BODY_Z = 0.8     # Body center height
LEG_X = 0.12     # Leg X offset from center

# VERY HIGH stiffness from Cassie experiments
STIFF = 50000.0
DAMP = 5000.0
MAX_FORCE = 100000.0

def make_body(name, pos, half_size, color, mass, is_artic_root=False):
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

    if is_artic_root:
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        print(f"  ArticulationRoot on {name}")

    return path

# Create body - ArticulationRoot
body = make_body("body", (0, 0, BODY_Z), (0.1, 0.08, 0.06), (0.5, 0.5, 0.6), 5.0, is_artic_root=True)

# Left leg - positioned to stand on ground
# Upper leg hangs from body
l_upper = make_body("l_upper", (-LEG_X, 0, 0.55), (0.025, 0.025, 0.12), (0.4, 0.6, 0.4), 1.0)
# Lower leg - bent backward like Cassie
l_lower = make_body("l_lower", (-LEG_X, 0, 0.32), (0.02, 0.02, 0.1), (0.4, 0.4, 0.7), 1.0)
# Foot - BIG for stability
l_foot = make_body("l_foot", (-LEG_X, 0, 0.05), (0.08, 0.06, 0.03), (0.7, 0.7, 0.3), 2.0)

# Right leg
r_upper = make_body("r_upper", (LEG_X, 0, 0.55), (0.025, 0.025, 0.12), (0.4, 0.6, 0.4), 1.0)
r_lower = make_body("r_lower", (LEG_X, 0, 0.32), (0.02, 0.02, 0.1), (0.4, 0.4, 0.7), 1.0)
r_foot = make_body("r_foot", (LEG_X, 0, 0.05), (0.08, 0.06, 0.03), (0.7, 0.7, 0.3), 2.0)

print("  Created 7 bodies")

# Joints scope
joints_path = atst_path + "/joints"
UsdGeom.Scope.Define(stage, joints_path)

def make_joint(name, parent_path, child_path, parent_anchor, target_rad=0.0):
    path = joints_path + "/" + name
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([parent_path])
    joint.CreateBody1Rel().SetTargets([child_path])
    joint.CreateAxisAttr("X")  # Rotate around X (pitch)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*parent_anchor))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

    prim = stage.GetPrimAtPath(path)
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(STIFF)
    drive.CreateDampingAttr(DAMP)
    drive.CreateMaxForceAttr(MAX_FORCE)
    drive.CreateTargetPositionAttr(target_rad)

    print(f"  {name}: stiff={STIFF}, target={math.degrees(target_rad):.0f} deg")
    return path

# Hip joints - at body bottom connecting to upper legs
# Body bottom at 0.8 - 0.06 = 0.74
# Upper leg center at 0.55, so joint offset = (0.74 - 0.8) = -0.06
make_joint("hip_left", body, l_upper, (-LEG_X, 0, -0.06), target_rad=0.2)  # Slight forward lean
make_joint("hip_right", body, r_upper, (LEG_X, 0, -0.06), target_rad=0.2)

# Knee joints - at upper leg bottom connecting to lower
# Upper leg bottom at 0.55 - 0.12 = 0.43
# Lower leg center at 0.32, offset = (0.43 - 0.55) = -0.12
make_joint("knee_left", l_upper, l_lower, (0, 0, -0.12), target_rad=-0.4)  # Bent backward
make_joint("knee_right", r_upper, r_lower, (0, 0, -0.12), target_rad=-0.4)

# Ankle joints - at lower leg bottom connecting to feet
# Lower leg bottom at 0.32 - 0.1 = 0.22
# Foot center at 0.05, offset = (0.22 - 0.32) = -0.1
make_joint("ankle_left", l_lower, l_foot, (0, 0, -0.1), target_rad=0.3)  # Compensate for knee
make_joint("ankle_right", r_lower, r_foot, (0, 0, -0.1), target_rad=0.3)

print("  Created 6 joints with high stiffness")

# Camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util
viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(1.5, -1.5, 1.0), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 0.5), True)

print("")
print("=== AT-ST Built ===")
print("Foot bottoms at Z = 0.05 - 0.03 = 0.02 (just above ground)")
print("Press PLAY to test standing!")
'''

print("Building AT-ST with stiff drives...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken - Press PLAY in Isaac Sim!")
