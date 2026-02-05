# AT-ST simulation test - run physics and track what happens
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

atst_path = "/World/ATST"
atst = UsdGeom.Xform.Define(stage, atst_path)

BODY_Z = 0.6
LEG_X = 0.10

# Even higher stiffness
STIFF = 100000.0
DAMP = 10000.0
MAX_FORCE = 500000.0

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

    return path

# Body - heavier to keep center of mass low
body = make_body("body", (0, 0, BODY_Z), (0.08, 0.06, 0.05), (0.5, 0.5, 0.6), 10.0, is_artic_root=True)

# Legs - lighter
l_upper = make_body("l_upper", (-LEG_X, 0, 0.42), (0.02, 0.02, 0.1), (0.4, 0.6, 0.4), 0.5)
l_lower = make_body("l_lower", (-LEG_X, 0, 0.22), (0.015, 0.015, 0.1), (0.4, 0.4, 0.7), 0.5)
l_foot = make_body("l_foot", (-LEG_X, 0, 0.04), (0.06, 0.05, 0.02), (0.7, 0.7, 0.3), 1.0)

r_upper = make_body("r_upper", (LEG_X, 0, 0.42), (0.02, 0.02, 0.1), (0.4, 0.6, 0.4), 0.5)
r_lower = make_body("r_lower", (LEG_X, 0, 0.22), (0.015, 0.015, 0.1), (0.4, 0.4, 0.7), 0.5)
r_foot = make_body("r_foot", (LEG_X, 0, 0.04), (0.06, 0.05, 0.02), (0.7, 0.7, 0.3), 1.0)

joints_path = atst_path + "/joints"
UsdGeom.Scope.Define(stage, joints_path)

def make_joint(name, parent_path, child_path, parent_anchor, target_rad=0.0):
    path = joints_path + "/" + name
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([parent_path])
    joint.CreateBody1Rel().SetTargets([child_path])
    joint.CreateAxisAttr("X")
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*parent_anchor))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

    prim = stage.GetPrimAtPath(path)
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(STIFF)
    drive.CreateDampingAttr(DAMP)
    drive.CreateMaxForceAttr(MAX_FORCE)
    drive.CreateTargetPositionAttr(target_rad)
    return path

# Body bottom = 0.6 - 0.05 = 0.55
# Upper leg center = 0.42
make_joint("hip_left", body, l_upper, (-LEG_X, 0, -0.05), target_rad=0.0)
make_joint("hip_right", body, r_upper, (LEG_X, 0, -0.05), target_rad=0.0)

# Upper leg bottom = 0.42 - 0.1 = 0.32, lower center = 0.22
make_joint("knee_left", l_upper, l_lower, (0, 0, -0.1), target_rad=0.0)
make_joint("knee_right", r_upper, r_lower, (0, 0, -0.1), target_rad=0.0)

# Lower leg bottom = 0.22 - 0.1 = 0.12, foot center = 0.04
make_joint("ankle_left", l_lower, l_foot, (0, 0, -0.1), target_rad=0.0)
make_joint("ankle_right", r_lower, r_foot, (0, 0, -0.1), target_rad=0.0)

print("AT-ST built with VERY stiff joints")
print(f"Stiffness: {STIFF}, Damping: {DAMP}, MaxForce: {MAX_FORCE}")

# Reset world to initialize physics
world.reset()

# Run simulation
print("Running simulation...")
for step in range(300):
    world.step(render=True)

    if step % 60 == 0:
        body_prim = stage.GetPrimAtPath(atst_path + "/body")
        if body_prim:
            xf = UsdGeom.Xformable(body_prim)
            world_tf = xf.ComputeLocalToWorldTransform(0)
            body_z = world_tf.GetRow(3)[2]
            print(f"  Step {step}: body_z={body_z:.2f}")

# Camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util
viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(1.5, -1.5, 0.8), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 0.3), True)

print("Simulation done")
'''

print("Running AT-ST simulation...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
