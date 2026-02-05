# Much stronger knee joints for Cassie
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

# First reload fresh
script1 = '''
import omni.usd
omni.usd.get_context().new_stage()
import omni.kit.app
omni.kit.app.get_app().update()
print("Fresh stage")
'''
send_command("execute_script", {"code": script1})
time.sleep(2)

# Setup with MUCH stronger joints
script2 = '''
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf, UsdLux

stage = omni.usd.get_context().get_stage()

# World and ground
UsdGeom.Xform.Define(stage, "/World")
ground = UsdGeom.Cube.Define(stage, "/World/Ground")
ground.CreateSizeAttr(10)
ground.CreateDisplayColorAttr([(0.3, 0.3, 0.3)])
gprim = stage.GetPrimAtPath("/World/Ground")
gxf = UsdGeom.Xformable(gprim)
gxf.AddTranslateOp().Set(Gf.Vec3d(0, 0, -5))
UsdPhysics.CollisionAPI.Apply(gprim)

# Physics
physicsScene = UsdPhysics.Scene.Define(stage, "/physicsScene")
physicsScene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
physicsScene.CreateGravityMagnitudeAttr(9.81)

# Light
UsdLux.DistantLight.Define(stage, "/World/Light").CreateIntensityAttr(3000)

# Load Cassie
print("Loading Cassie...")
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path

assets_root = get_assets_root_path()
cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"
add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")

import omni.kit.app
for i in range(10):
    omni.kit.app.get_app().update()

# Position - feet at ground
cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    xf = UsdGeom.Xformable(cassie)
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(0, 0, 1.15))
            break

# MUCH STRONGER joint settings
# These need to be strong enough to support body weight
joint_config = {
    "hip_abduction": {"stiff": 5000, "damp": 500, "target": 0},
    "hip_rotation": {"stiff": 5000, "damp": 500, "target": 0},
    "hip_flexion": {"stiff": 8000, "damp": 800, "target": 5},
    "knee_joint": {"stiff": 10000, "damp": 1000, "target": -20},    # VERY strong
    "knee_to_shin": {"stiff": 10000, "damp": 1000, "target": -15},  # VERY strong
    "ankle_joint": {"stiff": 12000, "damp": 1200, "target": 45},    # STRONGEST - main support
    "toe_joint": {"stiff": 3000, "damp": 300, "target": -25},
}

print("\\nConfiguring STRONG joints:")
joints_set = 0
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/Cassie" not in path or "Joint" not in prim.GetTypeName():
        continue

    name = prim.GetName().lower()

    if prim.HasAPI(UsdPhysics.DriveAPI):
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if drive:
            # Find matching config
            config = {"stiff": 5000, "damp": 500, "target": 0}
            for key, cfg in joint_config.items():
                if key in name:
                    config = cfg
                    break

            drive.CreateStiffnessAttr(config["stiff"])
            drive.CreateDampingAttr(config["damp"])
            drive.CreateTargetPositionAttr(config["target"])
            joints_set += 1

            if config["target"] != 0:
                print(f"  {name}: stiff={config['stiff']}, target={config['target']}°")

print(f"\\nConfigured {joints_set} joints")
print("Knee/ankle stiffness: 10000-12000 (very strong)")

# Camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util
viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(2.5, -2.5, 1.5), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 0.8), True)

print("")
print("=== READY - Press PLAY ===")
'''

print("Setting up Cassie with STRONG knee joints...")
result = send_command("execute_script", {"code": script2})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
