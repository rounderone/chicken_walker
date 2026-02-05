# Deep reverse-knee bend for Cassie
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

# Fresh stage
script1 = '''
import omni.usd
omni.usd.get_context().new_stage()
import omni.kit.app
omni.kit.app.get_app().update()
'''
send_command("execute_script", {"code": script1})
time.sleep(2)

# Setup with DEEP knee bend
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

# Position - a bit higher since knees will bend more
cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    xf = UsdGeom.Xformable(cassie)
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(0, 0, 0.9))  # Lower since deep bend
            break

# DEEP knee bend - reverse knees way back
joint_config = {
    "hip_abduction": {"stiff": 8000, "damp": 800, "target": 0},
    "hip_rotation": {"stiff": 8000, "damp": 800, "target": 0},
    "hip_flexion": {"stiff": 10000, "damp": 1000, "target": 15},      # More forward lean
    "knee_joint": {"stiff": 15000, "damp": 1500, "target": -40},      # Deeper bend
    "knee_to_shin": {"stiff": 15000, "damp": 1500, "target": -30},    # Deeper
    "ankle_joint": {"stiff": 20000, "damp": 2000, "target": 90},      # WAY BACK - 90 degrees!
    "toe_joint": {"stiff": 5000, "damp": 500, "target": -40},         # Compensate
}

print("\\nDEEP reverse-knee bend:")
joints_set = 0
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/Cassie" not in path or "Joint" not in prim.GetTypeName():
        continue

    name = prim.GetName().lower()

    if prim.HasAPI(UsdPhysics.DriveAPI):
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if drive:
            config = {"stiff": 8000, "damp": 800, "target": 0}
            for key, cfg in joint_config.items():
                if key in name:
                    config = cfg
                    break

            drive.CreateStiffnessAttr(config["stiff"])
            drive.CreateDampingAttr(config["damp"])
            drive.CreateTargetPositionAttr(config["target"])
            joints_set += 1

            if config["target"] != 0:
                print(f"  {name}: {config['target']} degrees")

print(f"\\nAnkle (reverse knee): 90 degrees back!")
print("Knee: -40 degrees")

# Camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util
viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(2.0, -2.0, 1.0), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 0.5), True)

print("")
print("=== Press PLAY ===")
'''

print("Setting up Cassie with DEEP knee bend...")
result = send_command("execute_script", {"code": script2})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
