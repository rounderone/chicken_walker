# Cassie with PROPER Isaac Lab settings
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

# Fresh stage
send_command("execute_script", {"code": '''
import omni.usd
omni.usd.get_context().new_stage()
import omni.kit.app
omni.kit.app.get_app().update()
'''})
time.sleep(2)

# Apply PROPER Isaac Lab settings
script = '''
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf, UsdLux
import math

stage = omni.usd.get_context().get_stage()

# Scene setup
UsdGeom.Xform.Define(stage, "/World")
ground = UsdGeom.Cube.Define(stage, "/World/Ground")
ground.CreateSizeAttr(10)
ground.CreateDisplayColorAttr([(0.3, 0.3, 0.3)])
gprim = stage.GetPrimAtPath("/World/Ground")
UsdGeom.Xformable(gprim).AddTranslateOp().Set(Gf.Vec3d(0, 0, -5))
UsdPhysics.CollisionAPI.Apply(gprim)

physicsScene = UsdPhysics.Scene.Define(stage, "/physicsScene")
physicsScene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
physicsScene.CreateGravityMagnitudeAttr(9.81)

UsdLux.DistantLight.Define(stage, "/World/Light").CreateIntensityAttr(3000)

# Load Cassie
print("Loading Cassie with Isaac Lab settings...")
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path

assets_root = get_assets_root_path()
cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"
add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")

import omni.kit.app
for i in range(10):
    omni.kit.app.get_app().update()

# Position at Z=0.9 (Isaac Lab default)
cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    xf = UsdGeom.Xformable(cassie)
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(0, 0, 0.9))
            print("Position: Z=0.9 (Isaac Lab default)")
            break

# Isaac Lab joint settings (values in RADIANS for initial position)
# These are the INITIAL positions the joints should START at
isaac_lab_config = {
    "hip_abduction_left": {"init": 0.1, "stiff": 100.0, "damp": 3.0},
    "hip_rotation_left": {"init": 0.0, "stiff": 100.0, "damp": 3.0},
    "hip_flexion_left": {"init": 1.0, "stiff": 200.0, "damp": 6.0},
    "knee_joint_left": {"init": -1.8, "stiff": 200.0, "damp": 6.0},    # Deep bend!
    "knee_to_shin_left": {"init": -1.8, "stiff": 200.0, "damp": 6.0},
    "ankle_joint_left": {"init": 1.57, "stiff": 200.0, "damp": 6.0},   # 90 degrees
    "toe_joint_left": {"init": -1.57, "stiff": 20.0, "damp": 1.0},     # -90 degrees
    "hip_abduction_right": {"init": -0.1, "stiff": 100.0, "damp": 3.0},
    "hip_rotation_right": {"init": 0.0, "stiff": 100.0, "damp": 3.0},
    "hip_flexion_right": {"init": 1.0, "stiff": 200.0, "damp": 6.0},
    "knee_joint_right": {"init": -1.8, "stiff": 200.0, "damp": 6.0},
    "knee_to_shin_right": {"init": -1.8, "stiff": 200.0, "damp": 6.0},
    "ankle_joint_right": {"init": 1.57, "stiff": 200.0, "damp": 6.0},
    "toe_joint_right": {"init": -1.57, "stiff": 20.0, "damp": 1.0},
}

print("\\nApplying Isaac Lab joint settings:")
print("  (stiffness 100-200, damping 3-6)")
print("")

for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/Cassie" not in path or "Joint" not in prim.GetTypeName():
        continue

    name = prim.GetName()

    if name in isaac_lab_config:
        cfg = isaac_lab_config[name]

        if prim.HasAPI(UsdPhysics.DriveAPI):
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drive:
                # Set stiffness and damping
                drive.CreateStiffnessAttr(cfg["stiff"])
                drive.CreateDampingAttr(cfg["damp"])
                # Target = initial position (in degrees for USD)
                target_deg = math.degrees(cfg["init"])
                drive.CreateTargetPositionAttr(target_deg)

                if abs(cfg["init"]) > 0.01:
                    print(f"  {name}: init={cfg['init']:.2f} rad ({target_deg:.0f}°)")

print("")
print("=== Isaac Lab settings applied ===")
print("Press PLAY!")

# Camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util
viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(2.0, -2.0, 1.2), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 0.5), True)
'''

print("Applying proper Isaac Lab settings...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
