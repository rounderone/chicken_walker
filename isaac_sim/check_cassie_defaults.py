# Check Cassie's default/original joint settings and look for policies
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

# Load Cassie and check its ORIGINAL default settings
script2 = '''
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf, UsdLux

stage = omni.usd.get_context().get_stage()

# Basic scene
UsdGeom.Xform.Define(stage, "/World")
ground = UsdGeom.Cube.Define(stage, "/World/Ground")
ground.CreateSizeAttr(10)
gprim = stage.GetPrimAtPath("/World/Ground")
UsdGeom.Xformable(gprim).AddTranslateOp().Set(Gf.Vec3d(0, 0, -5))
UsdPhysics.CollisionAPI.Apply(gprim)

physicsScene = UsdPhysics.Scene.Define(stage, "/physicsScene")
physicsScene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
physicsScene.CreateGravityMagnitudeAttr(9.81)

UsdLux.DistantLight.Define(stage, "/World/Light").CreateIntensityAttr(3000)

# Load Cassie WITHOUT modifying anything
print("Loading Cassie with ORIGINAL settings...")
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path

assets_root = get_assets_root_path()
cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"
add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")

import omni.kit.app
for i in range(10):
    omni.kit.app.get_app().update()

# Position only
cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    xf = UsdGeom.Xformable(cassie)
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(0, 0, 1.15))
            break

# Read the ORIGINAL joint settings from the USD
print("\\n=== CASSIE'S ORIGINAL JOINT SETTINGS ===\\n")

for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/Cassie" not in path or "Joint" not in prim.GetTypeName():
        continue

    name = prim.GetName()

    if prim.HasAPI(UsdPhysics.DriveAPI):
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if drive:
            stiff = drive.GetStiffnessAttr().Get()
            damp = drive.GetDampingAttr().Get()
            target = drive.GetTargetPositionAttr().Get()

            print(f"{name}:")
            print(f"  stiffness: {stiff}")
            print(f"  damping: {damp}")
            print(f"  target: {target}")
            print()

# Check for Isaac Lab examples
print("\\n=== Checking for Isaac Lab Cassie examples ===")
try:
    import isaaclab
    print(f"Isaac Lab found: {isaaclab.__path__}")
except:
    print("Isaac Lab not imported as 'isaaclab'")

try:
    from omni.isaac.lab.envs import ManagerBasedRLEnv
    print("Isaac Lab RL environments available")
except:
    print("Isaac Lab RL environments not available in this context")

# Check for pre-trained policies
try:
    import omni.isaac.lab_tasks
    print("Isaac Lab tasks available")
except:
    pass

print("\\n=== Press PLAY to test original settings ===")
'''

print("Checking Cassie's original default settings...")
result = send_command("execute_script", {"code": script2})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
