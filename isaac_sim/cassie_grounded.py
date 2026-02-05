# Cassie starting with feet on ground - no drop
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

# First reload Cassie fresh
script1 = '''
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf
import math

stage = omni.usd.get_context().get_stage()

# Remove and reload Cassie
if stage.GetPrimAtPath("/World/Cassie"):
    stage.RemovePrim("/World/Cassie")

print("Reloading Cassie...")
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path

assets_root = get_assets_root_path()
cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"
add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")

# Wait for load
import omni.kit.app
for i in range(5):
    omni.kit.app.get_app().update()

cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    # Cassie's toe Z position when at origin is about -1.1
    # So set Z to 1.15 to have feet just touching ground
    xf = UsdGeom.Xformable(cassie)
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(0, 0, 1.15))  # Feet just touching ground
            break

    print("Cassie positioned with feet at ground level")

    # Set joint drives with moderate settings
    # Key: enough stiffness to hold pose, enough damping to absorb shocks
    STIFFNESS = 1000.0
    DAMPING = 200.0

    joints_configured = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "/World/Cassie" not in path:
            continue

        if "Joint" not in prim.GetTypeName():
            continue

        # Get or create drive
        if not prim.HasAPI(UsdPhysics.DriveAPI):
            UsdPhysics.DriveAPI.Apply(prim, "angular")

        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if drive:
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(STIFFNESS)
            drive.CreateDampingAttr(DAMPING)
            # Target = current position (hold pose)
            drive.CreateTargetPositionAttr(0.0)
            joints_configured += 1

    print(f"Configured {joints_configured} joints")
    print(f"Stiffness: {STIFFNESS}, Damping: {DAMPING}")
    print("")
    print("Feet should be touching ground - press PLAY!")
    print("Cassie should stand in place without falling")
'''

print("Setting up Cassie at ground level...")
result = send_command("execute_script", {"code": script1})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
