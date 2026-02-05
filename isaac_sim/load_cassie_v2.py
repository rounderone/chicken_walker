# Load Cassie using Isaac Lab
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

script = '''
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf

stage = omni.usd.get_context().get_stage()

# Remove only specific paths
paths_to_remove = ["/World/ATST", "/World/Cassie"]
for path in paths_to_remove:
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
        print(f"Removed {path}")

print("")
print("Loading Cassie...")

try:
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.nucleus import get_assets_root_path

    assets_root = get_assets_root_path()
    cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"

    add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")
    print("Loaded Cassie from Isaac assets")

    # Position above ground
    cassie_prim = stage.GetPrimAtPath("/World/Cassie")
    xf = UsdGeom.Xformable(cassie_prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.0))
    print("Positioned at Z=1.0")

except Exception as e:
    print(f"Load error: {e}")

print("")
print("Press PLAY to test Cassie's articulation")
'''

print("Loading Cassie...")
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
