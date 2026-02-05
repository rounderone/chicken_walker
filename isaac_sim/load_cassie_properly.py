# Load Cassie using Isaac Lab's proper API
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

# Clean up
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/ATST" in path or "/World/Cassie" in path:
        stage.RemovePrim(path)

print("Loading Cassie robot...")

# Try Isaac Sim's asset loading
try:
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.nucleus import get_assets_root_path

    assets_root = get_assets_root_path()
    print(f"Assets root: {assets_root}")

    cassie_path = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"
    print(f"Cassie USD: {cassie_path}")

    # Add Cassie to stage
    add_reference_to_stage(usd_path=cassie_path, prim_path="/World/Cassie")
    print("Cassie added to stage")

    # Position it above ground
    cassie_prim = stage.GetPrimAtPath("/World/Cassie")
    if cassie_prim:
        xf = UsdGeom.Xformable(cassie_prim)
        # Clear existing transforms and set new position
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.0))
        print("Positioned Cassie at Z=1.0")

        # Check if it has articulation
        has_artic = cassie_prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        print(f"Has ArticulationRootAPI: {has_artic}")

        # List joints
        joint_count = 0
        for child in stage.Traverse():
            cpath = str(child.GetPath())
            if "/World/Cassie" in cpath and child.GetTypeName() == "PhysicsRevoluteJoint":
                joint_count += 1
        print(f"Found {joint_count} revolute joints")

except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()

print("")
print("Press PLAY - Cassie should drop and land on its joints")
'''

print("Loading Cassie robot...")
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
