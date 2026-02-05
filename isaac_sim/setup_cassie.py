# Load Cassie robot and set it up properly
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

# Remove AT-ST
if stage.GetPrimAtPath("/World/ATST"):
    stage.RemovePrim("/World/ATST")

# Remove old Cassie if exists
if stage.GetPrimAtPath("/World/Cassie"):
    stage.RemovePrim("/World/Cassie")

print("Loading Cassie robot...")

try:
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.nucleus import get_assets_root_path

    assets_root = get_assets_root_path()
    cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"

    add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")
    print("Cassie loaded!")

    # Check Cassie's structure
    cassie = stage.GetPrimAtPath("/World/Cassie")
    if cassie:
        # Find articulation root
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if "/World/Cassie" in path:
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    print(f"ArticulationRoot: {path}")
                    break

        # Count joints
        joint_count = 0
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if "/World/Cassie" in path and "Joint" in prim.GetTypeName():
                joint_count += 1
        print(f"Joints: {joint_count}")

        # Position Cassie above ground
        # Cassie's pelvis is the root, need to move the whole reference
        cassie_xf = UsdGeom.Xformable(cassie)
        ops = cassie_xf.GetOrderedXformOps()

        # Modify existing translate op or add new one
        found_translate = False
        for op in ops:
            if "translate" in op.GetOpName():
                op.Set(Gf.Vec3d(0, 0, 1.0))  # Raise above ground
                found_translate = True
                print("Positioned Cassie at Z=1.0")
                break

        if not found_translate:
            cassie_xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.0))
            print("Added position at Z=1.0")

except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()

print("")
print("Press PLAY - Cassie should drop and balance on its legs!")
'''

print("Setting up Cassie...")
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
