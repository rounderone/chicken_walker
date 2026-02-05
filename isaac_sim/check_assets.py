# Check Nucleus asset paths and Cassie loading
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
from pxr import UsdGeom, Gf, Sdf

stage = omni.usd.get_context().get_stage()

# Check the assets root path
from isaacsim.core.utils.nucleus import get_assets_root_path
assets_root = get_assets_root_path()
print(f"Assets root: {assets_root}")

cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"
print(f"Cassie USD path: {cassie_usd}")

# Check if we can resolve/open this path
import omni.client
result, entry = omni.client.stat(cassie_usd)
print(f"Asset stat result: {result}")
if entry:
    print(f"  Size: {entry.size}")
    print(f"  Flags: {entry.flags}")

# Check the reference on Cassie prim
cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    refs = cassie.GetReferences()
    items = refs.GetAddedOrExplicitItems()
    print(f"\\nCassie references ({len(list(items))} items):")
    for item in refs.GetAddedOrExplicitItems():
        print(f"  Asset: {item.assetPath}")
        print(f"  Prim: {item.primPath}")

# Try to reload Cassie by removing and re-adding
print("\\nRemoving and reloading Cassie...")
stage.RemovePrim("/World/Cassie")

from isaacsim.core.utils.stage import add_reference_to_stage
try:
    add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")
    print("Cassie reloaded")

    # Position it
    cassie = stage.GetPrimAtPath("/World/Cassie")
    if cassie:
        xf = UsdGeom.Xformable(cassie)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.2))

        # Check for meshes now
        mesh_count = 0
        for prim in stage.Traverse():
            if "/World/Cassie" in str(prim.GetPath()) and prim.IsA(UsdGeom.Mesh):
                mesh_count += 1
        print(f"Meshes found after reload: {mesh_count}")

        # Check if there's a payload that needs loading
        if cassie.HasPayload():
            print("Cassie has payload - loading...")
            cassie.Load()
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
'''

print("Checking Cassie asset loading...")
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
