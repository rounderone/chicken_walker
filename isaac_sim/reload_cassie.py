# Reload Cassie and check asset loading
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
from pxr import UsdGeom, Gf

stage = omni.usd.get_context().get_stage()

# Get assets root
from isaacsim.core.utils.nucleus import get_assets_root_path
assets_root = get_assets_root_path()
print(f"Assets root: {assets_root}")

if assets_root is None:
    print("ERROR: Nucleus not connected! Assets root is None.")
    print("Please connect to Nucleus in Isaac Sim.")
else:
    cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"
    print(f"Cassie path: {cassie_usd}")

    # Remove old
    if stage.GetPrimAtPath("/World/Cassie"):
        stage.RemovePrim("/World/Cassie")
    if stage.GetPrimAtPath("/World/DebugCube"):
        stage.RemovePrim("/World/DebugCube")

    print("\\nLoading Cassie...")
    from isaacsim.core.utils.stage import add_reference_to_stage
    add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")

    # Wait for stage to update
    import omni.kit.app
    omni.kit.app.get_app().update()

    cassie = stage.GetPrimAtPath("/World/Cassie")
    if cassie:
        # Position
        xf = UsdGeom.Xformable(cassie)
        for op in xf.GetOrderedXformOps():
            if "translate" in op.GetOpName():
                op.Set(Gf.Vec3d(0, 0, 1.2))
                break
        else:
            xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.2))
        print("Positioned at Z=1.2")

        # Count meshes
        mesh_count = 0
        for prim in stage.Traverse():
            path_str = str(prim.GetPath())
            if "/World/Cassie" in path_str and prim.IsA(UsdGeom.Mesh):
                mesh_count += 1
                if mesh_count <= 3:
                    print(f"  Mesh: {path_str}")

        print(f"Total meshes: {mesh_count}")

        if mesh_count == 0:
            # Try checking what children exist
            print("\\nCassie children:")
            for child in cassie.GetAllChildren():
                print(f"  {child.GetPath()} - {child.GetTypeName()}")
                if len(list(cassie.GetAllChildren())) > 10:
                    break
'''

print("Reloading Cassie...")
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
