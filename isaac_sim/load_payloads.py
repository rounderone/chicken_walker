# Load Cassie payloads to get geometry
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
from pxr import UsdGeom, Gf, Usd

stage = omni.usd.get_context().get_stage()

cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    print("Checking Cassie for payloads...")

    # Load all payloads under Cassie
    payload_count = 0
    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        if "/World/Cassie" in path_str:
            if prim.HasPayload():
                print(f"  Loading payload: {path_str}")
                prim.Load()
                payload_count += 1

    print(f"Loaded {payload_count} payloads")

    # Force stage update
    import omni.kit.app
    omni.kit.app.get_app().update()

    # Recount meshes
    mesh_count = 0
    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        if "/World/Cassie" in path_str and prim.IsA(UsdGeom.Mesh):
            mesh_count += 1
    print(f"Meshes after payload load: {mesh_count}")

    # Check if pelvis has children now
    pelvis = stage.GetPrimAtPath("/World/Cassie/pelvis")
    if pelvis:
        children = list(pelvis.GetChildren())
        print(f"\\nPelvis has {len(children)} children")
        for c in children[:5]:
            print(f"  {c.GetPath()} - {c.GetTypeName()}")

    # Try loading the whole stage's payloads
    print("\\nLoading ALL stage payloads...")
    stage.Load()

    import omni.kit.app
    omni.kit.app.get_app().update()

    mesh_count = 0
    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        if "/World/Cassie" in path_str and prim.IsA(UsdGeom.Mesh):
            mesh_count += 1
    print(f"Meshes after full load: {mesh_count}")

# Position Cassie
cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    xf = UsdGeom.Xformable(cassie)
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(0, 0, 1.2))
            break
'''

print("Loading Cassie payloads...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(3)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
