# Debug why Cassie isn't visible
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
if not cassie:
    print("ERROR: Cassie not found!")
else:
    print("Cassie exists")

    # Check visibility
    imageable = UsdGeom.Imageable(cassie)
    vis = imageable.ComputeVisibility()
    print(f"Visibility: {vis}")

    # Check if it's a reference and if it loaded
    refs = cassie.GetReferences()
    print(f"Has references: {bool(refs)}")

    # Get world position
    xf = UsdGeom.Xformable(cassie)
    world_transform = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = world_transform.ExtractTranslation()
    print(f"World position: {pos}")

    # Check the pelvis (main body)
    pelvis = stage.GetPrimAtPath("/World/Cassie/pelvis")
    if pelvis:
        print("Pelvis found")
        pelvis_xf = UsdGeom.Xformable(pelvis)
        pelvis_world = pelvis_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pelvis_pos = pelvis_world.ExtractTranslation()
        print(f"Pelvis world pos: {pelvis_pos}")

        # Check if pelvis has geometry
        for child in pelvis.GetChildren():
            if child.IsA(UsdGeom.Mesh) or child.IsA(UsdGeom.Gprim):
                print(f"  Has geometry: {child.GetPath()}")
                break
    else:
        print("No pelvis found")

    # List a few mesh prims under Cassie
    print("\\nMeshes under Cassie:")
    count = 0
    for prim in stage.Traverse():
        if "/World/Cassie" in str(prim.GetPath()) and prim.IsA(UsdGeom.Mesh):
            print(f"  {prim.GetPath()}")
            count += 1
            if count >= 5:
                print("  ...")
                break
    if count == 0:
        print("  NO MESHES FOUND!")

# Also add a debug cube at origin to verify camera is pointing correctly
cube_path = "/World/DebugCube"
if not stage.GetPrimAtPath(cube_path):
    cube = UsdGeom.Cube.Define(stage, cube_path)
    cube.CreateSizeAttr(0.5)
    cube.CreateDisplayColorAttr([(1, 0, 0)])
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(cube_path))
    xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.25))
    print("\\nAdded red debug cube at origin")
'''

print("Debugging Cassie...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(1)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
