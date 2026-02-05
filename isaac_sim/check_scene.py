# Check what's in the scene now
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

print("Scene contents:")
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if path.count("/") <= 3:  # Only show top levels
        type_name = prim.GetTypeName()
        print(f"  {path} - {type_name}")

        # Get world position for geometry
        if prim.IsA(UsdGeom.Xformable):
            xf = UsdGeom.Xformable(prim)
            world_xform = xf.ComputeLocalToWorldTransform(0)
            pos = world_xform.ExtractTranslation()
            print(f"    World pos: {pos}")

# Check Cassie specifically
cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    print("\\nCassie details:")

    # Get transform
    xf = UsdGeom.Xformable(cassie)
    for op in xf.GetOrderedXformOps():
        print(f"  Op: {op.GetOpName()} = {op.Get()}")

    # Count children by type
    meshes = 0
    xforms = 0
    for child in stage.Traverse():
        if "/World/Cassie" in str(child.GetPath()):
            if child.IsA(UsdGeom.Mesh):
                meshes += 1
            elif child.GetTypeName() == "Xform":
                xforms += 1
    print(f"  Xforms: {xforms}, Meshes: {meshes}")

    # Reposition Cassie correctly
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(0, 0, 1.0))
            print("  Set translate to (0, 0, 1.0)")
            break
else:
    print("No Cassie found")

# Position camera closer
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util

viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(2.0, -2.0, 1.5), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 0.5), True)
    print("\\nCamera repositioned closer")
'''

print("Checking scene...")
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
