# Clean rebuild - delete everything and start fresh
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

# Delete EVERYTHING under /World except the ground plane infrastructure
print("Cleaning up...")
world = stage.GetPrimAtPath("/World")
if world:
    to_delete = []
    for child in world.GetChildren():
        path = str(child.GetPath())
        # Keep only essential ground plane stuff
        if "defaultGroundPlane" not in path and "Physics_Mat" not in path:
            to_delete.append(path)

    for path in to_delete:
        stage.RemovePrim(path)
        print(f"  Deleted {path}")

print("")

# Create a simple ground with collision
ground_path = "/World/Ground"
ground = UsdGeom.Cube.Define(stage, ground_path)
ground_prim = stage.GetPrimAtPath(ground_path)

# Use SetTypedValue to ensure clean transform
xformable = UsdGeom.Xformable(ground_prim)
translate_op = xformable.AddTranslateOp()
translate_op.Set(Gf.Vec3d(0, 0, -0.25))
scale_op = xformable.AddScaleOp()
scale_op.Set(Gf.Vec3f(25, 25, 0.25))

ground.CreateDisplayColorAttr([(0.2, 0.2, 0.25)])
UsdPhysics.CollisionAPI.Apply(ground_prim)
print("Created ground (top at Z=0)")

# Create a simple test cube first to verify transforms work
cube_path = "/World/Cube"
cube = UsdGeom.Cube.Define(stage, cube_path)
cube_prim = stage.GetPrimAtPath(cube_path)

xf = UsdGeom.Xformable(cube_prim)
xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.5))
xf.AddScaleOp().Set(Gf.Vec3f(0.2, 0.2, 0.2))
cube.CreateDisplayColorAttr([(1, 0, 0)])

UsdPhysics.RigidBodyAPI.Apply(cube_prim)
UsdPhysics.CollisionAPI.Apply(cube_prim)

# Verify the position
ops = xf.GetOrderedXformOps()
for op in ops:
    if "translate" in op.GetOpName():
        pos = op.Get()
        print(f"Cube position: {pos}")

print("")
print("Created red test cube at (0, 0, 0.5)")
print("Press PLAY to verify collision works")
'''

print("Clean rebuild...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(1)

# Take screenshot
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
