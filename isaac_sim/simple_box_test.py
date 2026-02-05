# Simple test - just one box to verify physics works
import socket
import json

def send_command(command_type, params=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(60)
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
from pxr import UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Usd

stage = omni.usd.get_context().get_stage()

# Delete old AT-ST
if stage.GetPrimAtPath("/World/PhysicsATST"):
    stage.RemovePrim("/World/PhysicsATST")
    print("Removed old PhysicsATST")

# Create a simple test box
test_path = "/World/TestBox"
if stage.GetPrimAtPath(test_path):
    stage.RemovePrim(test_path)

# Simple cube at X=-2, Z=1 (1 meter above ground)
box = UsdGeom.Cube.Define(stage, test_path)
box.AddTranslateOp().Set(Gf.Vec3d(-2.0, 0.0, 1.0))
box.AddScaleOp().Set(Gf.Vec3f(0.2, 0.2, 0.2))  # 0.4m cube

# Add physics
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(test_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(test_path))

# Check position
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
prim = stage.GetPrimAtPath(test_path)
bound = cache.ComputeWorldBound(prim)
mn = bound.GetRange().GetMin()
mx = bound.GetRange().GetMax()
print(f"TestBox at: X={mn[0]:.2f} to {mx[0]:.2f}, Z={mn[2]:.2f} to {mx[2]:.2f}")
print("Should drop and land on ground when you press PLAY")
'''

print("Creating simple test box...")
result = send_command("execute_script", {"code": script})

if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")
