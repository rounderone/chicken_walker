# Setup scene with proper camera view
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
import omni.kit.viewport.utility as vp_util

stage = omni.usd.get_context().get_stage()

# Clean up
for path in ["/World/Pendulum", "/World/PhysicsATST", "/World/TestBox"]:
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)

# Create a simple colored cube at origin, above ground
cube_path = "/World/TestCube"
cube = UsdGeom.Cube.Define(stage, cube_path)
cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.5))  # 0.5m above ground
cube.AddScaleOp().Set(Gf.Vec3f(0.3, 0.3, 0.3))  # 0.6m cube
cube.CreateDisplayColorAttr([(1.0, 0.2, 0.2)])  # Red color

# Add physics
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(cube_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(cube_path))

print("Created red test cube at (0, 0, 0.5)")

# Try to set camera view
try:
    viewport = vp_util.get_active_viewport()
    if viewport:
        # Set camera to look at the cube from a good angle
        from omni.kit.viewport.utility import frame_viewport_selection
        # This should frame the view on selected objects
        print("Viewport found")
except Exception as e:
    print(f"Viewport error: {e}")

# Use omni.kit.commands to frame all
try:
    import omni.kit.commands
    omni.kit.commands.execute("FrameAllCommand")
    print("Framed all objects")
except Exception as e:
    print(f"Frame command error: {e}")

print("Scene ready - you should see a red cube")
'''

print("Setting up test scene...")
result = send_command("execute_script", {"code": script})

if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

# Wait a moment for the viewport to update
time.sleep(1)

# Take screenshot
print("\nTaking screenshot...")
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print(f"Screenshot: {result.get('status')}")
