# Load and run the H1 Humanoid walking demo with pre-trained policy
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

print("Loading H1 Humanoid walking demo...")

# First, let's try to load the example via the extension
script = '''
import omni.kit.app
import asyncio

# Try to find and load the humanoid policy example
try:
    # Method 1: Try loading via example browser extension
    import omni.isaac.examples.browser as browser
    print("Found example browser extension")
except Exception as e:
    print(f"Example browser not available: {e}")

# Method 2: Try loading the H1 robot directly with policy
try:
    from omni.isaac.core.utils.extensions import enable_extension
    enable_extension("omni.isaac.robot_policy_examples")
    print("Enabled robot_policy_examples extension")
except Exception as e:
    print(f"Could not enable extension: {e}")

# Method 3: List available extensions to find the right one
import omni.kit.app
ext_manager = omni.kit.app.get_app().get_extension_manager()

# Find policy-related extensions
policy_exts = []
for ext in ext_manager.get_extensions():
    name = ext.get("name", "")
    if "policy" in name.lower() or "humanoid" in name.lower() or "h1" in name.lower():
        policy_exts.append(name)

print(f"Found {len(policy_exts)} potentially relevant extensions:")
for ext in policy_exts[:10]:
    print(f"  - {ext}")
'''

result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

print("\n--- Trying to load H1 directly ---")

# Try loading H1 from Nucleus assets
script2 = '''
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf
from omni.isaac.core.world import World
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Clear and create new world
try:
    World.clear_instance()
except:
    pass

omni.usd.get_context().new_stage()
import omni.kit.app
omni.kit.app.get_app().update()

print("Creating world...")
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Get assets path
assets_root = get_assets_root_path()
print(f"Assets root: {assets_root}")

# Try to find H1 robot
if assets_root:
    h1_path = f"{assets_root}/Isaac/Robots/Unitree/H1/h1.usd"
    print(f"Looking for H1 at: {h1_path}")

    # Try to load the H1 robot
    try:
        from omni.isaac.core.robots import Robot
        h1 = world.scene.add(
            Robot(
                prim_path="/World/H1",
                usd_path=h1_path,
                name="h1",
                position=[0, 0, 1.05],
            )
        )
        print("H1 robot loaded!")
    except Exception as e:
        print(f"Failed to load H1: {e}")

        # Try alternative path
        alt_paths = [
            f"{assets_root}/Isaac/Robots/Humanoid/H1/h1.usd",
            f"{assets_root}/Isaac/Robots/Unitree/h1.usd",
        ]
        for alt in alt_paths:
            print(f"  Trying: {alt}")
else:
    print("Could not get assets root path - Nucleus may not be connected")

# Set camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util
viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(3, -3, 2), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 1), True)

print("\\nDone! Check if H1 loaded in the viewport.")
'''

result = send_command("execute_script", {"code": script2})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
