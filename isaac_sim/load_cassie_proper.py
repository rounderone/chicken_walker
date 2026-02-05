# Load Cassie using Isaac Sim's proper robot loading
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

# Clean up
for path in ["/World/Cassie", "/World/DebugCube"]:
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)

# Try using Isaac Sim's Robot class
print("Trying to load Cassie via isaacsim Robot class...")
try:
    from isaacsim.core.robots import Robot
    from isaacsim.core.utils.nucleus import get_assets_root_path

    assets_root = get_assets_root_path()
    cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"

    # Create robot instance
    robot = Robot(
        prim_path="/World/Cassie",
        usd_path=cassie_usd,
        name="Cassie",
        position=[0, 0, 1.2]
    )

    print(f"Robot created: {robot}")
    print(f"Prim path: {robot.prim_path}")

    # Check if meshes loaded
    mesh_count = 0
    for prim in stage.Traverse():
        if "/World/Cassie" in str(prim.GetPath()) and prim.IsA(UsdGeom.Mesh):
            mesh_count += 1
    print(f"Meshes: {mesh_count}")

except Exception as e:
    print(f"Robot class failed: {e}")

    # Try ArticulationView approach
    print("\\nTrying ArticulationView...")
    try:
        from isaacsim.core.articulations import Articulation
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.nucleus import get_assets_root_path

        assets_root = get_assets_root_path()
        cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"

        add_reference_to_stage(cassie_usd, "/World/Cassie")

        # Try initializing as articulation
        art = Articulation(prim_path="/World/Cassie")
        print(f"Articulation: {art}")

        mesh_count = 0
        for prim in stage.Traverse():
            if "/World/Cassie" in str(prim.GetPath()) and prim.IsA(UsdGeom.Mesh):
                mesh_count += 1
        print(f"Meshes: {mesh_count}")

    except Exception as e2:
        print(f"Articulation also failed: {e2}")

# Check what we have
cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    print(f"\\nCassie exists, children: {len(list(cassie.GetAllChildren()))}")

# Try a simpler robot - Ant
print("\\n--- Trying simpler Ant robot ---")
try:
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.nucleus import get_assets_root_path

    assets_root = get_assets_root_path()
    ant_usd = assets_root + "/Isaac/Robots/Ant/ant.usd"

    add_reference_to_stage(ant_usd, "/World/Ant")

    ant = stage.GetPrimAtPath("/World/Ant")
    if ant:
        xf = UsdGeom.Xformable(ant)
        xf.AddTranslateOp().Set(Gf.Vec3d(1, 0, 0.5))

        mesh_count = 0
        for prim in stage.Traverse():
            if "/World/Ant" in str(prim.GetPath()) and prim.IsA(UsdGeom.Mesh):
                mesh_count += 1
        print(f"Ant meshes: {mesh_count}")

except Exception as e:
    print(f"Ant failed: {e}")
'''

print("Loading robots...")
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
