# Check and fix ground collision
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
from pxr import UsdGeom, UsdPhysics, Gf, PhysxSchema

stage = omni.usd.get_context().get_stage()

# Check ground plane collision
ground_paths = [
    "/World/defaultGroundPlane",
    "/World/defaultGroundPlane/GroundPlane",
    "/World/defaultGroundPlane/GroundPlane/CollisionPlane",
]

print("Checking ground collision...")
for path in ground_paths:
    prim = stage.GetPrimAtPath(path)
    if prim:
        has_collision = prim.HasAPI(UsdPhysics.CollisionAPI)
        print(f"  {path}: collision={has_collision}")

        # If it's the collision plane, make sure it has collision
        if "CollisionPlane" in path and not has_collision:
            UsdPhysics.CollisionAPI.Apply(prim)
            print(f"    -> Added CollisionAPI")

# Check physics scene
physics_scene = stage.GetPrimAtPath("/physicsScene")
if physics_scene:
    print("Physics scene exists")
else:
    print("No physics scene - creating one")
    UsdPhysics.Scene.Define(stage, "/physicsScene")

# Reset the test cube position (in case it fell)
cube = stage.GetPrimAtPath("/World/TestCube")
if cube:
    # Remove and recreate to reset physics state
    stage.RemovePrim("/World/TestCube")

# Create new test cube
cube_path = "/World/TestCube"
cube = UsdGeom.Cube.Define(stage, cube_path)
cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.0))  # 1m above ground
cube.AddScaleOp().Set(Gf.Vec3f(0.2, 0.2, 0.2))
cube.CreateDisplayColorAttr([(1.0, 0.2, 0.2)])

cube_prim = stage.GetPrimAtPath(cube_path)
UsdPhysics.RigidBodyAPI.Apply(cube_prim)
UsdPhysics.CollisionAPI.Apply(cube_prim)

# Add mass
mass = UsdPhysics.MassAPI.Apply(cube_prim)
mass.CreateMassAttr(1.0)

print("\\nRecreated test cube at Z=1.0")
print("Press STOP then PLAY to test again")
'''

print("Checking ground collision...")
result = send_command("execute_script", {"code": script})

if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")
