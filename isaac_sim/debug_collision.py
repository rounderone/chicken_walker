# Debug collision setup
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
from pxr import UsdGeom, UsdPhysics, Gf

stage = omni.usd.get_context().get_stage()

# List ALL prims with collision
print("All prims with CollisionAPI:")
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        print(f"  {prim.GetPath()}")

# List ALL physics scenes
print("\\nAll PhysicsScene prims:")
for prim in stage.Traverse():
    if prim.GetTypeName() == "PhysicsScene":
        print(f"  {prim.GetPath()}")

# Check if ground plane collision plane exists and examine it
collision_path = "/World/defaultGroundPlane/GroundPlane/CollisionPlane"
prim = stage.GetPrimAtPath(collision_path)
if prim:
    print(f"\\nCollisionPlane exists: {prim.GetTypeName()}")
    print(f"  Has CollisionAPI: {prim.HasAPI(UsdPhysics.CollisionAPI)}")

    # Get all applied APIs
    schemas = prim.GetAppliedSchemas()
    print(f"  Applied schemas: {schemas}")
else:
    print(f"\\nCollisionPlane NOT found at {collision_path}")

# Let's create a proper ground plane with collision
print("\\nCreating new ground plane with guaranteed collision...")

# Remove old test cube
if stage.GetPrimAtPath("/World/TestCube"):
    stage.RemovePrim("/World/TestCube")

# Create a simple ground collision plane
ground_path = "/World/MyGround"
if stage.GetPrimAtPath(ground_path):
    stage.RemovePrim(ground_path)

# Create a large flat cube as ground
ground = UsdGeom.Cube.Define(stage, ground_path)
ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))  # Top surface at Z=0
ground.AddScaleOp().Set(Gf.Vec3f(50, 50, 0.5))  # 100x100m, 1m thick
ground.CreateDisplayColorAttr([(0.3, 0.3, 0.35)])

ground_prim = stage.GetPrimAtPath(ground_path)
UsdPhysics.CollisionAPI.Apply(ground_prim)
print("Created ground cube with collision at Z=0")

# Create test cube above it
cube_path = "/World/TestCube"
cube = UsdGeom.Cube.Define(stage, cube_path)
cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.5))
cube.AddScaleOp().Set(Gf.Vec3f(0.2, 0.2, 0.2))
cube.CreateDisplayColorAttr([(1.0, 0.2, 0.2)])

cube_prim = stage.GetPrimAtPath(cube_path)
UsdPhysics.RigidBodyAPI.Apply(cube_prim)
UsdPhysics.CollisionAPI.Apply(cube_prim)
mass = UsdPhysics.MassAPI.Apply(cube_prim)
mass.CreateMassAttr(1.0)

print("Created test cube at Z=1.5")
print("\\nPress PLAY - cube should land on gray ground")
'''

print("Debugging collision...")
result = send_command("execute_script", {"code": script})

if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")
