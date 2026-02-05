# Create AT-ST using cube size attribute instead of scale
import socket
import json

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
from pxr import UsdGeom, UsdPhysics, UsdShade, Sdf, Gf

stage = omni.usd.get_context().get_stage()

# Clean up
for path in ["/World/PhysicsATST", "/World/TestBox"]:
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
        print(f"Removed {path}")

# Create AT-ST using Mesh boxes instead of Cube primitives
# This gives us more control over size

atst_path = "/World/PhysicsATST"

# Create root xform - this will be our articulation root
root = UsdGeom.Xform.Define(stage, atst_path)

# Position the whole thing at X=-2
root_prim = stage.GetPrimAtPath(atst_path)
xformable = UsdGeom.Xformable(root_prim)
xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(-2.0, 0.0, 0.0))

# Apply articulation to root
UsdPhysics.ArticulationRootAPI.Apply(root_prim)

# Material
mtl_path = atst_path + "/Material"
mtl = UsdShade.Material.Define(stage, mtl_path)
shader = UsdShade.Shader.Define(stage, mtl_path + "/Shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.5, 0.5, 0.55))
mtl.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

# Helper function to create a box mesh
def create_box(path, position, size, mass):
    """Create a box using Cube with proper size"""
    cube = UsdGeom.Cube.Define(stage, path)
    prim = stage.GetPrimAtPath(path)

    # Set size (side length)
    cube.CreateSizeAttr(1.0)  # Unit cube, we'll scale it

    # Set transform
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*position))
    xf.AddScaleOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(*size))

    # Material
    UsdShade.MaterialBindingAPI(prim).Bind(mtl)

    # Physics
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(mass)

    return path

# Dimensions (in meters, all positions relative to root at ground level)
# Body at height 1.0m
# Feet at height 0.1m (above ground)

BODY_Z = 1.0
UPPER_Z = 0.7
LOWER_Z = 0.35
FOOT_Z = 0.1
LEG_X = 0.25

# Create body (the articulation base)
body_path = create_box(
    atst_path + "/body",
    position=(0, 0, BODY_Z),
    size=(0.4, 0.3, 0.25),  # 0.4m wide, 0.3m deep, 0.25m tall
    mass=3.0
)
print(f"Created body at Z={BODY_Z}")

# Left upper leg
l_upper = create_box(
    atst_path + "/l_upper",
    position=(-LEG_X, 0, UPPER_Z),
    size=(0.08, 0.08, 0.3),
    mass=2.0
)

# Left lower leg
l_lower = create_box(
    atst_path + "/l_lower",
    position=(-LEG_X, 0, LOWER_Z),
    size=(0.06, 0.06, 0.25),
    mass=2.0
)

# Left foot
l_foot = create_box(
    atst_path + "/l_foot",
    position=(-LEG_X, 0, FOOT_Z),
    size=(0.25, 0.2, 0.04),  # Wide flat foot
    mass=25.0
)
print(f"Created left leg, foot at Z={FOOT_Z}")

# Right upper leg
r_upper = create_box(
    atst_path + "/r_upper",
    position=(LEG_X, 0, UPPER_Z),
    size=(0.08, 0.08, 0.3),
    mass=2.0
)

# Right lower leg
r_lower = create_box(
    atst_path + "/r_lower",
    position=(LEG_X, 0, LOWER_Z),
    size=(0.06, 0.06, 0.25),
    mass=2.0
)

# Right foot
r_foot = create_box(
    atst_path + "/r_foot",
    position=(LEG_X, 0, FOOT_Z),
    size=(0.25, 0.2, 0.04),
    mass=25.0
)
print(f"Created right leg, foot at Z={FOOT_Z}")

# Joint settings
STIFF = 100000.0  # Very stiff
DAMP = 10000.0

def create_joint(name, parent_path, child_path, parent_anchor, child_anchor):
    """Create a revolute joint with drive"""
    joint_path = atst_path + "/" + name
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([parent_path])
    joint.CreateBody1Rel().SetTargets([child_path])
    joint.CreateAxisAttr("X")  # Rotate around X (pitch)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*parent_anchor))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*child_anchor))

    # Add stiff drive
    drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(joint_path), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(STIFF)
    drive.CreateDampingAttr(DAMP)
    drive.CreateTargetPositionAttr(0.0)

    return joint_path

# Hip joints (body to upper legs)
# Body center at Z=1.0, extends from 0.875 to 1.125
# Upper leg at Z=0.7, extends from 0.55 to 0.85
# Hip joint at Z=0.85 (between them)
HIP_Z = 0.85
create_joint("joint_l_hip", body_path, l_upper,
    parent_anchor=(-LEG_X, 0, HIP_Z - BODY_Z),  # (−0.25, 0, −0.15)
    child_anchor=(0, 0, HIP_Z - UPPER_Z))       # (0, 0, 0.15)

create_joint("joint_r_hip", body_path, r_upper,
    parent_anchor=(LEG_X, 0, HIP_Z - BODY_Z),
    child_anchor=(0, 0, HIP_Z - UPPER_Z))

# Knee joints (upper to lower legs)
# Upper extends from 0.55 to 0.85, lower from 0.225 to 0.475
# Knee at Z=0.52
KNEE_Z = 0.52
create_joint("joint_l_knee", l_upper, l_lower,
    parent_anchor=(0, 0, KNEE_Z - UPPER_Z),
    child_anchor=(0, 0, KNEE_Z - LOWER_Z))

create_joint("joint_r_knee", r_upper, r_lower,
    parent_anchor=(0, 0, KNEE_Z - UPPER_Z),
    child_anchor=(0, 0, KNEE_Z - LOWER_Z))

# Ankle joints (lower legs to feet)
# Lower extends from 0.225 to 0.475, foot from 0.08 to 0.12
# Ankle at Z=0.2
ANKLE_Z = 0.2
create_joint("joint_l_ankle", l_lower, l_foot,
    parent_anchor=(0, 0, ANKLE_Z - LOWER_Z),
    child_anchor=(0, 0, ANKLE_Z - FOOT_Z))

create_joint("joint_r_ankle", r_lower, r_foot,
    parent_anchor=(0, 0, ANKLE_Z - LOWER_Z),
    child_anchor=(0, 0, ANKLE_Z - FOOT_Z))

print("Created 6 joints with stiffness=100000")

print("")
print("=== AT-ST Ready ===")
print("Position: X=-2 (left side)")
print("Feet at Z=0.1 (above ground)")
print("Press PLAY to test!")
'''

print("Creating AT-ST v2...")
result = send_command("execute_script", {"code": script})

if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")
    if result.get('traceback'):
        print(result.get('traceback')[:500])
