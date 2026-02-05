# Fix AT-ST - raise it above ground
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

# First check current bounding box
print("Checking current AT-ST position...")
result = send_command("get_prim_info", {"prim_path": "/World/PhysicsATST"})
if result.get("status") == "success":
    bbox = result.get("result", {}).get("bbox", {})
    print(f"Current bbox: min Z={bbox.get('min', [0,0,0])[2]:.3f}, max Z={bbox.get('max', [0,0,0])[2]:.3f}")

# Recreate with higher position
script = '''
import omni.usd
from pxr import UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Usd

stage = omni.usd.get_context().get_stage()

# Delete old
if stage.GetPrimAtPath("/World/PhysicsATST"):
    stage.RemovePrim("/World/PhysicsATST")
    print("Removed old PhysicsATST")

atst_path = "/World/PhysicsATST"
root = UsdGeom.Xform.Define(stage, atst_path)
# Start at X=-2, and raise the whole thing by 0.5m
root.AddTranslateOp().Set(Gf.Vec3d(-2.0, 0.0, 0.5))

# Material
mtl = UsdShade.Material.Define(stage, atst_path + "/GrayMtl")
shader = UsdShade.Shader.Define(stage, atst_path + "/GrayMtl/Shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.5, 0.5, 0.55))
mtl.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

# Positions (relative to root which is at Z=0.5)
# So absolute Z = these values + 0.5
FOOT_Z = 0.05       # absolute: 0.55
ANKLE_Z = 0.10
LOWER_Z = 0.30
KNEE_Z = 0.50
UPPER_Z = 0.70
HIP_Z = 0.90
BODY_Z = 1.05       # absolute: 1.55

LEG_X = 0.30

STIFFNESS = 50000.0  # Even stiffer
DAMPING = 5000.0

# === BODY ===
body_path = atst_path + "/body"
body = UsdGeom.Cube.Define(stage, body_path)
body.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, BODY_Z))
body.AddScaleOp().Set(Gf.Vec3f(0.25, 0.2, 0.15))
UsdShade.MaterialBindingAPI(body).Bind(mtl)
UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(body_path))
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(body_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(body_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(body_path))
mass_api.CreateMassAttr(2.0)

# === LEFT LEG ===
l_upper_path = atst_path + "/left_upper"
l_upper = UsdGeom.Cube.Define(stage, l_upper_path)
l_upper.AddTranslateOp().Set(Gf.Vec3d(-LEG_X, 0.0, UPPER_Z))
l_upper.AddScaleOp().Set(Gf.Vec3f(0.04, 0.04, 0.18))
UsdShade.MaterialBindingAPI(l_upper).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(l_upper_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(l_upper_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(l_upper_path))
mass_api.CreateMassAttr(2.0)

l_hip = UsdPhysics.RevoluteJoint.Define(stage, atst_path + "/joint_l_hip")
l_hip.CreateBody0Rel().SetTargets([body_path])
l_hip.CreateBody1Rel().SetTargets([l_upper_path])
l_hip.CreateAxisAttr("X")
l_hip.CreateLocalPos0Attr().Set(Gf.Vec3f(-LEG_X, 0.0, HIP_Z - BODY_Z))
l_hip.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, HIP_Z - UPPER_Z))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(atst_path + "/joint_l_hip"), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(STIFFNESS)
drive.CreateDampingAttr(DAMPING)
drive.CreateTargetPositionAttr(0.0)

l_lower_path = atst_path + "/left_lower"
l_lower = UsdGeom.Cube.Define(stage, l_lower_path)
l_lower.AddTranslateOp().Set(Gf.Vec3d(-LEG_X, 0.0, LOWER_Z))
l_lower.AddScaleOp().Set(Gf.Vec3f(0.035, 0.035, 0.18))
UsdShade.MaterialBindingAPI(l_lower).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(l_lower_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(l_lower_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(l_lower_path))
mass_api.CreateMassAttr(2.0)

l_knee = UsdPhysics.RevoluteJoint.Define(stage, atst_path + "/joint_l_knee")
l_knee.CreateBody0Rel().SetTargets([l_upper_path])
l_knee.CreateBody1Rel().SetTargets([l_lower_path])
l_knee.CreateAxisAttr("X")
l_knee.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, KNEE_Z - UPPER_Z))
l_knee.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, KNEE_Z - LOWER_Z))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(atst_path + "/joint_l_knee"), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(STIFFNESS)
drive.CreateDampingAttr(DAMPING)
drive.CreateTargetPositionAttr(0.0)

l_foot_path = atst_path + "/left_foot"
l_foot = UsdGeom.Cube.Define(stage, l_foot_path)
l_foot.AddTranslateOp().Set(Gf.Vec3d(-LEG_X, 0.0, FOOT_Z))
l_foot.AddScaleOp().Set(Gf.Vec3f(0.15, 0.12, 0.025))
UsdShade.MaterialBindingAPI(l_foot).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(l_foot_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(l_foot_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(l_foot_path))
mass_api.CreateMassAttr(30.0)

l_ankle = UsdPhysics.RevoluteJoint.Define(stage, atst_path + "/joint_l_ankle")
l_ankle.CreateBody0Rel().SetTargets([l_lower_path])
l_ankle.CreateBody1Rel().SetTargets([l_foot_path])
l_ankle.CreateAxisAttr("X")
l_ankle.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, ANKLE_Z - LOWER_Z))
l_ankle.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, ANKLE_Z - FOOT_Z))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(atst_path + "/joint_l_ankle"), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(STIFFNESS)
drive.CreateDampingAttr(DAMPING)
drive.CreateTargetPositionAttr(0.0)

# === RIGHT LEG ===
r_upper_path = atst_path + "/right_upper"
r_upper = UsdGeom.Cube.Define(stage, r_upper_path)
r_upper.AddTranslateOp().Set(Gf.Vec3d(LEG_X, 0.0, UPPER_Z))
r_upper.AddScaleOp().Set(Gf.Vec3f(0.04, 0.04, 0.18))
UsdShade.MaterialBindingAPI(r_upper).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(r_upper_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(r_upper_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(r_upper_path))
mass_api.CreateMassAttr(2.0)

r_hip = UsdPhysics.RevoluteJoint.Define(stage, atst_path + "/joint_r_hip")
r_hip.CreateBody0Rel().SetTargets([body_path])
r_hip.CreateBody1Rel().SetTargets([r_upper_path])
r_hip.CreateAxisAttr("X")
r_hip.CreateLocalPos0Attr().Set(Gf.Vec3f(LEG_X, 0.0, HIP_Z - BODY_Z))
r_hip.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, HIP_Z - UPPER_Z))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(atst_path + "/joint_r_hip"), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(STIFFNESS)
drive.CreateDampingAttr(DAMPING)
drive.CreateTargetPositionAttr(0.0)

r_lower_path = atst_path + "/right_lower"
r_lower = UsdGeom.Cube.Define(stage, r_lower_path)
r_lower.AddTranslateOp().Set(Gf.Vec3d(LEG_X, 0.0, LOWER_Z))
r_lower.AddScaleOp().Set(Gf.Vec3f(0.035, 0.035, 0.18))
UsdShade.MaterialBindingAPI(r_lower).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(r_lower_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(r_lower_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(r_lower_path))
mass_api.CreateMassAttr(2.0)

r_knee = UsdPhysics.RevoluteJoint.Define(stage, atst_path + "/joint_r_knee")
r_knee.CreateBody0Rel().SetTargets([r_upper_path])
r_knee.CreateBody1Rel().SetTargets([r_lower_path])
r_knee.CreateAxisAttr("X")
r_knee.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, KNEE_Z - UPPER_Z))
r_knee.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, KNEE_Z - LOWER_Z))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(atst_path + "/joint_r_knee"), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(STIFFNESS)
drive.CreateDampingAttr(DAMPING)
drive.CreateTargetPositionAttr(0.0)

r_foot_path = atst_path + "/right_foot"
r_foot = UsdGeom.Cube.Define(stage, r_foot_path)
r_foot.AddTranslateOp().Set(Gf.Vec3d(LEG_X, 0.0, FOOT_Z))
r_foot.AddScaleOp().Set(Gf.Vec3f(0.15, 0.12, 0.025))
UsdShade.MaterialBindingAPI(r_foot).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(r_foot_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(r_foot_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(r_foot_path))
mass_api.CreateMassAttr(30.0)

r_ankle = UsdPhysics.RevoluteJoint.Define(stage, atst_path + "/joint_r_ankle")
r_ankle.CreateBody0Rel().SetTargets([r_lower_path])
r_ankle.CreateBody1Rel().SetTargets([r_foot_path])
r_ankle.CreateAxisAttr("X")
r_ankle.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, ANKLE_Z - LOWER_Z))
r_ankle.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, ANKLE_Z - FOOT_Z))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(atst_path + "/joint_r_ankle"), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(STIFFNESS)
drive.CreateDampingAttr(DAMPING)
drive.CreateTargetPositionAttr(0.0)

# Check bounding box
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
atst_prim = stage.GetPrimAtPath(atst_path)
bound = cache.ComputeWorldBound(atst_prim)
mn = bound.GetRange().GetMin()
mx = bound.GetRange().GetMax()
print(f"AT-ST bounding box:")
print(f"  Min: X={mn[0]:.2f}, Y={mn[1]:.2f}, Z={mn[2]:.2f}")
print(f"  Max: X={mx[0]:.2f}, Y={mx[1]:.2f}, Z={mx[2]:.2f}")
print(f"  Height: {mx[2]-mn[2]:.2f}m")
print("")
print("Feet should be at Z~0.55 (above ground)")
print("Press PLAY - AT-ST will drop ~0.5m and land on feet")
'''

print("\nRecreating AT-ST raised above ground...")
result = send_command("execute_script", {"code": script})

if result.get("status") == "success":
    print("Success!")
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")
    if result.get('traceback'):
        print(result.get('traceback'))
