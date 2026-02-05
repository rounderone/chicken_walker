# Create a stable physics AT-ST with properly spaced parts
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

# Delete old PhysicsATST
if stage.GetPrimAtPath("/World/PhysicsATST"):
    stage.RemovePrim("/World/PhysicsATST")
    print("Removed old PhysicsATST")

# Create new AT-ST at X=-2.0, away from Cassie
atst_path = "/World/PhysicsATST"
root = UsdGeom.Xform.Define(stage, atst_path)
root.AddTranslateOp().Set(Gf.Vec3d(-2.0, 0.0, 0.0))  # Ground level, we'll position parts above

# Gray material
mtl = UsdShade.Material.Define(stage, atst_path + "/GrayMtl")
shader = UsdShade.Shader.Define(stage, atst_path + "/GrayMtl/Shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.5, 0.5, 0.55))
mtl.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

# === MEASUREMENTS ===
# Body (cockpit): 0.6m wide, 0.5m deep, 0.4m tall, centered at height 1.0m
# Upper legs: 0.16m thick, 0.5m long, angled outward
# Lower legs: 0.12m thick, 0.4m long
# Feet: 0.4m x 0.3m x 0.06m (wide flat feet)
# Total height: ~1.3m from ground to top of body

# Height values (absolute Z from ground)
FOOT_Z = 0.03       # Feet sit just above ground
ANKLE_Z = 0.06      # Ankle joint height
LOWER_Z = 0.25      # Lower leg center
KNEE_Z = 0.45       # Knee joint height
UPPER_Z = 0.65      # Upper leg center
HIP_Z = 0.85        # Hip joint height
BODY_Z = 1.0        # Body center height

# Leg spread
LEG_X = 0.35        # Distance from center to leg

# === BODY (cockpit) - the root rigid body ===
body_path = atst_path + "/body"
body = UsdGeom.Cube.Define(stage, body_path)
body.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, BODY_Z))
body.AddScaleOp().Set(Gf.Vec3f(0.3, 0.25, 0.2))  # 0.6 x 0.5 x 0.4
UsdShade.MaterialBindingAPI(body).Bind(mtl)

# Add articulation root to body
UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(body_path))
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(body_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(body_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(body_path))
mass_api.CreateMassAttr(5.0)  # Light body

print("Created body with ArticulationRoot")

# Joint settings - VERY stiff to hold position
STIFFNESS = 10000.0
DAMPING = 1000.0

# === LEFT LEG ===
# Upper leg
l_upper_path = atst_path + "/left_upper"
l_upper = UsdGeom.Cube.Define(stage, l_upper_path)
l_upper.AddTranslateOp().Set(Gf.Vec3d(-LEG_X, 0.0, UPPER_Z))
l_upper.AddScaleOp().Set(Gf.Vec3f(0.06, 0.06, 0.2))  # Thin cylinder-like
UsdShade.MaterialBindingAPI(l_upper).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(l_upper_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(l_upper_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(l_upper_path))
mass_api.CreateMassAttr(3.0)

# Left hip joint - connects body to upper leg
l_hip_path = atst_path + "/joint_left_hip"
l_hip = UsdPhysics.RevoluteJoint.Define(stage, l_hip_path)
l_hip.CreateBody0Rel().SetTargets([body_path])
l_hip.CreateBody1Rel().SetTargets([l_upper_path])
l_hip.CreateAxisAttr("X")
# Joint anchors in local coordinates of each body
l_hip.CreateLocalPos0Attr().Set(Gf.Vec3f(-LEG_X, 0.0, HIP_Z - BODY_Z))  # On body
l_hip.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, HIP_Z - UPPER_Z))   # On upper leg
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(l_hip_path), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(STIFFNESS)
drive.CreateDampingAttr(DAMPING)
drive.CreateTargetPositionAttr(0.0)

# Lower leg
l_lower_path = atst_path + "/left_lower"
l_lower = UsdGeom.Cube.Define(stage, l_lower_path)
l_lower.AddTranslateOp().Set(Gf.Vec3d(-LEG_X, 0.0, LOWER_Z))
l_lower.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.15))
UsdShade.MaterialBindingAPI(l_lower).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(l_lower_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(l_lower_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(l_lower_path))
mass_api.CreateMassAttr(3.0)

# Left knee joint
l_knee_path = atst_path + "/joint_left_knee"
l_knee = UsdPhysics.RevoluteJoint.Define(stage, l_knee_path)
l_knee.CreateBody0Rel().SetTargets([l_upper_path])
l_knee.CreateBody1Rel().SetTargets([l_lower_path])
l_knee.CreateAxisAttr("X")
l_knee.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, KNEE_Z - UPPER_Z))
l_knee.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, KNEE_Z - LOWER_Z))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(l_knee_path), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(STIFFNESS)
drive.CreateDampingAttr(DAMPING)
drive.CreateTargetPositionAttr(0.0)

# Left foot - WIDE and HEAVY for stability
l_foot_path = atst_path + "/left_foot"
l_foot = UsdGeom.Cube.Define(stage, l_foot_path)
l_foot.AddTranslateOp().Set(Gf.Vec3d(-LEG_X, 0.0, FOOT_Z))
l_foot.AddScaleOp().Set(Gf.Vec3f(0.2, 0.15, 0.03))  # Wide flat foot
UsdShade.MaterialBindingAPI(l_foot).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(l_foot_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(l_foot_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(l_foot_path))
mass_api.CreateMassAttr(20.0)  # HEAVY foot

# Left ankle joint
l_ankle_path = atst_path + "/joint_left_ankle"
l_ankle = UsdPhysics.RevoluteJoint.Define(stage, l_ankle_path)
l_ankle.CreateBody0Rel().SetTargets([l_lower_path])
l_ankle.CreateBody1Rel().SetTargets([l_foot_path])
l_ankle.CreateAxisAttr("X")
l_ankle.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, ANKLE_Z - LOWER_Z))
l_ankle.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, ANKLE_Z - FOOT_Z))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(l_ankle_path), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(STIFFNESS)
drive.CreateDampingAttr(DAMPING)
drive.CreateTargetPositionAttr(0.0)

print("Created left leg")

# === RIGHT LEG (mirror of left) ===
r_upper_path = atst_path + "/right_upper"
r_upper = UsdGeom.Cube.Define(stage, r_upper_path)
r_upper.AddTranslateOp().Set(Gf.Vec3d(LEG_X, 0.0, UPPER_Z))
r_upper.AddScaleOp().Set(Gf.Vec3f(0.06, 0.06, 0.2))
UsdShade.MaterialBindingAPI(r_upper).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(r_upper_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(r_upper_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(r_upper_path))
mass_api.CreateMassAttr(3.0)

r_hip_path = atst_path + "/joint_right_hip"
r_hip = UsdPhysics.RevoluteJoint.Define(stage, r_hip_path)
r_hip.CreateBody0Rel().SetTargets([body_path])
r_hip.CreateBody1Rel().SetTargets([r_upper_path])
r_hip.CreateAxisAttr("X")
r_hip.CreateLocalPos0Attr().Set(Gf.Vec3f(LEG_X, 0.0, HIP_Z - BODY_Z))
r_hip.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, HIP_Z - UPPER_Z))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(r_hip_path), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(STIFFNESS)
drive.CreateDampingAttr(DAMPING)
drive.CreateTargetPositionAttr(0.0)

r_lower_path = atst_path + "/right_lower"
r_lower = UsdGeom.Cube.Define(stage, r_lower_path)
r_lower.AddTranslateOp().Set(Gf.Vec3d(LEG_X, 0.0, LOWER_Z))
r_lower.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.15))
UsdShade.MaterialBindingAPI(r_lower).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(r_lower_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(r_lower_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(r_lower_path))
mass_api.CreateMassAttr(3.0)

r_knee_path = atst_path + "/joint_right_knee"
r_knee = UsdPhysics.RevoluteJoint.Define(stage, r_knee_path)
r_knee.CreateBody0Rel().SetTargets([r_upper_path])
r_knee.CreateBody1Rel().SetTargets([r_lower_path])
r_knee.CreateAxisAttr("X")
r_knee.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, KNEE_Z - UPPER_Z))
r_knee.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, KNEE_Z - LOWER_Z))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(r_knee_path), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(STIFFNESS)
drive.CreateDampingAttr(DAMPING)
drive.CreateTargetPositionAttr(0.0)

r_foot_path = atst_path + "/right_foot"
r_foot = UsdGeom.Cube.Define(stage, r_foot_path)
r_foot.AddTranslateOp().Set(Gf.Vec3d(LEG_X, 0.0, FOOT_Z))
r_foot.AddScaleOp().Set(Gf.Vec3f(0.2, 0.15, 0.03))
UsdShade.MaterialBindingAPI(r_foot).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(r_foot_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(r_foot_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(r_foot_path))
mass_api.CreateMassAttr(20.0)

r_ankle_path = atst_path + "/joint_right_ankle"
r_ankle = UsdPhysics.RevoluteJoint.Define(stage, r_ankle_path)
r_ankle.CreateBody0Rel().SetTargets([r_lower_path])
r_ankle.CreateBody1Rel().SetTargets([r_foot_path])
r_ankle.CreateAxisAttr("X")
r_ankle.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, ANKLE_Z - LOWER_Z))
r_ankle.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, ANKLE_Z - FOOT_Z))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(r_ankle_path), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(STIFFNESS)
drive.CreateDampingAttr(DAMPING)
drive.CreateTargetPositionAttr(0.0)

print("Created right leg")

print("")
print("=== Stable AT-ST Created ===")
print("Position: X=-2.0 (left of Cassie)")
print("Body: 5kg at 1.0m height")
print("Feet: 20kg each, wide flat base")
print("Joints: Stiffness=10000, Damping=1000")
print("")
print("Press PLAY to test!")
'''

print("Creating stable AT-ST with proper spacing...")
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
