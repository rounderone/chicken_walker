# Create a physics-enabled Box AT-ST with stable wide feet
import omni.usd
from pxr import UsdGeom, UsdPhysics, PhysxSchema, UsdShade, Sdf, Gf

stage = omni.usd.get_context().get_stage()

# Delete old BoxATST if exists
if stage.GetPrimAtPath("/World/BoxATST"):
    stage.RemovePrim("/World/BoxATST")
    print("Removed old BoxATST")

# Create new physics-enabled AT-ST
atst_path = "/World/PhysicsATST"
if stage.GetPrimAtPath(atst_path):
    stage.RemovePrim(atst_path)

# Root xform
root = UsdGeom.Xform.Define(stage, atst_path)
root.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.8))  # Start above ground

# Gray material
mtl = UsdShade.Material.Define(stage, atst_path + "/GrayMtl")
shader = UsdShade.Shader.Define(stage, atst_path + "/GrayMtl/Shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.5, 0.5, 0.55))
mtl.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

# === BODY (cockpit) - the root rigid body ===
body_path = atst_path + "/body"
body = UsdGeom.Cube.Define(stage, body_path)
body.AddScaleOp().Set(Gf.Vec3f(0.3, 0.25, 0.2))
UsdShade.MaterialBindingAPI(body).Bind(mtl)

# Add articulation root to body
UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(body_path))
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(body_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(body_path))

# Set body mass (light body, heavy feet for stability)
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(body_path))
mass_api.CreateMassAttr(10.0)  # 10 kg body

print("Created body with ArticulationRoot")

# === LEFT LEG ===
# Upper leg
l_upper_path = atst_path + "/left_upper"
l_upper = UsdGeom.Cube.Define(stage, l_upper_path)
l_upper.AddTranslateOp().Set(Gf.Vec3d(-0.2, 0.0, -0.3))
l_upper.AddScaleOp().Set(Gf.Vec3f(0.08, 0.08, 0.25))
UsdShade.MaterialBindingAPI(l_upper).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(l_upper_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(l_upper_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(l_upper_path))
mass_api.CreateMassAttr(5.0)

# Left hip joint
l_hip_path = atst_path + "/joint_left_hip"
l_hip = UsdPhysics.RevoluteJoint.Define(stage, l_hip_path)
l_hip.CreateBody0Rel().SetTargets([body_path])
l_hip.CreateBody1Rel().SetTargets([l_upper_path])
l_hip.CreateAxisAttr("X")
l_hip.CreateLocalPos0Attr().Set(Gf.Vec3f(-0.2, 0.0, -0.15))
l_hip.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.2))
# Add drive
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(l_hip_path), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(2000.0)
drive.CreateDampingAttr(200.0)
drive.CreateTargetPositionAttr(0.0)

# Lower leg
l_lower_path = atst_path + "/left_lower"
l_lower = UsdGeom.Cube.Define(stage, l_lower_path)
l_lower.AddTranslateOp().Set(Gf.Vec3d(-0.25, 0.0, -0.65))
l_lower.AddScaleOp().Set(Gf.Vec3f(0.06, 0.06, 0.2))
UsdShade.MaterialBindingAPI(l_lower).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(l_lower_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(l_lower_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(l_lower_path))
mass_api.CreateMassAttr(5.0)

# Left knee joint
l_knee_path = atst_path + "/joint_left_knee"
l_knee = UsdPhysics.RevoluteJoint.Define(stage, l_knee_path)
l_knee.CreateBody0Rel().SetTargets([l_upper_path])
l_knee.CreateBody1Rel().SetTargets([l_lower_path])
l_knee.CreateAxisAttr("X")
l_knee.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, -0.2))
l_knee.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.15))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(l_knee_path), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(2000.0)
drive.CreateDampingAttr(200.0)
drive.CreateTargetPositionAttr(0.0)

# Left foot - WIDE for stability
l_foot_path = atst_path + "/left_foot"
l_foot = UsdGeom.Cube.Define(stage, l_foot_path)
l_foot.AddTranslateOp().Set(Gf.Vec3d(-0.25, 0.0, -0.85))
l_foot.AddScaleOp().Set(Gf.Vec3f(0.2, 0.15, 0.03))  # Wide flat foot
UsdShade.MaterialBindingAPI(l_foot).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(l_foot_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(l_foot_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(l_foot_path))
mass_api.CreateMassAttr(50.0)  # HEAVY foot for stability

# Left ankle joint
l_ankle_path = atst_path + "/joint_left_ankle"
l_ankle = UsdPhysics.RevoluteJoint.Define(stage, l_ankle_path)
l_ankle.CreateBody0Rel().SetTargets([l_lower_path])
l_ankle.CreateBody1Rel().SetTargets([l_foot_path])
l_ankle.CreateAxisAttr("X")
l_ankle.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, -0.15))
l_ankle.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.02))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(l_ankle_path), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(2000.0)
drive.CreateDampingAttr(200.0)
drive.CreateTargetPositionAttr(0.0)

print("Created left leg with joints")

# === RIGHT LEG ===
# Upper leg
r_upper_path = atst_path + "/right_upper"
r_upper = UsdGeom.Cube.Define(stage, r_upper_path)
r_upper.AddTranslateOp().Set(Gf.Vec3d(0.2, 0.0, -0.3))
r_upper.AddScaleOp().Set(Gf.Vec3f(0.08, 0.08, 0.25))
UsdShade.MaterialBindingAPI(r_upper).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(r_upper_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(r_upper_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(r_upper_path))
mass_api.CreateMassAttr(5.0)

# Right hip joint
r_hip_path = atst_path + "/joint_right_hip"
r_hip = UsdPhysics.RevoluteJoint.Define(stage, r_hip_path)
r_hip.CreateBody0Rel().SetTargets([body_path])
r_hip.CreateBody1Rel().SetTargets([r_upper_path])
r_hip.CreateAxisAttr("X")
r_hip.CreateLocalPos0Attr().Set(Gf.Vec3f(0.2, 0.0, -0.15))
r_hip.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.2))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(r_hip_path), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(2000.0)
drive.CreateDampingAttr(200.0)
drive.CreateTargetPositionAttr(0.0)

# Lower leg
r_lower_path = atst_path + "/right_lower"
r_lower = UsdGeom.Cube.Define(stage, r_lower_path)
r_lower.AddTranslateOp().Set(Gf.Vec3d(0.25, 0.0, -0.65))
r_lower.AddScaleOp().Set(Gf.Vec3f(0.06, 0.06, 0.2))
UsdShade.MaterialBindingAPI(r_lower).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(r_lower_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(r_lower_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(r_lower_path))
mass_api.CreateMassAttr(5.0)

# Right knee joint
r_knee_path = atst_path + "/joint_right_knee"
r_knee = UsdPhysics.RevoluteJoint.Define(stage, r_knee_path)
r_knee.CreateBody0Rel().SetTargets([r_upper_path])
r_knee.CreateBody1Rel().SetTargets([r_lower_path])
r_knee.CreateAxisAttr("X")
r_knee.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, -0.2))
r_knee.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.15))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(r_knee_path), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(2000.0)
drive.CreateDampingAttr(200.0)
drive.CreateTargetPositionAttr(0.0)

# Right foot - WIDE for stability
r_foot_path = atst_path + "/right_foot"
r_foot = UsdGeom.Cube.Define(stage, r_foot_path)
r_foot.AddTranslateOp().Set(Gf.Vec3d(0.25, 0.0, -0.85))
r_foot.AddScaleOp().Set(Gf.Vec3f(0.2, 0.15, 0.03))  # Wide flat foot
UsdShade.MaterialBindingAPI(r_foot).Bind(mtl)
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(r_foot_path))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(r_foot_path))
mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(r_foot_path))
mass_api.CreateMassAttr(50.0)  # HEAVY foot for stability

# Right ankle joint
r_ankle_path = atst_path + "/joint_right_ankle"
r_ankle = UsdPhysics.RevoluteJoint.Define(stage, r_ankle_path)
r_ankle.CreateBody0Rel().SetTargets([r_lower_path])
r_ankle.CreateBody1Rel().SetTargets([r_foot_path])
r_ankle.CreateAxisAttr("X")
r_ankle.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, -0.15))
r_ankle.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.02))
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(r_ankle_path), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(2000.0)
drive.CreateDampingAttr(200.0)
drive.CreateTargetPositionAttr(0.0)

print("Created right leg with joints")

print("\n=== PhysicsATST Created ===")
print("- Light body (10kg)")
print("- Heavy feet (50kg each)")
print("- Wide feet for stability")
print("- 6 joints with drives")
print("\nPress STOP then PLAY to test!")
