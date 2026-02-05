# Scene setup script - to be executed via MCP
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.nucleus import get_assets_root_path
from pxr import UsdGeom, UsdShade, Sdf, Gf
import os

# Create world with physics
print("Creating world...")
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
print("Ground plane added")

stage = get_current_stage()

# Load Cassie
print("Loading Cassie...")
assets_root = get_assets_root_path()
cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"
add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")
print("Cassie loaded")

# Load mini AT-ST
atst_path = "D:/Projects/ATST/usd/atst.usdc"
if os.path.exists(atst_path):
    print("Loading mini AT-ST...")
    mini_xform = UsdGeom.Xform.Define(stage, "/World/MiniATST")
    mini_xform.AddTranslateOp().Set(Gf.Vec3d(2.0, 0.0, 0.0))
    mini_xform.AddScaleOp().Set(Gf.Vec3f(0.16, 0.16, 0.16))
    add_reference_to_stage(usd_path=atst_path, prim_path="/World/MiniATST/Model")
    print("Mini AT-ST loaded")

# Create Box AT-ST
print("Creating Box AT-ST...")
box_path = "/World/BoxATST"
box_xform = UsdGeom.Xform.Define(stage, box_path)
box_xform.AddTranslateOp().Set(Gf.Vec3d(-1.5, 0.0, 0.0))

# Material
mtl = UsdShade.Material.Define(stage, box_path + "/GrayMtl")
shader = UsdShade.Shader.Define(stage, box_path + "/GrayMtl/Shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.4, 0.4, 0.45))
mtl.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

# Cockpit
cockpit = UsdGeom.Cube.Define(stage, box_path + "/cockpit")
cockpit.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.1))
cockpit.AddScaleOp().Set(Gf.Vec3f(0.25, 0.2, 0.15))
UsdShade.MaterialBindingAPI(cockpit).Bind(mtl)

# Left leg
l_upper = UsdGeom.Cube.Define(stage, box_path + "/left_upper")
l_upper.AddTranslateOp().Set(Gf.Vec3d(-0.15, 0.0, 0.75))
l_upper.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, -15))
l_upper.AddScaleOp().Set(Gf.Vec3f(0.06, 0.06, 0.25))
UsdShade.MaterialBindingAPI(l_upper).Bind(mtl)

l_lower = UsdGeom.Cube.Define(stage, box_path + "/left_lower")
l_lower.AddTranslateOp().Set(Gf.Vec3d(-0.25, 0.0, 0.35))
l_lower.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 15))
l_lower.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.25))
UsdShade.MaterialBindingAPI(l_lower).Bind(mtl)

l_foot = UsdGeom.Cube.Define(stage, box_path + "/left_foot")
l_foot.AddTranslateOp().Set(Gf.Vec3d(-0.22, 0.0, 0.05))
l_foot.AddScaleOp().Set(Gf.Vec3f(0.1, 0.08, 0.03))
UsdShade.MaterialBindingAPI(l_foot).Bind(mtl)

# Right leg
r_upper = UsdGeom.Cube.Define(stage, box_path + "/right_upper")
r_upper.AddTranslateOp().Set(Gf.Vec3d(0.15, 0.0, 0.75))
r_upper.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 15))
r_upper.AddScaleOp().Set(Gf.Vec3f(0.06, 0.06, 0.25))
UsdShade.MaterialBindingAPI(r_upper).Bind(mtl)

r_lower = UsdGeom.Cube.Define(stage, box_path + "/right_lower")
r_lower.AddTranslateOp().Set(Gf.Vec3d(0.25, 0.0, 0.35))
r_lower.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, -15))
r_lower.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.25))
UsdShade.MaterialBindingAPI(r_lower).Bind(mtl)

r_foot = UsdGeom.Cube.Define(stage, box_path + "/right_foot")
r_foot.AddTranslateOp().Set(Gf.Vec3d(0.22, 0.0, 0.05))
r_foot.AddScaleOp().Set(Gf.Vec3f(0.1, 0.08, 0.03))
UsdShade.MaterialBindingAPI(r_foot).Bind(mtl)

print("Box AT-ST created")

# Set camera
set_camera_view(eye=[0.0, 5.0, 2.0], target=[0.0, 0.0, 0.6], camera_prim_path="/OmniverseKit_Persp")
print("Camera set")

print("Scene ready! Left: BoxATST, Center: Cassie, Right: MiniATST")
