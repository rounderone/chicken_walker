"""
AT-ST Setup - Cassie robot with joint control + mini AT-ST for reference
Run with: D:\Projects\ATST\venv_isaaclab\Scripts\python.exe D:\Projects\ATST\isaac_sim\setup_atst.py
"""
from isaacsim import SimulationApp
import sys
import os

# Start with GUI
simulation_app = SimulationApp({"headless": False})

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf, PhysxSchema, UsdShade, Sdf

def log(msg):
    print(msg)
    sys.stdout.flush()

log("=" * 60)
log("CASSIE WITH JOINT CONTROL + MINI AT-ST REFERENCE")
log("=" * 60)

try:
    # Import after SimulationApp starts
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
    from isaacsim.core.utils.viewports import set_camera_view
    from isaacsim.core.utils.prims import define_prim
    from isaacsim.core.utils.nucleus import get_assets_root_path
    from isaacsim.core.api.objects import DynamicCuboid

    # Create world
    log("\n[1] Creating World...")
    my_world = World(stage_units_in_meters=1.0)
    log("    World created")

    # Add ground plane
    log("\n[2] Adding ground plane...")
    my_world.scene.add_default_ground_plane()
    log("    Ground plane added")

    # Set camera to see the small-scale scene
    log("\n[3] Setting camera for small scale...")
    set_camera_view(
        eye=[4.0, 4.0, 2.0],
        target=[0.0, 0.0, 0.5],
        camera_prim_path="/OmniverseKit_Persp"
    )
    log("    Camera set")

    # Get the stage
    stage = get_current_stage()

    # =========================================
    # LOAD CASSIE AT NATIVE SCALE
    # =========================================
    log("\n[4] Loading Cassie at native 1.36m scale...")
    assets_root = get_assets_root_path()
    cassie_usd = f"{assets_root}/Isaac/Robots/Agility/Cassie/cassie.usd"
    log(f"    Cassie path: {cassie_usd}")

    # Load Cassie directly (no parent transform needed)
    add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")
    log("    Cassie loaded at origin")

    # =========================================
    # LOAD MINI AT-ST FOR REFERENCE
    # =========================================
    atst_path = "D:/Projects/ATST/usd/atst.usdc"
    log(f"\n[5] Loading mini AT-ST from {atst_path}...")

    if os.path.exists(atst_path):
        # Create scaled parent for AT-ST (scale down to ~1.36m to match Cassie)
        # AT-ST is about 8.6m, Cassie is 1.36m, so scale = 1.36/8.6 = 0.158
        mini_atst = define_prim("/World/MiniATST", "Xform")
        mini_prim = stage.GetPrimAtPath("/World/MiniATST")
        if mini_prim:
            xform = UsdGeom.Xformable(mini_prim)
            xform.AddTranslateOp().Set(Gf.Vec3d(2.0, 0.0, 0.0))  # 2m to the right
            xform.AddScaleOp().Set(Gf.Vec3f(0.16, 0.16, 0.16))  # Scale down

        add_reference_to_stage(usd_path=atst_path, prim_path="/World/MiniATST/Model")
        log("    Mini AT-ST loaded (scaled to match Cassie height)")
    else:
        log("    WARNING: AT-ST file not found, skipping")

    # Wait for loading
    simulation_app.update()

    # =========================================
    # LIST CASSIE'S STRUCTURE
    # =========================================
    log("\n[6] Analyzing Cassie's body structure...")

    body_parts = []
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if path.startswith("/World/Cassie/") and not "/joints/" in path:
            # Get depth level
            depth = path.replace("/World/Cassie/", "").count("/")
            if depth == 0:  # Top-level children only
                type_name = prim.GetTypeName()
                body_parts.append((path.split("/")[-1], type_name))

    log("    Cassie body parts:")
    for name, ptype in body_parts:
        log(f"        - {name} ({ptype})")

    # =========================================
    # CREATE BOX AT-ST (simple geometric version)
    # =========================================
    log("\n[7] Creating Box AT-ST...")

    box_path = "/World/BoxATST"
    box_atst = UsdGeom.Xform.Define(stage, box_path)
    box_atst.AddTranslateOp().Set(Gf.Vec3d(-1.5, 0.0, 0.0))  # Left of Cassie

    # Gray material
    mtl = UsdShade.Material.Define(stage, box_path + "/GrayMtl")
    shader = UsdShade.Shader.Define(stage, box_path + "/GrayMtl/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.4, 0.4, 0.45))
    mtl.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    # Cockpit (head)
    cockpit = UsdGeom.Cube.Define(stage, box_path + "/cockpit")
    cockpit.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.1))
    cockpit.AddScaleOp().Set(Gf.Vec3f(0.25, 0.2, 0.15))
    UsdShade.MaterialBindingAPI(cockpit).Bind(mtl)

    # Left leg (upper)
    l_upper = UsdGeom.Cube.Define(stage, box_path + "/left_upper")
    l_upper.AddTranslateOp().Set(Gf.Vec3d(-0.15, 0.0, 0.75))
    l_upper.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, -15))
    l_upper.AddScaleOp().Set(Gf.Vec3f(0.06, 0.06, 0.25))
    UsdShade.MaterialBindingAPI(l_upper).Bind(mtl)

    # Left leg (lower)
    l_lower = UsdGeom.Cube.Define(stage, box_path + "/left_lower")
    l_lower.AddTranslateOp().Set(Gf.Vec3d(-0.25, 0.0, 0.35))
    l_lower.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 15))
    l_lower.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.25))
    UsdShade.MaterialBindingAPI(l_lower).Bind(mtl)

    # Left foot
    l_foot = UsdGeom.Cube.Define(stage, box_path + "/left_foot")
    l_foot.AddTranslateOp().Set(Gf.Vec3d(-0.22, 0.0, 0.05))
    l_foot.AddScaleOp().Set(Gf.Vec3f(0.1, 0.08, 0.03))
    UsdShade.MaterialBindingAPI(l_foot).Bind(mtl)

    # Right leg (upper)
    r_upper = UsdGeom.Cube.Define(stage, box_path + "/right_upper")
    r_upper.AddTranslateOp().Set(Gf.Vec3d(0.15, 0.0, 0.75))
    r_upper.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 15))
    r_upper.AddScaleOp().Set(Gf.Vec3f(0.06, 0.06, 0.25))
    UsdShade.MaterialBindingAPI(r_upper).Bind(mtl)

    # Right leg (lower)
    r_lower = UsdGeom.Cube.Define(stage, box_path + "/right_lower")
    r_lower.AddTranslateOp().Set(Gf.Vec3d(0.25, 0.0, 0.35))
    r_lower.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, -15))
    r_lower.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.25))
    UsdShade.MaterialBindingAPI(r_lower).Bind(mtl)

    # Right foot
    r_foot = UsdGeom.Cube.Define(stage, box_path + "/right_foot")
    r_foot.AddTranslateOp().Set(Gf.Vec3d(0.22, 0.0, 0.05))
    r_foot.AddScaleOp().Set(Gf.Vec3f(0.1, 0.08, 0.03))
    UsdShade.MaterialBindingAPI(r_foot).Bind(mtl)

    log("    Box AT-ST created")

    # =========================================
    # SET CAMERA TO SEE ALL THREE
    # =========================================
    log("\n[8] Setting camera...")
    set_camera_view(
        eye=[0.0, 5.0, 2.0],
        target=[0.0, 0.0, 0.6],
        camera_prim_path="/OmniverseKit_Persp"
    )
    log("    Camera positioned to see all models")

    # =========================================
    # INFO
    # =========================================
    log("\n" + "=" * 60)
    log("SCENE READY - THREE MODELS")
    log("=" * 60)
    log("\nWhat you see (left to right):")
    log("  - Box AT-ST (simple geometry)")
    log("  - Cassie (physics robot)")
    log("  - Mini AT-ST (original model)")
    log("\nPress PLAY to test physics.")
    log("Cassie should stand briefly then fall (no controller).")
    log("\nClose window when done.")
    log("=" * 60)

    # Run until user closes window
    while simulation_app.is_running():
        my_world.step(render=True)

except Exception as e:
    log(f"\nERROR: {type(e).__name__}: {e}")
    import traceback
    traceback.print_exc()
    sys.stdout.flush()

simulation_app.close()
