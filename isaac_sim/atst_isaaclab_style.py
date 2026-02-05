# AT-ST using Isaac Lab style - active joint control every step
# Run with: set PYTHONPATH=D:\Projects\ATST\IsaacLab\source && "D:\Projects\ATST\venv_isaaclab\Scripts\python.exe" atst_isaaclab_style.py

import sys
print("Starting AT-ST demo (Isaac Lab style)...", flush=True)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

print("Isaac Sim started, importing modules...", flush=True)
sys.stdout.flush()

import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg

# Define AT-ST configuration (like Cassie config)
ATST_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="",  # We'll create it programmatically
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.65),
        joint_pos={
            ".*": 0.0,  # All joints at 0
        },
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=1000.0,
            damping=100.0,
        ),
    },
)

def create_atst_usd(stage, prim_path):
    """Create AT-ST articulation directly in USD"""
    print(f"Creating AT-ST at {prim_path}...", flush=True)

    # Dimensions
    FOOT_HALF = 0.025
    LOWER_HALF = 0.12
    UPPER_HALF = 0.12
    BODY_HALF = 0.08
    LEG_X = 0.15

    # Vertical positions (stacked)
    FOOT_Z = FOOT_HALF + 0.01
    ANKLE_Z = FOOT_Z + FOOT_HALF
    LOWER_Z = ANKLE_Z + LOWER_HALF
    KNEE_Z = LOWER_Z + LOWER_HALF
    UPPER_Z = KNEE_Z + UPPER_HALF
    HIP_Z = UPPER_Z + UPPER_HALF
    BODY_Z = HIP_Z + BODY_HALF

    # Root xform with ArticulationRoot
    root = UsdGeom.Xform.Define(stage, prim_path)
    root_prim = stage.GetPrimAtPath(prim_path)
    UsdPhysics.ArticulationRootAPI.Apply(root_prim)

    def make_link(name, pos, half_size, color, mass):
        path = f"{prim_path}/{name}"
        cube = UsdGeom.Cube.Define(stage, path)
        prim = stage.GetPrimAtPath(path)

        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        xf.AddScaleOp().Set(Gf.Vec3f(*half_size))
        cube.CreateDisplayColorAttr([color])

        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.CollisionAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(mass)

        return path

    # Create links
    body = make_link("body", (0, 0, BODY_Z), (0.12, 0.10, BODY_HALF), (0.5, 0.5, 0.6), 10.0)
    l_upper = make_link("l_upper", (-LEG_X, 0, UPPER_Z), (0.025, 0.025, UPPER_HALF), (0.4, 0.6, 0.4), 1.5)
    l_lower = make_link("l_lower", (-LEG_X, 0, LOWER_Z), (0.02, 0.02, LOWER_HALF), (0.4, 0.4, 0.7), 1.0)
    l_foot = make_link("l_foot", (-LEG_X, 0, FOOT_Z), (0.08, 0.06, FOOT_HALF), (0.7, 0.7, 0.3), 3.0)
    r_upper = make_link("r_upper", (LEG_X, 0, UPPER_Z), (0.025, 0.025, UPPER_HALF), (0.4, 0.6, 0.4), 1.5)
    r_lower = make_link("r_lower", (LEG_X, 0, LOWER_Z), (0.02, 0.02, LOWER_HALF), (0.4, 0.4, 0.7), 1.0)
    r_foot = make_link("r_foot", (LEG_X, 0, FOOT_Z), (0.08, 0.06, FOOT_HALF), (0.7, 0.7, 0.3), 3.0)

    print("  Created 7 links", flush=True)

    def make_revolute(name, parent, child, anchor_world, axis="X"):
        path = f"{prim_path}/{name}"
        joint = UsdPhysics.RevoluteJoint.Define(stage, path)
        joint.CreateBody0Rel().SetTargets([parent])
        joint.CreateBody1Rel().SetTargets([child])
        joint.CreateAxisAttr(axis)

        # Get parent/child positions
        p_prim = stage.GetPrimAtPath(parent)
        c_prim = stage.GetPrimAtPath(child)
        p_pos = list(UsdGeom.Xformable(p_prim).GetOrderedXformOps()[0].Get())
        c_pos = list(UsdGeom.Xformable(c_prim).GetOrderedXformOps()[0].Get())

        local0 = Gf.Vec3f(anchor_world[0] - p_pos[0], anchor_world[1] - p_pos[1], anchor_world[2] - p_pos[2])
        local1 = Gf.Vec3f(anchor_world[0] - c_pos[0], anchor_world[1] - c_pos[1], anchor_world[2] - c_pos[2])

        joint.CreateLocalPos0Attr().Set(local0)
        joint.CreateLocalPos1Attr().Set(local1)
        joint.CreateLowerLimitAttr(-30.0)
        joint.CreateUpperLimitAttr(30.0)

        # High stiffness drive
        prim = stage.GetPrimAtPath(path)
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(5000.0)
        drive.CreateDampingAttr(500.0)
        drive.CreateMaxForceAttr(10000.0)
        drive.CreateTargetPositionAttr(0.0)

        return path

    # Create joints
    make_revolute("hip_L", body, l_upper, (-LEG_X, 0, HIP_Z))
    make_revolute("hip_R", body, r_upper, (LEG_X, 0, HIP_Z))
    make_revolute("knee_L", l_upper, l_lower, (-LEG_X, 0, KNEE_Z))
    make_revolute("knee_R", r_upper, r_lower, (LEG_X, 0, KNEE_Z))
    make_revolute("ankle_L", l_lower, l_foot, (-LEG_X, 0, ANKLE_Z))
    make_revolute("ankle_R", r_lower, r_foot, (LEG_X, 0, ANKLE_Z))

    print("  Created 6 revolute joints", flush=True)
    return prim_path

def main():
    print("Setting up simulation...", flush=True)

    # Use Isaac Core World instead of Isaac Lab SimulationContext
    from omni.isaac.core.world import World

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    import omni.usd
    stage = omni.usd.get_context().get_stage()

    # Add light
    from pxr import UsdLux
    light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    light.CreateIntensityAttr(2000.0)

    # Create AT-ST in USD
    atst_path = create_atst_usd(stage, "/World/ATST")

    print("Starting simulation...", flush=True)

    # Reset world
    world.reset()

    # Try to create articulation wrapper
    try:
        from omni.isaac.core.articulations import Articulation as CoreArticulation
        atst = CoreArticulation(prim_path="/World/ATST")
        atst.initialize()

        dof_names = atst.dof_names
        num_dof = atst.num_dof
        print(f"AT-ST DOFs: {num_dof}", flush=True)
        print(f"Joint names: {dof_names}", flush=True)

        # Get default positions
        default_pos = atst.get_joint_positions()
        print(f"Default positions: {default_pos}", flush=True)

        has_articulation = True
    except Exception as e:
        print(f"Could not create articulation wrapper: {e}", flush=True)
        has_articulation = False

    print("\n=== AT-ST Simulation Running ===", flush=True)
    print("Close the window to exit.", flush=True)
    sys.stdout.flush()

    # Simulation loop with active joint control
    step = 0
    while simulation_app.is_running():
        # Active control: command target positions every step
        if has_articulation and step % 2 == 0:
            try:
                # Command joints to hold at zero (straight legs)
                target_pos = np.zeros(num_dof)
                atst.set_joint_position_targets(target_pos)
            except:
                pass

        world.step(render=True)
        step += 1

        # Status update
        if step % 200 == 0:
            if has_articulation:
                try:
                    pos = atst.get_world_pose()[0]
                    joint_pos = atst.get_joint_positions()
                    print(f"Step {step}: body_z={pos[2]:.3f}, joints={joint_pos}", flush=True)
                except:
                    pass

    simulation_app.close()
    print("Done!", flush=True)

if __name__ == "__main__":
    main()
