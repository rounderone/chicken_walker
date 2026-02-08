"""
AT-ST 9 Robots - DEEP CROUCH TEST
Testing different knee angles and hip compensation ratios
to keep torso/head level during deep crouch.
"""
print("=" * 60)
print("AT-ST DEEP CROUCH TEST")
print("=" * 60)
import sys
sys.stdout.flush()

from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=False)
simulation_app = app_launcher.app

import omni.usd
import omni.kit.app
import omni.timeline
from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Gf, PhysxSchema
import math

ASSEMBLED_USD = "D:/Projects/ATST/usd/atst_with_bars.usda"

PART_MASSES = {
    'ATST_torso': 20.0,
    'ATST_head': 10.0,
    'ATST_bar_L': 2.0,
    'ATST_bar_R': 2.0,
    'ATST_thigh_L': 4.0,
    'ATST_thigh_R': 4.0,
    'ATST_shin_L': 4.0,
    'ATST_shin_R': 4.0,
    'ATST_elbow_L': 3.0,
    'ATST_elbow_R': 3.0,
    'ATST_foot_L': 15.0,
    'ATST_foot_R': 15.0,
}

NO_COLLISION_PARTS = {'ATST_thigh_L', 'ATST_thigh_R', 'ATST_bar_L', 'ATST_bar_R'}

PART_POSITIONS = {
    'ATST_torso': (-0.031, 0.246, -0.191),
    'ATST_head': (0.080, -0.352, 0.642),
    'ATST_bar_L': (1.026, 1.788, -1.219),
    'ATST_bar_R': (-1.026, 1.788, -1.219),
    'ATST_thigh_L': (0.856, 0.708, -0.744),
    'ATST_thigh_R': (-0.856, 0.708, -0.744),
    'ATST_shin_L': (0.754, 1.387, -2.079),
    'ATST_shin_R': (-0.754, 1.387, -2.079),
    'ATST_elbow_L': (0.746, 1.488, -3.152),
    'ATST_elbow_R': (-0.746, 1.488, -3.152),
    'ATST_foot_L': (0.747, 0.455, -3.722),
    'ATST_foot_R': (-0.747, 0.455, -3.722),
}

# User-verified joint positions
JOINT_POSITIONS = {
    'neck': (0.0, 0.263, -0.669),
    'torsobar_L': (0.640, 1.050, -1.043),
    'legbar_L': (0.727, 0.449, -1.031),
    'knee_L': (0.944, 1.861, -1.230),
    'ankle_L': (0.776, 1.515, -3.452),
    'foot_L': (0.757, 1.010, -3.697),
    'torsobar_R': (-0.640, 1.050, -1.043),
    'legbar_R': (-0.727, 0.449, -1.031),
    'knee_R': (-0.944, 1.861, -1.230),
    'ankle_R': (-0.776, 1.515, -3.452),
    'foot_R': (-0.757, 1.010, -3.697),
}

# 9 robots - DEEP CROUCH with POSITIVE hip (front down to level torso)
ROBOT_CONFIGS = [
    # Row 1: K60 with different positive hip
    {'name': '0: K60 H+18', 'behavior': 'crouch', 'knee_max': 60, 'hip_ratio': 0.3},
    {'name': '1: K60 H+30', 'behavior': 'crouch', 'knee_max': 60, 'hip_ratio': 0.5},
    {'name': '2: K60 H+42', 'behavior': 'crouch', 'knee_max': 60, 'hip_ratio': 0.7},

    # Row 2: K70 deeper crouch
    {'name': '3: K70 H+21', 'behavior': 'crouch', 'knee_max': 70, 'hip_ratio': 0.3},
    {'name': '4: K70 H+35', 'behavior': 'crouch', 'knee_max': 70, 'hip_ratio': 0.5},
    {'name': '5: K70 H+49', 'behavior': 'crouch', 'knee_max': 70, 'hip_ratio': 0.7},

    # Row 3: K80 deepest crouch
    {'name': '6: K80 H+24', 'behavior': 'crouch', 'knee_max': 80, 'hip_ratio': 0.3},
    {'name': '7: K80 H+40', 'behavior': 'crouch', 'knee_max': 80, 'hip_ratio': 0.5},
    {'name': '8: K80 H+56', 'behavior': 'crouch', 'knee_max': 80, 'hip_ratio': 0.7},
]

GRID_SPACING = 8.0
GRID_COLS = 3


def setup_scene():
    print("Creating scene...")
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    physics = UsdPhysics.Scene.Define(stage, "/World/Physics")
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics.CreateGravityMagnitudeAttr(9.81)

    physx = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/Physics"))
    physx.CreateEnableCCDAttr(True)
    physx.CreateEnableStabilizationAttr(True)
    physx.CreateSolverTypeAttr("TGS")

    ground = UsdGeom.Mesh.Define(stage, "/World/Ground")
    size = 50.0
    ground.CreatePointsAttr([
        Gf.Vec3f(-size, -size, 0), Gf.Vec3f(size, -size, 0),
        Gf.Vec3f(size, size, 0), Gf.Vec3f(-size, size, 0)
    ])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateDisplayColorAttr([(0.3, 0.35, 0.3)])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(1500.0)

    return stage


def create_robot(stage, robot_id, x_offset, y_offset):
    robot_path = f"/World/ATST_{robot_id}"
    robot_prim = stage.DefinePrim(robot_path, "Xform")
    robot_xform = UsdGeom.Xformable(robot_prim)
    robot_xform.AddTranslateOp().Set(Gf.Vec3d(x_offset, y_offset, 4.05))
    robot_prim.GetReferences().AddReference(ASSEMBLED_USD)

    part_paths = {}
    processed_parts = set()
    robot_root = stage.GetPrimAtPath(robot_path)

    for child in robot_root.GetChildren():
        prim_name = child.GetName()
        if prim_name in processed_parts or prim_name not in PART_MASSES:
            continue

        processed_parts.add(prim_name)
        mass = PART_MASSES[prim_name]
        part_path = str(child.GetPath())

        UsdPhysics.RigidBodyAPI.Apply(child)
        mass_api = UsdPhysics.MassAPI.Apply(child)
        mass_api.CreateMassAttr(mass)

        if prim_name not in NO_COLLISION_PARTS:
            for descendant in Usd.PrimRange(child):
                if descendant.IsA(UsdGeom.Mesh):
                    UsdPhysics.CollisionAPI.Apply(descendant)
                    mesh_col = UsdPhysics.MeshCollisionAPI.Apply(descendant)
                    mesh_col.CreateApproximationAttr("convexHull")
                    break

        part_paths[prim_name] = part_path

    if 'ATST_torso' in part_paths:
        torso_prim = stage.GetPrimAtPath(part_paths['ATST_torso'])
        UsdPhysics.ArticulationRootAPI.Apply(torso_prim)
        physx_art = PhysxSchema.PhysxArticulationAPI.Apply(torso_prim)
        physx_art.CreateEnabledSelfCollisionsAttr(False)

    return robot_path, part_paths


def create_joint(stage, joints_path, joint_name, parent_path, child_path,
                 world_pos, parent_center, child_center, axis, limits, stiffness, damping):
    local_pos_parent = Gf.Vec3f(
        world_pos[0] - parent_center[0],
        world_pos[1] - parent_center[1],
        world_pos[2] - parent_center[2]
    )
    local_pos_child = Gf.Vec3f(
        world_pos[0] - child_center[0],
        world_pos[1] - child_center[1],
        world_pos[2] - child_center[2]
    )

    jpath = f"{joints_path}/{joint_name}"
    joint = UsdPhysics.RevoluteJoint.Define(stage, jpath)
    joint.CreateBody0Rel().SetTargets([parent_path])
    joint.CreateBody1Rel().SetTargets([child_path])
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(float(limits[0]))
    joint.CreateUpperLimitAttr(float(limits[1]))
    joint.CreateLocalPos0Attr(local_pos_parent)
    joint.CreateLocalPos1Attr(local_pos_child)
    joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(stiffness))
    drive.CreateDampingAttr(float(damping))
    drive.CreateMaxForceAttr(50000.0)
    drive.CreateTargetPositionAttr(0.0)

    joint.CreateBreakForceAttr(float('inf'))
    joint.CreateBreakTorqueAttr(float('inf'))


def create_joints(stage, robot_path, part_paths):
    joints_path = f"{robot_path}/joints"
    stage.DefinePrim(joints_path, "Xform")

    # Neck
    if 'ATST_torso' in part_paths and 'ATST_head' in part_paths:
        create_joint(stage, joints_path, 'neck',
                     part_paths['ATST_torso'], part_paths['ATST_head'],
                     JOINT_POSITIONS['neck'],
                     PART_POSITIONS['ATST_torso'], PART_POSITIONS['ATST_head'],
                     'Z', (-30, 30), 100, 5)

    # Bar joints (torso -> bar)
    if 'ATST_torso' in part_paths and 'ATST_bar_L' in part_paths:
        create_joint(stage, joints_path, 'torsobar_L',
                     part_paths['ATST_torso'], part_paths['ATST_bar_L'],
                     JOINT_POSITIONS['torsobar_L'],
                     PART_POSITIONS['ATST_torso'], PART_POSITIONS['ATST_bar_L'],
                     'Y', (-20, 20), 500, 50)

    if 'ATST_torso' in part_paths and 'ATST_bar_R' in part_paths:
        create_joint(stage, joints_path, 'torsobar_R',
                     part_paths['ATST_torso'], part_paths['ATST_bar_R'],
                     JOINT_POSITIONS['torsobar_R'],
                     PART_POSITIONS['ATST_torso'], PART_POSITIONS['ATST_bar_R'],
                     'Y', (-20, 20), 500, 50)

    # Hip joints (bar -> thigh)
    if 'ATST_bar_L' in part_paths and 'ATST_thigh_L' in part_paths:
        create_joint(stage, joints_path, 'hip_L',
                     part_paths['ATST_bar_L'], part_paths['ATST_thigh_L'],
                     JOINT_POSITIONS['legbar_L'],
                     PART_POSITIONS['ATST_bar_L'], PART_POSITIONS['ATST_thigh_L'],
                     'X', (-30, 50), 600, 30)

    if 'ATST_bar_R' in part_paths and 'ATST_thigh_R' in part_paths:
        create_joint(stage, joints_path, 'hip_R',
                     part_paths['ATST_bar_R'], part_paths['ATST_thigh_R'],
                     JOINT_POSITIONS['legbar_R'],
                     PART_POSITIONS['ATST_bar_R'], PART_POSITIONS['ATST_thigh_R'],
                     'X', (-30, 50), 600, 30)

    # Knees
    if 'ATST_thigh_L' in part_paths and 'ATST_shin_L' in part_paths:
        create_joint(stage, joints_path, 'knee_L',
                     part_paths['ATST_thigh_L'], part_paths['ATST_shin_L'],
                     JOINT_POSITIONS['knee_L'],
                     PART_POSITIONS['ATST_thigh_L'], PART_POSITIONS['ATST_shin_L'],
                     'X', (-50, 35), 700, 35)

    if 'ATST_thigh_R' in part_paths and 'ATST_shin_R' in part_paths:
        create_joint(stage, joints_path, 'knee_R',
                     part_paths['ATST_thigh_R'], part_paths['ATST_shin_R'],
                     JOINT_POSITIONS['knee_R'],
                     PART_POSITIONS['ATST_thigh_R'], PART_POSITIONS['ATST_shin_R'],
                     'X', (-50, 35), 700, 35)

    # Elbows (shin -> elbow)
    if 'ATST_shin_L' in part_paths and 'ATST_elbow_L' in part_paths:
        create_joint(stage, joints_path, 'elbow_L',
                     part_paths['ATST_shin_L'], part_paths['ATST_elbow_L'],
                     JOINT_POSITIONS['ankle_L'],
                     PART_POSITIONS['ATST_shin_L'], PART_POSITIONS['ATST_elbow_L'],
                     'X', (-40, 40), 700, 35)

    if 'ATST_shin_R' in part_paths and 'ATST_elbow_R' in part_paths:
        create_joint(stage, joints_path, 'elbow_R',
                     part_paths['ATST_shin_R'], part_paths['ATST_elbow_R'],
                     JOINT_POSITIONS['ankle_R'],
                     PART_POSITIONS['ATST_shin_R'], PART_POSITIONS['ATST_elbow_R'],
                     'X', (-40, 40), 700, 35)

    # Ankles (elbow -> foot)
    if 'ATST_elbow_L' in part_paths and 'ATST_foot_L' in part_paths:
        create_joint(stage, joints_path, 'ankle_L',
                     part_paths['ATST_elbow_L'], part_paths['ATST_foot_L'],
                     JOINT_POSITIONS['foot_L'],
                     PART_POSITIONS['ATST_elbow_L'], PART_POSITIONS['ATST_foot_L'],
                     'X', (-30, 30), 400, 20)

    if 'ATST_elbow_R' in part_paths and 'ATST_foot_R' in part_paths:
        create_joint(stage, joints_path, 'ankle_R',
                     part_paths['ATST_elbow_R'], part_paths['ATST_foot_R'],
                     JOINT_POSITIONS['foot_R'],
                     PART_POSITIONS['ATST_elbow_R'], PART_POSITIONS['ATST_foot_R'],
                     'X', (-30, 30), 400, 20)

    return joints_path


class WalkingGait:
    """Walk forward without crouching first."""
    def __init__(self, speed=0.5):
        self.phase = 0.0
        self.frequency = speed
        self.hip_swing = 15.0
        self.knee_lift = 20.0

    def get_targets(self, dt):
        self.phase += self.frequency * dt * 2 * math.pi
        if self.phase > 2 * math.pi:
            self.phase -= 2 * math.pi

        sin_p = math.sin(self.phase)
        lift_L = max(0, math.cos(self.phase)) ** 2
        lift_R = max(0, math.cos(self.phase + math.pi)) ** 2

        return {
            'torsobar_L': 0, 'torsobar_R': 0,
            'hip_L': 10 + self.hip_swing * sin_p,
            'hip_R': 10 - self.hip_swing * sin_p,
            'knee_L': -20 + self.knee_lift * lift_L,
            'knee_R': -20 + self.knee_lift * lift_R,
            'elbow_L': 5 + 10 * lift_L,
            'elbow_R': 5 + 10 * lift_R,
            'ankle_L': -5 * lift_L,
            'ankle_R': -5 * lift_R,
        }


class ShiftWeightGait:
    """Shift weight from foot to foot, try to balance on one leg."""
    def __init__(self, speed=0.4):
        self.phase = 0.0
        self.frequency = speed

    def get_targets(self, dt):
        self.phase += self.frequency * dt * 2 * math.pi
        if self.phase > 2 * math.pi:
            self.phase -= 2 * math.pi

        # Slow sine wave for weight shift
        shift = math.sin(self.phase)  # -1 to 1

        # When shift > 0.5, try to lift right foot
        # When shift < -0.5, try to lift left foot
        lift_R = max(0, (shift - 0.5) * 2) ** 2 if shift > 0.5 else 0
        lift_L = max(0, (-shift - 0.5) * 2) ** 2 if shift < -0.5 else 0

        # Hip tilt for balance
        hip_tilt = shift * 8

        return {
            'torsobar_L': -shift * 5,  # Lean toward standing leg
            'torsobar_R': shift * 5,
            'hip_L': 15 + hip_tilt + 15 * lift_L,
            'hip_R': 15 - hip_tilt + 15 * lift_R,
            'knee_L': -25 + 25 * lift_L,
            'knee_R': -25 + 25 * lift_R,
            'elbow_L': 5 + 15 * lift_L,
            'elbow_R': 5 + 15 * lift_R,
            'ankle_L': -10 * lift_L,
            'ankle_R': -10 * lift_R,
        }


class DeepCrouchGait:
    """Deep crouch while keeping torso/head level using hip compensation."""
    def __init__(self, knee_max=50, hip_ratio=0.5):
        self.phase = 0.0
        self.frequency = 0.25  # Slow crouch cycle
        self.knee_max = knee_max  # Maximum knee bend in degrees
        self.hip_ratio = hip_ratio  # Hip compensation = knee * hip_ratio

    def get_targets(self, dt):
        self.phase += self.frequency * dt * 2 * math.pi
        if self.phase > 2 * math.pi:
            self.phase -= 2 * math.pi

        # Smooth crouch cycle (0 to 1 to 0)
        crouch_amount = (1 - math.cos(self.phase)) / 2

        # Knee bends down (negative angle)
        knee_angle = -self.knee_max * crouch_amount

        # Hip compensates to keep torso level
        # Hip angle = knee_max * hip_ratio * crouch_amount
        hip_compensation = self.knee_max * self.hip_ratio * crouch_amount

        # Elbow compensates proportionally
        elbow_comp = self.knee_max * 0.3 * crouch_amount

        # Ankle adjusts for foot contact
        ankle_comp = -self.knee_max * 0.1 * crouch_amount

        return {
            'torsobar_L': 0, 'torsobar_R': 0,
            'hip_L': hip_compensation,
            'hip_R': hip_compensation,
            'knee_L': knee_angle,
            'knee_R': knee_angle,
            'elbow_L': elbow_comp,
            'elbow_R': elbow_comp,
            'ankle_L': ankle_comp,
            'ankle_R': ankle_comp,
        }


def main():
    stage = setup_scene()
    robots = []

    print("\nCreating 9 robots testing deep crouch...")
    print("-" * 60)

    for i, config in enumerate(ROBOT_CONFIGS):
        row = i // GRID_COLS
        col = i % GRID_COLS

        x_offset = (col - 1) * GRID_SPACING
        y_offset = (row - 1) * GRID_SPACING

        robot_path, part_paths = create_robot(stage, i, x_offset, y_offset)
        joints_path = create_joints(stage, robot_path, part_paths)

        # Create behavior controller
        if config['behavior'] == 'walk':
            controller = WalkingGait(speed=config.get('speed', 0.5))
        elif config['behavior'] == 'shift':
            controller = ShiftWeightGait(speed=config.get('speed', 0.4))
        else:  # crouch
            controller = DeepCrouchGait(
                knee_max=config.get('knee_max', 50),
                hip_ratio=config.get('hip_ratio', 0.5)
            )

        robots.append({
            'id': i,
            'path': robot_path,
            'joints_path': joints_path,
            'config': config,
            'controller': controller,
        })

        print(f"  Robot {i}: {config['name']}")

    save_path = "D:/Projects/ATST/usd/nine_behaviors.usda"
    stage.Export(save_path)
    print(f"\nSaved to: {save_path}")

    print("\n" + "=" * 60)
    print("DEEP CROUCH - POSITIVE HIP (front down)")
    print("=" * 60)
    print("""
Hip rotates FRONT DOWN to keep torso level during crouch.
K = knee bend, H = positive hip compensation

Row 1 (front): K60 crouch
  0: K60 H+18   1: K60 H+30   2: K60 H+42

Row 2 (middle): K70 deeper
  3: K70 H+21   4: K70 H+35   5: K70 H+49

Row 3 (back): K80 deepest
  6: K80 H+24   7: K80 H+40   8: K80 H+56

Press PLAY, then F to frame all.
""")
    print("=" * 60)
    sys.stdout.flush()

    app = omni.kit.app.get_app_interface()
    frame_count = 0
    physics_running = False
    dt = 1.0 / 60.0

    def set_joint_target(robot_joints_path, joint_name, angle):
        joint_path = f"{robot_joints_path}/{joint_name}"
        joint_prim = stage.GetPrimAtPath(joint_path)
        if joint_prim.IsValid():
            drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
            if drive:
                drive.GetTargetPositionAttr().Set(float(angle))

    try:
        while app.is_running():
            app.update()

            timeline = omni.timeline.get_timeline_interface()
            if timeline.is_playing():
                if not physics_running:
                    physics_running = True
                    frame_count = 0
                    print("\nPhysics started! Running behaviors...")
                    sys.stdout.flush()

                frame_count += 1

                # Run each robot's controller
                for robot in robots:
                    jp = robot['joints_path']
                    targets = robot['controller'].get_targets(dt)

                    for joint_name, angle in targets.items():
                        set_joint_target(jp, joint_name, angle)

                if frame_count > 0 and frame_count % 300 == 0:
                    print(f"[{frame_count}] Running...")
                    sys.stdout.flush()
            else:
                physics_running = False

    except KeyboardInterrupt:
        print("\nStopped")

    simulation_app.close()


if __name__ == "__main__":
    main()
