# AT-ST Environment Configuration for RL Training
# Based on H1 rough environment config

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as mdp

# Import locomotion-specific MDP functions (for biped rewards)
from isaaclab_tasks.manager_based.locomotion.velocity.mdp import rewards as locomotion_rewards

# Import our AT-ST config
from atst_robot_cfg import ATST_CFG


@configclass
class ATSTSceneCfg(InteractiveSceneCfg):
    """Configuration for the AT-ST scene."""

    # Ground plane
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        debug_vis=False,
    )

    # Robot
    robot = ATST_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # Contact sensors - track all body parts for termination and rewards
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*_link",
        history_length=3,
        track_air_time=True,
    )

    # Lights
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            color=(0.75, 0.75, 0.75),
        ),
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=0.5,
        use_default_offset=True,
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # Velocity command
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})

        # Base state
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))

        # Joint state
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-1.5, n_max=1.5))

        # Actions
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class CommandsCfg:
    """Command specifications for the MDP."""

    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),
        rel_standing_envs=0.02,
        rel_heading_envs=1.0,
        heading_command=True,
        heading_control_stiffness=0.5,
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.5, 1.0),  # Forward/backward
            lin_vel_y=(-0.0, 0.0),  # No sideways (biped)
            ang_vel_z=(-1.0, 1.0),  # Turning
            heading=(-3.14, 3.14),
        ),
    )


@configclass
class EventsCfg:
    """Event specifications for the MDP."""

    # Startup
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.8, 1.2),
            "dynamic_friction_range": (0.6, 0.8),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
    )

    # Reset
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.9, 1.1),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # Positive rewards - use locomotion-specific tracking functions
    track_lin_vel_xy_exp = RewTerm(
        func=locomotion_rewards.track_lin_vel_xy_yaw_frame_exp,
        weight=1.5,
        params={"command_name": "base_velocity", "std": 0.5},
    )
    track_ang_vel_z_exp = RewTerm(
        func=locomotion_rewards.track_ang_vel_z_world_exp,
        weight=0.75,
        params={"command_name": "base_velocity", "std": 0.5},
    )

    # Stay alive bonus
    alive = RewTerm(func=mdp.is_alive, weight=0.5)

    # Penalties
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)  # Penalize vertical bouncing
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)  # Penalize rolling
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-1.0)  # Stay upright

    # Energy efficiency
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    dof_torques_l2 = RewTerm(func=mdp.joint_torques_l2, weight=-0.0001)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)

    # Termination penalty
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)

    # Feet rewards - biped-specific air time reward
    feet_air_time = RewTerm(
        func=locomotion_rewards.feet_air_time_positive_biped,
        weight=0.25,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*ankle_link"),
            "threshold": 0.4,
        },
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    # Terminate if body touches ground
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="torso_link"), "threshold": 1.0},
    )


@configclass
class ATSTEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the AT-ST locomotion environment."""

    # Scene
    scene: ATSTSceneCfg = ATSTSceneCfg(num_envs=4096, env_spacing=2.5)

    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    events: EventsCfg = EventsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        """Post initialization."""
        # General settings
        self.decimation = 4
        self.episode_length_s = 20.0

        # Simulation settings
        self.sim.dt = 0.005  # 200Hz physics
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = True

        # Physics settings
        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625


@configclass
class ATSTEnvCfg_PLAY(ATSTEnvCfg):
    """Configuration for playing with trained AT-ST."""

    def __post_init__(self):
        super().__post_init__()

        # Smaller scene for play
        self.scene.num_envs = 50
        self.episode_length_s = 40.0

        # Disable noise during play
        self.observations.policy.enable_corruption = False
