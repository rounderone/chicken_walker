# AT-ST Robot Configuration for Isaac Lab
# Digitigrade (chicken walker) configuration with 8 joints

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

# AT-ST Articulation Configuration
# Joint structure per leg: hip_pitch -> hip_roll -> knee -> ankle
ATST_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="D:/Projects/ATST/usd/atst_rl.usda",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.80),  # Starting height (taller model ~0.75m)
        joint_pos={
            # Hip pitch - forward/backward swing (Y axis, -60 to 60 deg)
            ".*_hip_pitch": 0.0,
            # Hip roll - side-to-side tilt (X axis, -15 to 15 deg)
            ".*_hip_roll": 0.0,
            # Knee - main leg bend (Y axis, -120 to 30 deg)
            # Shin is already angled 45° in model, so 0 = digitigrade stance
            ".*_knee": 0.0,
            # Ankle - foot pitch (Y axis, -60 to 60 deg)
            ".*_ankle": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        # Main leg actuators - hip pitch and knee (high torque)
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_pitch", ".*_knee"],
            effort_limit_sim=300,
            stiffness={
                ".*_hip_pitch": 200.0,
                ".*_knee": 200.0,
            },
            damping={
                ".*_hip_pitch": 5.0,
                ".*_knee": 5.0,
            },
        ),
        # Hip roll - smaller range, used for balance
        "hip_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll"],
            effort_limit_sim=150,
            stiffness={".*_hip_roll": 100.0},
            damping={".*_hip_roll": 3.0},
        ),
        # Ankle - foot control
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle"],
            effort_limit_sim=100,
            stiffness={".*_ankle": 50.0},
            damping={".*_ankle": 4.0},
        ),
    },
)
"""Configuration for the AT-ST digitigrade bipedal walker.

Joint structure (8 joints total, 4 per leg):
- hip_pitch: Forward/backward leg swing (Y axis)
- hip_roll: Side-to-side leg tilt for balance (X axis)
- knee: Main leg bend, "reverse knee" joint (Y axis)
- ankle: Foot pitch control (Y axis)

The shin segments are pre-angled 45° backward in the USD model,
giving the characteristic "chicken walker" appearance.
"""
