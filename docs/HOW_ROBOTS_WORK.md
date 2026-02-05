# How Robots Work in Isaac Sim - Complete Technical Guide

This document explains how we got humanoid robots (Unitree H1, Cassie) and our custom AT-ST walker working in Isaac Sim with physics simulation and RL training.

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Loading Built-in Robots (H1, Cassie)](#loading-built-in-robots)
3. [Creating Custom Robots (AT-ST)](#creating-custom-robots)
4. [Isaac Lab RL Training Pipeline](#isaac-lab-rl-training-pipeline)
5. [Key Differences: Why Built-in Robots "Just Work"](#key-differences)
6. [Commands Reference](#commands-reference)

---

## Architecture Overview

Isaac Sim uses a layered architecture:

```
┌─────────────────────────────────────────────────────────┐
│                  Isaac Lab (RL Framework)                │
│  - ManagerBasedRLEnv for parallel training              │
│  - RSL-RL PPO algorithm                                 │
│  - Reward/termination managers                          │
├─────────────────────────────────────────────────────────┤
│                  Isaac Sim Core APIs                     │
│  - World, Scene, Robot classes                          │
│  - ArticulationController                               │
│  - PhysX GPU simulation                                 │
├─────────────────────────────────────────────────────────┤
│                  USD + PhysX Scene Graph                 │
│  - ArticulationRootAPI (robot base)                     │
│  - RigidBodyAPI (links/bodies)                          │
│  - RevoluteJoint (joints with limits & drives)          │
│  - CollisionAPI (collision geometry)                    │
├─────────────────────────────────────────────────────────┤
│                  Native USD Geometry                     │
│  - Cube, Capsule, Sphere, Mesh                          │
│  - These render automatically in viewport               │
└─────────────────────────────────────────────────────────┘
```

---

## Loading Built-in Robots

### Unitree H1 Humanoid

**File:** `isaac_sim/h1_standalone.py`

Built-in robots are loaded from NVIDIA's Nucleus asset server:

```python
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Get the assets path (requires Nucleus connection)
assets_root = get_assets_root_path()
h1_usd = f"{assets_root}/Isaac/Robots/Unitree/H1/h1.usd"

# Create world with ground
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Load robot at position
h1 = world.scene.add(
    Robot(
        prim_path="/World/H1",
        usd_path=h1_usd,
        name="h1",
        position=np.array([0, 0, 1.05]),  # Starting height
    )
)

world.reset()

# Control loop - maintain standing pose
while simulation_app.is_running():
    world.step(render=True)
    h1.set_joint_position_targets(h1.get_joint_positions())
```

**Why it works:**
- NVIDIA's USD files have pre-configured ArticulationRootAPI
- All physics properties (mass, inertia, collision) are set
- Visual meshes are embedded in the USD (not external references)
- Joint drives have proper stiffness/damping

### Cassie Bipedal Robot

**File:** `isaac_scripts/cassie_scene.py`

Similar loading, but with a gait controller:

```python
# Load from Nucleus
cassie_usd = f"{assets_root}/Isaac/Robots/Cassie/cassie.usd"

cassie = world.scene.add(
    Robot(
        prim_path="/World/Cassie",
        usd_path=cassie_usd,
        name="cassie",
        position=np.array([0, 0, 1.0]),
    )
)

# Configure articulation
cassie.set_joint_gains(
    kps=np.ones(num_dofs) * 100.0,  # Position gain
    kds=np.ones(num_dofs) * 0.1,     # Damping gain
)

# Create sinusoidal gait controller
class CassieGaitController:
    def __init__(self, num_dofs, cycle_time=1.0):
        self.neutral_pos = np.array([0, 0, -0.5, 0.5, 0, 0, -0.5, 0.5])
        self.amplitude = np.array([0.1, 0.3, 0.3, 0.2] * 2)

    def update(self, dt):
        phase_left = (2 * np.pi * self.t / self.cycle_time)
        phase_right = phase_left + np.pi  # 180 degrees out of phase

        target = self.neutral_pos.copy()
        target[0:4] += self.amplitude[0:4] * np.sin(phase_left)
        target[4:8] += self.amplitude[4:8] * np.sin(phase_right)
        return target
```

---

## Creating Custom Robots (AT-ST)

### Step 1: Create USD with Native Geometry

**File:** `isaac_sim/create_atst_usd.py`

**Key insight:** Use native USD geometry types (Cube, Capsule) that render automatically.

```python
from pxr import UsdGeom, UsdPhysics, Gf

# Create root with ArticulationRootAPI
root = UsdGeom.Xform.Define(stage, "/ATST")
UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())

def create_rigid_body(name, pos, size, mass, shape="capsule"):
    """Create a physics-enabled body with collision."""
    path = f"/ATST/{name}"

    if shape == "capsule":
        geom = UsdGeom.Capsule.Define(stage, path)
        geom.CreateRadiusAttr(size[0])
        geom.CreateHeightAttr(size[1])
    else:
        geom = UsdGeom.Cube.Define(stage, path)
        geom.CreateSizeAttr(1.0)

    prim = stage.GetPrimAtPath(path)

    # Transform
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if shape == "box":
        xf.AddScaleOp().Set(Gf.Vec3f(*size))

    # Physics APIs
    UsdPhysics.RigidBodyAPI.Apply(prim)      # Makes it dynamic
    UsdPhysics.CollisionAPI.Apply(prim)       # Enables collision
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)

    return path

# Create body parts
body = create_rigid_body("torso_link", (0, 0, 0.74), (0.3, 0.25, 0.25), 10.0, "box")
l_thigh = create_rigid_body("left_upper_leg_link", (0.17, 0.2, 0.42), (0.035, 0.23), 2.0, "capsule")
# ... etc for each body part
```

### Step 2: Create Joints with Drives

```python
def create_revolute_joint(name, parent_body, child_body, anchor_pos, axis="Y"):
    joint = UsdPhysics.RevoluteJoint.Define(stage, f"/ATST/joints/{name}")

    # Connect bodies
    joint.CreateBody0Rel().SetTargets([parent_body])
    joint.CreateBody1Rel().SetTargets([child_body])
    joint.CreateAxisAttr(axis)

    # Joint limits (degrees)
    joint.CreateLowerLimitAttr(-60.0)
    joint.CreateUpperLimitAttr(60.0)

    # Calculate local positions relative to each body
    joint.CreateLocalPos0Attr().Set(local0)
    joint.CreateLocalPos1Attr().Set(local1)

    # Add drive for position control (PD controller)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(200.0)   # P gain
    drive.CreateDampingAttr(5.0)        # D gain
    drive.CreateMaxForceAttr(300.0)
    drive.CreateTargetPositionAttr(0.0)
```

### Step 3: AT-ST Joint Structure (8 joints total)

```
/ATST (ArticulationRoot)
├── torso_link (Cube - main body)
├── left_hip_link (Capsule - hip connector)
├── left_upper_leg_link (Capsule - thigh)
├── left_lower_leg_link (Capsule - shin, rotated 45°)
├── left_ankle_link (Cube - foot)
├── right_hip_link
├── right_upper_leg_link
├── right_lower_leg_link
├── right_ankle_link
└── joints/
    ├── left_hip_pitch (Y axis, ±60°)
    ├── left_hip_roll (X axis, ±15°)
    ├── left_knee (Y axis, -120° to +30°)
    ├── left_ankle (Y axis, ±60°)
    └── (same for right leg)
```

**Key dimensions:**
- Total height: ~0.75m
- Shin angle: 45° backward (digitigrade/chicken walker look)
- Leg spacing: 0.40m
- Foot size: 0.18m × 0.12m × 0.03m

---

## Isaac Lab RL Training Pipeline

### Step 4: Robot Configuration

**File:** `isaac_sim/atst_robot_cfg.py`

```python
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

ATST_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="D:/Projects/ATST/usd/atst_rl.usda",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.80),  # Starting height
        joint_pos={".*": 0.0},  # All joints at neutral
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_pitch", ".*_knee"],
            effort_limit_sim=300,
            stiffness={".*_hip_pitch": 200.0, ".*_knee": 200.0},
            damping={".*_hip_pitch": 5.0, ".*_knee": 5.0},
        ),
        "hip_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll"],
            stiffness={".*_hip_roll": 100.0},
            damping={".*_hip_roll": 3.0},
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle"],
            stiffness={".*_ankle": 50.0},
            damping={".*_ankle": 4.0},
        ),
    },
)
```

### Step 5: Environment Configuration

**File:** `isaac_sim/atst_env_cfg.py`

```python
@configclass
class ATSTEnvCfg(ManagerBasedRLEnvCfg):
    # Scene with terrain and lighting
    scene: ATSTSceneCfg = ATSTSceneCfg(num_envs=4096, env_spacing=2.5)

    # Observations: velocity, gravity, joint state, last action
    observations: ObservationsCfg = ObservationsCfg()

    # Actions: joint position targets
    actions: ActionsCfg = ActionsCfg()

    # Velocity commands: forward/backward, turning
    commands: CommandsCfg = CommandsCfg()

    # Rewards (biped-specific)
    rewards: RewardsCfg = RewardsCfg()

    # Termination: timeout, body contact with ground
    terminations: TerminationsCfg = TerminationsCfg()
```

**Reward structure:**
```python
@configclass
class RewardsCfg:
    # Positive rewards
    track_lin_vel_xy_exp = RewTerm(weight=1.5)   # Track velocity command
    track_ang_vel_z_exp = RewTerm(weight=0.75)   # Track turning
    alive = RewTerm(weight=0.5)                   # Survive bonus
    feet_air_time = RewTerm(weight=0.25)         # Biped gait reward

    # Penalties
    lin_vel_z_l2 = RewTerm(weight=-2.0)          # No bouncing
    flat_orientation_l2 = RewTerm(weight=-1.0)   # Stay upright
    termination_penalty = RewTerm(weight=-200.0) # Death penalty
    action_rate_l2 = RewTerm(weight=-0.01)       # Smooth actions
```

### Step 6: Training Script

**File:** `isaac_sim/train_atst.py`

```python
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from rsl_rl.runners import OnPolicyRunner

# Create environment with 4096 parallel robots
env_cfg = ATSTEnvCfg()
env_cfg.scene.num_envs = 4096
env = ManagerBasedRLEnv(cfg=env_cfg)

# Wrap for RSL-RL
env = RslRlVecEnvWrapper(env, clip_actions=1.0)

# PPO configuration
agent_cfg = {
    "policy": {
        "actor_hidden_dims": [256, 256, 128],
        "critic_hidden_dims": [256, 256, 128],
    },
    "algorithm": {
        "class_name": "PPO",
        "learning_rate": 1.0e-3,
        "num_learning_epochs": 5,
        "gamma": 0.99,
    },
}

# Train
runner = OnPolicyRunner(env, agent_cfg, log_dir=log_dir, device="cuda:0")
runner.learn(num_learning_iterations=1500)
runner.save("model_final.pt")
```

### Step 7: Visualization

**File:** `isaac_sim/play_atst.py`

```python
# Load trained policy
runner.load("path/to/model_final.pt")
policy = runner.get_inference_policy(device="cuda:0")

# Inference loop
obs = env.get_observations()
while simulation_app.is_running():
    with torch.no_grad():
        actions = policy(obs)
    obs, rewards, dones, infos = env.step(actions)
```

---

## Key Differences: Why Built-in Robots "Just Work"

| Aspect | Built-in Robots (H1, Cassie) | Custom URDF Import | Custom USD (Our AT-ST) |
|--------|------------------------------|--------------------|-----------------------|
| **Geometry** | Embedded in USD | External mesh references | Native USD (Cube/Capsule) |
| **Rendering** | Automatic | May fail to resolve | Automatic |
| **Physics** | Pre-configured | Needs conversion | Must add APIs manually |
| **Joints** | Complete setup | Often incomplete | Must create with drives |
| **Mass/Inertia** | Calibrated | Often wrong | Must calculate |

**The critical insight:**
- Built-in robots use **native USD geometry** embedded in the file
- URDF imports reference **external mesh files** that Isaac Sim may not resolve
- Our working AT-ST uses **native USD geometry** (Cubes, Capsules) which always renders

---

## Commands Reference

### Run Pre-built Robot Demos
```bash
# H1 Humanoid standing
D:\Projects\ATST\venv_isaaclab\Scripts\python.exe D:\Projects\ATST\isaac_sim\h1_standalone.py

# Cassie walking gait
D:\Projects\ATST\venv_isaaclab\Scripts\python.exe D:\Projects\ATST\isaac_scripts\cassie_scene.py
```

### Train Custom AT-ST
```bash
# Training (1024 parallel environments, 1500 iterations)
D:\Projects\ATST\venv_isaaclab\Scripts\python.exe D:\Projects\ATST\isaac_sim\train_atst.py --num_envs 1024 --max_iterations 1500

# Headless training (faster)
D:\Projects\ATST\venv_isaaclab\Scripts\python.exe D:\Projects\ATST\isaac_sim\train_atst.py --headless --num_envs 4096
```

### Visualize Trained Policy
```bash
# Watch 16 AT-STs walk
D:\Projects\ATST\venv_isaaclab\Scripts\python.exe D:\Projects\ATST\isaac_sim\play_atst.py --num_envs 16
```

### Generate New USD Model
```bash
# Create fresh atst_rl.usda with primitive geometry
D:\Projects\ATST\venv_isaaclab\Scripts\python.exe D:\Projects\ATST\isaac_sim\create_atst_usd.py
```

---

## File Reference

| File | Purpose |
|------|---------|
| `usd/atst_rl.usda` | Working physics model with native geometry |
| `isaac_sim/create_atst_usd.py` | Script that generates the USD |
| `isaac_sim/atst_robot_cfg.py` | Isaac Lab articulation config |
| `isaac_sim/atst_env_cfg.py` | RL environment with rewards/terminations |
| `isaac_sim/train_atst.py` | RSL-RL PPO training script |
| `isaac_sim/play_atst.py` | Trained policy visualization |
| `logs/atst_training/.../model_final.pt` | Trained neural network weights |

---

## Summary

1. **Built-in robots work** because they have embedded USD geometry and pre-configured physics
2. **URDF imports fail** because external mesh references don't resolve in Isaac Sim
3. **Our AT-ST works** because we used native USD geometry (Cubes, Capsules)
4. **RL training** uses 4096 parallel environments with PPO, achieving ~82% survival rate
5. **Next step**: Convert OBJ visual meshes to USD and overlay on physics model
