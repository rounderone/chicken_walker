# AT-ST Policy Visualization Script
# Watch the trained AT-ST walk in Isaac Sim
# Run with: D:\Projects\ATST\venv_isaaclab\Scripts\python.exe play_atst.py

import argparse
import os
import sys

print("=" * 60, flush=True)
print("AT-ST Policy Visualization", flush=True)
print("=" * 60, flush=True)

# Parse arguments before launching Isaac Sim
parser = argparse.ArgumentParser(description="Visualize trained AT-ST policy")
parser.add_argument("--checkpoint", type=str,
                    default="D:/Projects/ATST/logs/atst_training/2026-02-04_18-25-00/model_final.pt",
                    help="Path to trained model checkpoint")
parser.add_argument("--num_envs", type=int, default=16, help="Number of environments to visualize")
args = parser.parse_args()

print(f"Loading checkpoint: {args.checkpoint}", flush=True)
print(f"Number of environments: {args.num_envs}", flush=True)

# Launch Isaac Sim WITH rendering (not headless)
print("\nLaunching Isaac Sim with visualization...", flush=True)
from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=False)
simulation_app = app_launcher.app

print("Isaac Sim launched!", flush=True)

# Now import everything else
import torch
from rsl_rl.runners import OnPolicyRunner

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

# Add our config directory to path
sys.path.insert(0, "D:/Projects/ATST/isaac_sim")

# Import our AT-ST configs
from atst_env_cfg import ATSTEnvCfg_PLAY

print("\nAT-ST configs imported!", flush=True)


def create_rsl_rl_config():
    """Create RSL-RL config for inference."""
    return {
        "seed": 42,
        "device": "cuda:0",
        "num_steps_per_env": 24,
        "max_iterations": 1,
        "save_interval": 1,
        "experiment_name": "atst_play",
        "obs_groups": {
            "policy": ["policy"],
        },
        "policy": {
            "class_name": "ActorCritic",
            "init_noise_std": 1.0,
            "actor_obs_normalization": False,
            "critic_obs_normalization": False,
            "actor_hidden_dims": [256, 256, 128],
            "critic_hidden_dims": [256, 256, 128],
            "activation": "elu",
        },
        "algorithm": {
            "class_name": "PPO",
            "value_loss_coef": 1.0,
            "use_clipped_value_loss": True,
            "clip_param": 0.2,
            "entropy_coef": 0.01,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "learning_rate": 1.0e-3,
            "schedule": "adaptive",
            "gamma": 0.99,
            "lam": 0.95,
            "desired_kl": 0.01,
            "max_grad_norm": 1.0,
        },
    }


def main():
    print("\n" + "=" * 60, flush=True)
    print("Setting up visualization environment...", flush=True)
    print("=" * 60, flush=True)

    # Use play config (fewer envs, no noise)
    env_cfg = ATSTEnvCfg_PLAY()
    env_cfg.scene.num_envs = args.num_envs

    print(f"Environment: {args.num_envs} AT-ST robots", flush=True)

    # Create the environment
    print("\nCreating environment...", flush=True)
    env = ManagerBasedRLEnv(cfg=env_cfg)
    print("Environment created!", flush=True)

    # Wrap for RSL-RL
    env = RslRlVecEnvWrapper(env, clip_actions=1.0)

    # Create runner and load checkpoint
    agent_cfg = create_rsl_rl_config()
    runner = OnPolicyRunner(env, agent_cfg, log_dir=None, device="cuda:0")

    print(f"\nLoading trained policy from: {args.checkpoint}", flush=True)
    runner.load(args.checkpoint)
    print("Policy loaded successfully!", flush=True)

    # Get the policy
    policy = runner.get_inference_policy(device="cuda:0")

    # Run visualization loop
    print("\n" + "=" * 60, flush=True)
    print("RUNNING VISUALIZATION", flush=True)
    print("Press Ctrl+C to stop", flush=True)
    print("=" * 60, flush=True)

    # Get initial observations (returns dict with 'policy' key)
    obs = env.get_observations()

    step = 0
    try:
        while simulation_app.is_running():
            # Get action from policy
            with torch.no_grad():
                actions = policy(obs)

            # Step environment - RSL-RL wrapper returns (obs, rewards, dones, infos)
            obs, rewards, dones, infos = env.step(actions)

            step += 1
            if step % 500 == 0:
                print(f"Step {step}...", flush=True)

    except KeyboardInterrupt:
        print("\n\nStopped by user.", flush=True)

    # Cleanup
    env.close()
    print("\nVisualization complete!", flush=True)


if __name__ == "__main__":
    main()
    simulation_app.close()
