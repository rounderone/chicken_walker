# AT-ST RL Training Script
# Train the AT-ST biped to walk using RSL-RL PPO
# Run with: D:\Projects\ATST\venv_isaaclab\Scripts\python.exe train_atst.py

import argparse
import os
import sys

print("=" * 60, flush=True)
print("AT-ST RL Training Script", flush=True)
print("=" * 60, flush=True)

# Parse arguments before launching Isaac Sim
parser = argparse.ArgumentParser(description="Train AT-ST walking policy with RSL-RL")
parser.add_argument("--num_envs", type=int, default=1024, help="Number of parallel environments")
parser.add_argument("--max_iterations", type=int, default=1500, help="Training iterations")
parser.add_argument("--headless", action="store_true", help="Run headless (no rendering)")
parser.add_argument("--resume", type=str, default=None, help="Path to checkpoint to resume from")
args = parser.parse_args()

print(f"Configuration:", flush=True)
print(f"  Environments: {args.num_envs}", flush=True)
print(f"  Max iterations: {args.max_iterations}", flush=True)
print(f"  Headless: {args.headless}", flush=True)
print(f"  Resume: {args.resume}", flush=True)

# Launch Isaac Sim
print("\nLaunching Isaac Sim...", flush=True)
from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=args.headless)
simulation_app = app_launcher.app

print("Isaac Sim launched successfully!", flush=True)

# Now import everything else
import time
from datetime import datetime

import gymnasium as gym
import torch
from rsl_rl.runners import OnPolicyRunner

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

# Add our config directory to path
sys.path.insert(0, "D:/Projects/ATST/isaac_sim")

# Import our AT-ST configs
from atst_env_cfg import ATSTEnvCfg
from atst_robot_cfg import ATST_CFG

print("\nAT-ST configs imported successfully!", flush=True)

# Enable TF32 for faster training on RTX GPUs
torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cudnn.deterministic = False
torch.backends.cudnn.benchmark = False


def create_rsl_rl_config():
    """Create RSL-RL PPO configuration for AT-ST training."""
    return {
        "seed": 42,
        "device": "cuda:0",
        "num_steps_per_env": 24,
        "max_iterations": args.max_iterations,
        "save_interval": 50,
        "experiment_name": "atst_walk",
        # Map environment observation groups to algorithm observation sets
        "obs_groups": {
            "policy": ["policy"],  # Maps to our ObservationsCfg.policy group
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
    print("Setting up training environment...", flush=True)
    print("=" * 60, flush=True)

    # Configure the environment
    env_cfg = ATSTEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.seed = 42

    print(f"\nEnvironment configuration:", flush=True)
    print(f"  Number of environments: {env_cfg.scene.num_envs}", flush=True)
    print(f"  Episode length: {env_cfg.episode_length_s}s", flush=True)
    print(f"  Physics dt: {env_cfg.sim.dt}s", flush=True)
    print(f"  Decimation: {env_cfg.decimation}", flush=True)

    # Create logging directory
    log_root = "D:/Projects/ATST/logs/atst_training"
    log_dir = os.path.join(log_root, datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
    os.makedirs(log_dir, exist_ok=True)
    print(f"\nLogging to: {log_dir}", flush=True)

    # Create the environment
    print("\nCreating environment...", flush=True)
    env = ManagerBasedRLEnv(cfg=env_cfg)
    print(f"Environment created!", flush=True)
    print(f"  Observation space: {env.observation_space}", flush=True)
    print(f"  Action space: {env.action_space}", flush=True)

    # Wrap for RSL-RL
    print("\nWrapping environment for RSL-RL...", flush=True)
    env = RslRlVecEnvWrapper(env, clip_actions=1.0)  # Clip actions to [-1, 1]

    # Create RSL-RL config
    agent_cfg = create_rsl_rl_config()

    # Create the runner
    print("\nCreating PPO runner...", flush=True)
    runner = OnPolicyRunner(env, agent_cfg, log_dir=log_dir, device="cuda:0")

    # Resume from checkpoint if provided
    if args.resume:
        print(f"\nLoading checkpoint from: {args.resume}", flush=True)
        runner.load(args.resume)

    # Start training
    print("\n" + "=" * 60, flush=True)
    print("STARTING TRAINING", flush=True)
    print("=" * 60, flush=True)
    print(f"Training for {args.max_iterations} iterations...", flush=True)
    print("Press Ctrl+C to stop training early.", flush=True)
    print("", flush=True)

    start_time = time.time()

    try:
        runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)
    except KeyboardInterrupt:
        print("\n\nTraining interrupted by user.", flush=True)

    elapsed = time.time() - start_time
    print(f"\nTraining completed in {elapsed:.1f} seconds ({elapsed/60:.1f} minutes)", flush=True)

    # Save final checkpoint
    final_checkpoint = os.path.join(log_dir, "model_final.pt")
    runner.save(final_checkpoint)
    print(f"\nFinal model saved to: {final_checkpoint}", flush=True)

    # Close environment
    env.close()
    print("\nEnvironment closed.", flush=True)

    return log_dir


if __name__ == "__main__":
    log_dir = main()
    print("\n" + "=" * 60, flush=True)
    print("TRAINING COMPLETE", flush=True)
    print(f"Logs and checkpoints saved to: {log_dir}", flush=True)
    print("=" * 60, flush=True)

    # Close Isaac Sim
    simulation_app.close()
