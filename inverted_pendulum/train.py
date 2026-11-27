"""
Train a policy to balance the cartpole with PPO (Stable-Baselines3).

Run:    python train.py
Watch:  tensorboard --logdir logs      (then open http://localhost:6006)
Result: models/ppo_cartpole.zip
"""
import os

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.env_util import make_vec_env

from environment import CartPoleEnv

HERE = os.path.dirname(__file__)
TOTAL_STEPS = 500_000   # how long to train. Recovering from pushes takes longer to learn than plain balancing.
NUM_ENVS = 4            # copies of the env run side by side to collect experience faster

if __name__ == "__main__":
    # Catches mistakes in our env (wrong shapes/dtypes etc.) before a long training run
    check_env(CartPoleEnv(random_pushes=True))

    # random_pushes=True: the pole gets shoved at random times, so the policy learns to recover
    env = make_vec_env(CartPoleEnv, n_envs=NUM_ENVS, env_kwargs={"random_pushes": True})

    # PPO with default settings: a small neural net (2 layers x 64) maps obs -> action
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=os.path.join(HERE, "logs"))
    model.learn(total_timesteps=TOTAL_STEPS)

    model.save(os.path.join(HERE, "models", "ppo_cartpole"))
    print("saved to models/ppo_cartpole.zip")
