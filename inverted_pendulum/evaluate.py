"""
Watch the trained policy balance the pole in the MuJoCo viewer, and push it with the mouse.

Run: python evaluate.py
Close the viewer window to stop.

Pushing with the mouse:
  1. Double-click the pole (or cart) to select it.
  2. Hold Ctrl and RIGHT-drag to pull it. The force is like a spring:
     the further you drag from the body, the harder it pulls.
     (Ctrl + LEFT-drag applies a twisting torque instead.)
  3. Let go to stop pushing. Double-click empty space to deselect.
"""
import os
import time

import mujoco
import mujoco.viewer
import numpy as np
from stable_baselines3 import PPO

from environment import CartPoleEnv

HERE = os.path.dirname(__file__)

# Multiplies the mouse force. 1.0 = MuJoCo's default spring strength, which on the
# pole is about 330 N per meter of drag (way more than the 20 N motor can fight).
# 0.2 -> about 66 N per meter, so a 10 cm drag pushes with ~6.6 N.
# Raise it for harder shoves, lower it for gentle nudges.
PUSH_STRENGTH = 0.2

env = CartPoleEnv()
model = PPO.load(os.path.join(HERE, "models", "ppo_cartpole"))
step_time = env.FRAME_SKIP * env.model.opt.timestep  # sim seconds per policy step (0.02)

obs, _ = env.reset()
episode_steps = 0

# launch_passive: the viewer just draws env.data; our loop below drives the simulation
with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
    while viewer.is_running():
        start = time.time()

        action, _ = model.predict(obs, deterministic=True)  # deterministic = no exploration noise

        # lock() so the viewer doesn't read data while we're changing it
        with viewer.lock():
            # xfrc_applied = external force [fx fy fz] and torque [tx ty tz] on each body.
            # Clear last step's push, then turn the mouse drag into a force on the selected body.
            env.data.xfrc_applied[:] = 0
            mujoco.mjv_applyPerturbForce(env.model, env.data, viewer.perturb)
            env.data.xfrc_applied *= PUSH_STRENGTH

            # xfrc_applied stays on for all FRAME_SKIP physics steps inside env.step
            obs, reward, terminated, truncated, _ = env.step(action)

        episode_steps += 1
        viewer.sync()

        # Show how hard you're pushing (newtons) while dragging
        push = np.linalg.norm(env.data.xfrc_applied[:, :3], axis=1).max()
        if push > 0:
            print(f"push force: {push:5.1f} N", end="\r")

        if terminated or truncated:
            print(f"episode ended after {episode_steps * step_time:.1f} s "
                  f"({'fell' if terminated else 'time limit'})")
            obs, _ = env.reset()
            episode_steps = 0

        # sleep so it plays at real-time speed instead of as fast as possible
        time.sleep(max(0.0, step_time - (time.time() - start)))
