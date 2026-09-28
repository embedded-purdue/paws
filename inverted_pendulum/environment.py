"""
Cartpole balancing environment, written directly on top of MuJoCo.

We subclass gymnasium.Env only so our env has the standard reset()/step()
interface that training libraries (Stable-Baselines3) expect. All the physics,
observations, rewards, and "when is the episode over" logic is ours, below.
The robot dog env will follow this exact same pattern.

The loop an RL algorithm runs:
    obs, info = env.reset()
    while True:
        action = policy(obs)
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            obs, info = env.reset()
"""
import os

import gymnasium as gym
import mujoco
import numpy as np

XML_PATH = os.path.join(os.path.dirname(__file__), "pendulum.xml")


class CartPoleEnv(gym.Env):

    # ---------------- Settings you can tweak ----------------
    FRAME_SKIP = 10            # physics steps per policy step. 10 x 0.002 s = 0.02 s -> policy runs at 50 Hz
    MAX_STEPS = 1000           # episode length cap (1000 x 0.02 s = 20 s)
    MAX_ANGLE = 0.4            # rad (~23 deg). Pole tilted past this = fell over, episode ends
    MAX_CART_POS = 1.7         # m. Cart past this = about to hit the end of the rail, episode ends
    START_NOISE = 0.05         # how far from perfectly still each episode starts (rad, m, and per-second)

    # Random pushes on the pole, so the policy learns to recover from bumps
    PUSH_CHANCE = 0.02         # chance per policy step that a new push starts (0.02 at 50 Hz = about 1 per second)
    PUSH_MAX_FORCE = 3.0       # N. Each push is a random sideways force between -this and +this.
                               # ~3 N is about the most the 20 N motor can physically recover from,
                               # so training sees some pushes it cannot survive. That is OK.
    PUSH_DURATION = (0.1, 0.3) # s. Each push lasts a random time in this range

    def __init__(self, random_pushes=False):
        # Load the model (the fixed description) and data (the changing state: positions, velocities...)
        self.model = mujoco.MjModel.from_xml_path(XML_PATH)
        self.data = mujoco.MjData(self.model)

        self.random_pushes = random_pushes
        self.pole_id = self.model.body("pole").id
        self.push_force = 0.0       # current push (N along x)
        self.push_steps_left = 0    # how many more policy steps the current push lasts

        # Action: one number in [-1, 1], sent to the cart motor (x20 gear = up to 20 N)
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)

        # Observation: [cart position, cart velocity, pole angle, pole angular velocity]
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)

        self.steps = 0

    def _get_obs(self):
        # sensordata holds the 4 sensors from the XML, in order:
        # cart_pos, cart_vel, pole_angle, pole_vel
        return self.data.sensordata.astype(np.float32).copy()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)  # sets up self.np_random so runs are reproducible with a seed

        # Put everything back to the XML's starting state (cart centered, pole straight up)
        mujoco.mj_resetData(self.model, self.data)

        # Nudge the start state a little so the policy can't memorize one exact situation
        n = self.START_NOISE
        self.data.qpos[:] = self.np_random.uniform(-n, n, size=self.model.nq)  # [cart x, pole angle]
        self.data.qvel[:] = self.np_random.uniform(-n, n, size=self.model.nv)  # [cart speed, pole speed]

        # Recompute sensors etc. for the new state without advancing time
        mujoco.mj_forward(self.model, self.data)

        self.steps = 0
        self.push_force = 0.0
        self.push_steps_left = 0
        return self._get_obs(), {}

    def _update_push(self):
        # Maybe start a new push, then apply the current one (or nothing) to the pole
        if self.push_steps_left == 0 and self.np_random.random() < self.PUSH_CHANCE:
            self.push_force = self.np_random.uniform(-self.PUSH_MAX_FORCE, self.PUSH_MAX_FORCE)
            step_time = self.FRAME_SKIP * self.model.opt.timestep
            self.push_steps_left = int(self.np_random.uniform(*self.PUSH_DURATION) / step_time)

        if self.push_steps_left > 0:
            self.push_steps_left -= 1
        else:
            self.push_force = 0.0

        # xfrc_applied = external [force xyz, torque xyz] on each body, applied at its center of mass
        self.data.xfrc_applied[self.pole_id, 0] = self.push_force

    def step(self, action):
        if self.random_pushes:
            self._update_push()

        # Send the action to the motor, then run the physics for FRAME_SKIP steps
        self.data.ctrl[:] = np.clip(action, -1.0, 1.0)
        mujoco.mj_step(self.model, self.data, nstep=self.FRAME_SKIP)
        self.steps += 1

        obs = self._get_obs()
        cart_pos, cart_vel, angle, angle_vel = obs

        # terminated = the task failed (pole fell or cart went too far)
        terminated = bool(abs(angle) > self.MAX_ANGLE or abs(cart_pos) > self.MAX_CART_POS)
        # truncated = we just ran out of time (not a failure)
        truncated = self.steps >= self.MAX_STEPS

        # Reward: +1 for every step the pole stays up, minus small penalties
        # for drifting from center and for using lots of force.
        reward = float(1.0 - 0.1 * cart_pos**2 - 0.01 * action[0] ** 2)
        if terminated:
            reward = 0.0

        return obs, reward, terminated, truncated, {}


if __name__ == "__main__":
    # Quick sanity check: random actions. The pole should fall within a second or two.
    env = CartPoleEnv()
    obs, _ = env.reset(seed=0)
    total = 0.0
    for i in range(1000):
        obs, r, term, trunc, _ = env.step(env.action_space.sample())
        total += r
        if term or trunc:
            break
    print(f"random policy lasted {i + 1} steps ({(i + 1) * 0.02:.2f} s), total reward {total:.1f}")
