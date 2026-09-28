# Inverted Pendulum (Cartpole) — MuJoCo + RL

A practice project for learning MuJoCo and reinforcement learning before working on the PAWS robot dog.
A cart on a rail has a pole hinged on top. A neural-network policy learns to push the cart so the pole
stays upright, including after being shoved.

## Quick start

All commands run from this folder, using the venv at `Documents/ESAP_PAWS_Project/.venv`.

```
python environment.py           # sanity check: random actions, pole falls in < 1 s
python train.py                 # train a policy (~7 min on CPU) -> models/ppo_cartpole.zip
python evaluate.py              # watch the trained policy in the MuJoCo viewer
tensorboard --logdir logs       # training curves at http://localhost:6006
python -m mujoco.viewer --mjcf=pendulum.xml   # just look at the model, no policy
```

## Files

| File | What it is |
|---|---|
| `pendulum.xml` | The MuJoCo model: cart, pole, motor, sensors. |
| `environment.py` | `CartPoleEnv`: our own environment. Physics, observations, reward, episode rules, random pushes. |
| `train.py` | Trains a policy with PPO (Stable-Baselines3). |
| `evaluate.py` | Plays the trained policy in the viewer; lets you push the pole with the mouse. |
| `models/ppo_cartpole.zip` | Current policy, trained **with** random pushes. |
| `models/ppo_cartpole_nopush.zip` | First policy, trained **without** pushes. Kept for comparison. |
| `logs/` | TensorBoard training logs. |

## The model (`pendulum.xml`)

- **Cart:** 1 kg, slides along x on a rail. Travel limited to ±1.8 m. Has damping, friction and armature
  (motor/pulley inertia) so it behaves more like real hardware.
- **Pole:** 0.6 m long, 0.2 kg, on a hinge 0.08 m above the cart center. Angle 0 = straight up.
  It has no joint limit; if it falls it collides with the cart. Optional tip mass (currently 0).
- **Motor:** control signal −1 to 1, gear 20 → up to 20 N on the cart. A 0.02 s filter means the force
  can't change instantly, like a real motor.
- **Sensors:** cart position, cart velocity, pole angle, pole angular velocity (like encoders).
- **Physics:** timestep 0.002 s (500 Hz), `implicitfast` integrator.

## About Gymnasium (why we use it even though we wrote our own env)

Gymnasium here is **only an interface**, not a pre-made pendulum. None of Gymnasium's built-in
environments are used. `CartPoleEnv` subclasses `gymnasium.Env` so it has the standard shape:

```python
obs, info = env.reset()
obs, reward, terminated, truncated, info = env.step(action)
```

That standard shape lets off-the-shelf training code (Stable-Baselines3 PPO) train on it. All the MuJoCo
code is ours. The PAWS repo plans the same stack for the dog (`requirements.txt`: gymnasium,
stable-baselines3, torch; `sim/envs/` holds Gymnasium environment wrappers), so the dog env will follow
this same pattern.

## The environment (`environment.py`)

| Piece | Choice |
|---|---|
| Observation | `[cart_pos, cart_vel, pole_angle, pole_vel]`, read from the XML sensors (`data.sensordata`). |
| Action | One number in [−1, 1] → motor (up to 20 N). |
| Policy rate | `FRAME_SKIP = 10` physics steps per action → 0.02 s → **50 Hz**. |
| Reset | Back to the XML start state plus small random noise (`START_NOISE = 0.05`) on positions and velocities, so the policy can't memorize one exact start. |
| Reward | `1 − 0.1·cart_pos² − 0.01·action²` per step: +1 for staying up, small penalties for drifting off-center and for using lots of force. 0 on the step it fails. |
| Terminated (failure) | Pole tilts past `MAX_ANGLE = 0.4` rad (~23°), or cart goes past `MAX_CART_POS = 1.7` m. |
| Truncated (time limit) | `MAX_STEPS = 1000` steps = **20 s**. |

### Random pushes

`CartPoleEnv(random_pushes=True)` shoves the pole sideways at random so the policy learns to recover.
They are **off by default**, so `evaluate.py` only has your mouse pushes.

| Setting | Value | Meaning |
|---|---|---|
| `PUSH_CHANCE` | 0.02 | Chance per step a new push starts (about 1 per second at 50 Hz). |
| `PUSH_MAX_FORCE` | 3.0 N | Each push is a random force between −3 and +3 N along x. |
| `PUSH_DURATION` | 0.1–0.3 s | How long each push lasts. |

Pushes are applied through `data.xfrc_applied`, which is an external force/torque on a body applied at its center of mass.

**Why 3 N is the limit:** a 3 N push on the pole's center of mass (0.3 m up) makes about 0.9 N·m of torque
at the pivot. The most the 20 N motor can counter by accelerating the cart is about 1 N·m, and the motor has
a 0.02 s delay. So 3 N is about the edge of what's physically recoverable, and some training pushes can't
be survived. That's fine; the policy still learns from them.

## Training (`train.py`)

- **Algorithm:** PPO from Stable-Baselines3, default settings (`MlpPolicy` = small network, 2 layers × 64).
- **Parallel envs:** 4 copies (`NUM_ENVS`) to collect experience faster.
- **Length:** 500,000 steps (`TOTAL_STEPS`), about 7 minutes on CPU.
- **Env check:** `check_env` runs first to catch shape/dtype mistakes. It caught a real bug: the reward
  was a numpy number instead of a Python `float`.

### Training history

1. **No pushes, 200k steps (~3 min).** Average episode length went from ~16 steps (random) to the full
   1000 steps (20 s). Saved as `ppo_cartpole_nopush.zip`.
2. **Random pushes up to 3 N, 500k steps.** Average training episode length plateaued around 250 steps
   (5 s), because some 3 N pushes are unrecoverable. It was still far better against pushes than run 1 (table below).
   **This is the current `ppo_cartpole.zip`.**
3. **Random pushes up to 2 N, 500k steps.** Tried an easier setting. It came out slightly worse at 2 N
   (11/20 vs 16/20 full runs). That's probably normal run-to-run randomness in RL, so run 2's model was kept
   and the setting put back to 3 N.

### Results

Each cell is from 20 test episodes with random pushes up to the given size. A full run means surviving the whole 20 s.

| Largest push | No-push policy (run 1) | Push-trained policy (run 2, current) |
|---|---|---|
| 0 N | 20/20 full runs | 20/20 |
| 1 N | 1/20 (avg 5.5 s) | 20/20 |
| 1.5 N | — | 19/20 |
| 2 N | 0/20 (avg 1.3 s) | 16/20 (avg 17.4 s) |
| 3 N | 0/20 (avg 1.2 s) | 0/20 (avg 5.0 s) |

**Takeaway:** a policy only handles what it saw in training. Training with disturbances made it far more robust.
This is the same idea (domain randomization) the dog will need to work on real hardware.

## Pushing it with the mouse (`evaluate.py`)

1. Double-click the pole (or cart) to select it.
2. **Ctrl + right-drag** to pull. It works like a spring: force grows with the distance between the mouse and the grab point.
   (Ctrl + left-drag applies a twisting torque instead.)
3. Let go to stop. The terminal prints the current push force in newtons.

How it works: each policy step, `evaluate.py` clears `data.xfrc_applied`, then calls
`mujoco.mjv_applyPerturbForce(model, data, viewer.perturb)` to turn the mouse drag into a force. It then
scales that force by `PUSH_STRENGTH` and steps the env. The force stays on for all 10 physics substeps.

`PUSH_STRENGTH = 0.2` by default. At 1.0 (MuJoCo's default), dragging the pole gives about 330 N per meter,
far more than the motor can fight. At 0.2 it's about 66 N per meter, so a 10 cm drag ≈ 6.6 N. That's still
more than the policy can survive, so small quick drags are the fair test.

## Setup notes

- Packages installed into the venv (versions from the repo's `requirements.txt`):
  `stable-baselines3==2.9.0`, `torch==2.8.0` (CPU build), `tensorboard==2.20.0`.
- The venv has **mujoco 3.13.0**, but `requirements.txt` pins `mujoco==3.3.5`. It works as is; worth syncing with the team.
- If VS Code shows "Import could not be resolved", pick the venv's Python: Ctrl+Shift+P →
  "Python: Select Interpreter" → the `.venv` in ESAP_PAWS_Project.

## Ideas for next steps

- **Swing-up:** start with the pole hanging down. You'd need to change the reset, the termination rule and the
  reward, and use `sin/cos` of the angle in the observation instead of the raw angle.
- **Harder robustness:** randomize masses, friction, and motor strength each episode, and add sensor noise.
- **Longer / tuned training:** more steps or PPO hyperparameter tweaks to handle 2–3 N pushes more reliably.
