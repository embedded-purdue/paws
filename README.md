# PAWS

PAWS is a 12-DOF quadruped robot built by a Purdue embedded systems club team. This repo is the shared monorepo for firmware, simulation, ML/AI, mechanical references, onboarding docs, and team tools.

## Repo skeleton

- `firmware/p4-main/` — ESP32-P4 control loop, CAN-FD bus master, and policy inference.
- `firmware/s3-bluetooth/` — ESP32-S3 Bluepad32 controller input and UART link to the P4.
- `firmware/components/` — shared ESP-IDF components used by firmware projects.
- `sim/models/` — MuJoCo MJCF model files.
- `sim/envs/` — Gymnasium environment wrappers.
- `sim/train/` — simulation training scripts and configs.
- `sim/exercises/` — onboarding exercises for simulation and controls.
- `ml/data/` — dataset configuration files only; do not commit images or large datasets.
- `ml/train/` — ML training code.
- `ml/deploy/` — quantization and ESP-DL export work.
- `cad/reference/` — vendor drawings and reference material, including MJ5208 and moteus r4.11 docs.
- `cad/exports/` — STEP/STL exports from Onshape for downstream use. Onshape remains the CAD source of truth.
- `docs/onboarding/` — setup instructions for each subteam.
- `docs/decisions/` — append-only decision records.
- `docs/hardware/` — hardware notes and bring-up documentation.
- `tools/` — repo utilities and scripts.
- `introductions/` — first pull request exercise for new members.

## Setup instructions

Start with `docs/onboarding/setup.md`.

That guide is split by subteam:

- Firmware: ESP-IDF 5.4 setup for ESP32-P4 and ESP32-S3 work.
- ML/AI: Python, MuJoCo, Gymnasium, Stable-Baselines3, and PyTorch setup.
- Mechanical: Onshape workspace access and vendor reference locations.

Only follow the section for your subteam unless a lead asks you to install another stack.

## First contribution: introduce yourself

Your first task is to practice the normal GitHub workflow: create a branch, make a tiny change, push the branch, and open a pull request.

Replace `your-name` with your actual name, using lowercase letters and hyphens if needed.

```sh
git checkout -b introductions/your-name
printf "Your Name\n" > introductions/your-name.txt
git add introductions/your-name.txt
git commit -m "Add Your Name introduction"
git push -u origin introductions/your-name
```

Then open a pull request on GitHub.

The pull request should contain only one new file:

```txt
introductions/your-name.txt
```

The file should contain your name. Nothing else is required for the first PR.
