# PAWS Environment Setup

PAWS is split into three subteams. Use only the section for your subteam unless a lead asks you to install another stack.

Version drift is the main setup failure mode. Do not install "latest" for semester work. Do not upgrade tools mid-semester unless the subteam lead announces a coordinated upgrade. If a build or training command fails, first check the version command in your section before debugging code.

## Firmware setup: ESP-IDF for ESP32-P4 and ESP32-S3

You need this section if you work on:

- `firmware/p4-main/`: ESP32-P4 control loop, CAN-FD bus master, and policy inference
- `firmware/s3-bluetooth/`: ESP32-S3 Bluepad32 controller input and UART to P4
- `firmware/components/`: shared ESP-IDF components

Pinned ESP-IDF version for PAWS: `5.4`

Do not install ESP-IDF "latest". Do not mix ESP-IDF minor versions across the team. The Bluepad32 controller code is sensitive to the ESP-IDF version it is built against; if half the room has a different minor version, the first Bluetooth exercise turns into build-error triage instead of robot work.

We pin the ESP-IDF version in this repo's docs, but each ESP-IDF project's `sdkconfig` is gitignored. That means version mismatches usually show up as confusing build errors, not as a clean Git diff.

### Recommended beginner path: VS Code ESP-IDF extension

Use this unless a firmware lead tells you to do the manual install.

1. Install VS Code: https://code.visualstudio.com/
2. Open VS Code.
3. Open the Extensions panel.
4. Install the extension named `Espressif IDF`.
5. Press `Ctrl+Shift+P` on Windows/Linux or `Cmd+Shift+P` on macOS.
6. Run `ESP-IDF: Configure ESP-IDF Extension`.
7. Choose `Express` setup.
8. When the extension asks for the ESP-IDF version, choose exactly `5.4`.
9. Let the extension install Python, the compiler toolchain, OpenOCD, and ESP-IDF tools.
10. Press `Ctrl+Shift+P` or `Cmd+Shift+P` again.
11. Run `ESP-IDF: Open ESP-IDF Terminal`.
12. Verify the version:

```sh
idf.py --version
```

The output must print ESP-IDF `v5.4`. If it prints a different version, stop and fix the install before building PAWS firmware.

### Manual fallback: macOS and Linux

Run these commands in a normal terminal:

```sh
mkdir -p ~/esp
cd ~/esp
git clone -b v5.4 --recursive https://github.com/espressif/esp-idf.git esp-idf-v5.4
cd esp-idf-v5.4
./install.sh all
. ./export.sh
idf.py --version
```

The final command must print ESP-IDF `v5.4`.

For every new terminal session, activate ESP-IDF again before running `idf.py`:

```sh
. ~/esp/esp-idf-v5.4/export.sh
```

### Manual fallback: Windows

Use Command Prompt, not PowerShell, for these commands:

```bat
mkdir C:\Esp
cd /d C:\Esp
git clone -b v5.4 --recursive https://github.com/espressif/esp-idf.git esp-idf-v5.4
cd esp-idf-v5.4
install.bat all
export.bat
idf.py --version
```

The final command must print ESP-IDF `v5.4`.

For every new Command Prompt session, activate ESP-IDF again before running `idf.py`:

```bat
C:\Esp\esp-idf-v5.4\export.bat
```

### First firmware build target: blink

Build Espressif's blink example before building any PAWS firmware. This proves your toolchain works before PAWS code is involved.

In an ESP-IDF terminal on macOS or Linux:

```sh
mkdir -p ~/paws-work
cp -R "$IDF_PATH/examples/get-started/blink" ~/paws-work/paws-blink
cd ~/paws-work/paws-blink
idf.py set-target esp32s3
idf.py build
```

In an ESP-IDF Command Prompt on Windows:

```bat
mkdir %USERPROFILE%\paws-work
xcopy /E /I "%IDF_PATH%\examples\get-started\blink" "%USERPROFILE%\paws-work\paws-blink"
cd /d "%USERPROFILE%\paws-work\paws-blink"
idf.py set-target esp32s3
idf.py build
```

If you are assigned an ESP32-P4 board first, use `idf.py set-target esp32p4` instead of `idf.py set-target esp32s3`.

If you are stuck, ask in the firmware subteam channel. Do not burn the session silently.

## ML/AI setup: MuJoCo, Gymnasium, RL, and onboard vision tooling

You need this section if you work on:

- `sim/models/`: MJCF model files
- `sim/envs/`: Gymnasium environment wrappers
- `sim/train/`: training scripts and configs
- `sim/exercises/`: onboarding exercises
- `ml/data/`: dataset config files only; do not commit images
- `ml/train/`: model training code
- `ml/deploy/`: quantization and ESP-DL export

Use the pinned Python packages in `requirements.txt`. Do not run unpinned `pip install gymnasium mujoco stable-baselines3 tensorboard torch`. Everyone should have identical package versions when debugging training and simulation.

Pinned package versions are committed in the repo root `requirements.txt`:

```txt
gymnasium==1.3.0
mujoco==3.3.5
stable-baselines3==2.9.0
tensorboard==2.20.0
torch==2.8.0
```

### macOS setup

Install Python 3.11 from https://www.python.org/downloads/ if `python3.11` is not already available.

From the repo root:

```sh
cd /path/to/paws
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install torch==2.8.0
python -m pip install -r requirements.txt
python -m pip list
```

Important for macOS: the interactive MuJoCo viewer must be launched with `mjpython`, not `python`. If the viewer crashes or opens a blank window on macOS, check that you used `mjpython` first.

Create a temporary MuJoCo smoke-test model and open it in the viewer:

```sh
cat > /tmp/paws-mujoco-smoke.xml <<'XML'
<mujoco model="paws-smoke-test">
  <worldbody>
    <light pos="0 0 2"/>
    <body name="box" pos="0 0 0.2">
      <geom type="box" size="0.1 0.1 0.1" rgba="0.2 0.4 0.8 1"/>
    </body>
  </worldbody>
</mujoco>
XML
mjpython -c "import mujoco, mujoco.viewer; m=mujoco.MjModel.from_xml_path('/tmp/paws-mujoco-smoke.xml'); d=mujoco.MjData(m); mujoco.viewer.launch(m, d)"
```

The verify step passes when a MuJoCo viewer window opens and you can see the blue box render.

### Linux setup

Install Python 3.11 and virtual environment support. On Ubuntu/Debian:

```sh
sudo apt update
sudo apt install -y python3.11 python3.11-venv python3.11-dev
```

From the repo root:

```sh
cd /path/to/paws
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install torch==2.8.0 --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r requirements.txt
python -m pip list
```

Install PyTorch before Stable-Baselines3. If you install Stable-Baselines3 first on Linux, pip can pull the multi-gigabyte CUDA PyTorch build even when you only need CPU training.

Create a temporary MuJoCo smoke-test model and open it in the viewer:

```sh
cat > /tmp/paws-mujoco-smoke.xml <<'XML'
<mujoco model="paws-smoke-test">
  <worldbody>
    <light pos="0 0 2"/>
    <body name="box" pos="0 0 0.2">
      <geom type="box" size="0.1 0.1 0.1" rgba="0.2 0.4 0.8 1"/>
    </body>
  </worldbody>
</mujoco>
XML
python -c "import mujoco, mujoco.viewer; m=mujoco.MjModel.from_xml_path('/tmp/paws-mujoco-smoke.xml'); d=mujoco.MjData(m); mujoco.viewer.launch(m, d)"
```

The verify step passes when a MuJoCo viewer window opens and you can see the blue box render.

### Windows setup

Install Python 3.11 from https://www.python.org/downloads/. In the installer, check `Add python.exe to PATH`.

Open PowerShell from the repo root:

```powershell
cd C:\path\to\paws
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install torch==2.8.0 --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r requirements.txt
python -m pip list
```

Install PyTorch before Stable-Baselines3. If you install Stable-Baselines3 first on Windows, pip can pull the multi-gigabyte CUDA PyTorch build even when you only need CPU training.

Create a temporary MuJoCo smoke-test model and open it in the viewer:

```powershell
@'
<mujoco model="paws-smoke-test">
  <worldbody>
    <light pos="0 0 2"/>
    <body name="box" pos="0 0 0.2">
      <geom type="box" size="0.1 0.1 0.1" rgba="0.2 0.4 0.8 1"/>
    </body>
  </worldbody>
</mujoco>
'@ | Set-Content $env:TEMP\paws-mujoco-smoke.xml
python -c "import os, mujoco, mujoco.viewer; p=os.path.join(os.environ['TEMP'], 'paws-mujoco-smoke.xml'); m=mujoco.MjModel.from_xml_path(p); d=mujoco.MjData(m); mujoco.viewer.launch(m, d)"
```

The verify step passes when a MuJoCo viewer window opens and you can see the blue box render.

If you are stuck, ask in the ML/AI subteam channel. Do not burn the session silently.

## Mechanical setup: Onshape CAD and vendor references

You need this section if you work on:

- Onshape robot CAD
- `cad/reference/`: vendor drawings for parts such as MJ5208 actuators and moteus r4.11 controllers
- `cad/exports/`: STEP/STL exports from Onshape for manufacturing, printing, or simulation handoff
- `docs/hardware/`: hardware notes and bring-up documentation

There is nothing to install. Mechanical work is browser-only.

1. Create an Onshape account at https://www.onshape.com/.
2. Ask a mechanical lead to add your account to the PAWS team workspace.
3. Open the shared PAWS Onshape document in a browser.
4. Confirm you can view the main assembly.
5. Confirm you can open or download the vendor reference drawings in `cad/reference/`.

Onshape is the source of truth for CAD. Files in `cad/exports/` are exports for downstream use, not the authoritative design. Large STEP/STL exports are gitignored to keep the repo small.

If you are stuck, ask in the mechanical subteam channel. Do not burn the session silently.
