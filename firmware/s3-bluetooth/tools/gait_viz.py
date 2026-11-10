#!/usr/bin/env python3
"""
gait_viz.py — real-time gait visualiser for the paws quadruped.

Reads serial output produced by gait_task on the ESP32 and renders:
  · 3D isometric robot view  — body + legs coloured by stance/swing state
  · Top-down XY foot trace   — Raibert placement is visible here when moving
  · Per-leg Z height strip   — validates the sinusoidal swing arcs

Usage:
    python gait_viz.py /dev/cu.usbserial-10   # live hardware
    python gait_viz.py --demo                  # generated playback, no hardware
    python gait_viz.py --list                  # list available serial ports
"""

import argparse
import glob
import math
import queue
import re
import sys
import threading
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401 — registers 3D projection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---------------------------------------------------------------------------
# Robot geometry — must match robot.h
# ---------------------------------------------------------------------------
HIP_X        = [ 0.17449,  0.17449, -0.17449, -0.17449]   # FL FR RL RR
HIP_Y        = [ 0.15941, -0.15940,  0.15941, -0.15940]
BODY_HEIGHT  = 0.325    # CoM above ground [m]
STEP_HEIGHT  = 0.060    # max foot lift    [m]
BODY_LEN     = 0.400    # full body length [m]
BODY_WID     = 0.160    # full body width  [m]

LEG_NAMES    = ['FL', 'FR', 'RL', 'RR']
LEG_COLORS   = ['#5B9BD5', '#E05252', '#52C87C', '#E09A52']  # blue red green orange
STANCE_DIM   = '#44475a'                                       # dimmed stance colour

HISTORY_XY   = 120   # foot trace length in top-down view (frames)
HISTORY_Z    = 200   # samples in Z-height strip

# ---------------------------------------------------------------------------
# Serial line parser
# ---------------------------------------------------------------------------
_LINE_RE = re.compile(
    r'lx=\s*(-?\d+)\s+ly=\s*(-?\d+)\s+rx=\s*(-?\d+)\s*\|'
    r'\s*vx=([+-]?\d+\.\d+)\s+vy=([+-]?\d+\.\d+)\s+w=([+-]?\d+\.\d+)\s*\|'
    r'\s*FL:([SW])\(([+-]?\d+\.\d+),([+-]?\d+\.\d+),([+-]?\d+\.\d+)\)'
    r'\s*FR:([SW])\(([+-]?\d+\.\d+),([+-]?\d+\.\d+),([+-]?\d+\.\d+)\)'
    r'\s*RL:([SW])\(([+-]?\d+\.\d+),([+-]?\d+\.\d+),([+-]?\d+\.\d+)\)'
    r'\s*RR:([SW])\(([+-]?\d+\.\d+),([+-]?\d+\.\d+),([+-]?\d+\.\d+)\)'
)

def parse_line(line):
    """Return a frame dict from a gait_task serial line, or None."""
    m = _LINE_RE.search(line)
    if not m:
        return None
    g = m.groups()
    return {
        'lx': int(g[0]), 'ly': int(g[1]), 'rx': int(g[2]),
        'vx': float(g[3]), 'vy': float(g[4]), 'omega': float(g[5]),
        'legs': [
            {
                'swing': g[6 + i * 4] == 'W',
                'x':     float(g[7 + i * 4]),
                'y':     float(g[8 + i * 4]),
                'z':     float(g[9 + i * 4]),
            }
            for i in range(4)
        ],
    }

# ---------------------------------------------------------------------------
# Serial reader thread
# ---------------------------------------------------------------------------
def _serial_reader(port, baud, q):
    try:
        import serial
    except ImportError:
        sys.exit('pyserial not installed — run: pip install pyserial')
    print(f'[serial] opening {port} @ {baud}')
    with serial.Serial(port, baud, timeout=1.0) as ser:
        while True:
            try:
                raw   = ser.readline()
                line  = raw.decode('ascii', errors='ignore').strip()
                frame = parse_line(line)
                if frame:
                    # Stay real-time: drop stale frames
                    while q.qsize() > 4:
                        try:
                            q.get_nowait()
                        except queue.Empty:
                            break
                    q.put_nowait(frame)
            except Exception as e:
                print(f'[serial] {e}')
                time.sleep(0.1)

# ---------------------------------------------------------------------------
# Demo mode: Python re-implementation of gait.c (same math, no hardware)
# ---------------------------------------------------------------------------
class _GaitSim:
    """Minimal Python mirror of gait_update() for demo/offline testing."""

    PHASE_OFFSET = [0.0, 0.5, 0.5, 0.0]   # FL FR RL RR — trot diagonals

    def __init__(self):
        self.master_phase = 0.0
        self.beta  = 0.5
        self.freq  = 1.5
        self.legs  = [
            {
                'p_contact': [HIP_X[i], HIP_Y[i], -BODY_HEIGHT],
                'p_lift':    [HIP_X[i], HIP_Y[i], -BODY_HEIGHT],
                'p_target':  [HIP_X[i], HIP_Y[i], -BODY_HEIGHT],
                'in_swing':  False,
            }
            for i in range(4)
        ]

    def _raibert(self, vx, vy, omega, i):
        T_st  = self.beta / self.freq
        vx_h  = vx - omega * HIP_Y[i]
        vy_h  = vy + omega * HIP_X[i]
        return [HIP_X[i] + vx_h * T_st * 0.5,
                HIP_Y[i] + vy_h * T_st * 0.5,
                -BODY_HEIGHT]

    def update(self, vx, vy, omega, dt):
        self.master_phase = (self.master_phase + self.freq * dt) % 1.0
        c = math.cos(omega * dt)
        s = math.sin(omega * dt)

        feet = []
        for i, leg in enumerate(self.legs):
            phi       = (self.master_phase + self.PHASE_OFFSET[i]) % 1.0
            was_swing = leg['in_swing']
            now_swing = phi >= self.beta
            leg['in_swing'] = now_swing

            if was_swing and not now_swing:
                leg['p_contact'] = list(leg['p_target'])
            if not was_swing and now_swing:
                leg['p_lift']   = list(leg['p_contact'])
                leg['p_target'] = self._raibert(vx, vy, omega, i)

            if not now_swing:
                cx, cy = leg['p_contact'][0], leg['p_contact'][1]
                leg['p_contact'][0] = c * cx + s * cy - vx * dt
                leg['p_contact'][1] = -s * cx + c * cy - vy * dt
                leg['p_contact'][2] = -BODY_HEIGHT
                p = leg['p_contact']
                feet.append({'swing': False, 'x': p[0], 'y': p[1], 'z': p[2]})
            else:
                t  = (phi - self.beta) / (1.0 - self.beta)
                pl = leg['p_lift']
                pt = leg['p_target']
                feet.append({
                    'swing': True,
                    'x': pl[0] + (pt[0] - pl[0]) * t,
                    'y': pl[1] + (pt[1] - pl[1]) * t,
                    'z': -BODY_HEIGHT + STEP_HEIGHT * math.sin(math.pi * t),
                })
        return feet


def _demo_thread(q):
    """
    Cycles through motion patterns so you can see every gait behaviour
    without touching a controller: forward walk, turn, strafe, combined.
    """
    sim = _GaitSim()
    dt  = 0.010   # 100 Hz sim, same as ESP32

    # (duration_s, vx, vy, omega)
    patterns = [
        (3.0,  0.4,  0.0,  0.0),   # walk forward
        (1.5,  0.0,  0.0,  0.0),   # pause
        (3.0, -0.4,  0.0,  0.0),   # walk backward
        (1.5,  0.0,  0.0,  0.0),
        (2.5,  0.0,  0.3,  0.0),   # strafe left
        (1.5,  0.0,  0.0,  0.0),
        (2.5,  0.0, -0.3,  0.0),   # strafe right
        (1.5,  0.0,  0.0,  0.0),
        (3.0,  0.0,  0.0,  1.5),   # spin left
        (1.5,  0.0,  0.0,  0.0),
        (3.0,  0.3,  0.0,  1.0),   # forward + yaw (arc turn)
        (1.5,  0.0,  0.0,  0.0),
    ]

    while True:
        for duration, vx, vy, omega in patterns:
            steps = int(duration / dt)
            for _ in range(steps):
                feet  = sim.update(vx, vy, omega, dt)
                frame = {
                    'lx': 0, 'ly': 0, 'rx': 0,
                    'vx': vx, 'vy': vy, 'omega': omega,
                    'legs': feet,
                    'demo': True,
                }
                while q.qsize() > 5:
                    try:
                        q.get_nowait()
                    except queue.Empty:
                        break
                q.put_nowait(frame)
                time.sleep(dt)

# ---------------------------------------------------------------------------
# Visualiser
# ---------------------------------------------------------------------------
class GaitViz:
    # Matplotlib colour scheme (dark)
    BG      = '#0d1117'
    FG      = '#c9d1d9'
    PANEL   = '#161b22'
    GRID    = '#21262d'
    ACCENT  = '#58a6ff'
    GROUND  = '#1f6feb'

    def __init__(self, q, demo=False):
        self.q        = q
        self.demo     = demo
        self.frame    = None

        self.xy_hist  = [deque(maxlen=HISTORY_XY) for _ in range(4)]
        self.z_hist   = [deque(maxlen=HISTORY_Z)  for _ in range(4)]

        self._build_figure()

    # ------------------------------------------------------------------ build
    def _build_figure(self):
        plt.rcParams.update({
            'figure.facecolor':  self.BG,
            'axes.facecolor':    self.PANEL,
            'axes.edgecolor':    self.GRID,
            'axes.labelcolor':   self.FG,
            'xtick.color':       self.FG,
            'ytick.color':       self.FG,
            'text.color':        self.FG,
            'grid.color':        self.GRID,
            'grid.linewidth':    0.5,
        })

        self.fig = plt.figure(figsize=(15, 8))
        gs = gridspec.GridSpec(
            2, 2,
            height_ratios=[1.8, 1],
            hspace=0.38, wspace=0.28,
            left=0.05, right=0.97, top=0.92, bottom=0.07,
        )
        self.ax3d  = self.fig.add_subplot(gs[0, 0], projection='3d')
        self.ax_xy = self.fig.add_subplot(gs[0, 1])
        self.ax_z  = self.fig.add_subplot(gs[1, :])

        title = 'paws gait visualiser' + ('  [DEMO]' if self.demo else '  [LIVE]')
        self.fig.suptitle(title, color=self.FG, fontsize=12, fontweight='bold')

        self._init_3d()
        self._init_xy()
        self._init_z()

    # ------------------------------------------------------------ 3D panel
    def _init_3d(self):
        ax = self.ax3d
        ax.set_facecolor(self.PANEL)
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor(self.GRID)
        ax.grid(True)
        ax.set_xlim(-0.42, 0.42)
        ax.set_ylim(-0.32, 0.32)
        ax.set_zlim(-BODY_HEIGHT - 0.04, 0.12)
        ax.set_xlabel('x fwd', fontsize=8)
        ax.set_ylabel('y left', fontsize=8)
        ax.set_zlabel('z up', fontsize=8)
        ax.view_init(elev=28, azim=-48)
        ax.set_title('3D body frame', color=self.FG, fontsize=10, pad=4)

        # Static: body outline
        bx, by = BODY_LEN / 2, BODY_WID / 2
        corners = np.array([[-bx,-by,0],[bx,-by,0],[bx,by,0],[-bx,by,0],[-bx,-by,0]])
        ax.plot(corners[:,0], corners[:,1], corners[:,2],
                color='#30363d', linewidth=2)

        # Forward marker
        ax.plot([bx, bx + 0.04], [0, 0], [0, 0], color='#f0e68c', linewidth=2)
        ax.text(bx + 0.045, 0, 0, 'fwd', color='#f0e68c', fontsize=7)

        # Hip dots (static)
        ax.scatter(HIP_X, HIP_Y, [0]*4, c='#58a6ff', s=18, zorder=6, depthshade=False)

        # Ground plane (semi-transparent polygon)
        gx = np.array([-0.42, 0.42, 0.42, -0.42])
        gy = np.array([-0.32, -0.32, 0.32, 0.32])
        gz = np.full(4, -BODY_HEIGHT)
        verts = [list(zip(gx, gy, gz))]
        ground = Poly3DCollection(verts, alpha=0.07,
                                  facecolor='#1f6feb', edgecolor='#30363d',
                                  linewidth=0.5)
        ax.add_collection3d(ground)

        # Dynamic: leg lines and foot dots (use plot() so set_data_3d works)
        self._leg_lines = []
        self._foot_dots = []
        for i in range(4):
            lx0, ly0 = HIP_X[i], HIP_Y[i]
            ln, = ax.plot([lx0, lx0], [ly0, ly0], [0, -BODY_HEIGHT],
                         color=LEG_COLORS[i], linewidth=2, alpha=0.9)
            fd, = ax.plot([lx0], [ly0], [-BODY_HEIGHT],
                         'o', color=LEG_COLORS[i], markersize=10,
                         markeredgewidth=0, zorder=10)
            self._leg_lines.append(ln)
            self._foot_dots.append(fd)

        # Velocity readout
        self._vel_text = ax.text2D(0.02, 0.97, '', transform=ax.transAxes,
                                   color='#f0e68c', fontsize=8,
                                   verticalalignment='top')

    # -------------------------------------------------------- top-down panel
    def _init_xy(self):
        ax = self.ax_xy
        ax.set_aspect('equal')
        ax.set_xlim(-0.44, 0.44)
        ax.set_ylim(-0.34, 0.34)
        ax.grid(True, linewidth=0.4)
        ax.set_xlabel('x (fwd →)', fontsize=8)
        ax.set_ylabel('y (left ↑)', fontsize=8)
        ax.set_title('Top-down foot trace', color=self.FG, fontsize=10)

        # Body outline
        bx, by = BODY_LEN / 2, BODY_WID / 2
        body_rect = plt.Polygon(
            [[-bx,-by],[bx,-by],[bx,by],[-bx,by]],
            closed=True, fill=True,
            facecolor='#161b22', edgecolor='#30363d', linewidth=1.5, zorder=2,
        )
        ax.add_patch(body_rect)
        ax.plot([bx, bx + 0.03], [0, 0], color='#f0e68c', linewidth=1.5, zorder=3)

        # Hip dots
        ax.scatter(HIP_X, HIP_Y, c='#58a6ff', s=12, zorder=4)

        # Traces and foot dots
        self._traces = []
        self._xy_dots = []
        for i in range(4):
            tr, = ax.plot([], [], '-', color=LEG_COLORS[i], alpha=0.35,
                         linewidth=1.2, zorder=3)
            fd, = ax.plot([], [], 'o', color=LEG_COLORS[i],
                         markersize=9, zorder=5)
            self._traces.append(tr)
            self._xy_dots.append(fd)

        # Swing state labels (updated each frame)
        self._state_texts = []
        for i in range(4):
            x = HIP_X[i] + (-0.06 if HIP_X[i] > 0 else 0.02)
            y = HIP_Y[i] + (0.04 if HIP_Y[i] > 0 else -0.06)
            t = ax.text(x, y, LEG_NAMES[i] + ':S', fontsize=7,
                       color=LEG_COLORS[i], zorder=6)
            self._state_texts.append(t)

        legend_patches = [mpatches.Patch(color=LEG_COLORS[i], label=LEG_NAMES[i])
                         for i in range(4)]
        ax.legend(handles=legend_patches, loc='lower right', fontsize=7,
                 facecolor=self.PANEL, edgecolor=self.GRID,
                 labelcolor=self.FG)

    # --------------------------------------------------- Z-height strip
    def _init_z(self):
        ax = self.ax_z
        ax.set_xlim(0, HISTORY_Z)
        y_lo = -BODY_HEIGHT - 0.01
        y_hi = -BODY_HEIGHT + STEP_HEIGHT + 0.015
        ax.set_ylim(y_lo, y_hi)
        ax.set_yticks(np.round(np.linspace(y_lo, y_hi, 5), 3))
        ax.grid(True, axis='y', linewidth=0.4)
        ax.axhline(-BODY_HEIGHT, color='#58a6ff', linewidth=0.8,
                  linestyle='--', label='ground (z = −{:.3f})'.format(BODY_HEIGHT))
        ax.set_xlabel('time →  (each tick = 100 ms)', fontsize=8)
        ax.set_ylabel('foot  z  (m)', fontsize=8)
        ax.set_title(
            'Per-leg Z height  —  flat line = stance, arc = swing',
            color=self.FG, fontsize=10,
        )
        ax.set_xticks([])

        self._z_lines = []
        for i in range(4):
            ln, = ax.plot([], [], color=LEG_COLORS[i],
                         linewidth=1.6, label=LEG_NAMES[i])
            self._z_lines.append(ln)

        ax.legend(loc='upper right', fontsize=7,
                 facecolor=self.PANEL, edgecolor=self.GRID,
                 labelcolor=self.FG)

    # ---------------------------------------------------------------- animate
    def _update(self, _frame_num):
        # Pull the freshest frame; skip if nothing new
        try:
            while True:
                self.frame = self.q.get_nowait()
        except queue.Empty:
            pass

        if self.frame is None:
            return

        f    = self.frame
        legs = f['legs']

        # Accumulate history
        for i, leg in enumerate(legs):
            self.xy_hist[i].append((leg['x'], leg['y']))
            self.z_hist[i].append(leg['z'])

        # --- 3D panel ---
        for i, leg in enumerate(legs):
            # Leg line: hip → foot
            self._leg_lines[i].set_data_3d(
                [HIP_X[i], leg['x']],
                [HIP_Y[i], leg['y']],
                [0,        leg['z']],
            )
            # Foot dot: bright when swinging, dim when on ground
            if leg['swing']:
                self._foot_dots[i].set_markerfacecolor(LEG_COLORS[i])
                self._foot_dots[i].set_markersize(12)
                self._leg_lines[i].set_linestyle('--')
                self._leg_lines[i].set_alpha(0.7)
            else:
                self._foot_dots[i].set_markerfacecolor('#556070')
                self._foot_dots[i].set_markersize(9)
                self._leg_lines[i].set_linestyle('-')
                self._leg_lines[i].set_alpha(1.0)
            self._foot_dots[i].set_data_3d([leg['x']], [leg['y']], [leg['z']])

        self._vel_text.set_text(
            f"vx={f['vx']:+.2f}  vy={f['vy']:+.2f}  ω={f['omega']:+.2f}"
        )

        # --- Top-down panel ---
        for i, leg in enumerate(legs):
            xs = [p[0] for p in self.xy_hist[i]]
            ys = [p[1] for p in self.xy_hist[i]]
            self._traces[i].set_data(xs, ys)
            self._xy_dots[i].set_data([leg['x']], [leg['y']])
            if leg['swing']:
                self._xy_dots[i].set_color(LEG_COLORS[i])
                self._xy_dots[i].set_markersize(10)
            else:
                self._xy_dots[i].set_color('#556070')
                self._xy_dots[i].set_markersize(7)
            label = LEG_NAMES[i] + (':W' if leg['swing'] else ':S')
            self._state_texts[i].set_text(label)

        # --- Z strip ---
        for i in range(4):
            zs = list(self.z_hist[i])
            self._z_lines[i].set_data(range(len(zs)), zs)

    def run(self):
        self.anim = FuncAnimation(
            self.fig,
            self._update,
            interval=100,          # 10 Hz — matches serial print rate
            blit=False,
            cache_frame_data=False,
        )
        plt.show()

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def _auto_detect_port():
    candidates = (
        glob.glob('/dev/cu.usbserial*') +
        glob.glob('/dev/ttyUSB*') +
        glob.glob('/dev/ttyACM*')
    )
    return candidates[0] if candidates else None


def main():
    parser = argparse.ArgumentParser(
        description='Real-time gait visualiser for the paws quadruped.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('port',   nargs='?', help='serial port path')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--demo', action='store_true',
                        help='run in demo mode (no hardware needed)')
    parser.add_argument('--list', action='store_true',
                        help='list available serial ports and exit')
    args = parser.parse_args()

    if args.list:
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            print(p.device, '-', p.description)
        return

    q = queue.Queue()

    if args.demo:
        t = threading.Thread(target=_demo_thread, args=(q,), daemon=True)
        t.start()
        print('[demo] running Python gait simulation — no hardware needed')
    else:
        port = args.port or _auto_detect_port()
        if port is None:
            sys.exit('No serial port found. Pass a port or use --demo.')
        t = threading.Thread(target=_serial_reader,
                             args=(port, args.baud, q), daemon=True)
        t.start()

    viz = GaitViz(q, demo=args.demo)
    viz.run()


if __name__ == '__main__':
    main()
