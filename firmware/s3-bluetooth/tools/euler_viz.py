#!/usr/bin/env python3
"""
Live 3D visualizer for the BNO085 Euler output printed by main/main.c:

    Roll=%.2f  Pitch=%.2f  Yaw=%.2f  (acc=%.1f deg)

Reconstructs orientation with the SAME rotation convention the firmware
uses in bno085_quaternion_to_euler() (bno085.c):

    roll  = atan2(2(wx+yz), 1-2(x^2+y^2))   -> rotation about body X
    pitch = asin(2(wy-zx))                  -> rotation about body Y
    yaw   = atan2(2(wz+xy), 1-2(y^2+z^2))   -> rotation about body Z

That pairing is the standard Z-Y-X (aerospace) Euler sequence:

    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

i.e. intrinsically: roll first (about body X), then pitch (about the
new Y), then yaw (about the new Z). The visualizer applies the exact
same R to a body-fixed shape, so if the on-screen model's motion
doesn't match the physical board's motion, the bug is real (axis
mapping, sign, or sensor mounting), not a visualization artifact.

Usage:
    tools/.venv/bin/python tools/euler_viz.py --list-ports
    tools/.venv/bin/python tools/euler_viz.py --port /dev/tty.usbserial-0001
    tools/.venv/bin/python tools/euler_viz.py --demo                # no hardware needed
    tools/.venv/bin/python tools/euler_viz.py --demo --save out.gif # headless smoke test
"""
from __future__ import annotations

import argparse
import math
import queue
import re
import sys
import threading
import time
from dataclasses import dataclass

import numpy as np
import matplotlib

LINE_RE = re.compile(
    r"Roll=(?P<roll>-?\d+\.?\d*)\s+"
    r"Pitch=(?P<pitch>-?\d+\.?\d*)\s+"
    r"Yaw=(?P<yaw>-?\d+\.?\d*)\s+"
    r"\(acc=(?P<acc>-?\d+\.?\d*)\s*deg\)"
)


@dataclass
class Sample:
    roll: float
    pitch: float
    yaw: float
    acc: float
    t: float


def parse_line(line: str) -> Sample | None:
    m = LINE_RE.search(line)
    if not m:
        return None
    return Sample(
        roll=float(m["roll"]),
        pitch=float(m["pitch"]),
        yaw=float(m["yaw"]),
        acc=float(m["acc"]),
        t=time.monotonic(),
    )


def rotation_zyx(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll) -- matches bno085_quaternion_to_euler()."""
    r, p, y = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


# ---------------------------------------------------------------- data sources

def serial_reader(port: str, baud: int, out_q: "queue.Queue[Sample]", stop: threading.Event):
    import serial  # local import: optional dependency, only needed for live mode

    try:
        ser = serial.Serial(port, baud, timeout=1)
    except Exception as exc:  # pragma: no cover - interactive failure path
        print(f"[serial] failed to open {port} @ {baud}: {exc}", file=sys.stderr)
        stop.set()
        return

    print(f"[serial] listening on {port} @ {baud}")
    with ser:
        while not stop.is_set():
            try:
                raw = ser.readline().decode("utf-8", errors="replace").strip()
            except Exception as exc:
                print(f"[serial] read error: {exc}", file=sys.stderr)
                continue
            if not raw:
                continue
            sample = parse_line(raw)
            if sample is None:
                continue  # ignore boot logs / unrelated lines
            # keep only the freshest sample so the plot never lags behind serial
            try:
                while True:
                    out_q.get_nowait()
            except queue.Empty:
                pass
            out_q.put_nowait(sample)


def demo_generator(out_q: "queue.Queue[Sample]", stop: threading.Event):
    """Synthetic sweep so the visualization logic can be verified without hardware."""
    t0 = time.monotonic()
    while not stop.is_set():
        t = time.monotonic() - t0
        sample = Sample(
            roll=40 * math.sin(0.6 * t),
            pitch=25 * math.sin(0.35 * t + 1.0),
            yaw=(60 * t) % 360,
            acc=2.0,
            t=time.monotonic(),
        )
        try:
            while True:
                out_q.get_nowait()
        except queue.Empty:
            pass
        out_q.put_nowait(sample)
        time.sleep(0.1)  # match firmware's 10 Hz rotation-vector rate


# ---------------------------------------------------------------- geometry

def board_shape():
    """Asymmetric flat board + nose marker, so roll/pitch/yaw are each
    visually unambiguous (a cube or sphere would hide axis-swap bugs)."""
    hx, hy, hz = 1.6, 0.8, 0.15  # half-extents: long on X, flat on Z
    verts = np.array(
        [
            [hx, hy, hz], [hx, -hy, hz], [-hx, -hy, hz], [-hx, hy, hz],   # top
            [hx, hy, -hz], [hx, -hy, -hz], [-hx, -hy, -hz], [-hx, hy, -hz],  # bottom
        ]
    )
    faces_idx = [
        [0, 1, 2, 3],  # top
        [4, 5, 6, 7],  # bottom
        [0, 1, 5, 4],  # +X nose face
        [2, 3, 7, 6],  # -X tail face
        [1, 2, 6, 5],  # -Y side
        [3, 0, 4, 7],  # +Y side
    ]
    nose = np.array([[hx + 0.6, 0, 0]])  # pyramid tip marking +X ("front")
    return verts, faces_idx, nose


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port, e.g. /dev/tty.usbserial-0001 or COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--demo", action="store_true", help="synthetic data, no hardware required")
    ap.add_argument("--list-ports", action="store_true", help="list available serial ports and exit")
    ap.add_argument("--save", metavar="PATH", help="render to a file (mp4/gif) instead of a live window")
    ap.add_argument("--duration", type=float, default=15.0, help="seconds to run when --save is used")
    args = ap.parse_args()

    if args.list_ports:
        import serial.tools.list_ports as lp
        ports = list(lp.comports())
        if not ports:
            print("No serial ports found.")
        for p in ports:
            print(f"{p.device}\t{p.description}")
        return 0

    if not args.demo and not args.port:
        ap.error("--port is required unless --demo is set")

    if args.save:
        matplotlib.use("Agg")  # headless backend for file output
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    data_q: "queue.Queue[Sample]" = queue.Queue()
    stop = threading.Event()

    if args.demo:
        src_thread = threading.Thread(target=demo_generator, args=(data_q, stop), daemon=True)
    else:
        src_thread = threading.Thread(target=serial_reader, args=(args.port, args.baud, data_q, stop), daemon=True)
    src_thread.start()

    verts, faces_idx, nose = board_shape()

    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_box_aspect([1, 1, 1])
    lim = 3
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # static world-frame reference axes (dashed, gray)
    for axis, color in zip(np.eye(3), ("r", "g", "b")):
        ax.plot([0, axis[0] * lim * 0.6], [0, axis[1] * lim * 0.6], [0, axis[2] * lim * 0.6],
                linestyle="--", color=color, alpha=0.3)

    poly = Poly3DCollection([], facecolor="deepskyblue", edgecolor="k", linewidths=0.8, alpha=0.85)
    ax.add_collection3d(poly)
    nose_scatter = ax.scatter([], [], [], color="red", s=60)

    body_axes = [ax.plot([], [], [], color=c, linewidth=2)[0] for c in ("r", "g", "b")]
    title = ax.set_title("waiting for data...")

    latest = Sample(0, 0, 0, 0, time.monotonic())

    def update(_frame):
        nonlocal latest
        try:
            latest = data_q.get_nowait()
        except queue.Empty:
            pass  # redraw with last-known orientation; no new sample yet

        R = rotation_zyx(latest.roll, latest.pitch, latest.yaw)
        rotated = verts @ R.T
        rotated_nose = nose @ R.T

        poly.set_verts([[rotated[i] for i in face] for face in faces_idx])
        nose_scatter._offsets3d = (rotated_nose[:, 0], rotated_nose[:, 1], rotated_nose[:, 2])

        for i, line in enumerate(body_axes):
            d = R[:, i] * 2.2
            line.set_data([0, d[0]], [0, d[1]])
            line.set_3d_properties([0, d[2]])

        stale = (time.monotonic() - latest.t) > 1.5
        status = "STALE (no recent serial data)" if stale else f"acc={latest.acc:.1f} deg"
        title.set_text(
            f"Roll={latest.roll:+.1f}  Pitch={latest.pitch:+.1f}  Yaw={latest.yaw:+.1f}   [{status}]"
        )
        return [poly, nose_scatter, title, *body_axes]

    frames = int(args.duration / 0.1) if args.save else None
    anim = FuncAnimation(fig, update, interval=100, blit=False, frames=frames, cache_frame_data=False)

    try:
        if args.save:
            anim.save(args.save, fps=10)
            print(f"saved {args.save}")
        else:
            plt.show()
    finally:
        stop.set()
        src_thread.join(timeout=1)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
