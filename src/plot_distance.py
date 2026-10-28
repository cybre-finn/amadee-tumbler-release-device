#!/usr/bin/env python3
import csv
import glob
import os
import re
import subprocess
import sys
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

WINDOW_S = 20
MAX_MM = 1200
LINE_RE = re.compile(r"U1\s+(-?\d+)\s*mm.*?U2\s+(-?\d+)\s*mm")
UNIT_LABELS = ("U1 (alt)", "U2 (neu)")

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def latest_session_file():
    files = sorted(glob.glob(os.path.join(DATA_DIR, "session_*.csv")))
    return files[-1] if files else None


def live_mode(port):
    os.makedirs(DATA_DIR, exist_ok=True)
    session_path = os.path.join(DATA_DIR, f"session_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    log_file = open(session_path, "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow(["t_s", "mm1", "mm2"])

    samples = deque()
    lock = threading.Lock()
    t_start = time.time()

    def reader():
        subprocess.run(["stty", "-F", port, "raw", "-echo"], check=True)
        with open(port, "r", errors="ignore") as f:
            for line in f:
                m = LINE_RE.search(line)
                if not m:
                    continue
                mm1, mm2 = int(m.group(1)), int(m.group(2))
                now = time.time()
                v1 = None if (mm1 < 0 or mm1 >= 8190) else mm1
                v2 = None if (mm2 < 0 or mm2 >= 8190) else mm2
                with lock:
                    samples.append((now, v1, v2))
                log_writer.writerow([f"{now - t_start:.3f}", "" if v1 is None else v1, "" if v2 is None else v2])
                log_file.flush()

    threading.Thread(target=reader, daemon=True).start()

    fig, axes = plt.subplots(1, 2, figsize=(11, 5), sharey=True)
    lines = []
    texts = []
    for ax, label in zip(axes, UNIT_LABELS):
        (line,) = ax.plot([], [], lw=1.5, marker=".", ms=3)
        text = ax.text(0.98, 0.95, "", transform=ax.transAxes, ha="right", va="top", fontsize=16)
        ax.set_xlim(-WINDOW_S, 0)
        ax.set_ylim(0, MAX_MM)
        ax.set_xlabel("Zeit [s]")
        ax.set_title(label)
        ax.grid(True, alpha=0.3)
        lines.append(line)
        texts.append(text)
    axes[0].set_ylabel("Distanz [mm]")
    fig.suptitle(f"VL53L0X live (2 Sensoren)  -  speichere nach {os.path.relpath(session_path)}")

    def update(_):
        now = time.time()
        with lock:
            while samples and now - samples[0][0] > WINDOW_S:
                samples.popleft()
            data = list(samples)
        xs = [t - now for t, _, _ in data]
        for i, line in enumerate(lines):
            ys = [float("nan") if d[i + 1] is None else d[i + 1] for d in data]
            line.set_data(xs, ys)
            last = data[-1][i + 1] if data else None
            texts[i].set_text("out of range" if data and last is None else (f"{last} mm" if last is not None else ""))
        return lines + texts

    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        log_file.close()


def load_session(path):
    ts, y1, y2 = [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        has_mm2 = "mm2" in (reader.fieldnames or [])
        for row in reader:
            ts.append(float(row["t_s"]))
            y1.append(float("nan") if row.get("mm1", row.get("mm", "")) == "" else float(row.get("mm1", row.get("mm"))))
            y2.append(float("nan") if not has_mm2 or row["mm2"] == "" else float(row["mm2"]))
    return ts, y1, y2


def history_mode(path):
    if path is None:
        path = latest_session_file()
    if path is None or not os.path.exists(path):
        sys.exit(f"Keine Session-Datei gefunden (gesucht in {DATA_DIR}).")

    ts, y1, y2 = load_session(path)
    if not ts:
        sys.exit(f"{path} enthaelt keine Daten.")

    duration = ts[-1] - ts[0]
    view = min(WINDOW_S, duration) if duration > 0 else WINDOW_S

    fig, axes = plt.subplots(1, 2, figsize=(11, 5), sharey=True)
    plt.subplots_adjust(bottom=0.2)
    for ax, ys, label in zip(axes, (y1, y2), UNIT_LABELS):
        ax.plot(ts, ys, lw=1.2, marker=".", ms=2)
        ax.set_xlim(ts[0], ts[0] + view)
        ax.set_ylim(0, MAX_MM)
        ax.set_xlabel("Zeit [s]")
        ax.set_title(label)
        ax.grid(True, alpha=0.3)
    axes[0].set_ylabel("Distanz [mm]")
    fig.suptitle(f"Verlauf: {os.path.basename(path)}  ({duration:.1f} s gesamt)")

    if duration > view:
        slider_ax = fig.add_axes([0.15, 0.05, 0.7, 0.04])
        slider = Slider(slider_ax, "Scroll", ts[0], ts[-1] - view, valinit=ts[0])

        def on_scroll(val):
            for ax in axes:
                ax.set_xlim(val, val + view)
            fig.canvas.draw_idle()

        slider.on_changed(on_scroll)

    plt.show()


if __name__ == "__main__":
    args = sys.argv[1:]
    if args and args[0] == "--history":
        history_mode(args[1] if len(args) > 1 else None)
    else:
        live_mode(args[0] if args else "/dev/ttyACM0")
