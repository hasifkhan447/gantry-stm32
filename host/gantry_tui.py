#!/usr/bin/env python3
"""Gantry TUI: WASD jog + per-axis settings + S-curve / trapezoid profiles.

Layout:
    [ X ]  state   hz=...  duration=...s  dwell=...s
    [ Y ]  state   hz=...
    [ Z ]  state   hz=...

  Jog keys
    w / s    Y +/-
    a / d    X -/+
    k / j    Z +/-
  Focus / edit
    1 2 3    select axis to edit
    + / -    hz step (5000)
    [ / ]    duration step (0.1 s)
    , / .    dwell step (0.1 s)
    e        toggle profile (scurve <-> trapezoid)
    h        toggle vacuum solenoid (PB0)
    q        quit

The dispatcher is a single worker thread that pulls jog requests off a
queue and decomposes each leg into constant-rate segments. Same idea as
fast_xyz.py, but with an S-curve option (half-cosine velocity ramp) and
a curses frontend. Jog requests for an already-busy axis are dropped.
"""

import argparse
import copy
import curses
import math
import queue
import sys
import threading
import time
from typing import Dict, List, Tuple

import serial  # pip install pyserial


VALID_AXES = ("X", "Y", "Z")
MIN_HZ = 2000
N_ACCEL = 5
N_DECEL = 5
N_CRUISE = 5
ACCEL_FRAC = 0.1
DECEL_FRAC = 0.1

HZ_STEP = 5000
DURATION_STEP = 0.1
DWELL_STEP = 0.1
FRAME_HZ_STEP = 0.5
MIN_DURATION = 0.2

PROFILES = ("scurve", "trapezoid")

# Damping ratio assumed for input shaping. Wooden frames are very lightly
# damped (zeta < 0.1); 0.05 is a reasonable middle of the road and the
# ZV shaper is fairly robust to mis-estimation of damping.
DEFAULT_ZETA = 0.05


# ---------- segment generation ---------------------------------------------

def _base_segments(peak_hz: int, duration: float,
                   profile: str) -> List[Tuple[int, int]]:
    """Decompose a leg into (steps, hz) constant-rate chunks.

    profile="trapezoid"  -> linear velocity ramps (constant accel, infinite jerk)
    profile="scurve"     -> half-cosine velocity ramp (zero accel at endpoints)
    """
    t_accel = ACCEL_FRAC * duration
    t_decel = DECEL_FRAC * duration
    t_cruise = duration - t_accel - t_decel
    if t_cruise < 0:
        raise ValueError("ACCEL_FRAC + DECEL_FRAC must be < 1")

    def v_frac(idx: int, n: int) -> float:
        """0..1 velocity fraction at midpoint of segment idx of n."""
        linear = (idx + 0.5) / n
        if profile == "scurve":
            return (1.0 - math.cos(math.pi * linear)) / 2.0
        return linear

    segs: List[Tuple[int, int]] = []

    dt_a = t_accel / N_ACCEL if N_ACCEL else 0.0
    for j in range(N_ACCEL):
        hz = MIN_HZ + (peak_hz - MIN_HZ) * v_frac(j, N_ACCEL)
        steps = int(round(hz * dt_a))
        if steps > 0:
            segs.append((steps, int(round(hz))))

    if N_CRUISE > 0 and t_cruise > 0:
        dt_c = t_cruise / N_CRUISE
        cruise_steps = int(round(peak_hz * dt_c))
        if cruise_steps > 0:
            for _ in range(N_CRUISE):
                segs.append((cruise_steps, peak_hz))

    dt_d = t_decel / N_DECEL if N_DECEL else 0.0
    for j in range(N_DECEL):
        hz = MIN_HZ + (peak_hz - MIN_HZ) * (1.0 - v_frac(j, N_DECEL))
        steps = int(round(hz * dt_d))
        if steps > 0:
            segs.append((steps, int(round(hz))))

    return segs


def _apply_zv_shaper(segs: List[Tuple[int, int]],
                     frame_hz: float,
                     zeta: float = DEFAULT_ZETA) -> List[Tuple[int, int]]:
    """Convolve a [(steps, hz)] velocity profile with a 2-impulse ZV shaper.

    The shaper produces v_shaped(t) = A1*v(t) + A2*v(t - T_d) where T_d is
    one half-period of the damped natural frequency. Output total
    displacement equals input total displacement (the impulses sum to 1).
    """
    if frame_hz <= 0.0 or not segs:
        return segs

    s = math.sqrt(max(1e-9, 1.0 - zeta * zeta))
    omega = 2.0 * math.pi * frame_hz
    T_d = math.pi / (omega * s)
    K = math.exp(-zeta * math.pi / s)
    A1 = 1.0 / (1.0 + K)
    A2 = K / (1.0 + K)

    # Original breakpoints in seconds.
    bps = [0.0]
    for steps, hz in segs:
        bps.append(bps[-1] + steps / float(hz))
    T_orig = bps[-1]

    def v_at(t: float) -> float:
        if t < 0.0 or t >= T_orig:
            return 0.0
        # Linear scan is fine: segs is at most ~50 long.
        for i, (_, hz) in enumerate(segs):
            if bps[i] <= t < bps[i + 1]:
                return float(hz)
        return 0.0

    # Merged breakpoints of the original and its delayed copy.
    merged = sorted(set(bps + [b + T_d for b in bps]))
    shaped: List[Tuple[int, int]] = []
    for i in range(len(merged) - 1):
        t_a = merged[i]
        t_b = merged[i + 1]
        dt = t_b - t_a
        if dt < 1e-6:
            continue
        t_mid = (t_a + t_b) * 0.5
        v_combined = A1 * v_at(t_mid) + A2 * v_at(t_mid - T_d)
        if v_combined < 1.0:
            continue
        steps = int(round(v_combined * dt))
        if steps > 0:
            shaped.append((steps, int(round(v_combined))))
    return shaped


def make_segments(peak_hz: int, duration: float, profile: str,
                  frame_hz: float = 0.0) -> List[Tuple[int, int]]:
    """Build the segment list, optionally passed through a ZV input shaper.

    frame_hz <= 0  ->  no shaping (returns the raw trapezoid/scurve).
    frame_hz > 0   ->  Zero-Vibration shaper tuned to that frequency.
                       The total move displacement is unchanged but the
                       residual oscillation at frame_hz is cancelled.
                       Move duration is extended by ~0.5/frame_hz seconds.
    """
    base = _base_segments(peak_hz, duration, profile)
    return _apply_zv_shaper(base, frame_hz)


# ---------- serial helpers --------------------------------------------------

def send_locked(ser: serial.Serial, lock: threading.Lock, line: str) -> str:
    with lock:
        ser.write((line + "\r").encode())
        return ser.readline().decode(errors="replace").strip()


def send_move(ser: serial.Serial, lock: threading.Lock, line: str) -> bool:
    for _ in range(200):
        reply = send_locked(ser, lock, line)
        if reply == "ACK":
            return True
        if reply == "ERR queue full":
            time.sleep(0.005)
            continue
        return False
    return False


# ---------- controller ------------------------------------------------------

class Controller:
    """Owns the serial port. Single dispatcher thread serializes wire access."""

    def __init__(self, ser: serial.Serial, settings: Dict[str, dict],
                 profile: str):
        self.ser = ser
        self.lock = threading.Lock()
        self.settings = settings
        self.profile = profile
        self.state = {a: "idle" for a in VALID_AXES}  # idle | queued | moving
        self.vacuum = False
        self.last_msg = "ready"
        self.q: queue.Queue = queue.Queue()
        self._stop = False
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def jog(self, axis: str, direction: int) -> bool:
        if self.state[axis] != "idle":
            return False
        self.state[axis] = "queued"
        snapshot = copy.deepcopy(self.settings[axis])
        self.q.put((axis, direction, snapshot, self.profile))
        return True

    def toggle_vacuum(self) -> None:
        try:
            reply = send_locked(self.ser, self.lock, "VAC TOGGLE")
        except Exception as e:
            self.last_msg = f"vac error: {e}"
            return
        if reply == "VAC ON":
            self.vacuum = True
        elif reply == "VAC OFF":
            self.vacuum = False
        else:
            self.last_msg = f"vac unexpected reply: {reply!r}"
            return
        self.last_msg = f"vacuum {'ON' if self.vacuum else 'OFF'}"

    def stop(self) -> None:
        self._stop = True
        self.q.put(None)

    def _worker(self) -> None:
        while True:
            item = self.q.get()
            if item is None:
                return
            axis, direction, settings, profile = item
            self.state[axis] = "moving"
            sign = "+" if direction > 0 else "-"
            self.last_msg = (f"{axis}{sign} peak={settings['hz']} Hz "
                             f"dur={settings['duration']:.2f}s [{profile}]")
            try:
                segs = make_segments(settings["hz"], settings["duration"],
                                     profile, settings.get("frame_hz", 0.0))
            except Exception as e:
                self.last_msg = f"profile error: {e}"
                self.state[axis] = "idle"
                continue
            ok = True
            for steps, hz in segs:
                cmd = f"MOVE {axis} {direction * steps} F {hz}"
                if not send_move(self.ser, self.lock, cmd):
                    self.last_msg = f"send failed: {cmd}"
                    ok = False
                    break
            if ok:
                self._wait_axis_idle(axis)
            self.state[axis] = "idle"

    def _wait_axis_idle(self, axis: str) -> None:
        token = f"{axis}:busy"
        while not self._stop:
            reply = send_locked(self.ser, self.lock, "STAT")
            if reply and token not in reply:
                return
            time.sleep(0.05)


# ---------- TUI -------------------------------------------------------------

def _safe_addnstr(stdscr, row: int, col: int, text: str, attr: int = 0) -> None:
    h, w = stdscr.getmaxyx()
    if row < 0 or row >= h or col >= w:
        return
    n = max(0, w - col - 1)
    if n == 0:
        return
    try:
        stdscr.addnstr(row, col, text, n, attr)
    except curses.error:
        pass


KEY_HELP = [
    "Jog        w/s = Y +/-     a/d = X -/+     k/j = Z +/-",
    "Focus      1 = X    2 = Y    3 = Z",
    "Edit hz    + / -          (step 5000)",
    "Edit dur   [ / ]          (step 0.1 s)",
    "Edit dwell , / .          (step 0.1 s, unused in TUI but kept for parity)",
    "Profile    e              toggles scurve <-> trapezoid",
    "Vacuum     h              toggles pick/place solenoid",
    "Quit       q",
]


def draw(stdscr, controller: Controller, focused: str) -> None:
    stdscr.erase()
    _, w = stdscr.getmaxyx()
    _safe_addnstr(stdscr, 0, 0,
                  "Gantry TUI  —  WASD = X/Y, K/J = Z, q to quit",
                  curses.A_BOLD)
    _safe_addnstr(stdscr, 1, 0, "-" * (w - 1))

    row = 2
    _safe_addnstr(stdscr, row, 0, "Axis", curses.A_UNDERLINE)
    _safe_addnstr(stdscr, row, 10, "state", curses.A_UNDERLINE)
    _safe_addnstr(stdscr, row, 22, "peak hz", curses.A_UNDERLINE)
    _safe_addnstr(stdscr, row, 36, "leg dur", curses.A_UNDERLINE)
    _safe_addnstr(stdscr, row, 50, "dwell", curses.A_UNDERLINE)
    row += 1

    state_attr = {
        "idle":   curses.A_DIM,
        "queued": curses.A_NORMAL,
        "moving": curses.A_BOLD | curses.A_REVERSE,
    }

    for ax in VALID_AXES:
        s = controller.settings[ax]
        st = controller.state[ax]
        marker = ">" if ax == focused else " "
        focus_attr = curses.A_REVERSE if ax == focused else 0
        _safe_addnstr(stdscr, row, 0, f"{marker} {ax}", focus_attr)
        _safe_addnstr(stdscr, row, 10, f"{st:<8}", state_attr.get(st, 0))
        _safe_addnstr(stdscr, row, 22, f"{s['hz']:>7}")
        _safe_addnstr(stdscr, row, 36, f"{s['duration']:>5.2f} s")
        _safe_addnstr(stdscr, row, 50, f"{s['pause']:>5.2f} s")
        row += 1

    row += 1
    vac_str = "ON" if controller.vacuum else "OFF"
    vac_attr = curses.A_BOLD | curses.A_REVERSE if controller.vacuum else curses.A_DIM
    _safe_addnstr(stdscr, row, 0,
                  f"Profile: {controller.profile}    Focused axis: {focused}    Vacuum: ",
                  curses.A_BOLD)
    _safe_addnstr(stdscr, row, 64, vac_str, vac_attr)
    row += 2

    for line in KEY_HELP:
        _safe_addnstr(stdscr, row, 0, line)
        row += 1
    row += 1

    _safe_addnstr(stdscr, row, 0, f"Last: {controller.last_msg}",
                  curses.A_DIM)
    stdscr.refresh()


def tui_loop(stdscr, controller: Controller) -> None:
    curses.curs_set(0)
    stdscr.timeout(50)
    focused = "X"

    while True:
        draw(stdscr, controller, focused)

        try:
            ch = stdscr.getch()
        except curses.error:
            continue
        if ch == -1:
            continue

        # quit
        if ch in (ord('q'), ord('Q'), 27):
            return

        # jog
        elif ch in (ord('w'), ord('W')): controller.jog("X", +1)
        elif ch in (ord('s'), ord('S')): controller.jog("X", -1)
        elif ch in (ord('a'), ord('A')): controller.jog("Y", +1)
        elif ch in (ord('d'), ord('D')): controller.jog("Y", -1)
        elif ch in (ord('k'), ord('K')): controller.jog("Z", +1)
        elif ch in (ord('j'), ord('J')): controller.jog("Z", -1)

        # focus
        elif ch == ord('1'): focused = "X"
        elif ch == ord('2'): focused = "Y"
        elif ch == ord('3'): focused = "Z"

        # adjust focused axis
        elif ch in (ord('+'), ord('=')):
            controller.settings[focused]["hz"] += HZ_STEP
        elif ch in (ord('-'), ord('_')):
            controller.settings[focused]["hz"] = max(
                MIN_HZ, controller.settings[focused]["hz"] - HZ_STEP)
        elif ch == ord('['):
            controller.settings[focused]["duration"] = max(
                MIN_DURATION,
                controller.settings[focused]["duration"] - DURATION_STEP)
        elif ch == ord(']'):
            controller.settings[focused]["duration"] += DURATION_STEP
        elif ch == ord(','):
            controller.settings[focused]["pause"] = max(
                0.0, controller.settings[focused]["pause"] - DWELL_STEP)
        elif ch == ord('.'):
            controller.settings[focused]["pause"] += DWELL_STEP

        # profile toggle
        elif ch in (ord('e'), ord('E')):
            idx = PROFILES.index(controller.profile)
            controller.profile = PROFILES[(idx + 1) % len(PROFILES)]

        # vacuum toggle
        elif ch in (ord('h'), ord('H')):
            controller.toggle_vacuum()


# ---------- entry -----------------------------------------------------------

def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--profile", choices=PROFILES, default="scurve")
    for ax in VALID_AXES:
        lo = ax.lower()
        p.add_argument(f"--{lo}-hz", type=int, default=80000)
        p.add_argument(f"--{lo}-duration", type=float, default=2.0)
        p.add_argument(f"--{lo}-pause", type=float, default=0.5)
    args = p.parse_args()

    settings = {
        ax: {
            "hz":       getattr(args, f"{ax.lower()}_hz"),
            "duration": getattr(args, f"{ax.lower()}_duration"),
            "pause":    getattr(args, f"{ax.lower()}_pause"),
        }
        for ax in VALID_AXES
    }

    print(f"Opening {args.port} ...")
    with serial.Serial(args.port, 115200, timeout=1.0) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(b"PING\r")
        reply = ser.readline().decode(errors="replace").strip()
        if reply != "PONG":
            print(f"No PONG (got {reply!r}). Is the firmware running?",
                  file=sys.stderr)
            return 1

        controller = Controller(ser, settings, args.profile)
        try:
            curses.wrapper(tui_loop, controller)
        finally:
            controller.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
