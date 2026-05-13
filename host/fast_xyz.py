#!/usr/bin/env python3
"""Gantry trapezoid driver: interactive control + reversal scan.

Each axis (X, Y, Z) keeps its own peak rate, leg duration, and reversal
dwell. You can drive a single axis or any combination concurrently.

Interactive (default):
    fx | fy | fz | fxy | fxyz | ...   forward leg on each named axis
    bx | by | bz | ...                backward leg
    rxy [N]                           N cycles of fwd / dwell / back / dwell
    sx <hz>   sy <hz>   sz <hz>       set peak step rate per axis
    dx <s>    dy <s>    dz <s>        set leg duration per axis
    px <s>    py <s>    pz <s>        set reversal dwell per axis
    ?                                 show settings + help
    q                                 quit

  The axis suffix is any combination of {x, y, z}. The space form still
  works too: `f xy` is equivalent to `fxy`, `s x 80000` to `sx 80000`.

Scan (--scan): repeats fwd / dwell / back / dwell with a dwell that
shrinks each pass. Use --scan-axes to pick which axes participate
(default x). Watch the gantry; Ctrl-C when you see racking.
"""

import argparse
import sys
import time
from typing import Dict, Iterable, List, Tuple

import serial  # pip install pyserial


VALID_AXES = ("X", "Y", "Z")
CMD_LETTERS = ("f", "b", "r", "s", "d", "p")
WORDS = {"q", "quit", "exit", "h", "help", "?"}

MIN_HZ = 2000
N_ACCEL = 20
N_DECEL = 20
N_CRUISE = 8
ACCEL_FRAC = 0.2
DECEL_FRAC = 0.2


def trapezoid_segments(peak_hz: int, duration: float) -> List[Tuple[int, int]]:
    t_accel = ACCEL_FRAC * duration
    t_decel = DECEL_FRAC * duration
    t_cruise = duration - t_accel - t_decel
    if t_cruise < 0:
        raise ValueError("ACCEL_FRAC + DECEL_FRAC must be < 1")

    segs: List[Tuple[int, int]] = []

    dt_a = t_accel / N_ACCEL
    for j in range(N_ACCEL):
        hz = MIN_HZ + (peak_hz - MIN_HZ) * (j + 0.5) / N_ACCEL
        steps = int(round(hz * dt_a))
        if steps > 0:
            segs.append((steps, int(round(hz))))

    dt_c = t_cruise / N_CRUISE
    cruise_steps = int(round(peak_hz * dt_c))
    if cruise_steps > 0:
        for _ in range(N_CRUISE):
            segs.append((cruise_steps, peak_hz))

    dt_d = t_decel / N_DECEL
    for j in range(N_DECEL):
        hz = peak_hz - (peak_hz - MIN_HZ) * (j + 0.5) / N_DECEL
        steps = int(round(hz * dt_d))
        if steps > 0:
            segs.append((steps, int(round(hz))))

    return segs


# ---------- serial helpers --------------------------------------------------

def send(ser: serial.Serial, line: str) -> str:
    ser.write((line + "\r").encode())
    return ser.readline().decode(errors="replace").strip()


def send_move(ser: serial.Serial, line: str) -> bool:
    for _ in range(200):
        reply = send(ser, line)
        if reply == "ACK":
            return True
        if reply == "ERR queue full":
            time.sleep(0.005)
            continue
        print(f"  unexpected reply to {line!r}: {reply!r}", file=sys.stderr)
        return False
    print(f"  giving up on {line!r}: queue stayed full", file=sys.stderr)
    return False


def wait_axes_idle(ser: serial.Serial,
                   axes: Iterable[str],
                   poll_hz: float = 20.0) -> None:
    period = 1.0 / poll_hz
    busy_toks = [f"{a}:busy" for a in axes]
    while True:
        reply = send(ser, "STAT")
        if reply and not any(t in reply for t in busy_toks):
            return
        time.sleep(period)


# ---------- motion ----------------------------------------------------------

def run_leg(ser: serial.Serial,
            axes: List[str],
            settings: Dict[str, dict],
            direction: int) -> bool:
    """Run one trapezoidal leg on each named axis concurrently."""
    per_axis = {a: trapezoid_segments(settings[a]["hz"],
                                       settings[a]["duration"])
                for a in axes}
    max_len = max(len(s) for s in per_axis.values())
    for i in range(max_len):
        for a in axes:
            segs = per_axis[a]
            if i < len(segs):
                steps, hz = segs[i]
                if not send_move(ser, f"MOVE {a} {direction * steps} F {hz}"):
                    return False
    return True


def reversal_cycle(ser: serial.Serial,
                   axes: List[str],
                   settings: Dict[str, dict],
                   dwell: float) -> bool:
    for direction, tag in ((+1, "forward"), (-1, "reverse")):
        print(f"  {tag} {''.join(axes)}")
        if not run_leg(ser, axes, settings, direction):
            return False
        wait_axes_idle(ser, axes)
        print(f"  dwell {dwell:.3f} s")
        time.sleep(dwell)
    return True


# ---------- parsing ---------------------------------------------------------

def parse_axes(s: str) -> List[str]:
    """'x'->['X'], 'xyz'->['X','Y','Z'], 'zx'->['Z','X']. Unknown chars dropped."""
    out: List[str] = []
    for c in s.upper():
        if c in VALID_AXES and c not in out:
            out.append(c)
    return out


def split_command(raw: str) -> Tuple[str, List[str]]:
    """Return (cmd, args). Supports both `fxy` and `f xy`, etc.

    The first token is `fxy` -> cmd='f', args=['xy', ...trailing args].
    The first token is `f`  -> cmd='f', args unchanged from parts[1:].
    """
    parts = raw.split()
    head = parts[0]
    tail = parts[1:]
    low = head.lower()

    if low in WORDS:
        return low, tail
    if len(low) >= 2 and low[0] in CMD_LETTERS:
        return low[0], [low[1:]] + tail
    return low, tail


# ---------- REPL ------------------------------------------------------------

HELP = """\
Commands (axis letters are any of x, y, z; combine them like xy, xyz):
  f<axes>            forward leg on each axis           (e.g. fx, fxy, fxyz)
  b<axes>            backward leg on each axis
  r<axes> [N]        N forward/dwell/back/dwell cycles  (default N=1)
  s<axis> <hz>       set peak rate
  d<axis> <secs>     set leg duration
  p<axis> <secs>     set reversal dwell
  ?                  show settings + help
  q                  quit
Space form also works:  `f xy`, `s x 80000`, etc."""


def show_state(settings: Dict[str, dict]) -> None:
    for a in VALID_AXES:
        s = settings[a]
        print(f"  {a}: {s['hz']:>7} Hz peak | "
              f"{s['duration']:.2f} s legs | "
              f"{s['pause']:.2f} s dwell")


def cycle_dwell(axes: List[str], settings: Dict[str, dict]) -> float:
    return max(settings[a]["pause"] for a in axes)


def interactive(ser: serial.Serial, settings: Dict[str, dict]) -> None:
    print(HELP)
    show_state(settings)
    while True:
        try:
            raw = input("gantry> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not raw:
            continue
        cmd, args = split_command(raw)
        try:
            if cmd in ("q", "quit", "exit"):
                return

            elif cmd in ("f", "b"):
                if not args:
                    print(f"usage: {cmd}<axes>  (e.g. {cmd}x, {cmd}xy)")
                    continue
                axes = parse_axes(args[0])
                if not axes:
                    print(f"no valid axes in {args[0]!r}")
                    continue
                direction = +1 if cmd == "f" else -1
                run_leg(ser, axes, settings, direction)
                wait_axes_idle(ser, axes)

            elif cmd == "r":
                if not args:
                    print("usage: r<axes> [N]")
                    continue
                axes = parse_axes(args[0])
                if not axes:
                    print(f"no valid axes in {args[0]!r}")
                    continue
                n = int(args[1]) if len(args) > 1 else 1
                dwell = cycle_dwell(axes, settings)
                for i in range(n):
                    print(f"-- cycle {i + 1}/{n}")
                    if not reversal_cycle(ser, axes, settings, dwell):
                        break

            elif cmd in ("s", "d", "p"):
                if len(args) < 2:
                    print(f"usage: {cmd}<axis> <value>")
                    continue
                ax = parse_axes(args[0])
                if not ax:
                    print(f"unknown axis: {args[0]!r}")
                    continue
                axis = ax[0]
                if cmd == "s":
                    settings[axis]["hz"] = int(args[1])
                elif cmd == "d":
                    settings[axis]["duration"] = float(args[1])
                else:
                    settings[axis]["pause"] = float(args[1])
                show_state(settings)

            elif cmd in ("?", "h", "help"):
                show_state(settings)
                print(HELP)

            else:
                print(f"unknown command: {cmd!r} (try ?)")

        except (IndexError, ValueError) as e:
            print(f"bad args for {cmd!r}: {e}")


# ---------- scan ------------------------------------------------------------

def scan(ser: serial.Serial,
         axes: List[str],
         settings: Dict[str, dict],
         pause_start: float,
         pause_step: float,
         pause_min: float) -> None:
    print(f"\nScan {''.join(axes)}: dwell {pause_start:.3f} -> "
          f"{pause_min:.3f} step {pause_step:.3f}")
    print("Ctrl-C the moment you see racking. The last completed dwell "
          "is the minimum safe value.\n")
    dwell = pause_start
    iteration = 0
    last_completed = None
    try:
        while dwell >= pause_min - 1e-9:
            iteration += 1
            print(f"--- iter {iteration}: dwell = {dwell:.3f} s ---")
            if not reversal_cycle(ser, axes, settings, dwell):
                break
            last_completed = dwell
            dwell -= pause_step
    except KeyboardInterrupt:
        print()
    if last_completed is not None:
        print(f"\nLast completed dwell: {last_completed:.3f} s")


# ---------- entry -----------------------------------------------------------

def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--port", default="/dev/ttyACM0")
    for ax in VALID_AXES:
        lo = ax.lower()
        p.add_argument(f"--{lo}-hz", type=int, default=80000)
        p.add_argument(f"--{lo}-duration", type=float, default=5.0)
        p.add_argument(f"--{lo}-pause", type=float, default=1.0)

    p.add_argument("--scan", action="store_true",
                   help="run reversal scan instead of REPL")
    p.add_argument("--scan-axes", default="x",
                   help="axes to drive in scan mode (e.g. x, xy, xyz)")
    p.add_argument("--pause-step", type=float, default=0.1)
    p.add_argument("--pause-min", type=float, default=0.0)
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
        if send(ser, "PING") != "PONG":
            print("No PONG — is the firmware running?", file=sys.stderr)
            return 1
        print(f"  VER:  {send(ser, 'VER')}")
        print(f"  STAT: {send(ser, 'STAT')}\n")

        if args.scan:
            axes = parse_axes(args.scan_axes)
            if not axes:
                print(f"no valid axes in --scan-axes {args.scan_axes!r}",
                      file=sys.stderr)
                return 1
            start_dwell = max(settings[a]["pause"] for a in axes)
            scan(ser, axes, settings,
                 start_dwell, args.pause_step, args.pause_min)
        else:
            interactive(ser, settings)

    return 0


if __name__ == "__main__":
    sys.exit(main())
