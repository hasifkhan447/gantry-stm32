#!/usr/bin/env python3
"""XYZ-coordinate -> MOVE-command translator.

Holds the steps-per-mm table in one place. Tune STEPS_PER_MM if you
change a pulley, regear, or repoint the Sigma-5 electronic gear
(Pn20E numerator / Pn210 denominator).

Current rig:
    pulley diameter        80 mm
    pulley circumference   π * 80 = 251.327 mm  per pulley rev
    gearbox                1:20   (20 motor rev = 1 pulley rev)
    linear per motor rev   251.327 / 20 = 12.566 mm
    Sigma-5 command pulses 1000 per motor rev
    => steps/mm            1000 / 12.566 = 79.577

Speed table at the resulting 79.577 steps/mm:
    feed_hz     rpm     mm/s    m/s     regime
    50_000      3000    628     0.63    continuous-safe (motor rated)
    80_000      4800    1006    1.00    short bursts only
    100_000     6000    1257    1.26    motor peak — don't sustain

Usage as a library:
    from coords import xyz_to_steps, move_commands
    cmds = move_commands((x_mm, y_mm, z_mm),
                         current=(cx, cy, cz),
                         feed_hz=80000)
    for c in cmds:
        ser.write((c + "\\r").encode())

Usage from the shell:
    python3 coords.py 100 50 -20            # absolute, from (0,0,0)
    python3 coords.py --from 10,0,0 100 50 -20
    python3 coords.py --feed 50000 200 0 0
"""

import argparse
import math
import time
from typing import Iterable, List, Tuple

AXES: Tuple[str, str, str] = ("X", "Y", "Z")

# Transmission constants — edit these, not STEPS_PER_MM, when something
# physical changes. STEPS_PER_MM is derived below.
PULLEY_DIAMETER_MM = 80.0
GEARBOX_RATIO      = 20.0    # motor rev per pulley rev
STEPS_PER_MOTOR_REV = 1000   # Sigma-5 electronic gear (Pn20E / Pn210)

_MM_PER_MOTOR_REV = math.pi * PULLEY_DIAMETER_MM / GEARBOX_RATIO   # 12.566
_STEPS_PER_MM = STEPS_PER_MOTOR_REV / _MM_PER_MOTOR_REV            # 79.577

# All three axes share the transmission today. If Z gets a different
# screw / belt, override the Z entry directly.
STEPS_PER_MM = {
    "X": _STEPS_PER_MM,
    "Y": _STEPS_PER_MM,
    "Z": _STEPS_PER_MM,
}

# 50 kHz keeps the motor at 3000 rpm (its continuous rating).
# Up the feed when you call move_commands() for short, bursty moves.
DEFAULT_FEED_HZ = 50000


def mm_to_steps(axis: str, mm: float) -> int:
    """Round mm to the nearest step count for `axis`."""
    return int(round(mm * STEPS_PER_MM[axis]))


def steps_to_mm(axis: str, steps: int) -> float:
    return steps / STEPS_PER_MM[axis]


def xyz_to_steps(target_mm: Iterable[float]) -> Tuple[int, int, int]:
    x, y, z = target_mm
    return (mm_to_steps("X", x),
            mm_to_steps("Y", y),
            mm_to_steps("Z", z))


def move_commands(target_mm: Iterable[float],
                  current_mm: Iterable[float] = (0.0, 0.0, 0.0),
                  feed_hz: int = DEFAULT_FEED_HZ) -> List[str]:
    """Return the MOVE lines that take you from current_mm to target_mm.

    Axes that don't move are omitted. The firmware accepts each axis
    independently; if you want strictly coordinated motion, drive them
    one at a time and STAT-poll between.
    """
    out: List[str] = []
    for axis, t, c in zip(AXES, target_mm, current_mm):
        delta = mm_to_steps(axis, t - c)
        if delta != 0:
            out.append(f"MOVE {axis} {delta} F {feed_hz}")
    return out


def _parse_triple(s: str) -> Tuple[float, float, float]:
    parts = [float(x) for x in s.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected x,y,z")
    return parts[0], parts[1], parts[2]


# ---------------------------------------------------------------------------
# Homing helpers
# ---------------------------------------------------------------------------
#
# Firmware contract used here:
#   MOVE <axis> <signed_steps> F <hz>   -- start a move, ACK on accept
#   STAT                                -- "X:idle Y:busy Z:idle"
#   LIM                                 -- "X1:0/0 X2:0/0 Y1:1/1 Y2:0/0 ES:0/0"
#                                          (active/latched per limit)
#   STOP <axis>                         -- host-driven abort
#   RESUME                              -- clear ESTOP + limit latches
#
# When a limit switch trips, the firmware EXTI aborts the axis inside the
# ISR (PWM killed, queue drained on the axis task wake-up). The host
# does not need to send STOP — it just notices the axis went idle while
# a latch is asserted.
#
# Travel limits and back-off distance below are conservative defaults.
# Tune HOMING_TRAVEL_MM up if you home from the far end, and BACKOFF_MM
# to whatever clears the switch with a comfortable margin.

HOMING_TRAVEL_MM = 5000.0   # bigger than any axis on this rig (4 m max)
HOMING_FEED_HZ   = 20000    # ~0.25 m/s — slow enough to land on the switch
BACKOFF_MM       = 5.0
BACKOFF_FEED_HZ  = 5000


def _send_line(ser, line: str, timeout: float = 1.0) -> str:
    ser.write((line + "\r").encode())
    deadline = time.monotonic() + timeout
    buf = b""
    while time.monotonic() < deadline:
        chunk = ser.read(1)
        if not chunk:
            continue
        buf += chunk
        if buf.endswith(b"\n"):
            return buf.decode(errors="replace").strip()
    raise TimeoutError(f"no reply to {line!r} in {timeout}s")


def _axis_busy(ser, axis: str) -> bool:
    reply = _send_line(ser, "STAT")
    return f"{axis}:busy" in reply


def _limits_state(ser):
    """Return a dict like {'X1': (active, latched), ...}."""
    reply = _send_line(ser, "LIM")
    out = {}
    for tok in reply.split():
        name, val = tok.split(":")
        active, latched = val.split("/")
        out[name] = (active == "1", latched == "1")
    return out


def home_axis(ser, axis: str, direction: int = -1) -> bool:
    """Drive `axis` into its limit switch and zero the host-side
    bookkeeping. Returns True on success.

    direction: -1 homes toward X1/Y1 (the negative end), +1 toward X2/Y2.
    The caller is expected to update its own current_mm[axis] = 0 after
    this returns True. STEPS_PER_MM is used to size the back-off only.
    """
    if axis not in AXES:
        raise ValueError(f"bad axis {axis!r}")
    if direction not in (-1, +1):
        raise ValueError("direction must be -1 or +1")

    # 1. clear any stale latches from a previous run
    _send_line(ser, "RESUME")

    # 2. seek: long slow move toward the switch. The firmware will
    #    abort it the moment the limit fires.
    travel_steps = direction * int(round(HOMING_TRAVEL_MM * STEPS_PER_MM[axis]))
    ack = _send_line(ser, f"MOVE {axis} {travel_steps} F {HOMING_FEED_HZ}")
    if ack != "ACK":
        raise RuntimeError(f"seek MOVE rejected: {ack!r}")

    # 3. wait for the axis to go idle. The only ways out are:
    #    - we hit the switch (limit latched, axis idle)
    #    - we somehow walked off the end without latching (failure)
    timeout = HOMING_TRAVEL_MM / (HOMING_FEED_HZ / STEPS_PER_MM[axis]) + 2.0
    deadline = time.monotonic() + timeout
    while _axis_busy(ser, axis):
        if time.monotonic() > deadline:
            _send_line(ser, f"STOP {axis}")
            raise TimeoutError(f"homing {axis}: axis never idled")
        time.sleep(0.05)

    end_name = "X1" if axis == "X" and direction < 0 else \
               "X2" if axis == "X"                   else \
               "Y1" if axis == "Y" and direction < 0 else \
               "Y2" if axis == "Y"                   else \
               None
    if end_name is None:
        # Z has no switch wired yet; treat the seek as authoritative
        return True

    lim = _limits_state(ser)
    if not lim[end_name][1]:
        raise RuntimeError(f"homing {axis}: idle but {end_name} not latched")

    # 4. back off until the switch releases. RESUME clears the latch
    #    so we can issue MOVE again.
    _send_line(ser, "RESUME")
    backoff_steps = -direction * int(round(BACKOFF_MM * STEPS_PER_MM[axis]))
    ack = _send_line(ser, f"MOVE {axis} {backoff_steps} F {BACKOFF_FEED_HZ}")
    if ack != "ACK":
        raise RuntimeError(f"backoff MOVE rejected: {ack!r}")
    while _axis_busy(ser, axis):
        time.sleep(0.02)

    # 5. confirm the switch is no longer asserted
    lim = _limits_state(ser)
    if lim[end_name][0]:
        raise RuntimeError(f"homing {axis}: {end_name} still active after backoff")
    return True


def home_all(ser, order=("X", "Y")) -> None:
    """Home each named axis in turn. Z is intentionally skipped by
    default because there's no Z limit wired yet."""
    for axis in order:
        home_axis(ser, axis, direction=-1)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("x", type=float)
    p.add_argument("y", type=float)
    p.add_argument("z", type=float)
    p.add_argument("--from", dest="origin", type=_parse_triple,
                   default=(0.0, 0.0, 0.0),
                   help="current position as x,y,z mm (default 0,0,0)")
    p.add_argument("--feed", type=int, default=DEFAULT_FEED_HZ,
                   help="pulse rate in Hz (default %(default)s)")
    args = p.parse_args()

    for cmd in move_commands((args.x, args.y, args.z), args.origin, args.feed):
        print(cmd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
