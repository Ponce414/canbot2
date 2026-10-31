#!/usr/bin/env python3
"""bench.py — bench tests for the drive base, run with the `bringup` firmware flashed.

Run with the WHEELS OFF THE GROUND (robot on a box) until step 6 of the README.

  python3 bench.py --port /dev/ttyACM0 counts                  # turn wheels by hand, watch counts
  python3 bench.py duty --wheel FL --duty 0.3 --seconds 3       # open loop: direction check
  python3 bench.py step --wheel all --target 8 --seconds 3      # closed loop: step response
  python3 bench.py gains --wheel -1 0.02 0.2 0.05 0.04          # set gains (until reboot)
  python3 bench.py monitor --seconds 10                         # just log telemetry

Every run that moves a wheel writes a CSV (logs/<test>_<time>.csv) you can open in
PlotJuggler, a spreadsheet, or pandas. Tuning is done by looking at plots, not numbers
scrolling past.
"""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

from canbot_link import WHEELS, BaseLink, Frame, describe_faults, fmt_cmd, wheel_vector

LOG_DIR = Path(__file__).resolve().parent / "logs"


def save_csv(name: str, frames: list[Frame]) -> Path:
    LOG_DIR.mkdir(exist_ok=True)
    path = LOG_DIR / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        header = ["t_ms"]
        for key in ("count", "vel", "target", "duty"):
            header += [f"{key}_{wh}" for wh in WHEELS]
        w.writerow(header + ["faults", "battery_v", "loop_us_max"])
        for fr in frames:
            w.writerow([fr.t_ms, *fr.counts, *fr.vel, *fr.target, *fr.duty,
                        fr.faults, fr.battery_v, fr.loop_us_max])
    return path


def step_metrics(frames: list[Frame], i: int, target: float) -> str:
    """Rise time (10→90%), overshoot and steady-state error for wheel i."""
    if not frames or target == 0:
        return "no data"
    t0 = frames[0].t_ms
    t = [(fr.t_ms - t0) / 1000.0 for fr in frames]
    v = [fr.vel[i] * (1 if target > 0 else -1) for fr in frames]  # make the step positive
    goal = abs(target)
    t10 = next((tt for tt, vv in zip(t, v) if vv >= 0.1 * goal), None)
    t90 = next((tt for tt, vv in zip(t, v) if vv >= 0.9 * goal), None)
    rise = f"{(t90 - t10) * 1000:.0f} ms" if t10 is not None and t90 is not None else "never reached 90%"
    overshoot = max(0.0, (max(v) - goal) / goal * 100)
    tail = v[int(len(v) * 0.7):] or v  # last 30% of the run = "steady state"
    ss_err = (sum(tail) / len(tail) - goal) / goal * 100
    return f"rise {rise}, overshoot {overshoot:.1f}%, steady-state error {ss_err:+.1f}%"


def cmd_counts(link: BaseLink, args) -> None:
    link.command("Z")
    print("Encoders zeroed. Turn each wheel FORWARD by hand (the way it turns when the robot")
    print("drives forward). Every count should go UP. Ctrl-C to stop.")
    print("To measure counts/rev: turn one wheel exactly 10 turns, divide its count by 10.")
    try:
        while True:
            fr = link.latest()
            print("  ".join(f"{w}:{c:>8d}" for w, c in zip(WHEELS, fr.counts)), end="\r", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


def cmd_duty(link: BaseLink, args) -> None:
    vec = wheel_vector(args.wheel, args.duty)
    print(f"Open loop {fmt_cmd('D', vec)} for {args.seconds} s ...")
    frames = link.hold(fmt_cmd("D", vec), args.seconds)
    link.command("S")
    path = save_csv("duty", frames)
    tail = frames[len(frames) // 2:] or frames
    for i, w in enumerate(WHEELS):
        if vec[i] == 0:
            continue
        mean_v = sum(fr.vel[i] for fr in tail) / max(1, len(tail))
        ok = "OK" if (mean_v > 0) == (vec[i] > 0) else "WRONG SIGN: flip kMotorInvert or kEncoderInvert"
        print(f"  {w}: duty {vec[i]:+.2f} -> {mean_v:+.2f} rad/s   {ok}")
    print(f"  faults at end: {describe_faults(frames[-1].faults) if frames else 'n/a'}")
    print(f"  log: {path}")


def cmd_step(link: BaseLink, args) -> None:
    vec = wheel_vector(args.wheel, args.target)
    print(f"Velocity step {fmt_cmd('V', vec)} for {args.seconds} s ...")
    frames = link.hold(fmt_cmd("V", vec), args.seconds)
    link.command("S")
    path = save_csv("step", frames)
    for i, w in enumerate(WHEELS):
        if vec[i] != 0:
            print(f"  {w}: {step_metrics(frames, i, vec[i])}")
    if frames:
        print(f"  worst control step: {frames[-1].loop_us_max} us (budget 10000 us)")
        print(f"  faults at end: {describe_faults(frames[-1].faults)}")
    print(f"  log: {path}")


def cmd_gains(link: BaseLink, args) -> None:
    link.command(f"G,{args.wheel},{args.kp},{args.ki},{args.kff},{args.kstatic}")
    print("Gains set (RAM only: copy them into config.h once you're happy with them).")


def cmd_monitor(link: BaseLink, args) -> None:
    link.drain()
    time.sleep(args.seconds)
    frames = link.drain()
    print(f"{len(frames)} frames, {len(frames) / args.seconds:.0f} Hz. Log: {save_csv('monitor', frames)}")
    if frames:
        print(f"faults: {describe_faults(frames[-1].faults)}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0", help="serial port (default /dev/ttyACM0)")
    sub = p.add_subparsers(dest="test", required=True)

    sub.add_parser("counts", help="stream encoder counts while you turn wheels by hand")

    d = sub.add_parser("duty", help="open-loop duty on one wheel or all")
    d.add_argument("--wheel", default="FL", help="FL, FR, RL, RR or all")
    d.add_argument("--duty", type=float, default=0.3)
    d.add_argument("--seconds", type=float, default=3.0)

    s = sub.add_parser("step", help="closed-loop velocity step and metrics")
    s.add_argument("--wheel", default="all")
    s.add_argument("--target", type=float, default=8.0, help="rad/s")
    s.add_argument("--seconds", type=float, default=3.0)

    g = sub.add_parser("gains", help="set PID gains")
    g.add_argument("--wheel", type=int, default=-1, help="0..3, or -1 for all")
    for name in ("kp", "ki", "kff", "kstatic"):
        g.add_argument(name, type=float)

    m = sub.add_parser("monitor", help="log telemetry without commanding anything")
    m.add_argument("--seconds", type=float, default=5.0)

    args = p.parse_args()
    handlers = {"counts": cmd_counts, "duty": cmd_duty, "step": cmd_step,
                "gains": cmd_gains, "monitor": cmd_monitor}
    with BaseLink(args.port) as link:
        handlers[args.test](link, args)


if __name__ == "__main__":
    main()
