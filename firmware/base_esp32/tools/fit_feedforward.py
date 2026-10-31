#!/usr/bin/env python3
"""fit_feedforward.py — measure kff and kstatic for each wheel. WHEELS OFF THE GROUND.

What it does: runs every wheel open-loop at a ladder of duties (0.10, 0.15, ... 0.90),
waits for the speed to settle at each rung, and records the steady speed. A DC motor's
steady speed is close to linear in its average voltage, so the points fall near a line:

        duty  ≈  kff * speed  +  kstatic

  kff      slope: extra duty per rad/s
  kstatic  intercept: duty spent just overcoming friction before anything moves

Rungs where a wheel didn't move are left out of the fit; they still tell you roughly
where static friction is. The result is printed as a ready-to-send "G,..." command and the
matching lines for config.h.

Measured off the ground, kff is the no-load value. On the floor the load adds friction,
which the PI terms absorb. Re-run it if you change motors, gearing or wheels.
"""

from __future__ import annotations

import argparse

from canbot_link import WHEELS, BaseLink, fmt_cmd
from bench import save_csv


def linear_fit(xs: list[float], ys: list[float]) -> tuple[float, float]:
    """Least-squares y = a*x + b. Written out instead of numpy so the Pi needs nothing extra."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    a = sxy / sxx
    return a, my - a * mx


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--settle", type=float, default=1.0, help="seconds per rung (default 1.0)")
    p.add_argument("--max-duty", type=float, default=0.9)
    args = p.parse_args()

    duties = [round(0.10 + 0.05 * k, 2) for k in range(int((args.max_duty - 0.10) / 0.05) + 1)]
    points: list[list[tuple[float, float]]] = [[] for _ in WHEELS]
    all_frames = []

    with BaseLink(args.port) as link:
        print("Wheels must be off the ground. Running", len(duties), "rungs on all four wheels.")
        for direction in (+1, -1):
            for d in duties:
                frames = link.hold(fmt_cmd("D", [direction * d] * 4), args.settle)
                all_frames += frames
                settled = frames[len(frames) // 2:]  # discard the acceleration half
                if not settled:
                    continue
                for i in range(4):
                    v = sum(fr.vel[i] for fr in settled) / len(settled)
                    points[i].append((direction * d, v))
                print(f"  duty {direction * d:+.2f}: " +
                      "  ".join(f"{w} {pts[-1][1]:+6.2f}" for w, pts in zip(WHEELS, points)))
            link.command("S")
        print(f"log: {save_csv('feedforward', all_frames)}")

    print("\nResults (fit on |duty| vs |speed|, both directions pooled):")
    results = []
    for i, w in enumerate(WHEELS):
        moving = [(abs(d), abs(v)) for d, v in points[i] if abs(v) > 0.5 and (d > 0) == (v > 0)]
        wrong_sign = [(d, v) for d, v in points[i] if abs(v) > 0.5 and (d > 0) != (v > 0)]
        if wrong_sign:
            print(f"  {w}: speed has the WRONG SIGN for its duty: fix kMotorInvert/kEncoderInvert first")
            results.append(None)
            continue
        if len(moving) < 3:
            print(f"  {w}: barely moved; check wiring and power")
            results.append(None)
            continue
        kff, kstatic = linear_fit([v for _, v in moving], [d for d, _ in moving])
        top_speed = max(v for _, v in moving)
        print(f"  {w}: kff={kff:.4f}  kstatic={kstatic:.4f}   (top speed {top_speed:.1f} rad/s)")
        results.append((kff, kstatic))

    good = [r for r in results if r]
    if good:
        kff = sum(r[0] for r in good) / len(good)
        kst = max(0.0, sum(r[1] for r in good) / len(good))
        print("\nAverage over wheels. To try them now (keeps the current kp/ki defaults):")
        print(f"  python3 bench.py gains --wheel -1 0.02 0.20 {kff:.4f} {kst:.4f}")
        print("To keep them, edit kDefaultGains in include/config.h:")
        print(f"  /*kff=*/{kff:.4f}f, /*kstatic=*/{kst:.4f}f")
        spread = max(r[0] for r in good) / min(r[0] for r in good)
        if spread > 1.15:
            print(f"Note: kff differs {100 * (spread - 1):.0f}% between wheels. Consider per-wheel gains "
                  "(bench.py gains --wheel N ...) or check for a binding wheel.")


if __name__ == "__main__":
    main()
