#!/usr/bin/env python3
"""Interactive servo calibration via Pi 5 hardware PWM (sysfs).

Requires `dtoverlay=pwm-2chan` in /boot/firmware/config.txt.
GPIO 12 -> channel 0, GPIO 13 -> channel 1 (verify on your system).

Usage:  sudo python3 servo_calibrate.py --chip 0 --channel 0
Keys:   a/d = -/+ 10 us   A/D = -/+ 50 us   c = go to 1500 us
        m = save current as MIN   M = save as MAX   z = save as CENTER
        q = quit (PWM disabled)
Someone must stay next to the servos. Stop as soon as the servo hums
or presses against the bracket: that position is beyond the limit.
"""
import argparse
import sys
import termios
import tty
from pathlib import Path

PERIOD_NS = 20_000_000          # 50 Hz
HARD_MIN_US, HARD_MAX_US = 600, 2440   # absolute safety clamp


def write(path: Path, value) -> None:
    path.write_text(str(value))


def getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--chip", type=int, default=0)
    ap.add_argument("--channel", type=int, default=0)
    args = ap.parse_args()

    chip = Path(f"/sys/class/pwm/pwmchip{args.chip}")
    pwm = chip / f"pwm{args.channel}"
    if not pwm.exists():
        write(chip / "export", args.channel)

    us = 1500
    saved = {}
    write(pwm / "period", PERIOD_NS)
    write(pwm / "duty_cycle", us * 1000)
    write(pwm / "enable", 1)

    steps = {"a": -10, "d": 10, "A": -50, "D": 50}
    try:
        while True:
            print(f"\r{us:5d} us   saved: {saved}      ", end="", flush=True)
            k = getch()
            if k == "q":
                break
            if k in steps:
                us = max(HARD_MIN_US, min(HARD_MAX_US, us + steps[k]))
            elif k == "c":
                us = 1500
            elif k in ("m", "M", "z"):
                saved[{"m": "min_us", "M": "max_us", "z": "center_us"}[k]] = us
                continue
            write(pwm / "duty_cycle", us * 1000)
    finally:
        write(pwm / "enable", 0)
        print(f"\nPWM disabled. Values: {saved}")


if __name__ == "__main__":
    main()
