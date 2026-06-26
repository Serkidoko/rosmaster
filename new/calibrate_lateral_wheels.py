"""Calibrate lateral wheel multipliers from raw encoder deltas.

This script runs the mecanum lateral profile without wheel sync or yaw
correction, then recommends the direction-specific lateral calibration.
By default it sends equal raw command magnitude to all four wheels, similar to
calibrate_wheels.py, so a bad old calibration cannot hide a weak wheel.
"""

import argparse
import atexit
import sys
import time
from datetime import datetime
from pathlib import Path

from config import (
    LATERAL_LEFT_MOTOR_CALIBRATION,
    LATERAL_LEFT_PROFILE,
    LATERAL_RIGHT_MOTOR_CALIBRATION,
    LATERAL_RIGHT_PROFILE,
    MAX_MOTOR_COMMAND,
    MOTOR_NAMES,
)


DEFAULT_LOG_DIR = Path(__file__).resolve().parent.parent


class TeeStream:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, data):
        for stream in self.streams:
            stream.write(data)
            stream.flush()

    def flush(self):
        for stream in self.streams:
            stream.flush()


def clamp(value, low, high):
    return max(low, min(high, value))


def read_encoder(bot, retries=5):
    enc = [0, 0, 0, 0]
    for _ in range(retries):
        enc = list(bot.get_motor_encoder())
        if any(value != 0 for value in enc):
            return enc
        time.sleep(0.2)
    return enc


def default_log_path(direction):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return DEFAULT_LOG_DIR / f"lateral_calib_{direction}_{timestamp}.log"


def setup_output_log(args):
    if args.no_log or (not args.run and not args.log):
        return None

    log_path = Path(args.log) if args.log else default_log_path(args.direction)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", encoding="utf-8")
    original_stdout = sys.stdout
    original_stderr = sys.stderr
    sys.stdout = TeeStream(original_stdout, log_file)
    sys.stderr = TeeStream(original_stderr, log_file)

    def close_log():
        sys.stdout.flush()
        sys.stderr.flush()
        sys.stdout = original_stdout
        sys.stderr = original_stderr
        log_file.close()

    atexit.register(close_log)
    print(f"[INFO] Log file: {log_path}")
    return log_path


def main():
    parser = argparse.ArgumentParser(description="Calibrate lateral wheel speeds")
    parser.add_argument("--direction", choices=("left", "right"), default="left")
    parser.add_argument(
        "--speed",
        type=float,
        default=32.0,
        help="Base raw motor command. Keep low enough to avoid command saturation.",
    )
    parser.add_argument("--duration", type=float, default=0.8)
    parser.add_argument(
        "--min-valid-delta",
        type=int,
        default=100,
        help="Minimum absolute encoder delta required for each wheel.",
    )
    parser.add_argument(
        "--allow-invalid-suggestion",
        action="store_true",
        help="Print suggested config even when the sanity checks fail.",
    )
    parser.add_argument(
        "--use-current-calibration",
        action="store_true",
        help=(
            "Use the current direction-specific calibration while measuring. "
            "Default is identity calibration for a clean raw-wheel test."
        ),
    )
    parser.add_argument(
        "--log",
        help="Optional log file path. With --run, a timestamped log is written by default.",
    )
    parser.add_argument(
        "--no-log",
        action="store_true",
        help="Disable automatic log file creation in --run mode.",
    )
    parser.add_argument("--run", action="store_true", help="Actually move robot")
    args = parser.parse_args()

    if args.speed <= 0:
        parser.error("--speed must be positive")
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.min_valid_delta < 0:
        parser.error("--min-valid-delta must be zero or positive")

    if args.direction == "left":
        profile = LATERAL_LEFT_PROFILE
        calibration = LATERAL_LEFT_MOTOR_CALIBRATION
        config_name = "LATERAL_LEFT_MOTOR_CALIBRATION"
    else:
        profile = LATERAL_RIGHT_PROFILE
        calibration = LATERAL_RIGHT_MOTOR_CALIBRATION
        config_name = "LATERAL_RIGHT_MOTOR_CALIBRATION"

    setup_output_log(args)

    command_calibration = calibration if args.use_current_calibration else (1.0, 1.0, 1.0, 1.0)
    base_speed = clamp(args.speed, 0, MAX_MOTOR_COMMAND)
    commands = [
        clamp(profile[i] * base_speed * command_calibration[i], -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND)
        for i in range(4)
    ]
    saturated_motors = [
        f"M{i + 1}"
        for i, command in enumerate(commands)
        if abs(command) >= MAX_MOTOR_COMMAND
    ]

    print(f"[INFO] Direction: {args.direction}")
    print(f"[INFO] Current {config_name} = {calibration}")
    if args.use_current_calibration:
        print("[INFO] Command mode: current calibration")
    else:
        print("[INFO] Command mode: identity calibration")
    print(f"[INFO] Raw command: set_motor({commands[0]:.1f}, {commands[1]:.1f}, {commands[2]:.1f}, {commands[3]:.1f})")
    print(f"[INFO] Duration: {args.duration:.2f}s")
    if saturated_motors:
        print(
            "[WARN] Command saturation on "
            f"{', '.join(saturated_motors)}; lower --speed before trusting "
            "the suggested calibration."
        )

    if not args.run:
        print("[DRY-RUN] Add --run when the robot has clear space.")
        return

    from Rosmaster_Lib import Rosmaster

    bot = Rosmaster()
    bot.clear_auto_report_data()
    bot.create_receive_threading()
    time.sleep(1.0)

    start_enc = read_encoder(bot)
    print(f"[INFO] Start encoder: {start_enc}")

    try:
        bot.set_motor(commands[0], commands[1], commands[2], commands[3])
        time.sleep(args.duration)
    finally:
        bot.set_motor(0, 0, 0, 0)
        time.sleep(0.4)

    end_enc = read_encoder(bot)
    deltas = [end_enc[i] - start_enc[i] for i in range(4)]
    abs_deltas = [abs(delta) for delta in deltas]
    reference = sum(abs_deltas) / 4.0
    print(f"[INFO] End encoder:   {end_enc}")
    print(f"[INFO] Delta:         {deltas}")
    print(f"[INFO] Abs delta:     {abs_deltas}")

    if reference == 0:
        print("[ERROR] All encoder deltas are zero; robot did not move.")
        return 2

    weak_motors = [
        f"M{i + 1} {MOTOR_NAMES[i]}"
        for i, delta in enumerate(abs_deltas)
        if delta < args.min_valid_delta
    ]
    invalid_reasons = []
    if saturated_motors:
        invalid_reasons.append(f"command saturation on {', '.join(saturated_motors)}")
    if weak_motors:
        invalid_reasons.append(
            "encoder delta below "
            f"{args.min_valid_delta} on {', '.join(weak_motors)}"
        )
    calibration_valid = not invalid_reasons

    suggested = []
    print("[INFO] Lateral wheel delta report:")
    for i, name in enumerate(MOTOR_NAMES):
        ratio = abs_deltas[i] / reference if reference else 0.0
        if abs_deltas[i] == 0:
            next_value = command_calibration[i]
        else:
            next_value = command_calibration[i] * reference / abs_deltas[i]
        suggested.append(next_value)
        next_text = (
            f"{next_value:.3f}"
            if calibration_valid or args.allow_invalid_suggestion
            else "invalid"
        )
        print(
            f"  M{i + 1} {name:>11}: "
            f"delta={deltas[i]:>7} ratio={ratio:.3f} next={next_text}"
        )

    if not calibration_valid:
        print("[ERROR] Calibration result failed sanity checks:")
        for reason in invalid_reasons:
            print(f"       - {reason}")
        print(
            "[ERROR] Do not copy these values into config.py. Test the weak "
            "motor(s), motor direction, contact with the floor, and speed "
            "deadband first."
        )
        if not args.allow_invalid_suggestion:
            return 2

    values = ", ".join(f"{value:.3f}" for value in suggested)
    print("[INFO] Suggested config:")
    print(f"{config_name} = ({values})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
