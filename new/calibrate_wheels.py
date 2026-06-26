"""Calibrate wheel feed-forward multipliers from encoder deltas.

Run on the floor with clear space. The script sends the same raw motor command
to all 4 wheels, measures encoder delta, then recommends MOTOR_CALIBRATION.
"""

import argparse
import time

from config import MAX_MOTOR_COMMAND, MOTOR_NAMES


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


def compute_calibration(deltas):
    abs_deltas = [abs(delta) for delta in deltas]
    reference = max(abs_deltas)
    if reference == 0:
        raise RuntimeError("All encoder deltas are zero; robot did not move.")

    calibration = []
    for delta in abs_deltas:
        if delta == 0:
            calibration.append(None)
        else:
            calibration.append(reference / delta)
    return reference, abs_deltas, calibration


def print_report(deltas):
    reference, abs_deltas, calibration = compute_calibration(deltas)
    fastest_index = abs_deltas.index(reference)

    print("[INFO] Wheel delta report:")
    for i, name in enumerate(MOTOR_NAMES):
        ratio = abs_deltas[i] / reference if reference else 0
        cal_text = "inf" if calibration[i] is None else f"{calibration[i]:.3f}"
        print(
            f"  M{i + 1} {name:>11}: "
            f"delta={deltas[i]:>7} ratio={ratio:.3f} calibration={cal_text}"
        )

    print(f"[INFO] Fastest wheel: M{fastest_index + 1} {MOTOR_NAMES[fastest_index]}")
    print("[INFO] Suggested config:")
    values = ["0.000" if value is None else f"{value:.3f}" for value in calibration]
    print(f"MOTOR_CALIBRATION = ({', '.join(values)})")


def main():
    parser = argparse.ArgumentParser(description="Calibrate 4 wheel speeds")
    parser.add_argument("--speed", type=float, default=30.0)
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument("--run", action="store_true", help="Actually move robot")
    args = parser.parse_args()

    speed = clamp(abs(args.speed), 0, MAX_MOTOR_COMMAND)
    if args.reverse:
        speed = -speed

    print(f"[INFO] Raw command: set_motor({speed}, {speed}, {speed}, {speed})")
    print(f"[INFO] Duration: {args.duration:.1f}s")

    if not args.run:
        print("[DRY-RUN] Add --run when the robot is on the floor with clear space.")
        return

    from Rosmaster_Lib import Rosmaster

    bot = Rosmaster()
    bot.clear_auto_report_data()
    bot.create_receive_threading()
    time.sleep(1.0)

    start_enc = read_encoder(bot)
    print(f"[INFO] Start encoder: {start_enc}")

    try:
        bot.set_motor(speed, speed, speed, speed)
        time.sleep(args.duration)
    finally:
        bot.set_motor(0, 0, 0, 0)
        time.sleep(0.4)

    end_enc = read_encoder(bot)
    deltas = [end_enc[i] - start_enc[i] for i in range(4)]
    print(f"[INFO] End encoder:   {end_enc}")
    print(f"[INFO] Delta:         {deltas}")
    print_report(deltas)


if __name__ == "__main__":
    main()
