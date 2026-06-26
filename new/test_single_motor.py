"""Short single-motor encoder test."""

import argparse
import time

from config import MAX_MOTOR_COMMAND, MOTOR_NAMES


def read_encoder(bot, retries=5):
    enc = [0, 0, 0, 0]
    for _ in range(retries):
        enc = list(bot.get_motor_encoder())
        if any(value != 0 for value in enc):
            return enc
        time.sleep(0.2)
    return enc


def main():
    parser = argparse.ArgumentParser(description="Run one motor briefly and print encoder delta")
    parser.add_argument("--motor", type=int, choices=(1, 2, 3, 4), required=True)
    parser.add_argument("--speed", type=float, required=True)
    parser.add_argument("--duration", type=float, default=0.5)
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args()

    if abs(args.speed) > MAX_MOTOR_COMMAND:
        parser.error(f"--speed must be in [-{MAX_MOTOR_COMMAND}, {MAX_MOTOR_COMMAND}]")
    if args.duration <= 0:
        parser.error("--duration must be positive")

    commands = [0.0, 0.0, 0.0, 0.0]
    commands[args.motor - 1] = args.speed
    print(f"[INFO] Motor M{args.motor} {MOTOR_NAMES[args.motor - 1]}")
    print(f"[INFO] Command: set_motor({commands[0]:.1f}, {commands[1]:.1f}, {commands[2]:.1f}, {commands[3]:.1f})")
    print(f"[INFO] Duration: {args.duration:.2f}s")

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
    print(f"[INFO] End encoder:   {end_enc}")
    print(f"[INFO] Delta:         {deltas}")


if __name__ == "__main__":
    main()
