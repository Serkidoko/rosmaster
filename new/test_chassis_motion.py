"""Small chassis motion test for bbox approach tuning.

Default mode is dry-run. Add --run only when the robot has clearance.
"""

import argparse

from config import DEFAULT_FORWARD_SPEED
from robot_driver import RobotDriver


def main():
    parser = argparse.ArgumentParser(description="Test one small chassis movement.")
    parser.add_argument(
        "--direction",
        choices=("forward", "left", "right"),
        default="left",
        help="Movement direction to test.",
    )
    parser.add_argument("--distance-m", type=float, default=0.05)
    parser.add_argument("--speed", type=float, default=DEFAULT_FORWARD_SPEED)
    parser.add_argument("--lateral-speed-mps", type=float, default=0.16)
    parser.add_argument(
        "--lateral-mode",
        choices=("calibrated", "firmware"),
        default="calibrated",
        help="Use separate wheel calibration for left/right, or firmware set_car_motion.",
    )
    parser.add_argument("--debug", action="store_true")
    parser.add_argument(
        "--run",
        action="store_true",
        help="Move the real robot. Without this flag this is dry-run.",
    )
    args = parser.parse_args()

    if args.distance_m <= 0:
        parser.error("--distance-m must be positive")
    if args.speed <= 0:
        parser.error("--speed must be positive")
    if args.lateral_speed_mps <= 0:
        parser.error("--lateral-speed-mps must be positive")

    driver = RobotDriver(dry_run=not args.run, debug=args.debug)
    if args.direction == "forward":
        driver.go_forward(args.distance_m, speed=args.speed)
    elif args.lateral_mode == "calibrated":
        driver.move_lateral_calibrated(
            args.direction,
            args.distance_m,
            speed_mps=args.lateral_speed_mps,
        )
    else:
        driver.move_lateral_timed(
            args.direction,
            args.distance_m,
            speed_mps=args.lateral_speed_mps,
        )


if __name__ == "__main__":
    main()
