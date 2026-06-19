"""CLI smoke test for fixed-waypoint movement.

Default mode is dry-run. Add --run only when the robot is on the field and the
route distances in config.py have been checked.
"""

import argparse

from config import DEFAULT_FORWARD_SPEED, FULL_MOVE_FLOW, ROUTES
from navigation_node import NavigationNode
from robot_driver import RobotDriver


def print_routes():
    for (start, goal), route in ROUTES.items():
        print(f"{start} -> {goal}: {route}")


def main():
    parser = argparse.ArgumentParser(description="Test fixed-waypoint movement")
    parser.add_argument("--start", default="home", help="Start waypoint")
    parser.add_argument("--goal", help="Goal waypoint")
    parser.add_argument("--full-flow", action="store_true", help="Run map flow")
    parser.add_argument("--forward", type=float, help="Move forward this many meters")
    parser.add_argument("--turn", type=float, help="Turn by this many degrees")
    parser.add_argument("--speed", type=float, default=DEFAULT_FORWARD_SPEED)
    parser.add_argument("--debug", action="store_true", help="Print wheel sync data")
    parser.add_argument("--list", action="store_true", help="List configured routes")
    parser.add_argument(
        "--run",
        action="store_true",
        help="Move the real robot. Without this flag the script only prints steps.",
    )
    args = parser.parse_args()

    if args.list:
        print_routes()
        return

    driver = RobotDriver(dry_run=not args.run, debug=args.debug)

    if args.forward is not None:
        driver. (args.forward, speed=args.speed)
        return

    if args.turn is not None:
        driver.turn_by(args.turn)
        return

    nav = NavigationNode(driver=driver)

    if args.full_flow:
        nav.move_flow(FULL_MOVE_FLOW, speed=args.speed)
        return

    if not args.goal:
        parser.error("--goal is required unless --full-flow or --list is used")

    nav.move_between(args.start, args.goal, speed=args.speed)


if __name__ == "__main__":
    main()
