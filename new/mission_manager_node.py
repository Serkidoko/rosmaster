"""Simple FSM mission manager for the pick_1 preparation step.

Default mode is dry-run. Add --run only when the robot is on the field and the
arm has enough clearance to move.
"""

import argparse

from arm_stage_calibrator import (
    ArmStageController,
    ArmStageLogger,
    get_stage_angles,
)
from config import DEFAULT_FORWARD_SPEED
from navigation_node import NavigationNode
from robot_driver import RobotDriver


class MissionManager:
    def __init__(
        self,
        navigation,
        arm,
        arm_logger,
        arm_run_time_ms,
        arm_settle_s,
    ):
        self.navigation = navigation
        self.arm = arm
        self.arm_logger = arm_logger
        self.arm_run_time_ms = arm_run_time_ms
        self.arm_settle_s = arm_settle_s
        self.stage_angles = get_stage_angles()

    def set_arm_stage(self, stage):
        angles = self.stage_angles[stage]
        print(f"[FSM] ARM_{stage.upper()}")
        self.arm.move(stage, angles, self.arm_run_time_ms, self.arm_settle_s)
        self.arm_logger.write(stage, angles, self.arm_run_time_ms, self.arm.dry_run)

    def prepare_pick_1(self, start, speed, skip_arm_default=False):
        print("[FSM] START_PREPARE_PICK_1")

        if not skip_arm_default:
            self.set_arm_stage("default")

        print(f"[FSM] MOVE_{start.upper()}_TO_PICK_1")
        self.navigation.move_between(start, "pick_1", speed=speed)

        # Important: only move the arm into pick_1 after navigation confirms
        # the robot has arrived at the pick_1 waypoint.
        self.set_arm_stage("pick_1")

        print("[FSM] PICK_1_ARM_READY")


def main():
    parser = argparse.ArgumentParser(
        description="Move to pick_1, then set the arm to pick_1 state."
    )
    parser.add_argument("--start", default="home", help="Current waypoint.")
    parser.add_argument("--speed", type=float, default=DEFAULT_FORWARD_SPEED)
    parser.add_argument("--debug", action="store_true", help="Print wheel sync data.")
    parser.add_argument(
        "--skip-arm-default",
        action="store_true",
        help="Do not move the arm to default before navigation.",
    )
    parser.add_argument(
        "--arm-run-time-ms",
        type=int,
        default=10000,
        help="Arm servo movement time in milliseconds. Larger value moves slower.",
    )
    parser.add_argument(
        "--arm-settle-s",
        type=float,
        default=1.0,
        help="Seconds to wait after each arm command.",
    )
    parser.add_argument(
        "--arm-log",
        default="arm_stage_calibration.log",
        help="CSV log output path for arm stages.",
    )
    parser.add_argument(
        "--run",
        action="store_true",
        help="Move the real robot and arm. Without this flag this is dry-run.",
    )
    args = parser.parse_args()

    if args.arm_run_time_ms <= 0:
        parser.error("--arm-run-time-ms must be positive")
    if args.arm_settle_s < 0:
        parser.error("--arm-settle-s must be zero or positive")

    driver = RobotDriver(dry_run=not args.run, debug=args.debug)
    navigation = NavigationNode(driver=driver)
    arm = ArmStageController(dry_run=not args.run)
    arm_logger = ArmStageLogger(args.arm_log)

    try:
        mission = MissionManager(
            navigation=navigation,
            arm=arm,
            arm_logger=arm_logger,
            arm_run_time_ms=args.arm_run_time_ms,
            arm_settle_s=args.arm_settle_s,
        )
        mission.prepare_pick_1(
            start=args.start,
            speed=args.speed,
            skip_arm_default=args.skip_arm_default,
        )
    finally:
        arm_logger.close()
        print(f"[INFO] Arm log saved to {args.arm_log}")


if __name__ == "__main__":
    main()
