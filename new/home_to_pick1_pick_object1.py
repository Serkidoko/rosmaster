"""FSM from home to pick_1, then pick object 1 by bbox.

Default mode is dry-run. Add --run only when the robot is on the field, the arm
has clearance, and the color-specific gripper config has been checked.
"""

from arm_stage_calibrator import ArmStageController, ArmStageLogger, get_stage_angles
from navigation_node import NavigationNode
from pick1_sequence_node import (
    PICK_AFTER_READY_STAGES,
    VISUAL_RESULT_MESSAGES,
    build_parser as build_pick1_parser,
    validate_args,
)
from robot_driver import RobotDriver
from visual_approach_node import (
    disable_show_if_headless,
    load_gripper_config,
    run_camera,
    run_image,
)


class HomeToPick1Object1:
    def __init__(self, navigation, arm, arm_logger, arm_run_time_ms, arm_settle_s):
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

    def wait_until_object_ready(self, args, driver):
        print("[FSM] DETECT_OBJECT_1_BY_SLOT_SCAN_AND_4_ZONE_BBOX")
        if args.image:
            return run_image(args)
        return run_camera(args, driver=driver)

    def run(self, args, driver):
        print("[FSM] START_HOME_TO_PICK_1_OBJECT_1")
        self.set_arm_stage("default")

        if args.skip_navigation:
            print("[FSM] SKIP_NAVIGATION_TO_PICK_1")
        else:
            print(f"[FSM] MOVE_{args.start.upper()}_TO_PICK_1")
            self.navigation.move_between(args.start, "pick_1", speed=args.speed)

        # Only enter pick_1 arm pose after navigation has arrived at pick_1.
        self.set_arm_stage("pick_1")
        args.arm_k1_angle = self.stage_angles["pick_1"][0]
        print(f"[FSM] ARM_STATUS k1={args.arm_k1_angle}")

        visual_result = self.wait_until_object_ready(args, driver)
        if visual_result != 0:
            message = VISUAL_RESULT_MESSAGES.get(visual_result, "unknown_error")
            print(f"[FSM] STOP_BEFORE_PICKING_1 visual_result={message}")
            return visual_result

        print("[FSM] OBJECT_READY_FOR_PICKING_1")
        for stage in PICK_AFTER_READY_STAGES:
            self.set_arm_stage(stage)

        print("[FSM] HOME_TO_PICK_1_OBJECT_1_DONE")
        return 0


def build_parser():
    parser = build_pick1_parser()
    parser.description = (
        "Run FSM: home -> pick_1 -> arm pick_1 -> slot color scan -> "
        "cardinal bbox approach -> picking_1 -> grasp_1 -> picked_1 -> hold."
    )
    parser.add_argument("--start", default="home", help="Current waypoint.")
    parser.add_argument(
        "--skip-navigation",
        action="store_true",
        help="Skip home->pick_1 movement; useful when robot is already at pick_1.",
    )
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    load_gripper_config(args)
    disable_show_if_headless(args)
    validate_args(parser, args)

    driver = RobotDriver(dry_run=not args.run, debug=args.debug)
    navigation = NavigationNode(driver=driver)
    shared_bot = driver.bot if not driver.dry_run else None
    arm = ArmStageController(dry_run=not args.run, bot=shared_bot)
    arm_logger = ArmStageLogger(args.arm_log)

    try:
        mission = HomeToPick1Object1(
            navigation=navigation,
            arm=arm,
            arm_logger=arm_logger,
            arm_run_time_ms=args.arm_run_time_ms,
            arm_settle_s=args.arm_settle_s,
        )
        return mission.run(args, driver)
    finally:
        arm_logger.close()
        print(f"[INFO] Arm log saved to {args.arm_log}")


if __name__ == "__main__":
    raise SystemExit(main())
