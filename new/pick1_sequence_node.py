"""FSM for pick_1: arm prep, visual bbox check, then grasp.

Default mode is dry-run for robot/arm movement. Use --image for an offline
vision check, or camera options for the real pick_1 setup.
"""

import argparse

from arm_stage_calibrator import ArmStageController, ArmStageLogger, get_stage_angles
from config import (
    DEFAULT_ALIGN_LATERAL_SPEED_MPS,
    DEFAULT_FORWARD_SPEED,
    MIN_ALIGN_SIDE_SPEED_MPS,
)
from robot_driver import RobotDriver
from vision_color_node import COLOR_RANGES
from visual_approach_node import (
    DEFAULT_GRIPPER_BOX,
    PICK1_LEFT_ROI,
    disable_show_if_headless,
    load_gripper_config,
    run_camera,
    run_image,
)


PICK_AFTER_READY_STAGES = ("picking_1", "grasp_1", "picked_1", "hold")
VISUAL_RESULT_MESSAGES = {
    0: "ready",
    2: "not_found",
    3: "not_aligned",
    4: "not_ready_or_timeout",
    130: "interrupted",
}


class Pick1Sequence:
    def __init__(self, arm, arm_logger, arm_run_time_ms, arm_settle_s):
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

    def run_pick_1(self, args, driver):
        print("[FSM] START_PICK_1_SEQUENCE")

        self.set_arm_stage("default")
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

        print("[FSM] PICK_1_SEQUENCE_DONE")
        return 0


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Run pick_1 FSM: default -> pick_1 -> slot color scan -> "
            "bbox approach -> picking_1 -> grasp_1 -> picked_1 -> hold."
        )
    )
    parser.add_argument("--image", help="Use an image file instead of the camera.")
    parser.add_argument("--camera-id", type=int, default=0, help="Camera index.")
    parser.add_argument("--warmup-frames", type=int, default=10)
    parser.add_argument("--color", choices=sorted(COLOR_RANGES))
    parser.add_argument("--min-area", type=float, default=1200)
    parser.add_argument(
        "--roi",
        nargs=4,
        type=float,
        default=PICK1_LEFT_ROI,
        metavar=("X", "Y", "W", "H"),
        help="Normalized object search ROI. Default is the left-side pick_1 tray area.",
    )
    parser.add_argument(
        "--gripper-box",
        nargs=4,
        type=float,
        default=DEFAULT_GRIPPER_BOX,
        metavar=("X", "Y", "W", "H"),
        help="Normalized fixed gripper target box in image.",
    )
    parser.add_argument(
        "--gripper-config",
        default="pick1_gripper_box.json",
        help="JSON output from calibrate_pick_bbox.py.",
    )
    parser.add_argument("--ready-min-area", type=float, default=18000.0)
    parser.add_argument("--ready-min-height", type=int, default=100)
    parser.add_argument("--center-tolerance-ratio", type=float, default=0.08)
    parser.add_argument(
        "--max-approach-s",
        type=float,
        default=12.0,
        help="Safety timeout for visual approach; robot stops when bbox is ready.",
    )
    parser.add_argument(
        "--approach-control-s",
        type=float,
        default=0.15,
        help="Seconds between visual approach velocity updates.",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=4,
        help="Deprecated; visual approach now stops by ready bbox or timeout.",
    )
    parser.add_argument(
        "--step-distance-m",
        type=float,
        default=0.05,
        help="Deprecated; visual approach now uses velocity until bbox is ready.",
    )
    parser.add_argument("--speed", type=float, default=DEFAULT_FORWARD_SPEED)
    parser.add_argument("--approach-forward-speed-mps", type=float, default=0.12)
    parser.add_argument(
        "--arm-k1-angle",
        type=int,
        default=0,
        help="Arm joint 1 angle fallback: 0=robot-left, 90=robot-forward, 180=robot-right.",
    )
    parser.add_argument("--lateral-speed-mps", type=float, default=0.22)
    parser.add_argument(
        "--align-lateral-speed-mps",
        type=float,
        default=DEFAULT_ALIGN_LATERAL_SPEED_MPS,
    )
    parser.add_argument(
        "--min-align-side-speed-mps",
        type=float,
        default=MIN_ALIGN_SIDE_SPEED_MPS,
    )
    parser.add_argument("--invert-lateral", action="store_true")
    parser.add_argument(
        "--disable-auto-invert-lateral",
        action="store_true",
        help="Deprecated; arm-relative vector control does not auto-invert.",
    )
    parser.add_argument("--align-progress-px", type=int, default=8)
    parser.add_argument("--align-no-progress-frames", type=int, default=6)
    parser.add_argument("--auto-invert-min-dx", type=int, default=120)
    parser.add_argument("--lost-frame-grace", type=int, default=4)
    parser.add_argument(
        "--scan-colors",
        nargs="+",
        choices=sorted(COLOR_RANGES),
        default=("red", "green", "yellow"),
        help="Colors to inspect while scanning the 3-object row.",
    )
    parser.add_argument(
        "--max-scan-objects",
        type=int,
        default=3,
        help="Maximum wrong-color slots to inspect before failing.",
    )
    parser.add_argument(
        "--arm-right-scan-speed-mps",
        type=float,
        default=0.08,
        help="Arm-right speed used to move from one object slot to the next.",
    )
    parser.add_argument(
        "--slot-center-tolerance-ratio",
        type=float,
        default=0.12,
        help="Normalized tolerance for deciding which object is in the current slot.",
    )
    parser.add_argument(
        "--slot-stable-frames",
        type=int,
        default=4,
        help="Consecutive wrong-color frames required before counting one object slot.",
    )
    parser.add_argument(
        "--scan-timeout-s",
        type=float,
        default=10.0,
        help="Safety timeout for wrong-color row scanning.",
    )
    parser.add_argument(
        "--search-direction",
        choices=("forward", "backward", "left", "right", "none"),
        default="forward",
        help="Cardinal arm-frame direction to move when the object is not found.",
    )
    parser.add_argument(
        "--search-speed-mps",
        type=float,
        default=0.08,
        help="Arm-forward/backward search speed when no object is visible.",
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=0.4,
        help="Seconds to wait after each bbox approach movement.",
    )
    parser.add_argument("--save-debug", help="Path to save annotated debug image.")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--pause-on-step", action="store_true")
    parser.add_argument("--preview-ms", type=int, default=1)
    parser.add_argument("--display-scale", type=float, default=0.6)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--arm-run-time-ms", type=int, default=10000)
    parser.add_argument("--arm-settle-s", type=float, default=1.0)
    parser.add_argument(
        "--arm-log",
        default="arm_stage_calibration.log",
        help="CSV log output path for arm stages.",
    )
    parser.add_argument(
        "--run",
        action="store_true",
        help="Move the real robot and arm. Without this flag movement is dry-run.",
    )
    parser.set_defaults(
        continuous=False,
        print_every=10,
        continuous_delay_s=0.05,
    )
    return parser


def validate_args(parser, args):
    if args.color is None:
        parser.error("--color is required when --gripper-config has no color")
    if args.min_area <= 0:
        parser.error("--min-area must be positive")
    if args.ready_min_area <= 0:
        parser.error("--ready-min-area must be positive")
    if args.ready_min_height <= 0:
        parser.error("--ready-min-height must be positive")
    if args.center_tolerance_ratio <= 0:
        parser.error("--center-tolerance-ratio must be positive")
    if args.max_approach_s <= 0:
        parser.error("--max-approach-s must be positive")
    if args.approach_control_s <= 0:
        parser.error("--approach-control-s must be positive")
    if args.max_steps < 0:
        parser.error("--max-steps must be zero or positive")
    if args.step_distance_m <= 0:
        parser.error("--step-distance-m must be positive")
    if args.approach_forward_speed_mps <= 0:
        parser.error("--approach-forward-speed-mps must be positive")
    if args.arm_k1_angle < 0 or args.arm_k1_angle > 180:
        parser.error("--arm-k1-angle must be in range 0..180")
    if args.lateral_speed_mps <= 0:
        parser.error("--lateral-speed-mps must be positive")
    if args.align_lateral_speed_mps <= 0:
        parser.error("--align-lateral-speed-mps must be positive")
    if args.min_align_side_speed_mps <= 0:
        parser.error("--min-align-side-speed-mps must be positive")
    if args.align_progress_px < 0:
        parser.error("--align-progress-px must be zero or positive")
    if args.align_no_progress_frames <= 0:
        parser.error("--align-no-progress-frames must be positive")
    if args.auto_invert_min_dx < 0:
        parser.error("--auto-invert-min-dx must be zero or positive")
    if args.lost_frame_grace < 0:
        parser.error("--lost-frame-grace must be zero or positive")
    if args.max_scan_objects <= 0:
        parser.error("--max-scan-objects must be positive")
    if args.arm_right_scan_speed_mps <= 0:
        parser.error("--arm-right-scan-speed-mps must be positive")
    if args.slot_center_tolerance_ratio <= 0:
        parser.error("--slot-center-tolerance-ratio must be positive")
    if args.slot_stable_frames <= 0:
        parser.error("--slot-stable-frames must be positive")
    if args.scan_timeout_s <= 0:
        parser.error("--scan-timeout-s must be positive")
    if args.search_speed_mps <= 0:
        parser.error("--search-speed-mps must be positive")
    if args.settle_s < 0:
        parser.error("--settle-s must be zero or positive")
    if args.preview_ms < 1:
        parser.error("--preview-ms must be at least 1")
    if args.display_scale <= 0:
        parser.error("--display-scale must be positive")
    if args.warmup_frames < 0:
        parser.error("--warmup-frames must be zero or positive")
    if args.arm_run_time_ms <= 0:
        parser.error("--arm-run-time-ms must be positive")
    if args.arm_settle_s < 0:
        parser.error("--arm-settle-s must be zero or positive")


def main():
    parser = build_parser()
    args = parser.parse_args()
    load_gripper_config(args)
    disable_show_if_headless(args)
    validate_args(parser, args)

    driver = None
    if not args.image:
        driver = RobotDriver(dry_run=not args.run, debug=args.debug)

    shared_bot = driver.bot if driver is not None and not driver.dry_run else None
    arm = ArmStageController(dry_run=not args.run, bot=shared_bot)
    arm_logger = ArmStageLogger(args.arm_log)

    try:
        sequence = Pick1Sequence(
            arm=arm,
            arm_logger=arm_logger,
            arm_run_time_ms=args.arm_run_time_ms,
            arm_settle_s=args.arm_settle_s,
        )
        return sequence.run_pick_1(args, driver)
    finally:
        arm_logger.close()
        print(f"[INFO] Arm log saved to {args.arm_log}")


if __name__ == "__main__":
    raise SystemExit(main())
