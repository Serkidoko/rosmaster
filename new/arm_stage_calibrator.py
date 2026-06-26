"""Move the robot arm through default and pick stages, with a CSV log.

Default mode is dry-run. Add --run only when the robot arm is clear to move.
"""

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path


DEFAULT_ANGLES = (90, 180, 0, 50, 90, 90)
PICK_ANGLES_1 = (0, 180, 0, 50, 90, 90)
PICK_ANGLES_2 = (90, 180, 0, 50, 90, 90)
PICKING_ANGLES_1 = (0, 90, 90, 65, 90, 30)
PICKING_ANGLES_2 = (90, 90, 90, 65, 90, 30)

GRIPPER_CLOSED_ANGLE = 132
STAGE_ORDER = ("default", "pick_1")
PICK_OBJECT_ORDER = ("default", "pick_1", "picking_1", "grasp_1", "picked_1", "hold")

# Angle tuple order:
#   s1 -> khop 1 / base yaw
#   s2 -> khop 2 / shoulder
#   s3 -> khop 3 / elbow
#   s4 -> khop 4 / wrist pitch
#   s5 -> khop 5 / wrist roll
#   s6 -> khop 6 / gripper
JOINT_LABELS = (
    "khop 1 / base yaw",
    "khop 2 / shoulder",
    "khop 3 / elbow",
    "khop 4 / wrist pitch",
    "khop 5 / wrist roll",
    "khop 6 / gripper",
)


def validate_angles(angles):
    if len(angles) != 6:
        raise ValueError("Expected 6 servo angles: khop 1 2 3 4 5 6")
    for index, angle in enumerate(angles, start=1):
        if angle < 0 or angle > 180:
            raise ValueError(f"Servo {index} angle must be in range 0..180")
    return tuple(int(angle) for angle in angles)


class ArmStageLogger:
    def __init__(self, log_path):
        self.log_path = Path(log_path)
        self.start_time = time.time()
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.log_path.open("a", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        if self.log_path.stat().st_size == 0:
            self.writer.writerow(
                [
                    "timestamp",
                    "elapsed_s",
                    "stage",
                    "khop_1",
                    "khop_2",
                    "khop_3",
                    "khop_4",
                    "khop_5",
                    "khop_6",
                    "run_time_ms",
                    "dry_run",
                ]
            )
            self.file.flush()

    def write(self, stage, angles, run_time_ms, dry_run):
        now = time.time()
        self.writer.writerow(
            [
                datetime.now().isoformat(timespec="seconds"),
                f"{now - self.start_time:.3f}",
                stage,
                *angles,
                run_time_ms,
                int(dry_run),
            ]
        )
        self.file.flush()

    def close(self):
        self.file.close()


class ArmStageController:
    def __init__(self, dry_run=True, bot=None):
        self.dry_run = dry_run
        self.bot = None
        if self.dry_run:
            print("[DRY-RUN] ArmStageController initialized without hardware.")
            return

        if bot is None:
            from Rosmaster_Lib import Rosmaster

            self.bot = Rosmaster()
            self.bot.create_receive_threading()
        else:
            self.bot = bot
        self.bot.set_uart_servo_torque(True)
        time.sleep(0.5)
        print("[INFO] Arm servo torque enabled.")

    def move(self, stage, angles, run_time_ms, settle_s):
        print(f"[INFO] Stage={stage} run_time_ms={run_time_ms}")
        for label, angle in zip(JOINT_LABELS, angles):
            print(f"       {label}: {angle} deg")
        if self.dry_run:
            print("[DRY-RUN] Skip hardware servo command.")
            time.sleep(settle_s)
        else:
            self.bot.set_uart_servo_angle_array(list(angles), run_time=run_time_ms)
            wait_s = run_time_ms / 1000.0 + settle_s
            print(f"[INFO] Waiting {wait_s:.1f}s for arm stage to settle.")
            time.sleep(wait_s)


def parse_stage_angles(raw_angles):
    if raw_angles is None:
        return None
    return validate_angles(raw_angles)


def with_gripper_angle(angles, gripper_angle):
    return tuple(angles[:5]) + (gripper_angle,)


def get_stage_angles(default_angles=None, pick_angles=None):
    default_angles = default_angles or DEFAULT_ANGLES
    pick_1_angles = pick_angles or PICK_ANGLES_1
    pick_2_angles = PICK_ANGLES_2
    picking_1_angles = PICKING_ANGLES_1
    picking_2_angles = PICKING_ANGLES_2
    return {
        "default": default_angles,
        "pick": pick_1_angles,
        "pick_1": pick_1_angles,
        "pick_2": pick_2_angles,
        "picking": picking_1_angles,
        "picking_1": picking_1_angles,
        "picking_2": picking_2_angles,
        "grasp": with_gripper_angle(picking_1_angles, GRIPPER_CLOSED_ANGLE),
        "grasp_1": with_gripper_angle(picking_1_angles, GRIPPER_CLOSED_ANGLE),
        "grasp_2": with_gripper_angle(picking_2_angles, GRIPPER_CLOSED_ANGLE),
        "picked_1": with_gripper_angle(pick_1_angles, GRIPPER_CLOSED_ANGLE),
        "picked_2": with_gripper_angle(pick_2_angles, GRIPPER_CLOSED_ANGLE),
        "hold": with_gripper_angle(default_angles, GRIPPER_CLOSED_ANGLE),
    }


def build_stage_map(args):
    default_angles = parse_stage_angles(args.default_angles)
    pick_angles = parse_stage_angles(args.pick_angles)
    return get_stage_angles(default_angles=default_angles, pick_angles=pick_angles)


def selected_stages(stage):
    if stage == "both":
        return STAGE_ORDER
    if stage == "pick_object":
        return PICK_OBJECT_ORDER
    return (stage,)


def main():
    parser = argparse.ArgumentParser(
        description="Move ROSMASTER X3 Plus arm to default and pick stages."
    )
    parser.add_argument(
        "--stage",
        choices=(
            "default",
            "pick",
            "pick_1",
            "pick_2",
            "picking",
            "picking_1",
            "picking_2",
            "grasp",
            "grasp_1",
            "grasp_2",
            "picked_1",
            "picked_2",
            "hold",
            "both",
            "pick_object",
        ),
        default="both",
        help=(
            "Arm stage to run. pick_object is an arm-only bench test and does "
            "not run camera detection. Use pick1_sequence_node.py for real pick_1."
        ),
    )
    parser.add_argument(
        "--default-angles",
        nargs=6,
        type=int,
        metavar=("K1", "K2", "K3", "K4", "K5", "K6"),
        help="Override default stage angles in khop 1..6 order.",
    )
    parser.add_argument(
        "--pick-angles",
        nargs=6,
        type=int,
        metavar=("K1", "K2", "K3", "K4", "K5", "K6"),
        help="Override pick stage angles in khop 1..6 order.",
    )
    parser.add_argument(
        "--run-time-ms",
        type=int,
        default=10000,
        help="Servo movement time in milliseconds. Larger value moves slower.",
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=1.0,
        help="Seconds to wait after each stage command.",
    )
    parser.add_argument(
        "--log",
        default="arm_stage_calibration.log",
        help="CSV log output path.",
    )
    parser.add_argument(
        "--run",
        action="store_true",
        help="Move the real arm. Without this flag the script only logs steps.",
    )
    args = parser.parse_args()

    if args.run_time_ms <= 0:
        parser.error("--run-time-ms must be positive")
    if args.settle_s < 0:
        parser.error("--settle-s must be zero or positive")

    stage_angles = build_stage_map(args)
    controller = ArmStageController(dry_run=not args.run)
    logger = ArmStageLogger(args.log)

    try:
        for stage in selected_stages(args.stage):
            angles = stage_angles[stage]
            controller.move(stage, angles, args.run_time_ms, args.settle_s)
            logger.write(stage, angles, args.run_time_ms, dry_run=not args.run)
    except KeyboardInterrupt:
        print("[INFO] Arm calibration interrupted.")
        raise
    finally:
        logger.close()
        print(f"[INFO] Log saved to {args.log}")


if __name__ == "__main__":
    main()
