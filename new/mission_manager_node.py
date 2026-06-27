"""Run detect -> align object into pick bbox -> pick."""

import argparse

import cv2

from config import CAMERA_ID, FRAME_HEIGHT, FRAME_WIDTH, HSV_RANGES
from gripper_node import GripperNode
from navigation_node import NavigationNode
from robot_driver import RobotDriver
from vision_color_node import ColorObjectDetector


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Align colored cube and pick it")
    parser.add_argument("--color", required=True, choices=sorted(HSV_RANGES))
    parser.add_argument("--camera", type=int, default=CAMERA_ID)
    parser.add_argument("--run", action="store_true", help="Move real robot. Default is dry-run.")
    parser.add_argument("--align-only", action="store_true", help="Do not run the gripper pick sequence.")
    return parser


def main():
    args = build_arg_parser().parse_args()
    driver = RobotDriver(dry_run=not args.run)
    detector = ColorObjectDetector()
    navigation = NavigationNode(driver=driver, detector=detector)
    gripper = GripperNode(driver=driver)

    camera = cv2.VideoCapture(args.camera)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if not camera.isOpened():
        raise SystemExit(f"cannot open camera {args.camera}")

    try:
        gripper.home()
        result = navigation.align_object_to_bbox(camera, target_color=args.color)
        print(f"[RESULT] {result}")
        if not result.done:
            return 1

        if not args.align_only:
            gripper.pick()
        return 0
    finally:
        camera.release()
        driver.stop()


if __name__ == "__main__":
    raise SystemExit(main())
