"""Move the base until the detected object center enters the pick bbox."""

from dataclasses import dataclass

from config import (
    ALIGN_FORWARD_SPEED,
    ALIGN_MAX_STEPS,
    ALIGN_STEP_SECONDS,
    ALIGN_STRAFE_SPEED,
    FORWARD_SIGN,
    PICK_TARGET_BBOX,
    STRAFE_LEFT_SIGN,
)
from robot_driver import RobotDriver
from vision_color_node import ColorObjectDetector


@dataclass
class AlignResult:
    done: bool
    steps: int
    center: tuple[int, int] | None = None
    message: str = ""


class NavigationNode:
    def __init__(self, driver=None, detector=None):
        self.driver = driver or RobotDriver()
        self.detector = detector or ColorObjectDetector()

    def align_object_to_bbox(
        self,
        camera,
        target_color,
        target_bbox=PICK_TARGET_BBOX,
        max_steps=ALIGN_MAX_STEPS,
    ):
        x1, y1, x2, y2 = target_bbox
        last_center = None

        for step in range(1, max_steps + 1):
            ok, frame = camera.read()
            if not ok:
                self.driver.stop()
                return AlignResult(False, step, last_center, "camera_read_failed")

            detection = self.detector.detect(frame, target_color=target_color)
            if not detection.found:
                self.driver.stop()
                return AlignResult(False, step, last_center, detection.message)

            cx, cy = detection.center
            last_center = detection.center
            print(f"[ALIGN] step={step} color={detection.color} center={detection.center} bbox={detection.bbox}")

            if x1 <= cx <= x2 and y1 <= cy <= y2:
                self.driver.stop()
                return AlignResult(True, step, detection.center, "object_in_pick_bbox")

            if cx < x1:
                self.driver.move_for(
                    strafe_left_mps=ALIGN_STRAFE_SPEED * STRAFE_LEFT_SIGN,
                    seconds=ALIGN_STEP_SECONDS,
                )
            elif cx > x2:
                self.driver.move_for(
                    strafe_left_mps=-ALIGN_STRAFE_SPEED * STRAFE_LEFT_SIGN,
                    seconds=ALIGN_STEP_SECONDS,
                )
            elif cy < y1:
                self.driver.move_for(
                    forward_mps=ALIGN_FORWARD_SPEED * FORWARD_SIGN,
                    seconds=ALIGN_STEP_SECONDS,
                )
            else:
                self.driver.move_for(
                    forward_mps=-ALIGN_FORWARD_SPEED * FORWARD_SIGN,
                    seconds=ALIGN_STEP_SECONDS,
                )

        self.driver.stop()
        return AlignResult(False, max_steps, last_center, "alignment_timeout")
