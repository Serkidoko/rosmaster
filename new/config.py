"""Small config for color-object pick alignment."""

CAMERA_ID = 2
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Only search where the cube appears in the current camera setup.
DETECT_ROI = (120, 170, 520, 430)  # x1, y1, x2, y2

# The robot will move until the detected object center is inside this box.
# Tune this box to the arm's real pick reach.
PICK_TARGET_BBOX = (300, 250, 350, 325)  # x1, y1, x2, y2

# HSV values measured from pick1_manual_red/green/yellow.jpg.
HSV_RANGES = {
    "red": [
        ((8, 50, 150), (25, 210, 255)),
        ((170, 70, 50), (180, 255, 255)),
    ],
    "green": [((55, 25, 130), (90, 145, 255))],
    "yellow": [((25, 45, 180), (38, 190, 255))],
}

MIN_CONTOUR_AREA = 3500
MIN_BOX_W = 55
MAX_BOX_W = 220
MIN_BOX_H = 50
MAX_BOX_H = 180
MIN_BOX_RATIO = 0.70
MAX_BOX_RATIO = 1.80
MIN_FILL_RATIO = 0.55
AMBIGUOUS_SCORE_RATIO = 1.30

ALIGN_MAX_STEPS = 25
ALIGN_STEP_SECONDS = 0.15
ALIGN_FORWARD_SPEED = 0.06
ALIGN_STRAFE_SPEED = 0.06

# Flip these to -1 if the real robot moves opposite to the image correction.
FORWARD_SIGN = 1.0
STRAFE_LEFT_SIGN = 1.0

ARM_HOME_ANGLES = (167, 180, 0, 0, 90, 42)
ARM_GRIP_ANGLES = (180, 0, 60, 120, 90, 42)
ARM_HOLD_ANGLES = (180, 90, 0, 90, 90, 128)
GRIPPER_OPEN_ANGLE = 37
GRIPPER_CLOSE_ANGLE = 128
