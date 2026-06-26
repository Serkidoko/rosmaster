"""Configuration for fixed-waypoint robot movement.

Distances below are initial placeholders from simple_map.png. Measure and tune
them on the real field before running a full route.
"""

PULSES_PER_REV = 1850
WHEEL_DIAMETER_M = 0.065
CALIBRATION_FACTOR = 1.0
READ_INTERVAL_S = 0.05

YAW_KP = 4.0

# Encoder synchronization: keep all 4 wheel distances equal while moving
# straight. Units are motor command per meter of wheel-distance error.
WHEEL_SYNC_KP = 160.0
WHEEL_SYNC_KI = 8.0
WHEEL_SYNC_MAX_CORRECTION = 10.0
YAW_MAX_CORRECTION = 6.0
WHEEL_SYNC_WARN_ERROR_M = 0.015
STOP_ON_SLOWEST_WHEEL = False
FORWARD_MIN_MOTOR_COMMAND = 0.0
MAX_FORWARD_SEGMENT_M = 0.50
SEGMENT_PAUSE_S = 0.2
REALIGN_AFTER_SEGMENT = False
REALIGN_YAW_TOLERANCE_DEG = 2.0
REALIGN_TURN_SPEED = 0.18
REALIGN_TIMEOUT_S = 4.0
FORWARD_MIN_TIMEOUT_S = 4.0
FORWARD_TIMEOUT_PER_M = 30.0
FORWARD_DISTANCE_TOLERANCE_M = 0.02
ENABLE_STALL_DETECTION = False
STALL_PROGRESS_EPS_M = 0.003
STALL_TIMEOUT_S = 1.2
SATURATION_MARGIN = 0.5

MOTION_KP = 1.2
MOTION_KI = 0.1
MOTION_KD = 0.1

# Assumed motor order for Rosmaster set_motor(m1, m2, m3, m4):
# left-front, left-rear, right-front, right-rear.
MOTOR_NAMES = ("left_front", "left_rear", "right_front", "right_rear")
MOTOR_CALIBRATION = (1.043, 1.000, 1.010, 1.006)

# Separate lateral tuning. Keep this independent from MOTOR_CALIBRATION so
# left/right strafing can be tuned without changing straight-line motion.
LATERAL_LEFT_MOTOR_CALIBRATION = (1.250, 0.930, 1.220, 0.900)
LATERAL_RIGHT_MOTOR_CALIBRATION = (1.250, 0.930, 1.220, 0.900)
LATERAL_LEFT_PROFILE = (-1, 1, 1, -1)
LATERAL_RIGHT_PROFILE = (1, -1, -1, 1)
LATERAL_BASE_MOTOR_COMMAND = 36
LATERAL_REFERENCE_SPEED_MPS = 0.16
LATERAL_WHEEL_SYNC_KP = 160.0
LATERAL_WHEEL_SYNC_KI = 8.0
LATERAL_WHEEL_SYNC_MAX_CORRECTION = 10.0
LATERAL_YAW_KP = 4.0
LATERAL_YAW_MAX_CORRECTION = 12.0
LATERAL_ABORT_YAW_ERROR_DEG = 10.0
LATERAL_TIMEOUT_SCALE = 3.0
LATERAL_REALIGN_AFTER_MOVE = True
MAX_LATERAL_SEGMENT_M = 0.10
LATERAL_SEGMENT_PAUSE_S = 0.15

DEFAULT_FORWARD_SPEED = 30
MAX_MOTOR_COMMAND = 42
DEFAULT_ALIGN_LATERAL_SPEED_MPS = 0.16
MIN_ALIGN_SIDE_SPEED_MPS = 0.10
DEFAULT_TURN_SPEED = 0.24
TURN_TOLERANCE_DEG = 2.0

# Movement flow from simple_map.png:
# home -> pick_1 -> drop_1 -> pick_2 -> drop_2.
#
# Replace the placeholder distances with measured values in meters.
ROUTES = {
    ("home", "pick_1"): (
        ("forward", 1.8),
    ),
    ("pick_1", "drop_1"): (
        ("turn_right", 90.0),
        ("forward", 0.80),
    ),
    ("drop_1", "pick_2"): (
        ("turn_right", 90.0),
        ("forward", 0.35),
    ),
    ("pick_2", "drop_2"): (
        ("turn_right", 45.0),
        ("forward", 0.80),
    ),
}

FULL_MOVE_FLOW = ("home", "pick_1", "drop_1", "pick_2", "drop_2")
