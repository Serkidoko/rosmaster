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
WHEEL_SYNC_KP = 120.0
WHEEL_SYNC_KI = 5.0
WHEEL_SYNC_MAX_CORRECTION = 8.0
YAW_MAX_CORRECTION = 8.0
WHEEL_SYNC_WARN_ERROR_M = 0.015
STOP_ON_SLOWEST_WHEEL = False
FORWARD_MIN_MOTOR_COMMAND = 18.0
MAX_FORWARD_SEGMENT_M = 0.10
SEGMENT_PAUSE_S = 0.4
REALIGN_AFTER_SEGMENT = True
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
# Temporary hardware compensation: M3/right_front is weak.
# On-ground test showed 0.31 over-boosts M3 at startup, so use a softer
# feed-forward ratio and let encoder sync add correction when needed.
M3_FORWARD_RATIO = 0.45
M3_BASE_CALIBRATION = 1.017
MOTOR_CALIBRATION = (
    0.990,
    0.995,
    M3_BASE_CALIBRATION / M3_FORWARD_RATIO,
    0.992,
)

DEFAULT_FORWARD_SPEED = 22
MAX_MOTOR_COMMAND = 42
DEFAULT_TURN_SPEED = 0.24
TURN_TOLERANCE_DEG = 2.0

# Movement flow from simple_map.png:
# home -> pick_1 -> drop_1 -> pick_2 -> drop_2.
#
# Replace the placeholder distances with measured values in meters.
ROUTES = {
    ("home", "pick_1"): (
        ("forward", 0.50),
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
