import time
import math
from Rosmaster_Lib import Rosmaster

# === USER CONFIGURATION ===
PULSES_PER_REV = 1800          # measured encoder pulses per revolution
WHEEL_DIAMETER = 0.065         # meters (e.g., 65 mm)
CALIBRATION_FACTOR = 1.0       # overall distance calibration factor
TARGET_DISTANCE = 1.0          # meters to travel
BASE_SPEED = 37                # nominal speed command (0–100 per SDK)
READ_INTERVAL = 0.05           # seconds between encoder reads
YAW_KP = 10.0                  # Proportional gain for yaw correction (tune as needed)
YAW_TARGET = 0.0               # Target yaw in degrees

# Steering PI gains tuned to reduce drift
STEER_KP = 5.0                 # P gain for steering correction
STEER_KI = 0.5                 # I gain for steering correction

# Built-in motor velocity PID gains retuned for smoother stop
MOTION_KP = 1.2                 # increased P for better velocity tracking
MOTION_KI = 0.1                 # slightly increased I
MOTION_KD = 0.1                 # small D term to damp oscillations

# Per-wheel calibration multipliers updated from latest experiment
# Orders: [LF, LR, RF, RR]
# LF overshoots by ~5%, RF undershoots by ~4%, others near unity
MOTOR_CALIBRATION = [1.2, 0.95, 1.1, 0.9]    # when pin position is middle inside the body

# Precompute circumference
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * math.pi * CALIBRATION_FACTOR


def ticks_to_meters(ticks):
    """Convert encoder ticks to linear distance in meters."""
    revolutions = ticks / PULSES_PER_REV
    return revolutions * WHEEL_CIRCUMFERENCE


def main(init_yaw):
    # configure built-in motor velocity PID
    bot.set_pid_param(MOTION_KP, MOTION_KI, MOTION_KD, forever=False)
    kp, ki, kd = bot.get_motion_pid()
    print(f"[INFO] Motor PID set to Kp={kp}, Ki={ki}, Kd={kd}")

    # read initial encoder ticks for all 4 motors (M1=LF, M2=LR, M3=RF, M4=RR)
    start_ticks = list(bot.get_motor_encoder())
    print(f"[INFO] Initial ticks: {start_ticks}")

    error_int = [0.0] * 4
    traveled = 0.0
    debug_bool = 1.0
    print(f"[INFO] Moving {TARGET_DISTANCE:.2f} m at base speed {BASE_SPEED}")

    try:
        while traveled < TARGET_DISTANCE:
            enc = list(bot.get_motor_encoder())
            deltas = [enc[i] - start_ticks[i] for i in range(4)]
            distances = [ticks_to_meters(d) for d in deltas]
            traveled = sum(distances) / 4.0

            # steering errors and integral for each wheel
            errors = [d - traveled for d in distances]
            for i in range(4):
                error_int[i] += errors[i] * READ_INTERVAL

            # PI steering corrections
            corrections = [STEER_KP * errors[i] + STEER_KI * error_int[i] for i in range(4)]

            # get IMU data
            _, _, yaw = bot.get_imu_attitude_data()
            yaw_error = yaw - init_yaw
            yaw_correction = YAW_KP * yaw_error

            # Apply yaw correction: subtract from left wheels, add to right wheels
            speeds = []
            for i in range(4):
                if i in [0, 1]:  # Left wheels (LF, LR)
                    spd_cmd = (BASE_SPEED - corrections[i] + yaw_correction) * MOTOR_CALIBRATION[i]
                    # spd_cmd = (BASE_SPEED - corrections[i] - yaw_correction) * MOTOR_CALIBRATION[i]
                else:            # Right wheels (RF, RR)
                    spd_cmd = (BASE_SPEED - corrections[i] - yaw_correction) * MOTOR_CALIBRATION[i]
                    # spd_cmd = (BASE_SPEED - corrections[i] + yaw_correction) * MOTOR_CALIBRATION[i]
                speeds.append(max(0, min(100, spd_cmd)))

            # send speed commands; set_motor uses velocity PID internally
            bot.set_motor(speeds[0], speeds[1], speeds[2], speeds[3])

            # debug info
            if traveled >= debug_bool:
                debug_bool += 0.5
                print(f"[DEBUG] Dists: {[f'{d:.3f}' for d in distances]}, Avg: {traveled:.3f}")
                print(f"        Speeds: {[f'{s:.1f}' for s in speeds]}, Errors: {[f'{e:.3f}' for e in errors]}")
                print(f"        IMU Yaw: {yaw:.3f}, Error: {yaw_error:.3f}, Correction: {yaw_correction:.3f}")
            
            time.sleep(READ_INTERVAL)

    except KeyboardInterrupt:
        print("[INFO] Interrupted by user.")
    finally:
        bot.set_motor(0, 0, 0, 0)
        print(f"[INFO] Stopped at {traveled:.3f} m.")
        print(f"Voltage remain: {bot.get_battery_voltage()}")


if __name__ == '__main__':
    bot = Rosmaster()
    # start encoder report thread
    bot.clear_auto_report_data()
    bot.create_receive_threading()
    time.sleep(1)

    _, _, init_yaw = bot.get_imu_attitude_data()
    print('Initial Yaw:', init_yaw)
    main(init_yaw)