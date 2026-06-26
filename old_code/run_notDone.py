import time
import cv2
import math
import numpy as np
from Rosmaster_Lib import Rosmaster


PULSES_PER_REV = 2300          # measured encoder pulses per revolution
WHEEL_DIAMETER = 0.065         # meters (e.g., 65 mm)
CALIBRATION_FACTOR = 1.0       # overall distance calibration factor
TARGET_DISTANCE = 3.0          # meters to travel
BASE_SPEED = 37                # nominal speed command (0–100 per SDK)
READ_INTERVAL = 0.05           # seconds between encoder reads
YAW_KP = 0.5                   # Proportional gain for yaw correction (tune as needed)
STEER_KP = 5.0                 # P gain for steering correction (reduced)
STEER_KI = 0.5                 # I gain for steering correction (reduced)
MOTION_KP = 1.2                # increased P for better velocity tracking
MOTION_KI = 0.1                # slightly increased I
MOTION_KD = 0.1                # small D term to damp oscillations
MOTOR_CALIBRATION = [0.94, 0.94, 1.0, 0.96]
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * math.pi * CALIBRATION_FACTOR
grip_angles = (180, 0, 60, 120, 90, 42)
hold_angles = (180, 90, 0, 90, 90, 128)


def ticks_to_meters(ticks):
    """Convert encoder ticks to linear distance in meters."""
    revolutions = ticks / PULSES_PER_REV
    return revolutions * WHEEL_CIRCUMFERENCE

def init_arm_servo():
    bot.set_uart_servo_angle_array([167, 180, 0 , 0, 90, 42], run_time=8000)
    return

def arm_servo(s1, s2, s3, s4, s5, s6):
    bot.set_uart_servo_angle_array([s1, s2, s3, s4, s5, s6], run_time=8000)
    return s1, s2, s3, s4, s5, s6

def gripe_right_side_box():
    print("Setting arm servo angles to:", grip_angles)
    bot.set_uart_servo_angle(1, 180, run_time=8000)
    time.sleep(1)
    arm_servo(*grip_angles)
    bot.set_uart_servo_torque(True)
    time.sleep(2.4)
    bot.set_uart_servo_angle(6, 128)
    time.sleep(1)
    print("Setting arm servo angles to:", hold_angles)
    bot.set_uart_servo_angle(2, 60, run_time=8000)
    time.sleep(1)
    arm_servo(*hold_angles)
    time.sleep(1)

def run_motor_straight(dist, angle):
    bot.set_pid_param(MOTION_KP, MOTION_KI, MOTION_KD, forever=False)
    kp, ki, kd = bot.get_motion_pid()
    print(f"[INFO] Motor PID set to Kp={kp}, Ki={ki}, Kd={kd}")

    start_ticks = list(bot.get_motor_encoder())
    print(f"[INFO] Initial ticks: {start_ticks}")

    error_int = [0.0] * 4
    traveled = 0.0
    print(f"[INFO] Moving {dist:.2f} m at base speed {BASE_SPEED}")

    try:
        while traveled < dist:
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
            roll, pitch, yaw = bot.get_imu_attitude_data()
            yaw_error = yaw - angle
            yaw_correction = YAW_KP * yaw_error

            # Apply yaw correction: subtract from left wheels, add to right wheels
            speeds = []
            for i in range(4):
                if i in [0, 1]:  # Left wheels (LF, LR)
                    spd_cmd = (BASE_SPEED - corrections[i] - yaw_correction) * MOTOR_CALIBRATION[i]
                else:            # Right wheels (RF, RR)
                    spd_cmd = (BASE_SPEED - corrections[i] + yaw_correction) * MOTOR_CALIBRATION[i]
                speeds.append(max(0, min(100, spd_cmd)))

            # send speed commands; set_motor uses velocity PID internally
            bot.set_motor(speeds[0], speeds[1], speeds[2], speeds[3])

            time.sleep(READ_INTERVAL)
        
            # debug info
            print(f"[DEBUG] Dists: {[f'{d:.3f}' for d in distances]}, Avg: {traveled:.3f}")
            print(f"        Speeds: {[f'{s:.1f}' for s in speeds]}, Errors: {[f'{e:.3f}' for e in errors]}")
            print(f"        IMU: [{roll:.1f}, {pitch:.1f}, {yaw:.1f}]")

    except KeyboardInterrupt:
        print("[INFO] Interrupted by user.")
    finally:
        bot.set_motor(0, 0, 0, 0)
        print(f"[INFO] Stopped at {traveled:.3f} m.")
        print(f"Voltage remain: {bot.get_battery_voltage()}")

def run_motor_rotate_left(angular_speed=0.24):
    target_yaw = 90
    bot.set_car_motion(0, 0, angular_speed)
    _, _, current_yaw = bot.get_imu_attitude_data()

    while current_yaw - target_yaw < 0:
        _, _, current_yaw = bot.get_imu_attitude_data()
        # print(f"Current yaw: {current_yaw:.2f}")
        time.sleep(0.05)

    bot.set_car_motion(0, 0, 0)
    print("Rotation complete.")
    print(f"Voltage remain: {bot.get_battery_voltage()}")

if __name__ == '__main__':
    bot = Rosmaster()
    bot.clear_auto_report_data()
    bot.create_receive_threading()
    _, _, init_yaw = bot.get_imu_attitude_data()
    print('Initial Yaw:', init_yaw)
    time.sleep(3.6)

    init_arm_servo()
    time.sleep(0.5)
    run_motor_straight(dist=3.0, angle=init_yaw)
    time.sleep(1)
    # run_motor_rotate_left(angular_speed=0.24)
    # time.sleep(1)
    # run_motor_straight(dist=0.05, angle=90)
    # time.sleep(1)
    # gripe_right_side_box()
    # time.sleep(1)