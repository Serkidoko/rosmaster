"""Low-level movement wrapper for Rosmaster.

This module intentionally contains no mission or color-detection logic.
"""

import math
import time

from config import (
    CALIBRATION_FACTOR,
    DEFAULT_FORWARD_SPEED,
    ENABLE_STALL_DETECTION,
    FORWARD_MIN_MOTOR_COMMAND,
    DEFAULT_TURN_SPEED,
    MOTION_KD,
    MOTION_KI,
    MOTION_KP,
    MAX_MOTOR_COMMAND,
    MOTOR_CALIBRATION,
    PULSES_PER_REV,
    READ_INTERVAL_S,
    REALIGN_TIMEOUT_S,
    REALIGN_TURN_SPEED,
    REALIGN_YAW_TOLERANCE_DEG,
    FORWARD_MIN_TIMEOUT_S,
    FORWARD_TIMEOUT_PER_M,
    MOTOR_NAMES,
    STOP_ON_SLOWEST_WHEEL,
    SATURATION_MARGIN,
    STALL_PROGRESS_EPS_M,
    STALL_TIMEOUT_S,
    TURN_TOLERANCE_DEG,
    WHEEL_DIAMETER_M,
    WHEEL_SYNC_KI,
    WHEEL_SYNC_KP,
    WHEEL_SYNC_MAX_CORRECTION,
    WHEEL_SYNC_WARN_ERROR_M,
    YAW_KP,
    YAW_MAX_CORRECTION,
)


WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * math.pi * CALIBRATION_FACTOR


def normalize_angle(angle_deg):
    """Normalize angle to (-180, 180]."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def ticks_to_meters(ticks):
    return ticks / PULSES_PER_REV * WHEEL_CIRCUMFERENCE_M


def clamp(value, low, high):
    return max(low, min(high, value))


class RobotDriver:
    def __init__(self, dry_run=False, debug=False):
        self.dry_run = dry_run
        self.debug = debug
        self.bot = None

        if self.dry_run:
            self.initial_yaw = 0.0
            print("[DRY-RUN] RobotDriver initialized without hardware.")
            return

        from Rosmaster_Lib import Rosmaster

        self.bot = Rosmaster()
        self.bot.clear_auto_report_data()
        self.bot.create_receive_threading()
        time.sleep(1.0)
        self.initial_yaw = self.get_yaw()
        print(f"[INFO] Initial yaw: {self.initial_yaw:.2f}")
        print(f"[INFO] Battery voltage: {self.bot.get_battery_voltage()}")

    def get_yaw(self):
        _, _, yaw = self.bot.get_imu_attitude_data()
        return normalize_angle(yaw)

    def stop(self):
        if self.dry_run:
            print("[DRY-RUN] stop()")
            return
        self.bot.set_motor(0, 0, 0, 0)
        self.bot.set_car_motion(0, 0, 0)

    def _wheel_distances_from(self, start_ticks):
        enc = list(self.bot.get_motor_encoder())
        deltas = [enc[i] - start_ticks[i] for i in range(4)]
        return [ticks_to_meters(delta) for delta in deltas]

    def go_forward(self, distance_m, speed=DEFAULT_FORWARD_SPEED):
        if distance_m <= 0:
            raise ValueError("distance_m must be positive")

        if self.dry_run:
            print(f"[DRY-RUN] forward {distance_m:.2f} m at speed {speed}")
            return distance_m

        self.bot.set_pid_param(MOTION_KP, MOTION_KI, MOTION_KD, forever=False)
        start_ticks = list(self.bot.get_motor_encoder())
        target_yaw = self.get_yaw()
        wheel_error_int = [0.0] * 4
        progress = 0.0
        average_distance = 0.0
        max_wheel_error = 0.0
        next_debug_time = time.time()
        start_time = time.time()
        last_progress = 0.0
        last_progress_time = start_time
        timeout_s = max(FORWARD_MIN_TIMEOUT_S, distance_m * FORWARD_TIMEOUT_PER_M)

        print(f"[INFO] Moving forward {distance_m:.2f} m at speed {speed}")
        print("[INFO] Encoder sync enabled for all 4 wheels.")
        if STOP_ON_SLOWEST_WHEEL:
            print("[INFO] Stop condition uses slowest wheel distance.")
        try:
            while progress < distance_m:
                distances = self._wheel_distances_from(start_ticks)
                average_distance = sum(distances) / 4.0
                progress = min(distances) if STOP_ON_SLOWEST_WHEEL else average_distance

                if time.time() - start_time > timeout_s:
                    print("[WARN] Forward timeout reached.")
                    break

                # Positive error means this wheel is behind the average and
                # needs more command. Negative error means it is ahead.
                wheel_errors = [average_distance - dist for dist in distances]
                for i in range(4):
                    wheel_error_int[i] += wheel_errors[i] * READ_INTERVAL_S

                sync_corrections = [
                    clamp(
                        WHEEL_SYNC_KP * wheel_errors[i]
                        + WHEEL_SYNC_KI * wheel_error_int[i],
                        -WHEEL_SYNC_MAX_CORRECTION,
                        WHEEL_SYNC_MAX_CORRECTION,
                    )
                    for i in range(4)
                ]

                yaw_error = normalize_angle(self.get_yaw() - target_yaw)
                yaw_correction = clamp(
                    YAW_KP * yaw_error,
                    -YAW_MAX_CORRECTION,
                    YAW_MAX_CORRECTION,
                )

                speeds = []
                for i in range(4):
                    if i in (0, 1):
                        cmd = speed + sync_corrections[i] - yaw_correction
                    else:
                        cmd = speed + sync_corrections[i] + yaw_correction
                    calibrated = cmd * MOTOR_CALIBRATION[i]
                    motor_cmd = clamp(calibrated, 0, MAX_MOTOR_COMMAND)
                    if 0 < motor_cmd < FORWARD_MIN_MOTOR_COMMAND:
                        motor_cmd = FORWARD_MIN_MOTOR_COMMAND
                    speeds.append(motor_cmd)

                self.bot.set_motor(speeds[0], speeds[1], speeds[2], speeds[3])
                max_wheel_error = max(max_wheel_error, max(abs(err) for err in wheel_errors))

                if ENABLE_STALL_DETECTION:
                    if progress >= last_progress + STALL_PROGRESS_EPS_M:
                        last_progress = progress
                        last_progress_time = time.time()
                    elif time.time() - last_progress_time >= STALL_TIMEOUT_S:
                        saturated = [
                            MOTOR_NAMES[i]
                            for i, cmd in enumerate(speeds)
                            if cmd >= MAX_MOTOR_COMMAND - SATURATION_MARGIN
                        ]
                        print("[WARN] Forward stall detected: progress is not increasing.")
                        if saturated:
                            print(f"[WARN] Saturated motors: {', '.join(saturated)}")
                        break

                if self.debug and time.time() >= next_debug_time:
                    next_debug_time = time.time() + 0.3
                    dist_text = ", ".join(f"{dist:.3f}" for dist in distances)
                    err_text = ", ".join(f"{err:.3f}" for err in wheel_errors)
                    cmd_text = ", ".join(f"{cmd:.1f}" for cmd in speeds)
                    print(
                        f"[DEBUG] progress={progress:.3f} "
                        f"avg={average_distance:.3f} yaw_err={yaw_error:.2f}"
                    )
                    print(f"[DEBUG] wheel_dist=[{dist_text}]")
                    print(f"[DEBUG] wheel_err=[{err_text}]")
                    print(f"[DEBUG] motor_cmd=[{cmd_text}]")

                time.sleep(READ_INTERVAL_S)

        except KeyboardInterrupt:
            print("[INFO] Movement interrupted.")
            raise
        finally:
            self.stop()
            time.sleep(0.5)
            print(f"[INFO] Stopped at progress {progress:.3f} m")
            print(f"[INFO] Average wheel distance: {average_distance:.3f} m")
            print(f"[INFO] Max wheel sync error: {max_wheel_error:.3f} m")
            if max_wheel_error > WHEEL_SYNC_WARN_ERROR_M:
                print("[WARN] Wheel sync error is high; tune WHEEL_SYNC_KP/KI.")

        return progress

    def realign_to_yaw(self, target_yaw):
        if self.dry_run:
            print(f"[DRY-RUN] realign yaw to {target_yaw:.2f}")
            return 0.0

        start_time = time.time()
        print(f"[INFO] Realigning to yaw {target_yaw:.2f}")
        try:
            while True:
                current_yaw = self.get_yaw()
                error = normalize_angle(target_yaw - current_yaw)
                if abs(error) <= REALIGN_YAW_TOLERANCE_DEG:
                    break

                if time.time() - start_time > REALIGN_TIMEOUT_S:
                    print("[WARN] Realign timeout reached.")
                    break

                omega = REALIGN_TURN_SPEED if error > 0 else -REALIGN_TURN_SPEED
                self.bot.set_car_motion(0, 0, omega)
                time.sleep(READ_INTERVAL_S)
        finally:
            self.stop()
            time.sleep(0.3)

        final_error = normalize_angle(target_yaw - self.get_yaw())
        print(f"[INFO] Realign yaw error: {final_error:.2f}")
        return final_error

    def turn_by(self, angle_deg, angular_speed=DEFAULT_TURN_SPEED):
        if angle_deg == 0:
            return 0.0

        if self.dry_run:
            direction = "left" if angle_deg > 0 else "right"
            print(f"[DRY-RUN] turn {direction} {abs(angle_deg):.1f} deg")
            return angle_deg

        target_yaw = normalize_angle(self.get_yaw() + angle_deg)
        timeout_s = max(4.0, abs(angle_deg) / 90.0 * 8.0)
        start_time = time.time()

        print(f"[INFO] Turning by {angle_deg:.1f} deg to yaw {target_yaw:.2f}")
        try:
            while True:
                current_yaw = self.get_yaw()
                error = normalize_angle(target_yaw - current_yaw)
                if abs(error) <= TURN_TOLERANCE_DEG:
                    break

                if time.time() - start_time > timeout_s:
                    print("[WARN] Turn timeout reached.")
                    break

                omega = angular_speed if error > 0 else -angular_speed
                self.bot.set_car_motion(0, 0, omega)
                time.sleep(READ_INTERVAL_S)

        except KeyboardInterrupt:
            print("[INFO] Turn interrupted.")
            raise
        finally:
            self.stop()
            time.sleep(0.5)
            print(f"[INFO] Current yaw: {self.get_yaw():.2f}")

        return normalize_angle(self.get_yaw() - target_yaw)
