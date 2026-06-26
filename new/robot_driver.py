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
    LATERAL_BASE_MOTOR_COMMAND,
    LATERAL_LEFT_MOTOR_CALIBRATION,
    LATERAL_LEFT_PROFILE,
    LATERAL_REALIGN_AFTER_MOVE,
    LATERAL_REFERENCE_SPEED_MPS,
    LATERAL_RIGHT_MOTOR_CALIBRATION,
    LATERAL_RIGHT_PROFILE,
    LATERAL_TIMEOUT_SCALE,
    LATERAL_WHEEL_SYNC_KI,
    LATERAL_WHEEL_SYNC_KP,
    LATERAL_WHEEL_SYNC_MAX_CORRECTION,
    LATERAL_ABORT_YAW_ERROR_DEG,
    LATERAL_YAW_KP,
    LATERAL_YAW_MAX_CORRECTION,
    LATERAL_SEGMENT_PAUSE_S,
    MAX_LATERAL_SEGMENT_M,
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
    def __init__(self, dry_run=False, debug=False, bot=None):
        self.dry_run = dry_run
        self.debug = debug
        self.bot = None

        if self.dry_run:
            self.initial_yaw = 0.0
            print("[DRY-RUN] RobotDriver initialized without hardware.")
            return

        if bot is None:
            from Rosmaster_Lib import Rosmaster

            self.bot = Rosmaster()
            self.bot.clear_auto_report_data()
            self.bot.create_receive_threading()
            time.sleep(1.0)
        else:
            self.bot = bot
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

    def set_approach_motion(
        self,
        direction,
        forward_speed_mps=0.12,
        lateral_speed_mps=0.16,
    ):
        if direction not in ("forward", "left", "right"):
            raise ValueError("direction must be forward, left, or right")
        if forward_speed_mps <= 0:
            raise ValueError("forward_speed_mps must be positive")
        if lateral_speed_mps <= 0:
            raise ValueError("lateral_speed_mps must be positive")

        if direction == "forward":
            vx, vy = forward_speed_mps, 0.0
        elif direction == "left":
            vx, vy = 0.0, lateral_speed_mps
        else:
            vx, vy = 0.0, -lateral_speed_mps

        if self.dry_run:
            print(
                f"[DRY-RUN] approach motion direction={direction} "
                f"vx={vx:.2f} vy={vy:.2f}"
            )
            return

        self.bot.set_car_motion(vx, vy, 0)

    def set_velocity_motion(self, vx, vy, omega=0.0):
        if self.dry_run:
            print(
                f"[DRY-RUN] velocity motion "
                f"vx={vx:.2f} vy={vy:.2f} omega={omega:.2f}"
            )
            return

        self.bot.set_car_motion(vx, vy, omega)

    def set_arm_relative_motion(
        self,
        arm_k1_angle,
        forward_speed_mps=0.0,
        side_speed_mps=0.0,
    ):
        """Move in the arm coordinate frame using Rosmaster omnidirectional drive.

        K1=90 means arm-forward is robot-forward. K1=0 means arm-forward is
        robot-left. K1=180 means arm-forward is robot-right.
        """
        angle_rad = math.radians(arm_k1_angle)
        forward_x = math.sin(angle_rad)
        forward_y = math.cos(angle_rad)
        side_x = math.cos(angle_rad)
        side_y = -math.sin(angle_rad)

        vx = forward_speed_mps * forward_x + side_speed_mps * side_x
        vy = forward_speed_mps * forward_y + side_speed_mps * side_y
        if self.dry_run:
            print(
                f"[DRY-RUN] arm-relative motion k1={arm_k1_angle} "
                f"arm_forward={forward_speed_mps:.2f} "
                f"arm_side={side_speed_mps:.2f} "
                f"=> vx={vx:.2f} vy={vy:.2f}"
            )
            return

        self.bot.set_car_motion(vx, vy, 0)

    def _wheel_distances_from(self, start_ticks):
        enc = list(self.bot.get_motor_encoder())
        deltas = [enc[i] - start_ticks[i] for i in range(4)]
        return [ticks_to_meters(delta) for delta in deltas]

    def _abs_wheel_distances_from(self, start_ticks):
        return [abs(dist) for dist in self._wheel_distances_from(start_ticks)]

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
                        cmd = speed + sync_corrections[i] + yaw_correction
                    else:
                        cmd = speed + sync_corrections[i] - yaw_correction
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

    def move_lateral_timed(self, direction, distance_m, speed_mps=0.16):
        """Move sideways for a short distance using Rosmaster car-motion."""
        if direction not in ("left", "right"):
            raise ValueError("direction must be left or right")
        if distance_m <= 0:
            raise ValueError("distance_m must be positive")
        if speed_mps <= 0:
            raise ValueError("speed_mps must be positive")

        duration_s = distance_m / speed_mps
        if self.dry_run:
            print(
                f"[DRY-RUN] strafe {direction} {distance_m:.2f} m "
                f"at {speed_mps:.2f} m/s for {duration_s:.2f} s"
            )
            return distance_m

        vy = speed_mps if direction == "left" else -speed_mps
        print(
            f"[INFO] Strafing {direction} {distance_m:.2f} m "
            f"at {speed_mps:.2f} m/s for {duration_s:.2f} s"
        )
        try:
            start_time = time.time()
            while time.time() - start_time < duration_s:
                self.bot.set_car_motion(0, vy, 0)
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("[INFO] Lateral movement interrupted.")
            raise
        finally:
            self.stop()
            time.sleep(0.3)

        return distance_m

    def move_lateral_calibrated(self, direction, distance_m, speed_mps=0.16):
        """Move sideways with a separate calibrated mecanum motor profile."""
        if direction not in ("left", "right"):
            raise ValueError("direction must be left or right")
        if distance_m <= 0:
            raise ValueError("distance_m must be positive")
        if speed_mps <= 0:
            raise ValueError("speed_mps must be positive")

        if self.dry_run:
            segment_count = int(math.ceil(distance_m / MAX_LATERAL_SEGMENT_M))
            print(
                f"[DRY-RUN] calibrated strafe {direction} {distance_m:.2f} m "
                f"at {speed_mps:.2f} m/s in {segment_count} segment(s)"
            )
            return distance_m

        target_yaw = self.get_yaw()
        if distance_m <= MAX_LATERAL_SEGMENT_M:
            return self._move_lateral_calibrated_once(
                direction,
                distance_m,
                speed_mps,
                target_yaw,
                segment_label=None,
            )

        remaining_m = distance_m
        total_progress = 0.0
        segment_count = int(math.ceil(distance_m / MAX_LATERAL_SEGMENT_M))
        segment_index = 1
        print(
            f"[INFO] Segmented lateral move: {distance_m:.2f} m "
            f"as {segment_count} x <= {MAX_LATERAL_SEGMENT_M:.2f} m"
        )

        while remaining_m > 0:
            segment_m = min(remaining_m, MAX_LATERAL_SEGMENT_M)
            progress = self._move_lateral_calibrated_once(
                direction,
                segment_m,
                speed_mps,
                target_yaw,
                segment_label=f"{segment_index}/{segment_count}",
            )
            total_progress += progress
            remaining_m = max(0.0, remaining_m - progress)

            if progress < segment_m * 0.7:
                print("[WARN] Lateral segment made too little progress; stopping sequence.")
                break

            if remaining_m > 0:
                time.sleep(LATERAL_SEGMENT_PAUSE_S)
            segment_index += 1

        print(f"[INFO] Segmented lateral total progress: {total_progress:.3f} m")
        return total_progress

    def _move_lateral_calibrated_once(
        self,
        direction,
        distance_m,
        speed_mps,
        target_yaw,
        segment_label=None,
    ):
        """Move one lateral segment while holding the caller's target yaw."""
        if direction == "left":
            motor_profile = LATERAL_LEFT_PROFILE
            motor_calibration = LATERAL_LEFT_MOTOR_CALIBRATION
        else:
            motor_profile = LATERAL_RIGHT_PROFILE
            motor_calibration = LATERAL_RIGHT_MOTOR_CALIBRATION
        base_command = clamp(
            speed_mps / LATERAL_REFERENCE_SPEED_MPS * LATERAL_BASE_MOTOR_COMMAND,
            0,
            MAX_MOTOR_COMMAND,
        )
        start_ticks = list(self.bot.get_motor_encoder())
        wheel_error_int = [0.0] * 4
        progress = 0.0
        average_distance = 0.0
        max_wheel_error = 0.0
        duration_s = distance_m / speed_mps
        timeout_s = max(1.0, duration_s * LATERAL_TIMEOUT_SCALE)

        label_text = f" segment {segment_label}" if segment_label else ""
        print(
            f"[INFO] Calibrated strafe{label_text} {direction} {distance_m:.2f} m "
            f"with base motor command {base_command:.1f}"
        )
        try:
            start_time = time.time()
            while progress < distance_m:
                distances = self._abs_wheel_distances_from(start_ticks)
                average_distance = sum(distances) / 4.0
                progress = min(distances) if STOP_ON_SLOWEST_WHEEL else average_distance

                if time.time() - start_time > timeout_s:
                    print("[WARN] Lateral timeout reached.")
                    break

                wheel_errors = [average_distance - dist for dist in distances]
                for i in range(4):
                    wheel_error_int[i] += wheel_errors[i] * READ_INTERVAL_S

                sync_corrections = [
                    clamp(
                        LATERAL_WHEEL_SYNC_KP * wheel_errors[i]
                        + LATERAL_WHEEL_SYNC_KI * wheel_error_int[i],
                        -LATERAL_WHEEL_SYNC_MAX_CORRECTION,
                        LATERAL_WHEEL_SYNC_MAX_CORRECTION,
                    )
                    for i in range(4)
                ]

                yaw_error = normalize_angle(self.get_yaw() - target_yaw)
                if abs(yaw_error) > LATERAL_ABORT_YAW_ERROR_DEG:
                    print(
                        "[WARN] Lateral yaw error exceeded "
                        f"{LATERAL_ABORT_YAW_ERROR_DEG:.1f} deg; stopping segment."
                    )
                    break

                yaw_correction = clamp(
                    LATERAL_YAW_KP * yaw_error,
                    -LATERAL_YAW_MAX_CORRECTION,
                    LATERAL_YAW_MAX_CORRECTION,
                )

                speeds = []
                for i in range(4):
                    signed_base = motor_profile[i] * (base_command + sync_corrections[i])
                    signed_yaw = (1 if i in (0, 1) else -1) * yaw_correction
                    calibrated = (signed_base + signed_yaw) * motor_calibration[i]
                    speeds.append(clamp(calibrated, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND))

                self.bot.set_motor(speeds[0], speeds[1], speeds[2], speeds[3])
                max_wheel_error = max(max_wheel_error, max(abs(err) for err in wheel_errors))

                if self.debug:
                    dist_text = ", ".join(f"{dist:.3f}" for dist in distances)
                    err_text = ", ".join(f"{err:.3f}" for err in wheel_errors)
                    cmd_text = ", ".join(f"{cmd:.1f}" for cmd in speeds)
                    print(
                        f"[DEBUG] lateral progress={progress:.3f} "
                        f"avg={average_distance:.3f} yaw_err={yaw_error:.2f}"
                    )
                    print(f"[DEBUG] lateral_wheel_dist=[{dist_text}]")
                    print(f"[DEBUG] lateral_wheel_err=[{err_text}]")
                    print(f"[DEBUG] lateral_motor_cmd=[{cmd_text}]")

                time.sleep(READ_INTERVAL_S)
        except KeyboardInterrupt:
            print("[INFO] Lateral movement interrupted.")
            raise
        finally:
            self.stop()
            time.sleep(0.3)
            print(f"[INFO] Lateral stopped at progress {progress:.3f} m")
            print(f"[INFO] Lateral average wheel distance: {average_distance:.3f} m")
            print(f"[INFO] Lateral max wheel sync error: {max_wheel_error:.3f} m")
            final_yaw_error = normalize_angle(self.get_yaw() - target_yaw)
            print(f"[INFO] Lateral final yaw error: {final_yaw_error:.2f}")
            if LATERAL_REALIGN_AFTER_MOVE and abs(final_yaw_error) > REALIGN_YAW_TOLERANCE_DEG:
                self.realign_to_yaw(target_yaw)

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
