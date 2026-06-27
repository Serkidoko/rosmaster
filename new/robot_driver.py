"""Tiny Rosmaster wrapper used by the pick alignment code."""

import time


class RobotDriver:
    def __init__(self, dry_run=False):
        self.dry_run = dry_run
        self.bot = None

        if self.dry_run:
            print("[DRY-RUN] RobotDriver")
            return

        from Rosmaster_Lib import Rosmaster

        self.bot = Rosmaster()
        self.bot.clear_auto_report_data()
        self.bot.create_receive_threading()
        time.sleep(1.0)
        print(f"[INFO] Battery voltage: {self.bot.get_battery_voltage()}")

    def stop(self):
        if self.dry_run:
            print("[DRY-RUN] stop")
            return
        self.bot.set_motor(0, 0, 0, 0)
        self.bot.set_car_motion(0, 0, 0)

    def move_for(self, forward_mps=0.0, strafe_left_mps=0.0, turn_rad_s=0.0, seconds=0.15):
        if self.dry_run:
            print(
                "[DRY-RUN] move_for "
                f"forward={forward_mps:.2f} strafe_left={strafe_left_mps:.2f} "
                f"turn={turn_rad_s:.2f} seconds={seconds:.2f}"
            )
            return

        try:
            self.bot.set_car_motion(forward_mps, strafe_left_mps, turn_rad_s)
            time.sleep(seconds)
        finally:
            self.stop()
            time.sleep(0.1)

    def set_servo_array(self, angles, run_time=8000):
        if self.dry_run:
            print(f"[DRY-RUN] servo_array {angles} run_time={run_time}")
            return
        self.bot.set_uart_servo_torque(True)
        self.bot.set_uart_servo_angle_array(list(angles), run_time=run_time)

    def set_servo_angle(self, servo_id, angle, run_time=1000):
        if self.dry_run:
            print(f"[DRY-RUN] servo {servo_id} -> {angle} run_time={run_time}")
            return
        self.bot.set_uart_servo_torque(True)
        self.bot.set_uart_servo_angle(servo_id, angle, run_time=run_time)
