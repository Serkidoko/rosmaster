"""Simple servo sequence for pick/place."""

import time

from config import (
    ARM_GRIP_ANGLES,
    ARM_HOLD_ANGLES,
    ARM_HOME_ANGLES,
    GRIPPER_CLOSE_ANGLE,
    GRIPPER_OPEN_ANGLE,
)
from robot_driver import RobotDriver


class GripperNode:
    def __init__(self, driver=None):
        self.driver = driver or RobotDriver()

    def home(self):
        self.driver.set_servo_array(ARM_HOME_ANGLES)
        time.sleep(3.0)

    def pick(self):
        self.driver.set_servo_angle(6, GRIPPER_OPEN_ANGLE, run_time=1000)
        time.sleep(0.5)
        self.driver.set_servo_array(ARM_GRIP_ANGLES)
        time.sleep(2.5)
        self.driver.set_servo_angle(6, GRIPPER_CLOSE_ANGLE, run_time=1000)
        time.sleep(1.0)
        self.driver.set_servo_array(ARM_HOLD_ANGLES)
        time.sleep(3.0)

    def place(self):
        self.driver.set_servo_angle(6, GRIPPER_OPEN_ANGLE, run_time=1000)
        time.sleep(1.0)
