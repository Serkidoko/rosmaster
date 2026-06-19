#coding=utf-8
import time
from Rosmaster_Lib import Rosmaster


def arm_servo(s1, s2, s3, s4, s5, s6):
    bot.set_uart_servo_angle_array([s1, s2, s3, s4, s5, s6], run_time=8000)
    return s1, s2, s3, s4, s5, s6

if __name__ == "__main__":
    bot = Rosmaster()
    bot.create_receive_threading()
    angles = (167, 180, 0 , 0, 90, 42)
    print("Setting arm servo angles to:", angles)
    arm_servo(*angles)
    time.sleep(1)  # Wait for a second to observe the servo movement
    bot.set_uart_servo_torque(True)