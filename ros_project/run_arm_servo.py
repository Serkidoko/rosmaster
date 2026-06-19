#coding=utf-8
import time
from Rosmaster_Lib import Rosmaster

grip_angles = (180, 0, 60, 120, 90, 42)
hold_angles = (180, 90, 0, 90, 90, 128)

def arm_servo(s1, s2, s3, s4, s5, s6):
    bot.set_uart_servo_angle_array([s1, s2, s3, s4, s5, s6], run_time=8000)
    return s1, s2, s3, s4, s5, s6

def gripe_right_side_box(bot):
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


if __name__ == "__main__":
    bot = Rosmaster()
    bot.create_receive_threading()
    gripe_right_side_box(bot)