import time

def rotate_left_90(bot, angular_speed=0.5):
    # Target yaw
    target_yaw = 90

    print(f"Target yaw: {target_yaw:.2f}")

    # Start rotating left
    bot.set_car_motion(0, 0, angular_speed)
    _, _, current_yaw = bot.get_imu_attitude_data()

    while current_yaw - target_yaw < 0:
        _, _, current_yaw = bot.get_imu_attitude_data()
        print(f"Current yaw: {current_yaw:.2f}")
        time.sleep(0.05)

    bot.set_car_motion(0, 0, 0)  # Stop
    print("Rotation complete.")

def rotate_right_90(bot, angular_speed=0.5):
    # Target yaw
    target_yaw = 0

    print(f"Target yaw: {target_yaw:.2f}")

    # Start rotating right
    bot.set_car_motion(0, 0, -angular_speed)
    _, _, current_yaw = bot.get_imu_attitude_data()

    while current_yaw - target_yaw > 0:
        _, _, current_yaw = bot.get_imu_attitude_data()
        print(f"Current yaw: {current_yaw:.2f}")
        time.sleep(0.05)

    bot.set_car_motion(0, 0, 0)  # Stop
    print("Rotation complete.")

if __name__ == '__main__':
    from Rosmaster_Lib import Rosmaster
    bot = Rosmaster()

    # start encoder report thread
    bot.clear_auto_report_data()
    bot.create_receive_threading()
    time.sleep(0.1)

    rotate_left_90(bot=bot, angular_speed=0.5)
    # rotate_right_90(bot=bot, angular_speed=0.5)
    # bot.set_car_motion(0, 0, 0)