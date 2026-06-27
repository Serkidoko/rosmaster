import argparse
import threading
from Rosmaster_Lib import Rosmaster
from utils import *

parser = argparse.ArgumentParser(description="Robot color detection")
parser.add_argument('--color', type=str, choices=['red', 'green', 'blue', 'yellow'], default='red', help='Target color')
args = parser.parse_args()
color = args.color

bot = Rosmaster()
bot.clear_auto_report_data()
bot.create_receive_threading()

dist = 3.0
dist_calib = 0.13 / 198
grip_angles = (180, 0, 60, 120, 90, 42)
hold_angles = (180, 90, 0, 90, 90, 128)

color_ranges = {
    'red':   [(170, 70, 50), (180, 255, 255)],
    'green': [(35, 40, 40),  (85, 255, 255)],
    'blue':  [(100, 150, 0), (140, 255, 255)],
    'yellow':[(10, 100, 100), (35, 255, 255)],
}

print(f"Voltage remain: {bot.get_battery_voltage()}")
_, _, init_yaw = bot.get_imu_attitude_data()
# init_yaw = map_imu_angle(init_yaw)
print('Initial Yaw:', init_yaw)
time.sleep(3.7)

init_arm_servo(bot)
go_straight(bot, init_yaw, base_speed=37, target_distance=dist)
time.sleep(3.7)


# share flag and bounding_boxes
detected_flag = [False]
bb = []
traveled_result = []
lock = threading.Lock()

t1 = threading.Thread(target=justify_straight_thread, args=(bot, init_yaw, 20, detected_flag, traveled_result, lock))
t2 = threading.Thread(target=run_Cam_thread, args=(2, 480, 640, 0.12, 1, color_ranges, detected_flag, bb, lock))
t1.start()
t2.start()
t1.join()
t2.join()

print(f'[INFO] {bb}')

if bb: 
    print(f'[INFO] Detected bounding boxes: {bb} after running {traveled_result[0]:.2f}')

    go_straight(bot, init_yaw, base_speed=20, target_distance=0.02)
    rotate_left(bot, normalize_angle(init_yaw + 90.0), angular_speed=0.24)

    for box in bb:
        if box[0] == color:
            speed_adjust, dist_adjust = calib_range(dist_calib, box[1])
    print(f'[INFO] Adjusted distance: {dist_adjust:.2f} m')
    if abs(dist_adjust) > 0.05:
        go_straight(bot, normalize_angle(init_yaw + 90.0), base_speed=speed_adjust, target_distance=dist_adjust)
    else:
        sign = math.copysign(1, dist_adjust)
        dist_adjust += 0.05 * sign
        go_straight(bot, normalize_angle(init_yaw + 90.0), base_speed=speed_adjust, target_distance=dist_adjust)
        go_straight(bot, normalize_angle(init_yaw + 90.0), base_speed=-speed_adjust, target_distance=-0.05*sign)

    # init_arm_RGBCam(bot)
    # run_Cam_thread(0, 480, 640, 1.0, 7, color_ranges, detected_flag, bb, lock)
    # print(f'[INFO] Box detected by RGB camera: {bb}')

    gripe_box_right(bot, grip_angles, hold_angles)
    go_straight(bot, normalize_angle(init_yaw + 90.0), base_speed=-speed_adjust, target_distance=-dist_adjust)

    rotate_left(bot, normalize_angle(init_yaw + 180.0), angular_speed=0.24)
    go_straight(bot, normalize_angle(init_yaw + 180.0), base_speed=37, target_distance=dist+traveled_result[0]+0.07) # 0.07m caused by rotation

    drop_box(bot)
    print('[INFO] DONE!')
    print(f'[INFO] Detected bounding boxes: {bb}')