# Robot ROSMASTER X3 Plus gắp - thả vật theo màu

Last updated: 2026-06-25

Đây là project robot/AI cho ROSMASTER X3 Plus: robot di chuyển qua các waypoint
cố định, nhận diện vật theo màu, gắp vật bằng tay máy, thả vật tại vị trí quy
định rồi quay về `home`.

Mục tiêu của project là xây dựng code đơn giản, dễ hiểu, dễ kiểm thử cho bài
robot pick and place trong môi trường trong nhà.

## 1. Bối cảnh bài toán

Robot hoạt động với các giả định:

- Môi trường trong nhà, mặt phẳng ổn định.
- Không có vật cản, không cần tránh vật cản.
- Waypoint cố định, khoảng cách/route được cấu hình trước.
- Robot có camera để nhận diện màu vật.
- Robot có tay máy và gripper để gắp/thả vật.
- Vật cần gắp được chọn theo màu, ví dụ `red`, `green`, `yellow`, `blue`.

## 2. Luồng nhiệm vụ chính

Luồng tổng quát:

```text
home -> pick_1 -> drop_1 -> pick_2 -> drop_2 -> home
```

Các bước chính:

1. Robot bắt đầu tại `home`.
2. Di chuyển đến `pick_1`.
3. Chuyển tay máy sang tư thế quan sát tại `pick_1`.
4. Camera scan hàng 3 vật `red/green/yellow` tại `pick_1` để tìm đúng màu.
5. Nếu vật ở slot hiện tại sai màu, robot đi `arm-right` chậm để sang vật kế tiếp.
6. Khi gặp đúng màu, camera kiểm tra bbox 4 vùng để căn vào điểm gắp.
7. Nếu vật đã sẵn sàng gắp, tay máy chạy chuỗi `picking_1 -> grasp_1 -> picked_1 -> hold`.
8. Robot di chuyển đến `drop_1` và thả vật.
9. Lặp lại với `pick_2` và `drop_2`.
10. Robot quay về `home`.

## 3. Thư mục và file quan trọng

Code active nằm trong:

```text
/home/pi/chay/new
```

Các file chính:

```text
new/config.py                    # cấu hình route, motor, calibration
new/robot_driver.py              # điều khiển chassis, motor, encoder/yaw
new/navigation_node.py           # di chuyển giữa waypoint
new/mission_manager_node.py      # FSM chuẩn bị đến pick_1
new/pick1_sequence_node.py       # FSM gắp vật tại pick_1
new/home_to_pick1_pick_object1.py # FSM home -> pick_1 -> gắp vật 1
new/vision_color_node.py         # detect vật theo màu HSV
new/visual_approach_node.py      # scan màu, detect bbox và approach vật
new/arm_stage_calibrator.py      # tư thế tay máy/gripper
new/calibrate_wheels.py          # calibration đi thẳng
new/calibrate_lateral_wheels.py  # calibration đi ngang
new/calibrate_pick_bbox.py       # calibration vùng bbox gắp
new/test_arm_positions_bbox_simple.ipynb # notebook test từng pose tay gắp + bbox
```

Tài liệu cần đọc trước khi sửa code:

```text
AGENTS.md
new/ROBOT_CALIBRATION_NOTES.md
new/ARM_POSITION_INSTRUCTIONS.md
```

## 4. Kiến trúc đề xuất

Project đang đi theo hướng chia module giống các node ROS:

- `vision_color_node`: nhận ảnh, chuyển HSV, threshold màu, tìm contour/bbox.
- `navigation_node`: nhận waypoint, chạy route cố định, báo hoàn thành.
- `arm_stage_calibrator` hoặc gripper logic: điều khiển tư thế tay máy.
- `mission_manager_node`: quản lý FSM cấp cao.
- `pick1_sequence_node`: FSM chi tiết cho chuỗi gắp tại `pick_1`.

Các trạng thái nên rõ ràng, ví dụ:

```text
move_done
object_found
ready_to_grasp
gripper_done
failed
timeout
```

## 5. Quy tắc an toàn và thực nghiệm

- Không detect/gắp tại `pick_1` trước khi robot đã tới waypoint và arm đã đứng
  yên ở tư thế `pick_1`.
- Không đóng gripper nếu vision chưa thấy đúng màu mục tiêu hoặc bbox chưa đạt
  điều kiện `ready`.
- Ở `pick_1`, nếu slot hiện tại sai màu thì chỉ chạy `arm-right` bằng
  `side_speed_mps > 0`, không trộn thêm forward/backward trong cùng chu kỳ.
- Không di chuyển robot tiếp khi arm/gripper chưa hoàn thành trạng thái cần thiết.
- Không trộn calibration đi thẳng với calibration đi ngang.
- Không đổi các giá trị user đã xác nhận ổn định nếu chưa có log robot thật tốt
  hơn.
- Nếu chạy robot thật, lưu log/ảnh debug có timestamp trong `/home/pi/chay`.

## 6. Lệnh kiểm tra thường dùng

Compile nhanh các script chính:

```bash
cd /home/pi/chay/new
python3 -m py_compile config.py robot_driver.py navigation_node.py mission_manager_node.py vision_color_node.py visual_approach_node.py arm_stage_calibrator.py pick1_sequence_node.py home_to_pick1_pick_object1.py
```

Chạy dry-run chuẩn bị tới `pick_1`:

```bash
cd /home/pi/chay/new
python3 mission_manager_node.py --start home --debug
```

Chạy dry-run chuỗi gắp tại `pick_1` bằng ảnh:

```bash
cd /home/pi/chay
python3 -B new/pick1_sequence_node.py --image new/pick_object_handcam.jpg --color green --debug
```

Chạy dry-run đầy đủ từ `home` tới `pick_1` rồi gắp vật 1 bằng ảnh chuẩn:

```bash
cd /home/pi/chay
python3 -B new/home_to_pick1_pick_object1.py --image pick1_manual_green.jpg --gripper-config pick1_gripper_box_green.json --debug
```

Kiểm tra case vật hiện tại sai màu, target là green:

```bash
cd /home/pi/chay
python3 -B new/visual_approach_node.py --image pick1_manual_red.jpg --gripper-config pick1_gripper_box_green.json --save-debug /home/pi/chay/slot_scan_wrong_red_for_green_debug.jpg
```

Chạy detect/approach bằng camera:

```bash
cd /home/pi/chay
python3 -B new/visual_approach_node.py --camera-id 0 --roi 0 0 1 0.8 --gripper-config pick1_gripper_box_green.json --show --continuous
```

Tham số scan hàng 3 vật tại `pick_1`:

```text
--scan-colors red green yellow
--max-scan-objects 3
--arm-right-scan-speed-mps 0.08
--slot-center-tolerance-ratio 0.12
--slot-stable-frames 4
--scan-timeout-s 10
```

Mở notebook đơn giản để chỉnh các tuple góc tay gắp, chụp ảnh và vẽ bbox:

```bash
jupyter lab /home/pi/chay/new/test_arm_positions_bbox_simple.ipynb
```

Chạy robot thật chỉ khi robot đã đặt đúng sân và có đủ khoảng trống:

```bash
cd /home/pi/chay/new
python3 home_to_pick1_pick_object1.py --run --gripper-config /home/pi/chay/pick1_gripper_box_green.json --debug
```

## 7. Quy tắc cập nhật tài liệu

Mỗi lần sửa code, calibration, route, FSM, camera, tay máy, test script hoặc
tham số chạy thật, phải cập nhật Markdown liên quan trong cùng lượt làm việc.

Ưu tiên cập nhật:

- `README.md`: tổng quan đề tài và cách chạy.
- `AGENTS.md`: luật làm việc cho AI/code agent.
- `new/ROBOT_CALIBRATION_NOTES.md`: calibration chassis, log, test thật.
- `new/ARM_POSITION_INSTRUCTIONS.md`: tư thế servo, thứ tự arm/gripper.

Nếu thay đổi chưa được test trên robot thật, ghi rõ là mới compile/dry-run và
chưa xác nhận hardware.
