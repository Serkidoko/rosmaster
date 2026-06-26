# Hướng dẫn sử dụng vị trí tay máy trong bài pick and place

Last updated: 2026-06-25

Tài liệu này mô tả quy ước dùng robot arm tại các waypoint cố định của bài
gắp - thả vật. Mission manager cần tuân thủ thứ tự: robot đến đúng waypoint
trước, sau đó mới chuyển trạng thái tay máy và detect vật/thùng.

## 1. Quy ước các khớp tay máy

Thứ tự góc servo trong code luôn là:

```text
K1 K2 K3 K4 K5 K6
```

Trong đó:

```text
K1: khớp 1 / base yaw / xoay đế tay máy trái - phải
K2: khớp 2 / shoulder / nâng - hạ cánh tay chính
K3: khớp 3 / elbow / gập - duỗi cánh tay
K4: khớp 4 / wrist pitch / chúc - ngửa cổ tay
K5: khớp 5 / wrist roll / xoay cổ tay
K6: khớp 6 / gripper / đóng - mở kẹp
```

Góc đang dùng trong `arm_stage_calibrator.py`:

```text
default   = 90 180 0 50 90 90
pick_1    = 0 180 0 50 90 90      # chuẩn bị để camera detect vật
pick_2    = 90 180 0 50 90 90     # hiện giống default
picking_1 = 0 90 90 65 90 30      # đưa tay vào vị trí gắp vật
grasp_1   = 0 90 90 65 90 132     # đóng gripper tại vị trí gắp
picked_1  = 0 180 0 50 90 132     # quay về pick_1 nhưng vẫn giữ vật
hold      = 90 180 0 50 90 132    # nâng vật lên và giữ kẹp đóng
```

Lưu ý: `pick_1` quay khớp 1 từ `90` về `0`, tức quay khoảng 90 độ từ
tọa độ `default` về bên trái. Nếu lắp servo thực tế bị ngược chiều, chỉ đổi
góc của K1, không đổi ý nghĩa FSM.

`PICK_ANGLES_1` không phải là góc gắp vật. Đây là góc chuẩn bị để camera nhìn
và detect vật. Góc thực sự đưa tay vào vị trí gắp là `PICKING_ANGLES_1`.

## 2. Trạng thái `default`

`default` là trạng thái an toàn của tay máy khi robot di chuyển giữa các
waypoint.

Sử dụng `default` khi:

```text
- robot bắt đầu tại home
- robot di chuyển giữa các waypoint
- robot chưa bắt đầu detect tại pick_1
- robot chuẩn bị detect tại pick_2
- robot đã gắp xong và cần giữ vật khi di chuyển
```

Tại `default`, camera có thể quan sát phía trước robot để detect vật ở
`pick_2` hoặc detect thùng xanh ở vùng drop.

## 3. Waypoint `pick_1`

Tại `pick_1`, robot không detect ngay ở trạng thái `default`.

Luồng bắt buộc:

```text
1. navigation_node di chuyển robot đến pick_1.
2. Chỉ khi move_done / Arrived at pick_1 thì mới điều khiển tay máy.
3. Arm chuyển từ default sang pick_1.
4. Ở pick_1, khớp 1 quay 90 độ về bên trái so với default.
5. Sau khi arm đứng yên, visual_approach_node scan hàng 3 vật
   `red/green/yellow` tại slot hiện tại.
6. Nếu vật ở slot hiện tại sai màu yêu cầu, robot chạy chậm `arm-right`
   để qua vật kế tiếp rồi detect lại liên tục.
7. Khi slot hiện tại đúng màu yêu cầu, dùng bounding box để ước lượng vật đã
   nằm đúng vùng tay kẹp chưa.
8. Nếu bbox vật chưa đủ gần:
   - robot chạy chậm theo hướng khớp 1
   - camera detect lại liên tục trong khi robot đang tiến gần
   - dừng ngay khi bbox đủ lớn và nằm trong vùng tay kẹp
9. Nếu detect đúng màu/vật và bbox đã ở vị trí gắp:
   - chuyển sang picking_1 để đưa tay vào vị trí gắp
   - chuyển sang grasp để đóng gripper
   - chuyển sang picked_1 để quay lại tư thế pick_1 nhưng vẫn giữ vật
   - chuyển sang hold để về default/giữ vật an toàn
10. Nếu không detect được:
   - không đóng gripper
   - báo lỗi hoặc thử detect lại vài lần
```

FSM rút gọn:

```text
MOVE_HOME_TO_PICK_1
-> ARM_PICK_1
-> DETECT_OBJECT_1_BY_SLOT_SCAN_AND_4_ZONE_BBOX
-> APPROACH_OBJECT_1_BY_BBOX
-> ARM_PICKING_1
-> GRASP_OBJECT_1
-> ARM_PICKED_1
-> HOLD_OBJECT_1
```

Điều kiện an toàn:

```text
- không chạy detect trước khi arm đã tới trạng thái pick_1
- nếu slot hiện tại sai màu, chỉ scan sang phải theo hệ arm bằng arm-side dương
- không chạy picking_1 nếu bbox vật chưa nằm trong vùng tay kẹp
- không chạy grasp nếu bbox vật chưa nằm trong vùng tay kẹp
- không chạy grasp nếu vision chưa báo object_found = true
- không di chuyển robot tiếp khi gripper chưa hoàn thành grasp/hold
```

Lệnh chạy toàn bộ chuỗi gắp tại `pick_1`:

```bash
python3 new/pick1_sequence_node.py --run
```

Chuỗi này thực hiện:

```text
default -> pick_1 -> slot scan -> bbox approach -> picking_1 -> grasp_1 -> picked_1 -> hold
```

Trong đó:

```text
pick_1    = tư thế camera detect vật
detect    = chỉ khi bbox vật ready mới cho phép picking_1
picking_1 = tư thế đưa tay vào điểm gắp
grasp_1   = đóng gripper tại picking_1
picked_1  = quay về pick_1 nhưng vẫn giữ vật
hold      = quay về default nhưng vẫn giữ vật
```

Nếu detect trả về `not_found`, `not_aligned`, `wrong_color_scan_right`,
`slot_search_right`, hoặc hết thời gian approach/scan, FSM sẽ dừng tại `pick_1`
và không chạy `picking_1`.

## 3.1. Scan slot và ước lượng bbox tại `pick_1`

Sau khi arm đã quay sang `pick_1`, camera dùng bounding box để ước lượng:

```text
- bbox của vật cần gắp
- vùng tay kẹp kỳ vọng trong ảnh, gọi là gripper target box
```

Trước khi căn bbox, node scan màu của slot hiện tại:

```text
- target color lấy từ file --gripper-config, ví dụ pick1_gripper_box_green.json
- detect các màu red / green / yellow trong ROI
- current slot = detection gần tâm gripper target box nhất
- nếu current slot sai màu: action = wrong_color_scan_right
- chỉ tăng wrong_slot_count khi màu sai ổn định đủ `--slot-stable-frames`
  và trước đó đã có khoảng slot trống; màu nhấp nháy trên cùng một vật không
  được tính thành nhiều vật
- nếu chưa có vật nằm trong slot: action = slot_search_right
- nếu đúng màu: action = target_found rồi mới chạy bbox approach
```

Hướng scan sang vật kế tiếp là `arm-right`:

```text
RobotDriver.set_arm_relative_motion(
    arm_k1_angle,
    forward_speed_mps=0.0,
    side_speed_mps=+arm_right_scan_speed_mps,
)
```

Tại `pick_1`, `K1 = 0`, nên `side_speed_mps > 0` là sang phải theo hệ arm.
Không dùng hướng phải/trái của ảnh để quyết định hướng scan hàng vật.

Nếu chưa gắn marker màu riêng lên tay kẹp, không detect trực tiếp tay kẹp.
Thay vào đó, dùng một khung cố định trong ảnh làm vị trí tay kẹp kỳ vọng.
Khung này cần hiệu chỉnh bằng ảnh thực tế sau khi arm đã ở trạng thái
`pick_1`.

Điều kiện sẵn sàng gắp:

```text
- object_found = true
- tâm bbox vật gần tâm gripper target box
- diện tích bbox đủ lớn
- chiều cao bbox đủ lớn
```

Quy tắc điều khiển xe:

```text
- nếu không thấy vật: robot chạy hướng search mặc định để tìm, đến timeout mới báo NOT_FOUND
- màn hình được chia 4 vùng bằng line dọc/ngang qua tâm gripper target bbox
- debug vẽ bbox tất cả màu detect được, target box, line dọc/ngang và label action
- nếu tâm bbox vật lệch khỏi line dọc: robot chỉ đi ngang trái/phải trước
- nếu tâm bbox vật đã đúng line dọc nhưng lệch line ngang: robot chỉ tiến/lùi
- nếu bbox còn nhỏ nhưng đã gần đúng tâm: robot chỉ tiến theo hướng khớp 1
- mỗi chu kỳ chỉ được đi một trục: ngang hoặc tiến/lùi, không trộn để đi chéo
- `--arm-k1-angle` chỉ được là `0`, `90`, hoặc `180`; góc khác bị reject để
  tránh robot-frame biến thành chuyển động chéo
- nếu vật thẳng và bbox đủ lớn: READY_TO_GRASP
```

Hướng tiến gần phụ thuộc vào góc khớp 1 của tay máy:

```text
K1 = 90  -> xe đi thẳng tới trước
K1 = 0   -> xe đi ngang thẳng sang trái
K1 = 180 -> xe đi ngang thẳng sang phải
```

Như vậy tại `pick_1`, vì arm quay trái 90 độ và K1 đang là `0`, xe cần tiến
gần vật bằng chuyển động ngang trái, không phải tiến thẳng theo đầu xe.

Script test:

```bash
python3 -B new/visual_approach_node.py --image new/pick_object_handcam.jpg --color green --save-debug approach_debug.jpg
```

Notebook tune thủ công tay gắp + bbox:

```bash
jupyter lab /home/pi/chay/new/test_arm_positions_bbox_simple.ipynb
```

Notebook này là bản đơn giản, cho phép:

```text
- sửa trực tiếp các tuple góc DEFAULT_ANGLES, PICK_ANGLES_1, PICKING_ANGLES_1...
- mỗi cell test đúng một pose, ví dụ DEFAULT_ANGLES, PICK_ANGLES_1, PICKING_ANGLES_1...
- gọi send_arm(...) để gửi góc servo, mặc định dry-run
- chụp ảnh camera bằng capture_image()
- chọn bbox bằng cv2.selectROI(...) hoặc tự nhập BBOX = (x, y, w, h)
- vẽ bbox màu tím lên ảnh bằng draw_bbox(BBOX)
- lưu /home/pi/chay/pick1_gripper_box_candidate.json để test
- copy candidate sang /home/pi/chay/pick1_gripper_box.json khi đã chắc chắn
```

Hiển thị bbox lên màn hình để tự ước lượng:

```bash
python3 -B new/visual_approach_node.py --image new/pick_object_handcam.jpg --color green --show --pause-on-step
```

Chạy camera ở robot:

```bash
python3 -B new/visual_approach_node.py --camera-id 2 --color green --save-debug approach_debug.jpg
```

Chạy camera và hiển thị quá trình detect trực tiếp:

```bash
python3 -B new/visual_approach_node.py --camera-id 2 --color green --arm-k1-angle 0 --show --pause-on-step
```

Nếu đã calibrate vị trí gắp lý tưởng bằng `pick1_gripper_box.json`, dùng:

```bash
python3 -B new/visual_approach_node.py --camera-id 0 --color yellow --roi 0 0 1 0.8 --gripper-config pick1_gripper_box.json --arm-k1-angle 0 --show --continuous
```

Calibration chuẩn hiện tại từ ảnh `/home/pi/chay/pick1_manual.jpg`:

```text
color = yellow
image_size = 640x480
detected_object_bbox_px = (247, 209, 155, 127)
detected_object_center_px = (324, 272)
gripper_box = [0.3625, 0.410417, 0.289062, 0.314583]
ready_min_area = 13804.0
ready_min_height = 101
center_tolerance_ratio = 0.08
debug_image = /home/pi/chay/pick1_manual_bbox.jpg
ready_check_image = /home/pi/chay/pick1_manual_ready_check.jpg
```

File chuẩn đã ghi:

```text
/home/pi/chay/pick1_gripper_box.json
/home/pi/chay/pick1_gripper_box_candidate.json
```

Kiểm tra offline trên chính ảnh chuẩn trả về `action=ready ready=1`.

Calibration red từ ảnh `/home/pi/chay/pick1_manual_red.jpg`:

```text
color = red
note = vat red thuc te bi sang choi nen camera nhin thanh cam-do
image_size = 640x480
detected_object_bbox_px = (264, 243, 157, 117)
detected_object_center_px = (342, 301)
gripper_box = [0.389062, 0.483333, 0.292187, 0.289583]
ready_min_area = 13014.8
ready_min_height = 93
debug_image = /home/pi/chay/pick1_manual_red_bbox.jpg
ready_check_image = /home/pi/chay/pick1_manual_red_ready_check.jpg
config = /home/pi/chay/pick1_gripper_box_red.json
```

Ngưỡng `red` trong `vision_color_node.py` đã được nới để bắt cả dải
cam-đỏ do chói (`H` khoảng 0..24 và 170..180). Ngưỡng `yellow` bắt đầu từ
`H=23` để giảm nhầm vật đỏ bị chói thành yellow. Test offline hiện tại:

```bash
cd /home/pi/chay
python3 -B new/visual_approach_node.py --image pick1_manual_red.jpg --gripper-config pick1_gripper_box_red.json
python3 -B new/visual_approach_node.py --image pick1_manual.jpg --gripper-config pick1_gripper_box_yellow.json
```

Khi chạy theo màu:

```bash
# Yellow
python3 -B new/visual_approach_node.py --camera-id 0 --gripper-config pick1_gripper_box_yellow.json --show --continuous

# Red
python3 -B new/visual_approach_node.py --camera-id 0 --gripper-config pick1_gripper_box_red.json --show --continuous
```

Calibration green từ ảnh `/home/pi/chay/pick1_manual_green.jpg`:

```text
color = green
image_size = 640x480
detected_object_bbox_px = (236, 254, 158, 113)
detected_object_center_px = (315, 310)
gripper_box = [0.345313, 0.50625, 0.29375, 0.28125]
ready_min_area = 11492.4
ready_min_height = 90
debug_image = /home/pi/chay/pick1_manual_green_bbox.jpg
ready_check_image = /home/pi/chay/pick1_manual_green_ready_check.jpg
config = /home/pi/chay/pick1_gripper_box_green.json
```

Test offline:

```bash
cd /home/pi/chay
python3 -B new/visual_approach_node.py --image pick1_manual_green.jpg --gripper-config pick1_gripper_box_green.json
```

Test slot scan offline ngày 2026-06-25:

```bash
cd /home/pi/chay
python3 -B new/visual_approach_node.py --image pick1_manual_green.jpg --gripper-config pick1_gripper_box_green.json --save-debug /home/pi/chay/slot_scan_green_ready_debug.jpg
python3 -B new/visual_approach_node.py --image pick1_manual_red.jpg --gripper-config pick1_gripper_box_green.json --save-debug /home/pi/chay/slot_scan_wrong_red_for_green_debug.jpg
```

Kết quả:

```text
green target + green image: action=target_found, approach=ready, exit=0
green target + red image: action=wrong_color_scan_right, exit=4
debug images:
/home/pi/chay/slot_scan_green_ready_debug.jpg
/home/pi/chay/slot_scan_wrong_red_for_green_debug.jpg
```

Chạy đầy đủ từ `home` tới khi gắp vật 1 bằng dry-run/ảnh:

```bash
cd /home/pi/chay
python3 -B new/home_to_pick1_pick_object1.py --image pick1_manual_green.jpg --gripper-config pick1_gripper_box_green.json --debug --arm-log /home/pi/chay/home_to_pick1_object1_dryrun_arm.log
```

Chạy thật từ `home` đến `pick_1`, sau đó arm vào `pick_1`, tìm vật và gắp:

```bash
cd /home/pi/chay
python3 -B new/home_to_pick1_pick_object1.py --run --gripper-config pick1_gripper_box_green.json --debug --save-debug /home/pi/chay/home_to_pick1_object1_green_live.jpg --arm-log /home/pi/chay/home_to_pick1_object1_green_live_arm.log
```

Dry-run camera không chạy motor ngày 2026-06-25:

```bash
cd /home/pi/chay
python3 -B new/visual_approach_node.py --camera-id 0 --gripper-config pick1_gripper_box_green.json --max-approach-s 3 --save-debug /home/pi/chay/home_to_pick1_object1_green_camera_dryrun.jpg --debug
```

Kết quả lần mới nhất sau khi thêm slot scan: camera mở được, không chạy motor
thật, action lặp lại là `slot_search_right` vì không có object đúng slot hiện
tại; dry-run in lệnh `arm_side=0.08 => vx=0.08 vy=0.00`, hết
`--max-approach-s` thì exit code `4`. Ảnh debug:
`/home/pi/chay/home_to_pick1_object1_green_camera_dryrun.jpg`.

Cho phép xe tự tiến gần cho đến khi bbox `ready`:

```bash
python3 new/visual_approach_node.py --camera-id 2 --color green --arm-k1-angle 0 --show --run
```

Cho xe tự tiến gần theo bbox đã calibrate:

```bash
python3 new/visual_approach_node.py --camera-id 0 --color yellow --roi 0 0 1 0.8 --gripper-config pick1_gripper_box.json --arm-k1-angle 0 --show --run
```

Các tham số cần hiệu chỉnh:

```text
--gripper-box X Y W H      vùng tay kẹp kỳ vọng trong ảnh, dạng normalize 0..1
--ready-min-area AREA      diện tích bbox tối thiểu để coi là đủ gần
--ready-min-height HEIGHT  chiều cao bbox tối thiểu để coi là đủ gần
--max-approach-s SEC       thời gian tối đa cho visual approach, chống chạy mãi
--approach-control-s SEC   chu kỳ cập nhật lệnh vận tốc theo bbox
--approach-forward-speed-mps SPEED  tốc độ đi thẳng khi K1 = 90
--search-direction DIR     hướng tìm khi mất vật: forward/backward/left/right/none
--search-speed-mps SPEED   tốc độ tìm khi mất vật
--scan-colors red green yellow
                           danh sách màu cần detect trong hàng pick_1
--max-scan-objects 3       tối đa số slot sai màu trước khi dừng
--arm-right-scan-speed-mps 0.08
                           tốc độ scan sang vật kế tiếp theo hệ arm
--slot-center-tolerance-ratio 0.12
                           tolerance để chọn vật đang ở slot hiện tại
--slot-stable-frames 4     số frame liên tiếp cùng màu sai trước khi count slot
--scan-timeout-s 10        timeout riêng cho scan hàng vật
--arm-k1-angle ANGLE       hướng arm: 90 đi thẳng, 0 sang trái, 180 sang phải;
                           chỉ nhận 0/90/180 để tránh đi chéo
--lateral-speed-mps SPEED  tốc độ đi ngang khi K1 = 0 hoặc 180
--invert-lateral           đảo trái/phải nếu robot thật chạy ngược lệnh
--display-scale SCALE      thu nhỏ cửa sổ preview, ví dụ 0.5 hoặc 0.6
```

Trong cửa sổ preview:

```text
- khung xanh lá: bbox vật được detect
- chấm đỏ: tâm bbox vật
- khung tím: vùng tay kẹp kỳ vọng
- chấm tím: tâm vùng tay kẹp
- chữ vàng góc trên: trạng thái ready / wrong_color_scan_right / target_found /
  move_right / move_forward
- nhấn q để thoát
```

Ghi chú: nếu không truyền `--continuous`, node sẽ tự chạy xe chậm cho đến khi
`ready`, `not_found`, `not_aligned`, hoặc hết `--max-approach-s`. Khi muốn
quan sát liên tục để tự ước lượng bbox mà không cho xe chạy, thêm
`--continuous`.

Nếu chạy qua SSH/headless mà không có desktop/VNC, không dùng được cửa sổ
OpenCV `--show`. Khi đó dùng `--save-debug` để lưu ảnh bbox:

```bash
python3 -B new/visual_approach_node.py --camera-id 0 --color yellow --roi 0 0 1 0.8 --gripper-config pick1_gripper_box.json --arm-k1-angle 0 --continuous --save-debug approach_debug.jpg
```

Nếu muốn mở cửa sổ preview thật trên Raspberry Pi desktop, chạy trong terminal
của desktop hoặc đặt display trước:

```bash
export DISPLAY=:0
```

Nếu bbox báo `action=forward` nhưng robot không di chuyển, test riêng chassis:

```bash
python3 new/test_chassis_motion.py --direction left --distance-m 0.05 --lateral-speed-mps 0.16 --run
```

Nếu lệnh trên vẫn không thấy xe đi ngang, tăng nhẹ tốc độ:

```bash
python3 new/test_chassis_motion.py --direction left --distance-m 0.08 --lateral-speed-mps 0.22 --run
```

## 4. Waypoint `pick_2`

Tại `pick_2`, robot detect vật ở trạng thái `default`.

Luồng bắt buộc:

```text
1. navigation_node di chuyển robot đến pick_2.
2. Giữ hoặc đưa arm về default.
3. Bắt đầu vision_color_node detect vật cần gắp.
4. Nếu detect đúng màu/vật:
   - chuyển arm sang tư thế pick phù hợp cho pick_2 nếu cần
   - đóng gripper bằng grasp
   - nâng vật bằng hold
5. Nếu không detect được:
   - không đóng gripper
   - báo lỗi hoặc thử detect lại vài lần
```

FSM rút gọn:

```text
MOVE_DROP_1_TO_PICK_2
-> ARM_DEFAULT
-> DETECT_OBJECT_2
-> GRASP_OBJECT_2
-> HOLD_OBJECT_2
```

Ghi chú: hiện tại chưa tách góc riêng cho `pick_2`. Nếu sau khi test thực tế
cần tư thế khác, thêm preset `pick_2` riêng nhưng vẫn giữ nguyên nguyên tắc:
detect tại `default` trước.

## 5. Waypoint `drop_1` và `drop_2`

Tại các vị trí drop, robot không thả vật ngay khi vừa tới waypoint.

Robot phải detect một thùng lớn màu xanh làm vị trí thả trước.

Luồng bắt buộc tại mỗi drop:

```text
1. navigation_node di chuyển robot đến drop_1 hoặc drop_2.
2. Arm giữ vật ở trạng thái hold.
3. vision_color_node detect thùng lớn màu xanh.
4. Nếu detect được thùng xanh:
   - xác nhận target_drop_found = true
   - mở gripper để thả vật
   - sau khi thả xong, đưa arm về default
5. Nếu không detect được thùng xanh:
   - không mở gripper
   - báo lỗi hoặc thử detect lại vài lần
```

FSM rút gọn cho `drop_1`:

```text
MOVE_PICK_1_TO_DROP_1
-> DETECT_BLUE_DROP_BIN_1
-> PLACE_OBJECT_1
-> ARM_DEFAULT
```

FSM rút gọn cho `drop_2`:

```text
MOVE_PICK_2_TO_DROP_2
-> DETECT_BLUE_DROP_BIN_2
-> PLACE_OBJECT_2
-> ARM_DEFAULT
```

## 6. Vision tại pick và drop

Tại vị trí pick:

```text
- detect vật nhỏ theo màu yêu cầu: red / green / yellow
- dùng contour lớn nhất trong ROI
- chỉ cho phép gắp nếu object_found = true
```

Tại vị trí drop:

```text
- detect thùng lớn màu xanh
- dùng ngưỡng diện tích lớn hơn vật nhỏ
- chỉ cho phép thả nếu blue_drop_bin_found = true
```

Không nên dùng cùng một ngưỡng diện tích cho vật nhỏ và thùng lớn. Thùng drop
lớn hơn vật pick nên cần `min_area` lớn hơn để tránh nhầm vật xanh nhỏ hoặc
nền màu xanh.

## 7. Chu trình tổng thể

Luồng nhiệm vụ đề xuất:

```text
ARM_DEFAULT
-> MOVE_HOME_TO_PICK_1
-> ARM_PICK_1
-> DETECT_OBJECT_1
-> APPROACH_OBJECT_1_BY_BBOX
-> ARM_PICKING_1
-> GRASP_OBJECT_1
-> ARM_PICKED_1
-> HOLD_OBJECT_1
-> MOVE_PICK_1_TO_DROP_1
-> DETECT_BLUE_DROP_BIN_1
-> PLACE_OBJECT_1
-> ARM_DEFAULT
-> MOVE_DROP_1_TO_PICK_2
-> DETECT_OBJECT_2_AT_DEFAULT
-> GRASP_OBJECT_2
-> HOLD_OBJECT_2
-> MOVE_PICK_2_TO_DROP_2
-> DETECT_BLUE_DROP_BIN_2
-> PLACE_OBJECT_2
-> ARM_DEFAULT
-> RETURN_HOME
```

Nguyên tắc quan trọng nhất:

```text
- pick_1: tới waypoint trước, quay arm trái 90 độ, rồi mới detect vật
- pick_2: tới waypoint trước, giữ arm default, rồi detect vật
- drop_1/drop_2: tới waypoint trước, detect thùng xanh lớn, rồi mới thả
```
