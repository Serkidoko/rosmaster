# Instruction cho agent code: Robot ROSMASTER X3 Plus gắp - thả vật theo màu

## 1. Vai trò của agent code

Bạn là agent code hỗ trợ xây dựng, mô phỏng và lập trình hệ thống robot gắp - thả vật sử dụng ROSMASTER X3 Plus.

Mục tiêu chính là tạo code đơn giản, dễ hiểu, dễ kiểm thử cho sinh viên làm project robot/AI. Khi viết code, hãy ưu tiên cấu trúc rõ ràng, có chú thích, tách module theo node ROS và điều khiển bằng FSM.

## 2. Bối cảnh bài toán

Robot sử dụng là ROSMASTER X3 Plus, hoạt động trong môi trường trong nhà:

- Mặt phẳng ổn định.
- Không có vật cản.
- Các vị trí làm việc là cố định.
- Robot có camera indoor hoặc cảm biến màu.
- Robot có cơ cấu gắp/thả vật.
- Robot có thể di chuyển đến waypoint cố định.
- Robot chọn vật cần gắp dựa trên màu sắc được chỉ định.

Không cần xử lý tránh vật cản trong bài toán này.

## 3. Các vị trí làm việc cố định

Hệ thống có 5 vị trí chính:

1. `home`
   - Vị trí bắt đầu.
   - Vị trí robot quay về sau khi hoàn thành chu trình.

2. `pick_1`
   - Vị trí gắp vật 1.
   - Robot dừng tại đây để nhận diện vật 1 theo màu được chỉ định.
   - Sau khi nhận diện đúng màu, robot gắp vật 1.

3. `drop_1`
   - Vị trí thả vật 1.
   - Robot di chuyển từ `pick_1` đến đây.
   - Robot mở gripper để thả vật 1.

4. `pick_2`
   - Vị trí gắp vật 2.
   - Robot dừng tại đây để nhận diện vật 2 theo màu được chỉ định.
   - Sau khi nhận diện đúng màu, robot gắp vật 2.

5. `drop_2`
   - Vị trí thả vật 2.
   - Robot di chuyển từ `pick_2` đến đây.
   - Robot mở gripper để thả vật 2.

## 4. Lộ trình hoạt động tổng quát

Chu trình robot cần thực hiện:

1. Robot bắt đầu tại `home`.
2. Robot di chuyển từ `home` đến `pick_1`.
3. Tại `pick_1`, robot nhận diện vật 1 theo màu được chỉ định.
4. Robot gắp vật 1.
5. Robot di chuyển đến `drop_1`.
6. Tại `drop_1`, robot thả vật 1.
7. Robot di chuyển đến `pick_2`.
8. Tại `pick_2`, robot nhận diện vật 2 theo màu được chỉ định.
9. Robot gắp vật 2.
10. Robot di chuyển đến `drop_2`.
11. Tại `drop_2`, robot thả vật 2.
12. Robot quay trở lại `home`.
13. Kết thúc chu trình.

Luồng rút gọn:

```text
home -> pick_1 -> drop_1 -> pick_2 -> drop_2 -> home
```

## 5. Kiến trúc ROS đề xuất

Ưu tiên tổ chức code thành các node sau:

1. `vision_color_node`
   - Nhận ảnh từ camera.
   - Chuyển ảnh RGB/BGR sang HSV.
   - Threshold màu theo màu mục tiêu.
   - Tìm contour lớn nhất.
   - Trả về trạng thái có tìm thấy vật hay không.
   - Có thể trả thêm tọa độ tâm vật trong ảnh.

2. `navigation_node`
   - Nhận tên waypoint cần đến.
   - Điều khiển robot di chuyển đến waypoint cố định.
   - Trả về trạng thái `move_done`.
   - Vì môi trường không có vật cản, có thể dùng điều hướng waypoint đơn giản.

3. `gripper_node`
   - Nhận lệnh `open`, `close`, `pick`, `place`.
   - Điều khiển cơ cấu gắp/thả.
   - Trả về trạng thái `gripper_done`.

4. `mission_manager_node`
   - Node quản lý nhiệm vụ chính.
   - Chạy FSM.

## 6. Luật bắt buộc cập nhật tài liệu

Agent AI phải coi tài liệu Markdown là một phần của code, không phải ghi chú phụ.
Mỗi lần sửa code, calibration, route, FSM, camera, tay máy, test script hoặc
tham số chạy thật, phải kiểm tra và cập nhật tài liệu liên quan ngay trong cùng
lượt làm việc.

Các file cần ưu tiên đọc trước khi sửa:

```text
/home/pi/chay/AGENTS.md
/home/pi/chay/README.md
/home/pi/chay/new/ROBOT_CALIBRATION_NOTES.md
/home/pi/chay/new/ARM_POSITION_INSTRUCTIONS.md
```

Khi thay đổi hành vi hoặc tham số, bắt buộc cập nhật ít nhất một trong các phần
sau nếu có liên quan:

1. Mục tiêu và phạm vi bài toán.
2. FSM, thứ tự trạng thái, điều kiện chuyển trạng thái.
3. Waypoint, route, khoảng cách, hướng di chuyển.
4. Calibration motor, lateral/straight movement, yaw correction.
5. Góc servo tay máy, ý nghĩa từng tư thế, thời gian chờ arm ổn định.
6. Vision: màu detect, HSV threshold, ROI, bbox, debug image, camera id.
7. Lệnh test đã chạy, lệnh test cần chạy, log path, ảnh debug path.
8. Kết quả thực nghiệm: pass/fail, hiện tượng robot thật, quyết định giữ/bỏ.
9. Known issue hoặc giả định tạm thời để agent sau không lặp lại lỗi cũ.

Nếu thay đổi làm khác cách giới thiệu đề tài, cấu trúc thư mục, lệnh chạy chính
hoặc danh sách tài liệu quan trọng, phải cập nhật `/home/pi/chay/README.md`.

Nếu chưa có file `.md` phù hợp, tạo file mới trong `/home/pi/chay/new` với tên
rõ nghĩa, ví dụ:

```text
MISSION_FLOW_NOTES.md
VISION_COLOR_NOTES.md
PICK_DROP_TEST_NOTES.md
```

Quy tắc khi viết tài liệu:

- Ghi ngắn gọn, dùng bullet rõ ràng, ưu tiên lệnh chạy thật và giá trị cụ thể.
- Mỗi tài liệu quan trọng nên có dòng `Last updated: YYYY-MM-DD`.
- Không để tài liệu nói ngược code hiện tại.
- Không xóa kết quả thực nghiệm cũ nếu nó giải thích vì sao không dùng một cấu
  hình nào đó.
- Nếu một log hoặc ảnh debug chứng minh hành vi robot, ghi lại đường dẫn tuyệt
  đối hoặc tương đối từ `/home/pi/chay`.
- Nếu không thể test trên robot thật, ghi rõ là chỉ compile/dry-run/chưa test
  hardware.
- Không cập nhật `.md` bằng nhận định mơ hồ như "có vẻ ổn"; phải ghi điều kiện
  kiểm chứng hoặc lý do giữ nguyên.

Trước khi kết thúc một thay đổi, agent phải tự hỏi:

```text
- Code mới có làm thay đổi cách robot chạy, detect, gắp, thả hoặc dừng không?
- Có tham số nào mới mà người sau cần biết không?
- Có test/log/ảnh nào cần lưu lại vào tài liệu không?
- Có hướng dẫn cũ nào đã sai vì thay đổi này không?
```

Nếu câu trả lời là "có", phải cập nhật `.md` trước khi báo hoàn thành.

## 7. Luật riêng cho bài toán ROSMASTER X3 Plus

### 7.1. Phạm vi code đang hoạt động

- Code active nằm trong `/home/pi/chay/new`.
- Không phục hồi hoặc chỉnh sửa `old_code`/`ros_project` trừ khi user yêu cầu rõ.
- Không trộn thay đổi unrelated vào cùng một lần sửa. Ví dụ đang tune chassis thì
  không tự ý đổi vision hoặc arm.

### 7.2. Kiến trúc điều khiển

- `mission_manager_node` là nơi điều phối FSM chính.
- `navigation_node` chỉ xử lý di chuyển waypoint hoặc chuyển động ngắn có kiểm soát.
- `vision_color_node` và `visual_approach_node` chỉ chịu trách nhiệm detect/alignment
  theo màu và bbox.
- `arm_stage_calibrator.py`/gripper logic chịu trách nhiệm tư thế tay máy.
- Các node nên giao tiếp bằng trạng thái rõ ràng như `move_done`,
  `object_found`, `ready_to_grasp`, `gripper_done`, `failed`, `timeout`.

### 7.3. Waypoint và mission flow

Luồng mặc định của bài toán là:

```text
home -> pick_1 -> drop_1 -> pick_2 -> drop_2 -> home
```

Không thêm tránh vật cản, SLAM hoặc path planning phức tạp nếu user không yêu
cầu, vì môi trường bài toán hiện giả định:

```text
- trong nhà
- mặt phẳng ổn định
- không có vật cản
- waypoint cố định
- robot chọn vật theo màu
```

### 7.4. Chassis và calibration

- Không trộn calibration đi thẳng với calibration đi ngang.
- Không đổi `MOTOR_CALIBRATION` khi đang tune lateral movement.
- Tune lateral bằng `LATERAL_LEFT_MOTOR_CALIBRATION` và
  `LATERAL_RIGHT_MOTOR_CALIBRATION` riêng.
- Không gộp lateral movement thành một lệnh dài nếu tài liệu calibration đang yêu
  cầu chia segment.
- Không đảo dấu yaw correction nếu không có log robot thật chứng minh.
- Giá trị user đã xác nhận ổn định là nguồn tin mạnh hơn suy luận từ một log lẻ.
- Mọi thay đổi calibration phải ghi vào `ROBOT_CALIBRATION_NOTES.md` kèm:
  giá trị cũ, giá trị mới, lệnh test, log path, kết luận.

### 7.5. Vision và màu vật

- Detect màu bằng HSV/ROI/bbox, không hard-code theo một ảnh duy nhất.
- Khi đổi threshold, ROI, camera id hoặc target color, phải ghi vào tài liệu.
- Nếu test bằng ảnh debug, lưu đường dẫn ảnh và mô tả kết quả.
- Nếu vật thật khác màu config cũ, dùng tham số override rõ ràng thay vì sửa bừa
  config chung.

### 7.6. Arm, gripper và an toàn gắp

- Thứ tự servo luôn là `K1 K2 K3 K4 K5 K6`.
- Khi robot di chuyển giữa waypoint, tay máy nên ở tư thế an toàn/default hoặc
  hold phù hợp.
- Không chạy detect tại `pick_1` trước khi arm đã tới tư thế `pick_1` và đã chờ
  đủ thời gian ổn định.
- Không chạy `picking_1` hoặc đóng gripper nếu bbox chưa đạt điều kiện gắp.
- Không di chuyển robot tiếp khi gripper/arm chưa báo hoàn thành trạng thái cần
  thiết.
- Mọi thay đổi góc servo hoặc thời gian chờ phải cập nhật
  `ARM_POSITION_INSTRUCTIONS.md`.

### 7.7. Test và xác nhận

Trước khi báo hoàn thành, ưu tiên chạy kiểm tra phù hợp với mức thay đổi:

```bash
cd /home/pi/chay/new
python3 -m py_compile config.py robot_driver.py navigation_node.py mission_manager_node.py vision_color_node.py visual_approach_node.py arm_stage_calibrator.py pick1_sequence_node.py
```

Nếu sửa một script cụ thể, compile script đó tối thiểu. Nếu chạy robot thật, lưu
log timestamp trong `/home/pi/chay` và cập nhật tài liệu. Nếu không chạy robot
thật, báo rõ giới hạn kiểm chứng.
