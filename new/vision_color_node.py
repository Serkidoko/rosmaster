"""Detect a colored pick object from an image or camera frame.

This is a plain Python vision node first. It can be wrapped as a ROS node later.
Exit code is 0 when the target object is found, 2 when it is not found.
"""

import argparse
import time
from dataclasses import dataclass

import cv2
import numpy as np


# HSV ranges are tuned for the red/green/yellow cube tops in the pick images.
# The real red cube can look orange under glare, so red includes the orange-red
# hue band seen in /home/pi/chay/pick1_manual_red.jpg.
# Blue is included for completeness, but a blue object on the blue tray is not
# reliable with color thresholding alone.
COLOR_RANGES = {
    "red": (
        ((0, 55, 50), (24, 255, 255)),
        ((170, 55, 50), (180, 255, 255)),
    ),
    "green": (
        ((35, 40, 40), (85, 255, 255)),
    ),
    "yellow": (
        ((23, 20, 120), (45, 255, 255)),
    ),
    "blue": (
        ((95, 70, 40), (130, 255, 255)),
    ),
}

DEFAULT_ROI = (0.0, 0.25, 1.0, 0.55)


@dataclass
class Detection:
    color: str
    found: bool
    center_x: int = 0
    center_y: int = 0
    x: int = 0
    y: int = 0
    width: int = 0
    height: int = 0
    area: float = 0.0


def parse_roi(frame, roi):
    if roi is None:
        return frame, 0, 0

    frame_h, frame_w = frame.shape[:2]
    x_ratio, y_ratio, w_ratio, h_ratio = roi
    x = int(frame_w * x_ratio)
    y = int(frame_h * y_ratio)
    w = int(frame_w * w_ratio)
    h = int(frame_h * h_ratio)

    x = max(0, min(frame_w - 1, x))
    y = max(0, min(frame_h - 1, y))
    right = max(x + 1, min(frame_w, x + w))
    bottom = max(y + 1, min(frame_h, y + h))
    return frame[y:bottom, x:right], x, y


def build_mask(hsv, color):
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in COLOR_RANGES[color]:
        partial = cv2.inRange(hsv, np.array(lower), np.array(upper))
        mask = cv2.bitwise_or(mask, partial)

    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_color_object(frame, color, min_area=1200, roi=None):
    roi_frame, roi_x, roi_y = parse_roi(frame, roi)
    hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
    mask = build_mask(hsv, color)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        candidates.append((area, x, y, w, h))

    if not candidates:
        return Detection(color=color, found=False), mask

    area, x, y, w, h = max(candidates, key=lambda item: item[0])
    center_x = roi_x + x + w // 2
    center_y = roi_y + y + h // 2
    return (
        Detection(
            color=color,
            found=True,
            center_x=center_x,
            center_y=center_y,
            x=roi_x + x,
            y=roi_y + y,
            width=w,
            height=h,
            area=area,
        ),
        mask,
    )


def draw_detection(frame, detection, roi=None):
    output = frame.copy()
    if roi is not None:
        frame_h, frame_w = frame.shape[:2]
        x = int(frame_w * roi[0])
        y = int(frame_h * roi[1])
        w = int(frame_w * roi[2])
        h = int(frame_h * roi[3])
        cv2.rectangle(output, (x, y), (x + w, y + h), (255, 0, 0), 2)

    if detection.found:
        x1 = detection.x
        y1 = detection.y
        x2 = detection.x + detection.width
        y2 = detection.y + detection.height
        cv2.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 3)
        cv2.circle(output, (detection.center_x, detection.center_y), 6, (0, 0, 255), -1)
        label = f"{detection.color} area={detection.area:.0f}"
        cv2.putText(output, label, (x1, max(20, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return output


def print_detection(detection):
    if not detection.found:
        print(f"NOT_FOUND color={detection.color}")
        return

    print(
        "FOUND "
        f"color={detection.color} "
        f"center=({detection.center_x},{detection.center_y}) "
        f"bbox=({detection.x},{detection.y},{detection.width},{detection.height}) "
        f"area={detection.area:.1f}"
    )


def read_image(path):
    frame = cv2.imread(path)
    if frame is None:
        raise RuntimeError(f"Cannot read image: {path}")
    return frame


def detect_from_image(args):
    frame = read_image(args.image)
    detection, _ = detect_color_object(frame, args.color, args.min_area, args.roi)
    print_detection(detection)

    if args.save_debug:
        cv2.imwrite(args.save_debug, draw_detection(frame, detection, args.roi))
        print(f"[INFO] Debug image saved to {args.save_debug}")

    if args.show:
        cv2.imshow("vision_color_node", draw_detection(frame, detection, args.roi))
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return 0 if detection.found else 2


def detect_from_camera(args):
    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {args.camera_id}")

    best_detection = Detection(color=args.color, found=False)
    stable_count = 0
    start_time = time.time()
    frame_index = 0
    last_frame = None

    try:
        while time.time() - start_time < args.timeout_s:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError(f"Cannot read frame from camera {args.camera_id}")
            last_frame = frame

            frame_index += 1
            if frame_index % args.process_every != 0:
                continue

            detection, _ = detect_color_object(frame, args.color, args.min_area, args.roi)
            if detection.found:
                best_detection = detection
                stable_count += 1
                if stable_count >= args.required_frames:
                    break
            else:
                stable_count = 0

            if args.show:
                cv2.imshow("vision_color_node", draw_detection(frame, detection, args.roi))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        cap.release()
        if args.show:
            cv2.destroyAllWindows()

    print_detection(best_detection)
    if args.save_debug and last_frame is not None:
        cv2.imwrite(args.save_debug, draw_detection(last_frame, best_detection, args.roi))
        print(f"[INFO] Debug image saved to {args.save_debug}")

    return 0 if best_detection.found else 2


def main():
    parser = argparse.ArgumentParser(description="Detect a colored pick object.")
    parser.add_argument("--image", help="Detect from image file instead of camera.")
    parser.add_argument("--camera-id", type=int, default=2, help="Camera index.")
    parser.add_argument("--color", choices=sorted(COLOR_RANGES), required=True)
    parser.add_argument("--min-area", type=float, default=1200, help="Minimum contour area.")
    parser.add_argument(
        "--roi",
        nargs=4,
        type=float,
        default=DEFAULT_ROI,
        metavar=("X", "Y", "W", "H"),
        help="Normalized ROI. Default is tray area: 0.0 0.25 1.0 0.55",
    )
    parser.add_argument("--timeout-s", type=float, default=5.0, help="Camera detection timeout.")
    parser.add_argument("--required-frames", type=int, default=3, help="Stable detections required.")
    parser.add_argument("--process-every", type=int, default=1, help="Process every N frames.")
    parser.add_argument("--save-debug", help="Path to save annotated debug image.")
    parser.add_argument("--show", action="store_true", help="Show OpenCV preview window.")
    args = parser.parse_args()

    if args.min_area <= 0:
        parser.error("--min-area must be positive")
    if args.timeout_s <= 0:
        parser.error("--timeout-s must be positive")
    if args.required_frames <= 0:
        parser.error("--required-frames must be positive")
    if args.process_every <= 0:
        parser.error("--process-every must be positive")

    if args.image:
        return detect_from_image(args)
    return detect_from_camera(args)


if __name__ == "__main__":
    raise SystemExit(main())
