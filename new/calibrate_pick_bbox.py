"""Calibrate the ideal pick bbox from the current camera view.

Run this when the object is already at the ideal grasp position. The script
detects the object bbox, saves an annotated image, and writes a JSON config
that visual_approach_node.py can reuse as its gripper target box.
"""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import cv2

from vision_color_node import COLOR_RANGES, DEFAULT_ROI, detect_color_object


DEFAULT_OUTPUT = "pick1_gripper_box.json"
DEFAULT_DEBUG_IMAGE = "pick1_gripper_box_debug.jpg"
AUTO_COLORS = ("red", "green", "yellow")


def capture_frame(camera_id, warmup_frames):
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {camera_id}")

    frame = None
    try:
        for _ in range(warmup_frames):
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError(f"Cannot read frame from camera {camera_id}")
            time.sleep(0.03)

        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f"Cannot read frame from camera {camera_id}")
        return frame
    finally:
        cap.release()


def normalize_box(frame, detection, padding_ratio):
    frame_h, frame_w = frame.shape[:2]
    pad_x = int(detection.width * padding_ratio)
    pad_y = int(detection.height * padding_ratio)
    x = max(0, detection.x - pad_x)
    y = max(0, detection.y - pad_y)
    right = min(frame_w, detection.x + detection.width + pad_x)
    bottom = min(frame_h, detection.y + detection.height + pad_y)
    return (
        x / frame_w,
        y / frame_h,
        (right - x) / frame_w,
        (bottom - y) / frame_h,
    )


def draw_calibration(frame, detection, gripper_box, roi):
    output = frame.copy()
    frame_h, frame_w = frame.shape[:2]

    if roi is not None:
        rx = int(frame_w * roi[0])
        ry = int(frame_h * roi[1])
        rw = int(frame_w * roi[2])
        rh = int(frame_h * roi[3])
        cv2.rectangle(output, (rx, ry), (rx + rw, ry + rh), (255, 0, 0), 2)

    cv2.rectangle(
        output,
        (detection.x, detection.y),
        (detection.x + detection.width, detection.y + detection.height),
        (0, 255, 0),
        3,
    )
    cv2.circle(output, (detection.center_x, detection.center_y), 6, (0, 0, 255), -1)

    gx = int(frame_w * gripper_box[0])
    gy = int(frame_h * gripper_box[1])
    gw = int(frame_w * gripper_box[2])
    gh = int(frame_h * gripper_box[3])
    cv2.rectangle(output, (gx, gy), (gx + gw, gy + gh), (255, 0, 255), 3)
    cv2.circle(output, (gx + gw // 2, gy + gh // 2), 6, (255, 0, 255), -1)

    label = f"calibrated {detection.color} bbox area={detection.area:.0f}"
    cv2.putText(
        output,
        label,
        (20, 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
    )
    return output


def detect_best(frame, color, min_area, roi):
    if color != "auto":
        detection, _ = detect_color_object(frame, color, min_area=min_area, roi=roi)
        return detection

    detections = []
    for candidate_color in AUTO_COLORS:
        detection, _ = detect_color_object(
            frame,
            candidate_color,
            min_area=min_area,
            roi=roi,
        )
        if detection.found:
            detections.append(detection)

    if not detections:
        return None
    return max(detections, key=lambda item: item.area)


def write_config(path, data):
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as file:
        json.dump(data, file, indent=2)
        file.write("\n")


def main():
    parser = argparse.ArgumentParser(
        description="Capture ideal pick bbox and save visual approach config."
    )
    parser.add_argument("--camera-id", type=int, default=0)
    parser.add_argument("--warmup-frames", type=int, default=20)
    parser.add_argument(
        "--color",
        choices=("auto", *sorted(COLOR_RANGES)),
        default="auto",
        help="Target object color. auto selects the largest detected color bbox.",
    )
    parser.add_argument("--min-area", type=float, default=200.0)
    parser.add_argument(
        "--roi",
        nargs=4,
        type=float,
        default=(0.0, 0.0, 1.0, 0.8),
        metavar=("X", "Y", "W", "H"),
        help="Normalized ROI for calibration.",
    )
    parser.add_argument("--padding-ratio", type=float, default=0.10)
    parser.add_argument("--area-ratio", type=float, default=0.80)
    parser.add_argument("--height-ratio", type=float, default=0.80)
    parser.add_argument("--center-tolerance-ratio", type=float, default=0.08)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--save-debug", default=DEFAULT_DEBUG_IMAGE)
    parser.add_argument("--raw-image", default="pick1_gripper_box_raw.jpg")
    args = parser.parse_args()

    if args.warmup_frames < 0:
        parser.error("--warmup-frames must be zero or positive")
    if args.min_area <= 0:
        parser.error("--min-area must be positive")
    if args.padding_ratio < 0:
        parser.error("--padding-ratio must be zero or positive")
    if args.area_ratio <= 0:
        parser.error("--area-ratio must be positive")
    if args.height_ratio <= 0:
        parser.error("--height-ratio must be positive")
    if args.center_tolerance_ratio <= 0:
        parser.error("--center-tolerance-ratio must be positive")

    frame = capture_frame(args.camera_id, args.warmup_frames)
    if args.raw_image:
        cv2.imwrite(args.raw_image, frame)
        print(f"[INFO] Raw image saved to {args.raw_image}")

    detection = detect_best(frame, args.color, args.min_area, args.roi)
    if detection is None or not detection.found:
        print("[ERROR] No object bbox found. Try --color or lower --min-area.")
        return 2

    gripper_box = normalize_box(frame, detection, args.padding_ratio)
    config = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "camera_id": args.camera_id,
        "color": detection.color,
        "frame_size": {
            "width": frame.shape[1],
            "height": frame.shape[0],
        },
        "roi": list(args.roi),
        "object_bbox_px": {
            "x": detection.x,
            "y": detection.y,
            "width": detection.width,
            "height": detection.height,
            "center_x": detection.center_x,
            "center_y": detection.center_y,
            "area": detection.area,
        },
        "gripper_box": [round(value, 6) for value in gripper_box],
        "ready_min_area": round(detection.area * args.area_ratio, 2),
        "ready_min_height": max(1, int(detection.height * args.height_ratio)),
        "center_tolerance_ratio": args.center_tolerance_ratio,
    }
    write_config(args.output, config)

    debug = draw_calibration(frame, detection, gripper_box, args.roi)
    cv2.imwrite(args.save_debug, debug)

    print(
        "CALIBRATED "
        f"color={detection.color} "
        f"bbox=({detection.x},{detection.y},{detection.width},{detection.height}) "
        f"center=({detection.center_x},{detection.center_y}) "
        f"area={detection.area:.1f}"
    )
    print(f"GRIPPER_BOX {config['gripper_box']}")
    print(f"READY_MIN_AREA {config['ready_min_area']}")
    print(f"READY_MIN_HEIGHT {config['ready_min_height']}")
    print(f"[INFO] Config saved to {args.output}")
    print(f"[INFO] Debug image saved to {args.save_debug}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
