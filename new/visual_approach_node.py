"""Approach a pick object using its bounding box in the camera image.

Default mode is dry-run. Add --run only after the gripper target box and
distance thresholds have been checked on the real camera.
"""

import argparse
import json
import os
import time
from dataclasses import dataclass
from pathlib import Path

import cv2

from config import (
    DEFAULT_ALIGN_LATERAL_SPEED_MPS,
    DEFAULT_FORWARD_SPEED,
    MIN_ALIGN_SIDE_SPEED_MPS,
)
from robot_driver import RobotDriver
from vision_color_node import (
    DEFAULT_ROI,
    COLOR_RANGES,
    detect_color_object,
    draw_detection,
    read_image,
)


DEFAULT_GRIPPER_BOX = (0.44, 0.34, 0.12, 0.14)
PICK1_LEFT_ROI = (0.0, 0.15, 0.65, 0.6)


@dataclass
class ApproachEstimate:
    ready: bool
    action: str
    center_error_x: int
    center_error_y: int
    area: float
    bbox_height: int
    message: str


@dataclass
class SlotScanEstimate:
    action: str
    current_detection: object
    target_detection: object
    detections: tuple
    slot_error_x: int
    slot_error_y: int
    message: str


def pixel_box(frame, box):
    frame_h, frame_w = frame.shape[:2]
    x = int(frame_w * box[0])
    y = int(frame_h * box[1])
    w = int(frame_w * box[2])
    h = int(frame_h * box[3])
    return x, y, w, h


def estimate_approach(
    frame,
    detection,
    gripper_box,
    ready_min_area,
    ready_min_height,
    center_tolerance_ratio,
):
    if detection is None or not detection.found:
        return ApproachEstimate(
            ready=False,
            action="not_found",
            center_error_x=0,
            center_error_y=0,
            area=0.0,
            bbox_height=0,
            message="object not found",
        )

    frame_h, frame_w = frame.shape[:2]
    gx, gy, gw, gh = pixel_box(frame, gripper_box)
    gripper_center_x = gx + gw // 2
    gripper_center_y = gy + gh // 2
    error_x = detection.center_x - gripper_center_x
    error_y = detection.center_y - gripper_center_y
    tolerance_x_px = int(frame_w * center_tolerance_ratio)
    tolerance_y_px = int(frame_h * center_tolerance_ratio)

    close_enough = (
        detection.area >= ready_min_area
        and detection.height >= ready_min_height
    )
    aligned_x = abs(error_x) <= tolerance_x_px
    aligned_y = abs(error_y) <= tolerance_y_px

    if aligned_x and aligned_y and close_enough:
        return ApproachEstimate(
            ready=True,
            action="ready",
            center_error_x=error_x,
            center_error_y=error_y,
            area=detection.area,
            bbox_height=detection.height,
            message="object is close enough and aligned with gripper target box",
        )

    # Cardinal movement only: correct horizontal error first, then vertical /
    # distance error. This avoids diagonal commands while still steering into
    # the target bbox quadrant by quadrant.
    if not aligned_x:
        side = "right" if error_x > 0 else "left"
        return ApproachEstimate(
            ready=False,
            action=f"move_{side}",
            center_error_x=error_x,
            center_error_y=error_y,
            area=detection.area,
            bbox_height=detection.height,
            message=(
                f"object is {'right' if error_x > 0 else 'left'} of the "
                "target vertical line; move sideways first"
            ),
        )

    if not aligned_y:
        direction = "forward" if error_y < 0 else "backward"
        return ApproachEstimate(
            ready=False,
            action=f"move_{direction}",
            center_error_x=error_x,
            center_error_y=error_y,
            area=detection.area,
            bbox_height=detection.height,
            message=(
                f"object is {'above' if error_y < 0 else 'below'} the target "
                f"horizontal line; move {direction}"
            ),
        )

    if not close_enough:
        return ApproachEstimate(
            ready=False,
            action="move_forward",
            center_error_x=error_x,
            center_error_y=error_y,
            area=detection.area,
            bbox_height=detection.height,
            message="object is centered but still small; move forward",
        )

    return ApproachEstimate(
        ready=False,
        action="move_backward",
        center_error_x=error_x,
        center_error_y=error_y,
        area=detection.area,
        bbox_height=detection.height,
        message="object is centered but bbox is not usable; move backward and recheck",
    )


def draw_gripper_box(frame, gripper_box):
    output = frame.copy()
    frame_h, frame_w = output.shape[:2]
    x, y, w, h = pixel_box(output, gripper_box)
    center_x = x + w // 2
    center_y = y + h // 2
    cv2.rectangle(output, (x, y), (x + w, y + h), (255, 0, 255), 3)
    cv2.line(output, (center_x, 0), (center_x, frame_h - 1), (255, 0, 255), 1)
    cv2.line(output, (0, center_y), (frame_w - 1, center_y), (255, 0, 255), 1)
    cv2.circle(output, (center_x, center_y), 6, (255, 0, 255), -1)
    cv2.putText(
        output,
        "gripper target",
        (x, max(20, y - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 0, 255),
        2,
    )
    return output


def draw_all_detections(frame, detections, roi=None):
    output = frame.copy()
    if roi is not None:
        frame_h, frame_w = frame.shape[:2]
        x = int(frame_w * roi[0])
        y = int(frame_h * roi[1])
        w = int(frame_w * roi[2])
        h = int(frame_h * roi[3])
        cv2.rectangle(output, (x, y), (x + w, y + h), (255, 0, 0), 2)

    colors = {
        "red": (0, 0, 255),
        "green": (0, 255, 0),
        "yellow": (0, 255, 255),
        "blue": (255, 0, 0),
    }
    for detection in detections:
        if not detection.found:
            continue
        color = colors.get(detection.color, (255, 255, 255))
        x1 = detection.x
        y1 = detection.y
        x2 = detection.x + detection.width
        y2 = detection.y + detection.height
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
        cv2.circle(output, (detection.center_x, detection.center_y), 5, color, -1)
        label = f"{detection.color} {detection.area:.0f}"
        cv2.putText(
            output,
            label,
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )
    return output


def draw_debug(frame, detection, estimate, roi, gripper_box, detections=None):
    if detections is not None:
        output = draw_all_detections(frame, detections, roi)
    elif detection is not None:
        output = draw_detection(frame, detection, roi)
    else:
        output = draw_all_detections(frame, (), roi)
    output = draw_gripper_box(output, gripper_box)
    cv2.putText(
        output,
        f"{estimate.action}: dx={estimate.center_error_x} area={estimate.area:.0f}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 255, 255),
        2,
    )
    return output


def resize_for_display(frame, scale):
    if scale == 1.0:
        return frame
    width = max(1, int(frame.shape[1] * scale))
    height = max(1, int(frame.shape[0] * scale))
    return cv2.resize(frame, (width, height))


def show_debug_window(args, frame, detection, estimate):
    detections = getattr(args, "last_scan_detections", None)
    debug_frame = draw_debug(
        frame,
        detection,
        estimate,
        args.roi,
        args.gripper_box,
        detections=detections,
    )
    debug_frame = resize_for_display(debug_frame, args.display_scale)
    cv2.imshow("visual_approach_node", debug_frame)

    if args.pause_on_step:
        key = cv2.waitKey(0)
    else:
        key = cv2.waitKey(args.preview_ms)
    return key & 0xFF != ord("q")


def print_estimate(detection, estimate):
    if detection is not None and detection.found:
        print(
            "BBOX "
            f"center=({detection.center_x},{detection.center_y}) "
            f"size=({detection.width},{detection.height}) "
            f"area={detection.area:.1f}"
        )
    print(
        "APPROACH "
        f"action={estimate.action} "
        f"ready={int(estimate.ready)} "
        f"dx={estimate.center_error_x} "
        f"dy={estimate.center_error_y} "
        f"height={estimate.bbox_height} "
        f"area={estimate.area:.1f} "
        f"message=\"{estimate.message}\""
    )


def print_slot_scan(slot_estimate):
    current = slot_estimate.current_detection
    if current is not None and current.found:
        print(
            "SLOT "
            f"action={slot_estimate.action} "
            f"current_color={current.color} "
            f"center=({current.center_x},{current.center_y}) "
            f"dx={slot_estimate.slot_error_x} "
            f"dy={slot_estimate.slot_error_y} "
            f"message=\"{slot_estimate.message}\""
        )
    else:
        colors = ",".join(
            detection.color for detection in slot_estimate.detections if detection.found
        )
        print(
            "SLOT "
            f"action={slot_estimate.action} "
            f"current_color=none "
            f"seen_colors={colors or 'none'} "
            f"message=\"{slot_estimate.message}\""
        )


def load_gripper_config(args):
    if not args.gripper_config:
        return

    config_path = resolve_input_path(args.gripper_config)
    with config_path.open("r", encoding="utf-8") as file:
        config = json.load(file)

    if "gripper_box" in config:
        args.gripper_box = tuple(config["gripper_box"])
    if "roi" in config:
        args.roi = tuple(config["roi"])
    if "color" in config and args.color is None:
        args.color = config["color"]
    if "ready_min_area" in config:
        args.ready_min_area = float(config["ready_min_area"])
    if "ready_min_height" in config:
        args.ready_min_height = int(config["ready_min_height"])
    if "center_tolerance_ratio" in config:
        args.center_tolerance_ratio = float(config["center_tolerance_ratio"])
    print(f"[INFO] Loaded gripper config from {config_path}")


def resolve_input_path(path):
    input_path = Path(path)
    if input_path.is_absolute() or input_path.exists():
        return input_path

    script_dir = Path(__file__).resolve().parent
    for base_dir in (script_dir.parent, script_dir):
        candidate = base_dir / input_path
        if candidate.exists():
            return candidate

    return input_path


def display_available():
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def disable_show_if_headless(args):
    if args.show and not display_available():
        print(
            "[WARN] --show disabled: no DISPLAY/WAYLAND_DISPLAY is available. "
            "Run from the Pi desktop/VNC, use SSH X forwarding, or omit --show."
        )
        args.show = False


def clamp(value, low, high):
    return max(low, min(high, value))


def normalized_error(error_px, frame_size_px, tolerance_ratio):
    deadband_px = frame_size_px * tolerance_ratio
    if abs(error_px) <= deadband_px:
        return 0.0

    usable_px = max(1.0, frame_size_px / 2.0 - deadband_px)
    signed_px = abs(error_px) - deadband_px
    return clamp(signed_px / usable_px, 0.0, 1.0) * (1.0 if error_px > 0 else -1.0)


def target_center(frame, gripper_box):
    gx, gy, gw, gh = pixel_box(frame, gripper_box)
    return gx + gw // 2, gy + gh // 2


def detect_scan_colors(frame, args):
    detections = []
    seen_colors = set()
    scan_colors = list(args.scan_colors)
    if args.color not in seen_colors and args.color not in scan_colors:
        scan_colors.append(args.color)
    for color in scan_colors:
        if color in seen_colors:
            continue
        seen_colors.add(color)
        detection, _ = detect_color_object(frame, color, args.min_area, args.roi)
        if detection.found:
            detections.append(detection)
    return tuple(detections)


def select_current_slot_detection(frame, args, detections):
    if not detections:
        return None, 0, 0

    target_x, target_y = target_center(frame, args.gripper_box)
    frame_h, frame_w = frame.shape[:2]
    tolerance_x = int(frame_w * args.slot_center_tolerance_ratio)
    tolerance_y = int(frame_h * args.slot_center_tolerance_ratio)

    scored = []
    for detection in detections:
        error_x = detection.center_x - target_x
        error_y = detection.center_y - target_y
        distance_sq = error_x * error_x + error_y * error_y
        scored.append((distance_sq, error_x, error_y, detection))
    _, error_x, error_y, current = min(scored, key=lambda item: item[0])

    if abs(error_x) > tolerance_x or abs(error_y) > tolerance_y:
        return None, error_x, error_y
    return current, error_x, error_y


def estimate_slot_scan(args, frame):
    detections = detect_scan_colors(frame, args)
    current, error_x, error_y = select_current_slot_detection(frame, args, detections)
    if current is None:
        return SlotScanEstimate(
            action="slot_search_right",
            current_detection=None,
            target_detection=None,
            detections=detections,
            slot_error_x=error_x,
            slot_error_y=error_y,
            message="no object is centered in the current gripper slot; scan arm-right",
        )

    if current.color != args.color:
        return SlotScanEstimate(
            action="wrong_color_scan_right",
            current_detection=current,
            target_detection=None,
            detections=detections,
            slot_error_x=error_x,
            slot_error_y=error_y,
            message=(
                f"current slot is {current.color}, target is {args.color}; "
                "scan arm-right to the next object"
            ),
        )

    return SlotScanEstimate(
        action="target_found",
        current_detection=current,
        target_detection=current,
        detections=detections,
        slot_error_x=error_x,
        slot_error_y=error_y,
        message=f"current slot color matches target {args.color}",
    )


def cardinal_side_speed(args, estimate, frame):
    frame_w = frame.shape[1]
    x_error = normalized_error(
        estimate.center_error_x,
        frame_w,
        args.center_tolerance_ratio,
    )
    if estimate.action == "move_left":
        x_error = min(x_error, -1e-6)
    elif estimate.action == "move_right":
        x_error = max(x_error, 1e-6)

    side_speed = x_error * args.align_lateral_speed_mps
    if args.invert_lateral:
        side_speed = -side_speed

    if abs(side_speed) < args.min_align_side_speed_mps:
        sign = 1.0 if side_speed >= 0 else -1.0
        side_speed = sign * args.min_align_side_speed_mps
    return side_speed


def compute_arm_relative_motion(args, estimate, frame):
    forward_speed = 0.0
    side_speed = 0.0

    if estimate.action == "move_forward":
        forward_speed = args.approach_forward_speed_mps
    elif estimate.action == "move_backward":
        forward_speed = -args.search_speed_mps
    elif estimate.action in ("move_left", "move_right"):
        side_speed = cardinal_side_speed(args, estimate, frame)
    elif estimate.action == "search":
        forward_speed, side_speed = compute_search_motion(args)

    return forward_speed, side_speed


def compute_search_motion(args):
    if args.search_direction == "none":
        return 0.0, 0.0
    if args.search_direction == "forward":
        return args.search_speed_mps, 0.0
    if args.search_direction == "backward":
        return -args.search_speed_mps, 0.0
    if args.search_direction == "left":
        return 0.0, -args.min_align_side_speed_mps
    if args.search_direction == "right":
        return 0.0, args.min_align_side_speed_mps
    raise ValueError(f"Unknown search direction: {args.search_direction}")


def command_slot_scan_motion(driver, args, slot_estimate):
    if driver is None:
        raise RuntimeError("Cannot command slot scan without RobotDriver")
    validate_cardinal_arm_angle(args)
    print(
        f"[INFO] arm-right slot scan: "
        f"k1={args.arm_k1_angle} "
        f"action={slot_estimate.action} "
        f"forward=0.00 "
        f"side={args.arm_right_scan_speed_mps:.2f}"
    )
    driver.set_arm_relative_motion(
        args.arm_k1_angle,
        forward_speed_mps=0.0,
        side_speed_mps=args.arm_right_scan_speed_mps,
    )


def validate_cardinal_arm_angle(args):
    if args.arm_k1_angle not in (0, 90, 180):
        raise ValueError(
            "Cardinal approach requires arm_k1_angle to be 0, 90, or 180 "
            "so robot motion cannot become diagonal."
        )


def command_approach_motion(driver, args, estimate, frame):
    if driver is None:
        raise RuntimeError("Cannot command approach motion without RobotDriver")

    validate_cardinal_arm_angle(args)
    forward_speed, side_speed = compute_arm_relative_motion(args, estimate, frame)
    if abs(forward_speed) > 1e-6 and abs(side_speed) > 1e-6:
        raise RuntimeError("Cardinal approach violated: diagonal motion requested")
    print(
        f"[INFO] arm-relative approach: "
        f"k1={args.arm_k1_angle} "
        f"action={estimate.action} "
        f"forward={forward_speed:.2f} "
        f"side={side_speed:.2f} "
        f"dx={estimate.center_error_x} "
        f"dy={estimate.center_error_y}"
    )
    driver.set_arm_relative_motion(
        args.arm_k1_angle,
        forward_speed_mps=forward_speed,
        side_speed_mps=side_speed,
    )


def estimate_frame(args, frame):
    detection, _ = detect_color_object(frame, args.color, args.min_area, args.roi)
    estimate = estimate_approach(
        frame=frame,
        detection=detection,
        gripper_box=args.gripper_box,
        ready_min_area=args.ready_min_area,
        ready_min_height=args.ready_min_height,
        center_tolerance_ratio=args.center_tolerance_ratio,
    )
    return detection, estimate


def estimate_frame_after_slot_scan(args, frame):
    slot_estimate = estimate_slot_scan(args, frame)
    args.last_scan_detections = slot_estimate.detections

    if slot_estimate.action != "target_found":
        current = slot_estimate.current_detection
        if current is None:
            area = 0.0
            height = 0
        else:
            area = current.area
            height = current.height
        estimate = ApproachEstimate(
            ready=False,
            action=slot_estimate.action,
            center_error_x=slot_estimate.slot_error_x,
            center_error_y=slot_estimate.slot_error_y,
            area=area,
            bbox_height=height,
            message=slot_estimate.message,
        )
        return current, estimate, slot_estimate

    detection = slot_estimate.target_detection
    estimate = estimate_approach(
        frame=frame,
        detection=detection,
        gripper_box=args.gripper_box,
        ready_min_area=args.ready_min_area,
        ready_min_height=args.ready_min_height,
        center_tolerance_ratio=args.center_tolerance_ratio,
    )
    return detection, estimate, slot_estimate


def run_image(args):
    frame = read_image(args.image)
    detection, estimate, slot_estimate = estimate_frame_after_slot_scan(args, frame)
    print_slot_scan(slot_estimate)
    print_estimate(detection, estimate)

    if args.save_debug:
        cv2.imwrite(
            args.save_debug,
            draw_debug(
                frame,
                detection,
                estimate,
                args.roi,
                args.gripper_box,
                detections=slot_estimate.detections,
            ),
        )
        print(f"[INFO] Debug image saved to {args.save_debug}")

    if args.show:
        try:
            show_debug_window(args, frame, detection, estimate)
        finally:
            cv2.destroyAllWindows()

    if estimate.ready:
        return 0
    if estimate.action == "not_found":
        return 2
    if estimate.action in ("wrong_color_scan_right", "slot_search_right"):
        return 4
    if estimate.action == "not_aligned":
        return 3
    return 4


def read_camera_frame(cap, camera_id):
    ok, frame = cap.read()
    if not ok:
        raise RuntimeError(f"Cannot read frame from camera {camera_id}")
    return frame


def run_camera(args, driver=None):
    if args.continuous and args.run:
        print("[WARN] --continuous is preview-only; movement is disabled.")
    if args.continuous:
        driver = None
    elif driver is None:
        driver = RobotDriver(dry_run=not args.run, debug=args.debug)
    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {args.camera_id}")

    last_frame = None
    last_detection = None
    last_estimate = None
    last_scan_estimate = None
    last_found_estimate = None
    last_found_frame = None
    lost_frames = 0
    wrong_slot_count = 0
    pending_wrong_slot_color = None
    wrong_slot_streak = 0
    slot_had_gap_since_count = True

    try:
        for _ in range(args.warmup_frames):
            read_camera_frame(cap, args.camera_id)
            time.sleep(0.03)

        frame_index = 0
        approach_start_time = time.time()
        scan_start_time = time.time()
        while True:
            frame = read_camera_frame(cap, args.camera_id)
            last_frame = frame
            detection, estimate, slot_estimate = estimate_frame_after_slot_scan(args, frame)
            last_detection = detection
            last_estimate = estimate
            last_scan_estimate = slot_estimate
            if detection is not None and detection.found:
                last_found_estimate = estimate
                last_found_frame = frame
                lost_frames = 0
            elapsed_s = time.time() - approach_start_time

            if not args.continuous or frame_index % args.print_every == 0:
                if args.continuous:
                    print(f"[INFO] approach_frame={frame_index}")
                else:
                    print(
                        f"[INFO] approach_time={elapsed_s:.1f}/"
                        f"{args.max_approach_s:.1f}s"
                    )
                print_slot_scan(slot_estimate)
                print_estimate(detection, estimate)

            if args.show and not show_debug_window(args, frame, detection, estimate):
                return 130

            if args.continuous:
                frame_index += 1
                if not args.show:
                    time.sleep(args.continuous_delay_s)
                continue

            if estimate.ready:
                return 0
            if estimate.action in ("wrong_color_scan_right", "slot_search_right"):
                if elapsed_s >= args.max_approach_s:
                    return 4
                if time.time() - scan_start_time >= args.scan_timeout_s:
                    return 2
                if estimate.action == "wrong_color_scan_right":
                    current_color = slot_estimate.current_detection.color
                    if current_color == pending_wrong_slot_color:
                        wrong_slot_streak += 1
                    else:
                        pending_wrong_slot_color = current_color
                        wrong_slot_streak = 1

                    if (
                        slot_had_gap_since_count
                        and wrong_slot_streak >= args.slot_stable_frames
                    ):
                        wrong_slot_count += 1
                        slot_had_gap_since_count = False
                        print(
                            f"[INFO] wrong_slot_count="
                            f"{wrong_slot_count}/{args.max_scan_objects} "
                            f"color={current_color} "
                            f"stable_frames={wrong_slot_streak}"
                        )
                    if wrong_slot_count >= args.max_scan_objects:
                        print("[WARN] Max scan objects reached without target color.")
                        return 2
                else:
                    pending_wrong_slot_color = None
                    wrong_slot_streak = 0
                    slot_had_gap_since_count = True
                command_slot_scan_motion(driver, args, slot_estimate)
                time.sleep(args.approach_control_s)
                frame_index += 1
                continue
            if estimate.action == "not_found":
                lost_frames += 1
                if elapsed_s >= args.max_approach_s:
                    return 2
                if last_found_estimate is not None and lost_frames <= args.lost_frame_grace:
                    print(
                        "[WARN] Object briefly lost; continuing from last bbox "
                        f"dx={last_found_estimate.center_error_x} "
                        f"lost_frame={lost_frames}/{args.lost_frame_grace}"
                    )
                    command_approach_motion(
                        driver,
                        args,
                        last_found_estimate,
                        last_found_frame,
                    )
                elif args.search_direction == "none":
                    return 2
                else:
                    search_estimate = ApproachEstimate(
                        ready=False,
                        action="search",
                        center_error_x=0,
                        center_error_y=0,
                        area=0.0,
                        bbox_height=0,
                        message=f"object not found; searching {args.search_direction}",
                    )
                    command_approach_motion(driver, args, search_estimate, frame)
                time.sleep(args.approach_control_s)
                frame_index += 1
                continue
            if elapsed_s >= args.max_approach_s:
                if estimate.action == "not_aligned":
                    return 3
                return 4

            command_approach_motion(
                driver,
                args,
                estimate,
                frame,
            )
            time.sleep(args.approach_control_s)
            frame_index += 1
    except KeyboardInterrupt:
        print("[INFO] Visual approach interrupted.")
        return 130
    finally:
        if driver is not None:
            driver.stop()
        cap.release()
        if args.show:
            cv2.destroyAllWindows()
        if args.save_debug and last_frame is not None and last_estimate is not None:
            detections = ()
            if last_scan_estimate is not None:
                detections = last_scan_estimate.detections
            cv2.imwrite(
                args.save_debug,
                draw_debug(
                    last_frame,
                    last_detection,
                    last_estimate,
                    args.roi,
                    args.gripper_box,
                    detections=detections,
                ),
            )
            print(f"[INFO] Debug image saved to {args.save_debug}")


def main():
    parser = argparse.ArgumentParser(
        description="Use object bbox to decide whether to move closer before picking."
    )
    parser.add_argument("--image", help="Estimate from image file instead of camera.")
    parser.add_argument("--camera-id", type=int, default=0, help="Camera index.")
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=10,
        help="Discard initial camera frames before detection.",
    )
    parser.add_argument("--color", choices=sorted(COLOR_RANGES))
    parser.add_argument("--min-area", type=float, default=1200, help="Minimum object contour area.")
    parser.add_argument(
        "--roi",
        nargs=4,
        type=float,
        default=PICK1_LEFT_ROI,
        metavar=("X", "Y", "W", "H"),
        help="Normalized object search ROI. Default is the left-side pick_1 tray area.",
    )
    parser.add_argument(
        "--gripper-box",
        nargs=4,
        type=float,
        default=DEFAULT_GRIPPER_BOX,
        metavar=("X", "Y", "W", "H"),
        help="Normalized fixed gripper target box in image.",
    )
    parser.add_argument(
        "--gripper-config",
        help="JSON output from calibrate_pick_bbox.py.",
    )
    parser.add_argument("--ready-min-area", type=float, default=18000.0)
    parser.add_argument("--ready-min-height", type=int, default=100)
    parser.add_argument("--center-tolerance-ratio", type=float, default=0.08)
    parser.add_argument(
        "--max-approach-s",
        type=float,
        default=12.0,
        help="Safety timeout for visual approach; robot stops when bbox is ready.",
    )
    parser.add_argument(
        "--approach-control-s",
        type=float,
        default=0.15,
        help="Seconds between visual approach velocity updates.",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=4,
        help="Deprecated; visual approach now stops by ready bbox or timeout.",
    )
    parser.add_argument(
        "--step-distance-m",
        type=float,
        default=0.05,
        help="Deprecated; visual approach now uses velocity until bbox is ready.",
    )
    parser.add_argument("--speed", type=float, default=DEFAULT_FORWARD_SPEED)
    parser.add_argument(
        "--approach-forward-speed-mps",
        type=float,
        default=0.12,
        help="Arm-forward velocity. K1=0 maps this to robot-left.",
    )
    parser.add_argument(
        "--arm-k1-angle",
        type=int,
        default=0,
        help="Arm joint 1 angle: 0=robot-left, 90=robot-forward, 180=robot-right.",
    )
    parser.add_argument(
        "--lateral-speed-mps",
        type=float,
        default=0.22,
        help="Deprecated alias kept for old commands; use --align-lateral-speed-mps.",
    )
    parser.add_argument(
        "--align-lateral-speed-mps",
        type=float,
        default=DEFAULT_ALIGN_LATERAL_SPEED_MPS,
        help="Maximum arm-side correction speed used to center the bbox.",
    )
    parser.add_argument(
        "--min-align-side-speed-mps",
        type=float,
        default=MIN_ALIGN_SIDE_SPEED_MPS,
        help="Minimum nonzero arm-side correction speed after bbox deadband.",
    )
    parser.add_argument(
        "--invert-lateral",
        action="store_true",
        help="Swap left/right if the real robot strafes opposite to the command.",
    )
    parser.add_argument(
        "--disable-auto-invert-lateral",
        action="store_true",
        help="Deprecated; arm-relative vector control does not auto-invert.",
    )
    parser.add_argument(
        "--align-progress-px",
        type=int,
        default=8,
        help="Minimum dx improvement in pixels before lateral motion is considered useful.",
    )
    parser.add_argument(
        "--align-no-progress-frames",
        type=int,
        default=6,
        help="Number of moving frames without dx improvement before auto-invert.",
    )
    parser.add_argument(
        "--auto-invert-min-dx",
        type=int,
        default=120,
        help="Only auto-invert lateral direction while absolute dx is at least this many pixels.",
    )
    parser.add_argument(
        "--lost-frame-grace",
        type=int,
        default=4,
        help="Continue from the last bbox for this many frames after a brief detection loss.",
    )
    parser.add_argument(
        "--scan-colors",
        nargs="+",
        choices=sorted(COLOR_RANGES),
        default=("red", "green", "yellow"),
        help="Colors to inspect while scanning the 3-object row.",
    )
    parser.add_argument(
        "--max-scan-objects",
        type=int,
        default=3,
        help="Maximum wrong-color slots to inspect before failing.",
    )
    parser.add_argument(
        "--arm-right-scan-speed-mps",
        type=float,
        default=0.08,
        help="Arm-right speed used to move from one object slot to the next.",
    )
    parser.add_argument(
        "--slot-center-tolerance-ratio",
        type=float,
        default=0.12,
        help="Normalized tolerance for deciding which object is in the current slot.",
    )
    parser.add_argument(
        "--slot-stable-frames",
        type=int,
        default=4,
        help="Consecutive wrong-color frames required before counting one object slot.",
    )
    parser.add_argument(
        "--scan-timeout-s",
        type=float,
        default=10.0,
        help="Safety timeout for wrong-color row scanning.",
    )
    parser.add_argument(
        "--search-direction",
        choices=("forward", "backward", "left", "right", "none"),
        default="forward",
        help="Cardinal arm-frame direction to move when the object is not found.",
    )
    parser.add_argument(
        "--search-speed-mps",
        type=float,
        default=0.08,
        help="Arm-forward/backward search speed when no object is visible.",
    )
    parser.add_argument("--settle-s", type=float, default=0.4)
    parser.add_argument("--save-debug", help="Path to save annotated debug image.")
    parser.add_argument("--show", action="store_true", help="Show live detection preview window.")
    parser.add_argument(
        "--continuous",
        action="store_true",
        help="Keep detecting until q/Ctrl-C; preview-only, no robot movement.",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=10,
        help="Print every N frames in --continuous mode.",
    )
    parser.add_argument(
        "--continuous-delay-s",
        type=float,
        default=0.05,
        help="Delay between frames when --continuous runs without --show.",
    )
    parser.add_argument(
        "--pause-on-step",
        action="store_true",
        help="Pause preview after each detection step until a key is pressed.",
    )
    parser.add_argument(
        "--preview-ms",
        type=int,
        default=1,
        help="OpenCV preview wait time in milliseconds when not using --pause-on-step.",
    )
    parser.add_argument(
        "--display-scale",
        type=float,
        default=0.6,
        help="Scale preview window to fit the screen.",
    )
    parser.add_argument("--debug", action="store_true", help="Print wheel sync data.")
    parser.add_argument(
        "--run",
        action="store_true",
        help="Move the real robot. Without this flag movement is dry-run.",
    )
    args = parser.parse_args()
    load_gripper_config(args)
    disable_show_if_headless(args)

    if args.color is None:
        parser.error("--color is required when --gripper-config has no color")
    if args.min_area <= 0:
        parser.error("--min-area must be positive")
    if args.ready_min_area <= 0:
        parser.error("--ready-min-area must be positive")
    if args.ready_min_height <= 0:
        parser.error("--ready-min-height must be positive")
    if args.center_tolerance_ratio <= 0:
        parser.error("--center-tolerance-ratio must be positive")
    if args.max_approach_s <= 0:
        parser.error("--max-approach-s must be positive")
    if args.approach_control_s <= 0:
        parser.error("--approach-control-s must be positive")
    if args.max_steps < 0:
        parser.error("--max-steps must be zero or positive")
    if args.step_distance_m <= 0:
        parser.error("--step-distance-m must be positive")
    if args.approach_forward_speed_mps <= 0:
        parser.error("--approach-forward-speed-mps must be positive")
    if args.arm_k1_angle < 0 or args.arm_k1_angle > 180:
        parser.error("--arm-k1-angle must be in range 0..180")
    if args.arm_k1_angle not in (0, 90, 180):
        parser.error("--arm-k1-angle must be 0, 90, or 180 for cardinal motion")
    if args.lateral_speed_mps <= 0:
        parser.error("--lateral-speed-mps must be positive")
    if args.align_lateral_speed_mps <= 0:
        parser.error("--align-lateral-speed-mps must be positive")
    if args.min_align_side_speed_mps <= 0:
        parser.error("--min-align-side-speed-mps must be positive")
    if args.align_progress_px < 0:
        parser.error("--align-progress-px must be zero or positive")
    if args.align_no_progress_frames <= 0:
        parser.error("--align-no-progress-frames must be positive")
    if args.auto_invert_min_dx < 0:
        parser.error("--auto-invert-min-dx must be zero or positive")
    if args.lost_frame_grace < 0:
        parser.error("--lost-frame-grace must be zero or positive")
    if args.max_scan_objects <= 0:
        parser.error("--max-scan-objects must be positive")
    if args.arm_right_scan_speed_mps <= 0:
        parser.error("--arm-right-scan-speed-mps must be positive")
    if args.slot_center_tolerance_ratio <= 0:
        parser.error("--slot-center-tolerance-ratio must be positive")
    if args.slot_stable_frames <= 0:
        parser.error("--slot-stable-frames must be positive")
    if args.scan_timeout_s <= 0:
        parser.error("--scan-timeout-s must be positive")
    if args.search_speed_mps <= 0:
        parser.error("--search-speed-mps must be positive")
    if args.settle_s < 0:
        parser.error("--settle-s must be zero or positive")
    if args.preview_ms < 1:
        parser.error("--preview-ms must be at least 1")
    if args.display_scale <= 0:
        parser.error("--display-scale must be positive")
    if args.warmup_frames < 0:
        parser.error("--warmup-frames must be zero or positive")
    if args.print_every <= 0:
        parser.error("--print-every must be positive")
    if args.continuous_delay_s < 0:
        parser.error("--continuous-delay-s must be zero or positive")

    if args.image:
        return run_image(args)
    return run_camera(args)


if __name__ == "__main__":
    raise SystemExit(main())
