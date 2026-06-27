"""Detect one colored cube in a camera frame."""

from dataclasses import dataclass

import cv2
import numpy as np

from config import (
    AMBIGUOUS_SCORE_RATIO,
    DETECT_ROI,
    HSV_RANGES,
    MAX_BOX_H,
    MAX_BOX_RATIO,
    MAX_BOX_W,
    MIN_BOX_H,
    MIN_BOX_RATIO,
    MIN_BOX_W,
    MIN_CONTOUR_AREA,
    MIN_FILL_RATIO,
    PICK_TARGET_BBOX,
)


@dataclass
class Detection:
    found: bool
    color: str | None = None
    bbox: tuple[int, int, int, int] | None = None
    center: tuple[int, int] | None = None
    area: float = 0.0
    score: float = 0.0
    message: str = ""


class ColorObjectDetector:
    def __init__(self, hsv_ranges=None, roi=DETECT_ROI):
        self.hsv_ranges = hsv_ranges or HSV_RANGES
        self.roi = roi
        self.kernel = np.ones((5, 5), np.uint8)

    def detect(self, frame, target_color=None):
        if target_color is not None:
            target_color = target_color.lower()
            if target_color not in self.hsv_ranges:
                return Detection(False, message=f"unknown target color: {target_color}")

        candidates = self._find_candidates(frame)
        if not candidates:
            return Detection(False, message="no cube-like color contour")

        candidates.sort(key=lambda item: item.score, reverse=True)
        best = candidates[0]

        if len(candidates) > 1:
            second = candidates[1]
            if _iou(best.bbox, second.bbox) > 0.30:
                ratio = best.score / max(second.score, 1.0)
                if ratio < AMBIGUOUS_SCORE_RATIO:
                    return Detection(False, message="ambiguous object color")

        if target_color and best.color != target_color:
            return Detection(
                False,
                color=best.color,
                bbox=best.bbox,
                center=best.center,
                area=best.area,
                score=best.score,
                message=f"saw {best.color}, target is {target_color}",
            )

        return best

    def _find_candidates(self, frame):
        x1, y1, x2, y2 = _clip_roi(self.roi, frame.shape[1], frame.shape[0])
        roi_frame = frame[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
        candidates = []

        for color, ranges in self.hsv_ranges.items():
            mask = self._mask_for_color(hsv, ranges)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                candidate = self._candidate_from_contour(color, contour, hsv, x1, y1)
                if candidate is not None:
                    candidates.append(candidate)

        return candidates

    def _mask_for_color(self, hsv, ranges):
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            current = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.bitwise_or(mask, current)

        mask = cv2.medianBlur(mask, 5)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)
        return mask

    def _candidate_from_contour(self, color, contour, hsv, offset_x, offset_y):
        area = float(cv2.contourArea(contour))
        if area < MIN_CONTOUR_AREA:
            return None

        x, y, w, h = cv2.boundingRect(contour)
        ratio = w / max(h, 1)
        fill = area / max(w * h, 1)
        if not (
            MIN_BOX_W <= w <= MAX_BOX_W
            and MIN_BOX_H <= h <= MAX_BOX_H
            and MIN_BOX_RATIO <= ratio <= MAX_BOX_RATIO
            and fill >= MIN_FILL_RATIO
        ):
            return None

        contour_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, -1)
        mean_saturation = cv2.mean(hsv[:, :, 1], mask=contour_mask)[0]
        score = area * fill * (mean_saturation / 255.0)

        bbox = (x + offset_x, y + offset_y, w, h)
        center = (bbox[0] + w // 2, bbox[1] + h // 2)
        return Detection(True, color, bbox, center, area, score, "found")


def draw_debug(frame, detection, target_bbox=PICK_TARGET_BBOX):
    x1, y1, x2, y2 = target_bbox
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
    if detection.found and detection.bbox:
        x, y, w, h = detection.bbox
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.circle(frame, detection.center, 4, (0, 255, 0), -1)
        cv2.putText(frame, detection.color, (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return frame


def _clip_roi(roi, width, height):
    x1, y1, x2, y2 = roi
    return max(0, x1), max(0, y1), min(width, x2), min(height, y2)


def _iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x1 = max(ax, bx)
    y1 = max(ay, by)
    x2 = min(ax + aw, bx + bw)
    y2 = min(ay + ah, by + bh)
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    union = aw * ah + bw * bh - intersection
    return intersection / union if union else 0.0


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Test color cube detection on one image")
    parser.add_argument("image")
    parser.add_argument("--color", choices=sorted(HSV_RANGES))
    args = parser.parse_args()

    image = cv2.imread(args.image)
    if image is None:
        raise SystemExit(f"cannot read image: {args.image}")

    result = ColorObjectDetector().detect(image, target_color=args.color)
    print(result)
