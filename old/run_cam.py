import cv2
import numpy as np
import time

def run_depthCam(id=2, height=480, width=640, height_ratio=0.1, process_every=5, color_ranges=None):
    cap = cv2.VideoCapture(id)
    if not cap.isOpened():
        print(f'Can not open Camera {id}')
        return

    frame_count = 0
    start_time = time.time()

    while time.time() - start_time < 10:  # Run for 10 seconds
        ret, frame = cap.read()
        if not ret:
            print(f'Can not receive frame from camera {id}')
            break

        box_top = height - int(height_ratio * height)
        box_bottom = height
        box_left = 0
        box_right = width

        # Draw predefined rectangle and vertical line
        mid_x = width // 2
        cv2.line(frame, (mid_x, box_top), (mid_x, box_bottom), (0, 255, 0), 2)
        cv2.rectangle(frame, (box_left, box_top), (box_right, box_bottom), (255, 0, 0), 2)
        cv2.putText(frame, f'Width: {width}, Height: {height}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Detection every N frames or display previous for N frames
        if frame_count % process_every == 0:
            frame_count = 1
            roi = frame[box_top:box_bottom, box_left:box_right]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            bounding_boxes = []

            for color, (lower, upper) in color_ranges.items():
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            y += box_top
            if w > 10 and h > 10:  # Filter small noise
                bounding_boxes.append((color, (x, y, w, h)))
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.imshow('Depth Camera', frame)
        frame_count += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # RUN "export DISPLAY=:0" if running on a remote machine
    
    # Define HSV ranges for colors
    color_ranges = {
        'red':   [(170, 70, 50), (180, 255, 255)],
        'green': [(35, 40, 40),  (85, 255, 255)],
        'blue':  [(100, 150, 0), (140, 255, 255)],
        'yellow':[(20, 100, 100), (35, 255, 255)],
    }

    run_depthCam(id=2, height_ratio=0.12, color_ranges=color_ranges)