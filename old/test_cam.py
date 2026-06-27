import cv2
import numpy as np

def run_cameras():
    cap0 = cv2.VideoCapture(0)
    cap1 = cv2.VideoCapture(2)

    if not cap0.isOpened():
        print("Cannot open camera 0")
        return
    if not cap1.isOpened():
        print("Cannot open camera 1")
        cap0.release()
        return

    while True:
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()
        if not ret0:
            print("Can't receive frame from camera 0. Exiting ...")
            break
        if not ret1:
            print("Can't receive frame from camera 1. Exiting ...")
            break

        # Resize frames to the same width if needed
        if frame0.shape[1] != frame1.shape[1]:
            width = min(frame0.shape[1], frame1.shape[1])
            frame0 = cv2.resize(frame0, (width, int(frame0.shape[0] * width / frame0.shape[1])))
            frame1 = cv2.resize(frame1, (width, int(frame1.shape[0] * width / frame1.shape[1])))

        # Stack horizontally
        combined = np.hstack((frame0, frame1))
        cv2.imshow('Combined Cameras', combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap0.release()
    cap1.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # RUN "export DISPLAY=:0" if running on a remote machine
    run_cameras()