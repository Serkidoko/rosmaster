import cv2
import os
import time

# Open the default camera (device 2 for Depth Camera)
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

i = 0
time.sleep(2)

ret, frame = cap.read()
if ret:
    filename = f'captured_image.jpg'
    cv2.imwrite(filename, frame)
    print(f"Image saved as {filename}")
else:
    print("Failed to capture image.")

cap.release()