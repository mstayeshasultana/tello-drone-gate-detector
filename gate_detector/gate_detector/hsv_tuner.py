#!/usr/bin/env python3
import cv2
import numpy as np

def nothing(x):
    pass

# 1) Open your camera
cap = cv2.VideoCapture(0)  # change index or URL if needed

# 2) Create a window and 6 trackbars for low/high H, S, V
cv2.namedWindow('HSV Tuner', cv2.WINDOW_NORMAL)
for name, default, maximum in [
    ('Low H',  30, 179),
    ('High H', 80, 179),
    ('Low S',  40, 255),
    ('High S', 255,255),
    ('Low V',  40, 255),
    ('High V', 255,255),
]:
    cv2.createTrackbar(name, 'HSV Tuner', default, maximum, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 3) Read slider positions
    l_h = cv2.getTrackbarPos('Low H',  'HSV Tuner')
    h_h = cv2.getTrackbarPos('High H', 'HSV Tuner')
    l_s = cv2.getTrackbarPos('Low S',  'HSV Tuner')
    h_s = cv2.getTrackbarPos('High S', 'HSV Tuner')
    l_v = cv2.getTrackbarPos('Low V',  'HSV Tuner')
    h_v = cv2.getTrackbarPos('High V', 'HSV Tuner')

    # 4) Build HSV mask
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low  = np.array((l_h, l_s, l_v))
    high = np.array((h_h, h_s, h_v))
    mask = cv2.inRange(hsv, low, high)

    # 5) Show original + mask
    cv2.imshow('Original', frame)
    cv2.imshow('Mask',     mask)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC to exit
        print(f'Final range: low={low}, high={high}')
        break

cap.release()
cv2.destroyAllWindows()
