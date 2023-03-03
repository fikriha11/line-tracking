import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time

camera = PiCamera()
camera.resolution = (640, 480)
camera.rotation = 180
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))

H_low = H_high = S_low = S_high = V_low = V_high = 0

# Allow the camera to warm up
time.sleep(0.1)

# Create a window to display the video stream
cv2.namedWindow("Video Stream", cv2.WINDOW_NORMAL)
cv2.namedWindow("control", cv2.WINDOW_NORMAL)


def callback(x):
   global H_low,H_high,S_low,S_high,V_low,V_high
   H_low = cv2.getTrackbarPos('low H','control')
   H_high = cv2.getTrackbarPos('high H','control')
   S_low = cv2.getTrackbarPos('low S','control')
   S_high = cv2.getTrackbarPos('high S','control')
   V_low = cv2.getTrackbarPos('low V','control')
   V_high = cv2.getTrackbarPos('high V','control')

cv2.createTrackbar('low H','control',15,225,callback)
cv2.createTrackbar('high H','control',93,225,callback)

cv2.createTrackbar('low S','control',140,255,callback)
cv2.createTrackbar('high S','control',255,255,callback)

cv2.createTrackbar('low V','control',112,255,callback)
cv2.createTrackbar('high V','control',255,255,callback)

# Capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # Grab the raw NumPy array representing the image
    image = frame.array

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


    # lower and upper hsv
    hsv_low = np.array([H_low, S_low, V_low], np.uint8)
    hsv_high = np.array([H_high, S_high, V_high], np.uint8) 

    # Membuat mask dengan warna yang terdeteksi
    mask = cv2.inRange(hsv, hsv_low, hsv_high)

    # Display the image in the window
    cv2.imshow("Video Stream", mask)

    # Wait for a key press and exit if 'q' is pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)


# Clean up
cv2.destroyAllWindows()
