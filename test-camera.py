import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

# Initialize the PiCamera

camera = PiCamera()
camera.resolution = (640, 480)
camera.rotation = 180
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))

# Allow the camera to warm up
time.sleep(0.1)

# Create a window to display the video stream
cv2.namedWindow("Video Stream", cv2.WINDOW_NORMAL)

# Capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # Grab the raw NumPy array representing the image
    image = frame.array

    # Display the image in the window
    cv2.imshow("Video Stream", image)

    # Wait for a key press and exit if 'q' is pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)

# Clean up
cv2.destroyAllWindows()
