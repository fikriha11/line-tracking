import cv2
import numpy as np
import imutils

# Menentukan range warna yang akan dideteksi
lower_red = np.array([0, 50, 50])
upper_red = np.array([10, 255, 255])

lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([30, 255, 255])

cv2.namedWindow("image", cv2.WINDOW_NORMAL)
cv2.namedWindow("control", cv2.WINDOW_NORMAL)

def callback(x):
   global H_low,H_high,S_low,S_high,V_low,V_high
   H_low = cv2.getTrackbarPos('low H','control')
   H_high = cv2.getTrackbarPos('high H','control')
   S_low = cv2.getTrackbarPos('low S','control')
   S_high = cv2.getTrackbarPos('high S','control')
   V_low = cv2.getTrackbarPos('low V','control')
   V_high = cv2.getTrackbarPos('high V','control')

def onChange(value):
    print(value)

cv2.createTrackbar('low H','control',20,225,callback)
cv2.createTrackbar('high H','control',30,225,callback)

cv2.createTrackbar('low S','control',100,255,callback)
cv2.createTrackbar('high S','control',255,255,callback)

cv2.createTrackbar('low V','control',100,255,callback)
cv2.createTrackbar('high V','control',255,255,callback)


# Display the window
while True:
    img = cv2.imread('line-image.jpg')
    img = cv2.resize(img,(640,480))
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

    # Mengonversi gambar ke ruang warna HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # lower and upper hsv
    hsv_low = np.array([H_low, S_low, V_low], np.uint8)
    hsv_high = np.array([H_high, S_high, V_high], np.uint8) 
    
    # Membuat mask dengan warna yang terdeteksi
    mask = cv2.inRange(hsv, hsv_low, hsv_high)
    cv2.imshow('image', mask)
    
    k = cv2.waitKey(1)
    if k == ord('q'):
        break


cv2.destroyAllWindows()
