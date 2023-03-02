import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import imutils
import RPi.GPIO as GPIO  



####################### INITIAL MOTOR #######################

Rleft = 23
Lleft = 24
enL = 25

Rright = 27
Lright = 17
enR = 22

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# SET LEFT WHEEL
GPIO.setup(Rleft,GPIO.OUT)
GPIO.setup(Lleft,GPIO.OUT)
GPIO.setup(enL,GPIO.OUT)

# SET RIGHT WHEEL
GPIO.setup(Rright,GPIO.OUT)
GPIO.setup(Lright,GPIO.OUT)
GPIO.setup(enR,GPIO.OUT)


# LOW WHEEL
GPIO.output(Rleft,GPIO.LOW)
GPIO.output(Lleft,GPIO.LOW)
GPIO.output(Rright,GPIO.LOW)
GPIO.output(Lright,GPIO.LOW)

# SET PWM
pwmL=GPIO.PWM(enL,1000)
pwmL.start(70)
pwmR=GPIO.PWM(enR,1000)
pwmR.start(70)

pwm = 70

def forward(speed): 
    pwmL.ChangeDutyCycle(speed)
    pwmR.ChangeDutyCycle(speed)
    GPIO.output(Rright,GPIO.HIGH)
    GPIO.output(Lright,GPIO.LOW)
    GPIO.output(Rleft,GPIO.LOW)
    GPIO.output(Lleft,GPIO.HIGH)

def backward(speed):
    pwmL.ChangeDutyCycle(speed)
    pwmR.ChangeDutyCycle(speed)
    GPIO.output(Rright,GPIO.LOW)
    GPIO.output(Lright,GPIO.HIGH)
    GPIO.output(Rleft,GPIO.HIGH)
    GPIO.output(Lleft,GPIO.LOW)

def right(speed):
    pwmL.ChangeDutyCycle(speed)
    pwmR.ChangeDutyCycle(speed)
    GPIO.output(Rright,GPIO.LOW)
    GPIO.output(Lright,GPIO.LOW)
    GPIO.output(Rleft,GPIO.LOW)
    GPIO.output(Lleft,GPIO.HIGH)

def left(speed):
    pwmL.ChangeDutyCycle(speed)
    pwmR.ChangeDutyCycle(speed)
    GPIO.output(Rright,GPIO.HIGH)
    GPIO.output(Lright,GPIO.LOW)
    GPIO.output(Rleft,GPIO.LOW)
    GPIO.output(Lleft,GPIO.LOW)



def stop():
    GPIO.output(Rleft,GPIO.LOW)
    GPIO.output(Lleft,GPIO.LOW)
    GPIO.output(Rright,GPIO.LOW)
    GPIO.output(Lright,GPIO.LOW)

####################### INITIAL MOTOR #######################



####################### INITIAL CAMERA #######################

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


# Menentukan range warna yang akan dideteksi
lower_yellow = np.array([18, 140, 112])
upper_yellow = np.array([93, 255, 255])

lower_red = np.array([0, 50, 50])
upper_red = np.array([10, 255, 255])


####################### INITIAL CAMERA #######################



####################### INITIAL PID #######################

kp = 0.2
kd = 0.2
lastime = 0
lasterror = 0
output = 0


def millis():
    return int(round(time.time() * 1000))


def pidCompute(setPoint, actual):
    global lasterror

    error = setPoint - actual
    P = kp * error
    D = kd * (error - lasterror)
    pid = P + D

    lasterror = error

    return pid    
        
####################### INITIAL PID #######################




####################### INITIAL CONTROL #######################

def controlManual():
    if (ex <= 100):
        print('belok kiri')
        left(70)
    elif (ex >= 400):
        print('belok kanan')
        right(70)
    else:
        print('lurus')
        forward(80)   

def controlMotor(lSpeed,rSpeed):
    try:
        pwmL.ChangeDutyCycle(lSpeed )
        pwmR.ChangeDutyCycle(rSpeed)
        GPIO.output(Rright,GPIO.HIGH)
        GPIO.output(Lright,GPIO.LOW)
        GPIO.output(Rleft,GPIO.LOW)
        GPIO.output(Lleft,GPIO.HIGH)
    except:
        print('error')



####################### INITIAL CONTROL #######################




# Capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # Grab the raw NumPy array representing the image
    image = frame.array
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
 
    # Membuat mask dengan warna yang terdeteksi
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # temukan kontur dari mask
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        try:
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), 160)
            ex = center[0]
        except ZeroDivisionError:
            print("divide by zero.")

    #### CONTROL PID ####

    if millis() - lastime >= 100:
        output = pidCompute(318,ex)
        lastime = millis()
    
        leftSpeed = 70 - output
        rightSpeed = 70 + output

        controlMotor(leftSpeed,rightSpeed)
        print(leftSpeed,rightSpeed)
    
    #### CONTROL PID ####


    cv2.circle(image, center, 20, (255,0,0), 2)

    # Display the image in the window
    cv2.imshow("Video Stream", mask)

    # Wait for a key press and exit if 'q' is pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)

# Clean up
stop()
cv2.destroyAllWindows()
