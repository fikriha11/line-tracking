import RPi.GPIO as GPIO          
from time import sleep

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
pwmL.start(50)
pwmR=GPIO.PWM(enR,1000)
pwmR.start(50)


print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    


def forward(): 
    GPIO.output(Rright,GPIO.HIGH)
    GPIO.output(Lright,GPIO.LOW)

    GPIO.output(Rleft,GPIO.LOW)
    GPIO.output(Lleft,GPIO.HIGH)


def backward():
    GPIO.output(Rright,GPIO.LOW)
    GPIO.output(Lright,GPIO.HIGH)
    
    GPIO.output(Rleft,GPIO.HIGH)
    GPIO.output(Lleft,GPIO.LOW)


def right():
    GPIO.output(Rright,GPIO.LOW)
    GPIO.output(Lright,GPIO.LOW)
    
    GPIO.output(Rleft,GPIO.LOW)
    GPIO.output(Lleft,GPIO.HIGH)

def left():
    GPIO.output(Rright,GPIO.HIGH)
    GPIO.output(Lright,GPIO.LOW)
    
    GPIO.output(Rleft,GPIO.LOW)
    GPIO.output(Lleft,GPIO.LOW)

def stop():
    GPIO.output(Rleft,GPIO.LOW)
    GPIO.output(Lleft,GPIO.LOW)

    GPIO.output(Rright,GPIO.LOW)
    GPIO.output(Lright,GPIO.LOW)

while(1):

    data = input("Input Character: ")
    
    if data == 'f':
        forward()

    elif data == 'b':
        backward()

    elif data == 's':
        stop()
     
    elif data == 'e':
        GPIO.cleanup()
        break
    
    else:
        print("<<<  wrong data  >>>")
        print("please enLter the defined data to continue.....")



