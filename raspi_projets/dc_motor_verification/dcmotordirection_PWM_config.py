import RPi.GPIO as GPIO
from time import sleep

Motor1A = 12
Motor1B = 18

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(Motor1A, GPIO.OUT)
GPIO.setup(Motor1B, GPIO.OUT)

def Backwards():
    #Going Backwards
    GPIO.output(Motor1A, GPIO.LOW) 
    GPIO.output(Motor1B, GPIO.HIGH) 
    p=GPIO.PWM(Motor1B, 10) #Direct Start with 1000 Frequency
    p.start(100)
    print("Going Backwards")


def Forwards():
    #Going Forwards
    GPIO.output(Motor1A, GPIO.HIGH)
    GPIO.output(Motor1B, GPIO.LOW)
    pwm=GPIO.PWM(Motor1A, 10) #Direct Start with 1000 Frequency
    pwm.start(100)
    print("Going Forwards")
    
    #sleep(1)
    #GPIO.output(Motor1A, GPIO.LOW)
    #GPIO.output(Motor1B, GPIO.LOW)
    #p.stop() #Stop
   # print("Going Forwards")

try:
    while True:
        Backwards()
        #p.stop() #Stop
        sleep(1)
        Forwards()
except KeyboardInterrupt:
    GPIO.cleanup


