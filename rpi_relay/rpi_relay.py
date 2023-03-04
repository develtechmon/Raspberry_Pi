import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Relay 1
GPIO.setup(14, GPIO.OUT)
# Relay 2
GPIO.setup(15, GPIO.OUT)

try:
    while True:
        GPIO.output(14, GPIO.HIGH)
        print('Relay 1 ON')
        time.sleep(1)
        #GPIO.output(15, GPIO.HIGH)
        #print('Relay 2 ON')
        time.sleep(1)
        GPIO.output(14, GPIO.LOW)
        print('Relay 1 OFF')
        time.sleep(1)
        #GPIO.output(15, GPIO.LOW)
        #print('Relay 2 OFF')
        time.sleep(1)
        
finally:
    GPIO.cleanup()
