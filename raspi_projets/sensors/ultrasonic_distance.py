#Import Libraries 
import RPi.GPIO as GPIO
import time

#Set GPIO Mode (Board / BCM)
#GPIO.setmode(GPIO.BCM)
GPIO.setmode(GPIO.BOARD)

# BCM
#Set GPIO Pins
#GPIO_TRIGGER = 12
#GPIO_ECHO = 13
#maxTime = 0.04

# BOARD
#Set GPIO Pins
GPIO_TRIGGER = 12
GPIO_ECHO = 13
maxTime = 0.04

GPIO.setwarnings(False)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def distance():
    GPIO.output(GPIO_TRIGGER, False)
    
    time.sleep(0.01)

    #Set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
    
    #Set Trigger after 0.01ms to LOW
    time.sleep(0.00001)

    GPIO.output(GPIO_TRIGGER, False)
    
    StartTime = time.time()
    timeout = StartTime + maxTime
    
    #Save Start Time
    while GPIO.input(GPIO_ECHO) == 0 and StartTime <  timeout:
        StartTime = time.time()
    
    StopTime = time.time()
    timeout = StopTime + maxTime

    #Save Time of Arrival
    while GPIO.input(GPIO_ECHO) == 1 and StopTime < timeout:
        StopTime = time.time()
        
    #Time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    #Multiply with the sonic speed (34300 cm/s)
    #and divide by 2 because there and back
    distance = (TimeElapsed *34300) /2
    
    return distance

try:
    while True:
        dist = distance()
        #print("Measure Distance = %.1f cm" % dist)
        print(dist)
        time.sleep(1)
        
        #if (dist <=20 and dist >=5):
        #    print("distance: ", dist, "cm")
            
       # else:
        #    print(dist)
        
    #Reset by pressing CTRL+C
except KeyboardInterrupt:
    print("Measurement Stopped By User")
    GPIO.cleanup()
    
        
    
    

    
    
    
    





