#!/usr/bin/env python

#Import Libraries 
import RPi.GPIO as GPIO
import time

#Set GPIO Mode (Board / BCM)
GPIO.setmode(GPIO.BCM)

#Set GPIO Pins
GPIO_TRIGGER = 14
GPIO_ECHO = 15
maxTime = 0.04

#Declare a motor pin
Motor1A = 12
Motor1B = 18

GPIO.setwarnings(False)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(Motor1A, GPIO.OUT)
GPIO.setup(Motor1B, GPIO.OUT)

loop=True

def Backwards():
    #Going Backwards
    GPIO.output(Motor1A, GPIO.LOW) 
    GPIO.output(Motor1B, GPIO.HIGH) 
    p=GPIO.PWM(Motor1B, 0.3) #Direct Start with 1000 Frequency
    p.start(100)
    print("Going Backwards")
    

def Forwards():
    #Going Forwards
    GPIO.output(Motor1A, GPIO.HIGH)
    GPIO.output(Motor1B, GPIO.LOW)
    pwm=GPIO.PWM(Motor1A, 0.3) #Direct Start with 1000 Frequency
    pwm.start(100)
    print("Going Forwards")
    
def distance():
    #Set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
    
    #Set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    
    StartTime = time.time()
    StopTime = time.time()
    timeout = StopTime + maxTime

    
    #Save Start Time
    while GPIO.input(GPIO_ECHO) == 0 and StartTime < timeout:
        StartTime = time.time()
        
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
    while loop:
        dist = distance()
        #print("Measure Distance = %.1f cm" % dist)
        time.sleep(0.004)
        if (dist <=50 ): #(dist <=50 and dist >=5):
            #print("distance: ", dist, "cm")
            Backwards()
        
        else:
            #print(dist)
            Forwards()
        
    #Reset by pressing CTRL+C
except KeyboardInterrupt:
    print("Measurement Stopped By User")
    GPIO.cleanup()
    
        
    
    

    
    
    
    





