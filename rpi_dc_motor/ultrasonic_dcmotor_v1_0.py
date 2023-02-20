import gpiozero 
from gpiozero import Buzzer
import time

buzzer = Buzzer(26)

TRIG=12
ECHO=13
maxTime=0.04

trigger = gpiozero.OutputDevice(TRIG)
echo = gpiozero.DigitalInputDevice(ECHO)

robot = gpiozero.Robot(left=(17,18), right=(27,22))

def get_distance(trigger, echo):
    trigger.on()
    time.sleep(0.00001)
    trigger.off()
    
    pulse_start = time.time()
    pulse_end = time.time()
    timeout = pulse_end + maxTime

    while echo.is_active == False and pulse_start < timeout:
        pulse_start = time.time()

    while echo.is_active == True and pulse_end < timeout:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = 34300 * (pulse_duration/2)
    round_distance = round(distance,1)

    return(round_distance)

try:
    while True:
        dist = get_distance(trigger,echo)
        #time.sleep(0.2)
        #print(dist)
        if dist <= 50:
            buzzer.beep(0.1, 0.1, 1)

            robot.forward(0.5)
            print("backward 1")
            time.sleep(0.8)

            robot.right(0.5)
            print("right")
            time.sleep(0.6)

            robot.backward(0.5)
            print("backward 2")
            time.sleep(0.3)
        else:
            robot.backward(0.4)
            time.sleep(0.2)
     
except KeyboardInterrupt:
    print("Measurement Stopped By User")
    robot.stop()


    