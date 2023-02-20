import gpiozero 
from gpiozero import Buzzer
import time

buzzer = Buzzer(26)
robot = gpiozero.Robot(left=(17,18), right=(27,22))

try:
    while True:
        buzzer.beep(0.1, 0.1, 1)
        robot.backward(0.5)
        print("backward")
        time.sleep(1)

        robot.left(0.5)
        print("left")
        time.sleep(1)

        robot.forward(0.5)
        print("forward")
        time.sleep(1)
       
        robot.right(0.5)
        print("right")
        time.sleep(1)
            
except KeyboardInterrupt:
    robot.stop()
    print("Measurement Stopped By User")


    
