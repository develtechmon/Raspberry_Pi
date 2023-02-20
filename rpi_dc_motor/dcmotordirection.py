from gpiozero import Buzzer
import pigpio
from os import system
from time import sleep

pi = pigpio.pi()
buzzer = Buzzer(26)

M1A = 17
M1B = 18

buzzer.beep(0.1, 0.1, 1)

pi.set_mode(M1A, pigpio.OUTPUT)
pi.set_mode(M1B, pigpio.OUTPUT)

def motormove_forward(speedLeft, speedRight):
       pi.set_PWM_dutycycle(M1A, 0)
       pi.set_PWM_dutycycle(M1B, speedRight)


def motormove_backward(speedLefts, speedRights):
       pi.set_PWM_dutycycle(M1A, speedLefts)
       pi.set_PWM_dutycycle(M1B, 0)

try:
    while True:
        print("forward")
        motormove_forward(250,100)
        sleep(1)
        #print("stop")
        #motormove_forward(0,0)
        #sleep(1)
        print("backward")
        motormove_backward(100,100)
        sleep(1)
        #print("stop")
        #motormove_backward(0,0)

except KeyboardInterrupt:
    buzzer.off()
    motormove_forward(0,0)
    motormove_backward(0,0)


