from gpiozero import LED
from time import sleep

LED1 = LED(17)
LED2 = LED(18)
LED3 = LED(27)
LED4 = LED(22)
LED5 = LED(25)
LED6 = LED(12)
LED7 = LED(13)
LED8 = LED(19)

while True:
    LED1.on()
    LED8.on()
    sleep(0.5)
    LED1.off()
    LED8.off()
    
    LED2.on()
    LED7.on()
    sleep(0.5)
    LED2.off()
    LED7.off()
    
    LED3.on()
    LED6.on()
    sleep(0.5)
    LED3.off()
    LED6.off()
    
    LED4.on()
    LED5.on()
    sleep(0.5)
    LED4.off()
    LED5.off()
    
    