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
    # Nyalakan semua LED
    LED1.on()
    LED2.on()
    LED3.on()
    LED4.on()
    LED5.on()
    LED6.on()
    LED7.on()
    LED8.on()

    sleep(1)

    # Padamkan smeua LED
    LED1.off()
    LED2.off()
    LED3.off()
    LED4.off()
    LED5.off()
    LED6.off()
    LED7.off()
    LED8.off()

    sleep(1)
