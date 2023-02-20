from picamera import PiCamera
from time import sleep

camera = PiCamera()
camera.start_preview()
for i in range(5):
    sleep(5)
    #Continously take the photos
    camera.capture('/home/pi/Desktop/image%s.jpg' % i)
camera.stop_preview()
    