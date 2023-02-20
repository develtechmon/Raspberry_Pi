from picamera import PiCamera
from time import sleep

camera = PiCamera()
camera.start_preview(fullscreen=False, window = (50,150,1024,576)) #--> x1,y1,w,h)

camera.image_effect = 'colorswap'
sleep(5)

camera.capture('/home/pi/Desktop/image%s.jpg' % i)
camera.stop_preview()

