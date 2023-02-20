from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.start_preview(fullscreen=False, window = (50,150,1024,576)) #--> x1,y1,w,h
sleep(9000)
camera.stop_preview
qqqq
