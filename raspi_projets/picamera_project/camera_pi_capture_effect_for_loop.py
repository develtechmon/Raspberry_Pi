from picamera import PiCamera
from time import sleep

camera = PiCamera()
camera.start_preview(fullscreen=False, window = (50,150,1024,576)) #--> x1,y1,w,h)

for effect in camera.IMAGE_EFFECTS:
    camera.image_effect = effect
    camera.annotate_text = "Effect: %s" %effect
    sleep(5)
camera.stop_preview()
