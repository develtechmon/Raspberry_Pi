import cv2
import numpy as np
import gpiozero
import sys
import time
import os
from threading import Thread
import importlib.util
from picamera2 import Picamera2

class RpiVideoStream:
    def __init__(self, src=0,name="RpiVideoStream"):   
    #def __init__(self, cam,name="RpiVideoStream"):         
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
        self.picam2.start() 
        
        #self.picam2=cam
                 
        self.name = name
        self.stopped = False
        
    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.update, name=self.name, args=())
        t.daemon = True
        t.start()
        return self
    
    def update(self):
        while True:
            if self.stopped:
               return
            self.frame=self.picam2.capture_array()
            #(self.grabbed, self.frame) = self.stream.read()
    
    def read(self):
        return self.frame
    
    def stop(self):
        self.stopped = True

#picam2 = Picamera2()
#picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
#picam2.start() 
              
if __name__ == "__main__":
    stream = RpiVideoStream(src=0).start()  # default camera
    #stream = RpiVideoStream(picam2).start()  # default camera
      
    time.sleep(1)
    while True:
        img = stream.read()
        cv2.imshow("Output", img)
        if cv2.waitKey(1) & 0XFF == ord('q'):
            break
        
cv2.destroyAllWindows()
WebcamVideoStream.stop()     