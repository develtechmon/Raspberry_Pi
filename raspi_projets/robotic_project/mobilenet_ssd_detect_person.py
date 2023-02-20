import cv2
import numpy as np
import gpiozero
import sys
import time
import os
from threading import Thread
import importlib.util

class VideoStream:
    """Camera object that controls video streaming from the Picamera"""
    def __init__(self,resolution=(640,480),framerate=30):
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(0)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.stream.set(3,resolution[0])
        ret = self.stream.set(4,resolution[1])
            
        # Read first frame from the stream
        (self.grabbed, self.frame) = self.stream.read()

	# Variable to control when the camera is stopped
        self.stopped = False

    def start(self):
	# Start the thread that reads frames from the video stream
        Thread(target=self.update,args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                self.stream.release()
                return

            # Otherwise, grab the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
	# Return the most recent frame
        return self.frame

    def stop(self):
	# Indicate that the camera and thread should be stopped
        self.stopped = True
        
robot = gpiozero.Robot(left=(17,18), right=(27,22))
width = 640
height = 480

#if cap.isOpened():
#    width = cap.get(3)
#    height = cap.get(4)
#    x = cap.get(0)
#    y = cap.get(1)
    #print(width, height, x, y)

image_width = width
image_height = height
center_image_x = image_width/2
center_image_y = image_height/2

minimum_area = 250
maximum_area = 200000
    
classNames =[]
classFile = '/home/pi/Desktop/Project/machine_learning/mobilenet_ssd/coco.names'

with open (classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')
    
configPath = '/home/pi/Desktop/Project/machine_learning/mobilenet_ssd/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = '/home/pi/Desktop/Project/machine_learning/mobilenet_ssd/frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

def detectObject(object_location):
    if object_location:
        if(object_location[0] > minimum_area) and (object_location[0] < maximum_area):
            if (object_location[1] > center_image_x + (image_width/7)):
                print("You are moving right")
                robot.left(0.45)
                
            elif (object_location[1] < center_image_x - (image_width/7)):
                print("You are moving left")
                robot.right(0.45)
                
            else:
                print("You are at centre")
                #robot.forward(0.5)
                
        elif(object_location[0] < minimum_area):
            #robot.left(0.5)
            print("Object is not large enough, searching..")
            
        elif(object_location[0] > maximum_area):
            robot.stop()
            print("Object is large enough, stopping")
    else:
        #robot.left(0.5)
        print("No object found")

                
def getObjects(img, thres, nms, draw=True, objects=[]):    
    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nms)
    #print(classIds,bbox)
    
    if len(objects) == 0 : objects = classNames
    objectInfo = []
    
    if len(classIds) != 0:
        for classId,confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            className = classNames[classId-1]
            if className in objects:
                objectInfo.append([box,className])
                if (draw):
                    cv2.rectangle(img,box,color=(0,255,0),thickness=2)
                    cv2.putText(img,className.upper(),(box[0]+10,box[1]+30),cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                    cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30),cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                    #if(classNames[classId-1].upper()) == 'PERSON':
                    face = box
        
                    x,y,w,h = face
                    object_x = 0
                    object_y = 0
                    object_area = 0
                
                    found_area = w*h
                    center_x = x + (w/2)
                    center_y = y + (h/2)
                
                    if (object_area < found_area):
                        object_x = center_x
                        object_y = center_y
                        object_area = found_area
                    if (object_area > 0):
                        object_location = [object_area, object_x, object_y]
                        #print(object_location)
                        detectObject(object_location)
    return img, objectInfo

                    
if __name__ == "__main__":
   videostream = VideoStream(resolution=(width,height),framerate=30).start()
   #time.sleep(1)
   #videostream = cv2.VideoCapture(0)
   #cap.set(3,width)
   #cap.set(4,height)
   #cap.set(10,70)
   while True:
       success, img = videostream.read()
       result, objectInfo = getObjects(img,0.45, 0.1, objects=['person'])
       cv2.imshow("Output", img)
       if cv2.waitKey(1) & 0XFF == ord('q'):
           break
        
cv2.destroyAllWindows()
videostream.stop()     