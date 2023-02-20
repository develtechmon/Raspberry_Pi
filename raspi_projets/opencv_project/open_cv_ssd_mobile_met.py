import cv2
#import open_cv_camera_threading as webstream


cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)
cap.set(10,70)

thres = 0.45
classNames =[]
classFile = '/home/pi/Desktop/Project/machine_learning/mobilenet_ssd/coco.names'

with open (classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')
    
    #print(classNames)
    #print(len(classNames))
    
configPath = '/home/pi/Desktop/Project/machine_learning/mobilenet_ssd/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = '/home/pi/Desktop/Project/machine_learning/mobilenet_ssd/frozen_inference_graph.pb'
net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

while True:
    success, img = cap.read()
    classIds, confs, bbox = net.detect(img, confThreshold=thres)
    #print(classIds,bbox)
    
    if len(classIds) != 0:
        for classId,confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            cv2.rectangle(img,box,color=(0,255,0),thickness=2)
            cv2.putText(img,classNames[classId-1].upper(),(box[0]+10,box[1]+30),cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
            cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30),cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
    
    cv2.imshow("Output", img)
    
    if cv2.waitKey(1) & 0XFF == ord('q'):
        break