'''
Here we're tracking black and dark line using OpenCV
See this link:
https://einsteiniumstudios.com/beaglebone-opencv-line-following-robot.html

'''
import numpy as np
import cv2

cap = cv2.VideoCapture(0)
cap.set(3,160)
cap.set(4,120)

while True:
    # Capture the frames
    ret, frame = cap.read()

    # Crop the image
    crop_img = frame[60:120, 0:160]

    # Convert to grayscale
    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

    # Gaussian Blur
    blur = cv2.GaussianBlur(gray,(5,5),0)

    # Color Thresholding
    ret, thresh1 = cv2.threshold(blur,60,255, cv2.THRESH_BINARY_INV)
    
    # Erode and dilate to remove accidental line detections
    mask = cv2.erode(thresh1, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find the contours of the frame
    contours,hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
    
    # Find the biggest contour (if detected)

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        # See this links : https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
        # Moments used to figure out the center of single blob in an image
        # blob is a group of connected pixel in an image that share common property such as grayscale value.
        M = cv2.moments(c) 

        if int(M['m10']) !=0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            cv2.circle(crop_img, (cx,cy),5,(255,255,255),-1)
            
            cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1)
            cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1)
            cv2.drawContours(crop_img, contours, -1, (0,255,0), 1)
            print (cx)
            print (cy)
        else:
            cx,cy = 0,0

    cv2.imshow("output",crop_img)
    if cv2.waitKey(1) & 0XFF == ord('q'):
        break

