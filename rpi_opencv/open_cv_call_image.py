import cv2
import numpy as np

print("Package Imported")
img = cv2.imread("/home/pi/Desktop/Project/opencv_project/resources/cy.jpeg")
cv2.imshow("Output", img)
cv2.waitKey(6000)



