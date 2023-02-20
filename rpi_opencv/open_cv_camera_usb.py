import cv2

cap = cv2.VideoCapture(0)

while True:
    success, img = cap.read()
    cv2.imshow("Output", img)
    if cv2.waitKey(1) & 0XFF == ord('q'):
        break
    
    