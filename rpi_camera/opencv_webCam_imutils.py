from imutils.video import VideoStream
import imutils
import time
import cv2

vs = VideoStream(src=0).start()
time.sleep(2.0)

while True:
    #grab the next frame
    frame = vs.read()

    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0XFF == ord('q'):
        break