#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Thread

class WebcamVideoStream:
    def __init__(self, src=0, name="WebcamVideoStream"):
        self.stream = cv2.VideoCapture(src)
        self.name = name
        self.stopped = False

    def start(self):
        t = Thread(target=self.update, name=self.name, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True

def main():
    rospy.init_node('image_publisher', anonymous=True)
    image_pub = rospy.Publisher('camera/image', Image, queue_size=10)
    bridge = CvBridge()
    cap = WebcamVideoStream(src=0).start()

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        frame = cap.read()
        if frame is not None:
            try:
                # Convert OpenCV image to ROS Image message
                ros_img = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                image_pub.publish(ros_img)
            except CvBridgeError as e:
                print(e)
        rate.sleep()

    cap.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
