#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('camera_subscriber', anonymous=True)
        
        # Create a CvBridge object to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()
        
        # Subscribe to the 'camera/image' topic
        rospy.Subscriber('camera/image', Image, self.callback)
        
        # Create a named window to display the image
        cv2.namedWindow("Camera Image", cv2.WINDOW_NORMAL)
        self.display_rate = rospy.Rate(30)  # Rate at which images are displayed
    
    def callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            
            # Display the image
            cv2.imshow("Camera Image", cv_img)
            cv2.waitKey(1)  # Display the image window and handle user events
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", str(e))

    def run(self):
        # Spin to keep the node active
        rospy.spin()
        
        # Destroy all OpenCV windows when done
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        subscriber = ImageSubscriber()
        subscriber.run()
    except rospy.ROSInterruptException:
        pass
