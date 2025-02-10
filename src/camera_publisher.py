#!/usr/bin/env python3

import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Output directory
OUTPUT_DIR = "/home/is/catkin_ws/src/z_output"

# Ensure output directory exists
os.makedirs(OUTPUT_DIR, exist_ok=True)

class CameraSubscriber:
    def __init__(self):
        rospy.init_node("camera_subscriber", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cobot/camera1/image_raw", Image, self.callback)
        self.image_counter = 0

    def callback(self, msg):
        """Callback function to process and save images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            image_path = os.path.join(OUTPUT_DIR, f"recent_frame.jpg")
            cv2.imshow("frame", cv_image)
            cv2.waitKey(1)
            cv2.imwrite(image_path, cv_image)
            print(f"Saved: {image_path}")
        except Exception as e:
            print(f"Error processing image: {e}")

    def run(self):
        rospy.spin()  # Keep the script running

if __name__ == "__main__":
    try:
        camera_subscriber = CameraSubscriber()
        camera_subscriber.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        print("\nShutting down camera subscriber...")




        print("100 reps in gym v3")
