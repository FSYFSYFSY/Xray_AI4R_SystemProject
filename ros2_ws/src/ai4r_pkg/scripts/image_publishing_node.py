#!/usr/bin/env python3

import cv2 

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

USB_PORT = 0

class ImagePublishingNode(Node):

    def __init__(self):
        super().__init__('image_publishing_node')
        
        # Open the camera
        self.get_logger().info("[LINE DETECTOR NODE] Now opening the camera capture.")
        self.cap = cv2.VideoCapture(USB_PORT)
        self.get_logger().info("[LINE DETECTOR NODE] Camera capture opened.")

        # Create a CvBridge object for converting between OpenCV images and ROS Image messages
        self.cv_bridge = CvBridge()

        # Read the a camera frame as a double check of the properties
        # > Read the frame
        return_flag , current_frame = self.cap.read()
        # > Get the dimensions of the frame
        dimensions = current_frame.shape
        # > Display the dimensions
        self.get_logger().info("[LINE DETECTOR NODE] Triple check of resolution, a frame captured just now has dimensions = " + str(dimensions))

        # For publishing the image 
        self.img_pub = self.create_publisher(
            # Image,"image",qos_profile_system_default
            Image,"image",qos_profile_sensor_data
        )

        # Timer callback to publish the image
        # timer_period = (1.0/self.cap.get(cv2.CAP_PROP_FPS))
        timer_period = (1.0/30.0)
        self.timer = self.create_timer(timer_period,self.camera_callback)

    # Callback function for publishing the image
    def camera_callback(self):
        try:
            ret, frame = self.cap.read()
            
            if ret == True:
                # # Publish image straight from camera
                # self.img_pub.publish(self.cv_bridge.cv2_to_imgmsg(self.frame, "bgr8"))

                frame_small = cv2.resize(frame, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_AREA)

                # Publish the modified image to the topic /image
                frame_mono = cv2.cvtColor(frame_small, cv2.COLOR_BGR2GRAY)
                self.img_pub.publish(self.cv_bridge.cv2_to_imgmsg(frame_mono, "mono8"))
            else:
                self.get_logger().error("[LINE_DETECTOR_NODE] No frame received from camera.")
        
        except CvBridgeError as e:
            self.get_logger().error("[LINE_DETECTOR_NODE] CvBridgeError: {0}".format(e))
 


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublishingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()