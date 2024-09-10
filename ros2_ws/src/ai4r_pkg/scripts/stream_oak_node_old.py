#!/usr/bin/env python3

# dependencies: pip install depthai; pip install opencv-python

import depthai as dai
import cv2 

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data, QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class OAKDPublishingNode(Node):

    def __init__(self):
        super().__init__('oakd_publishing_node')

        custom_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.get_logger().info("[STREAM OAKD NODE] Now setting up the camera.")

        # Create a pipeline
        self.pipeline = dai.Pipeline()

        # Define a source - color camera
        self.cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        self.cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        self.cam_rgb.setInterleaved(False)
        self.cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        # Adjust the camera to flip the image vertically
        self.cam_rgb.setImageOrientation(dai.CameraImageOrientation.VERTICAL_FLIP)

        # Create an XLink output to send the RGB video to the host
        self.xout_video = self.pipeline.create(dai.node.XLinkOut)
        self.xout_video.setStreamName("video")
        self.cam_rgb.video.link(self.xout_video.input)

        # # Connect to device and start the pipeline
        # with dai.Device(self.pipeline) as self.device:
        self.device = dai.Device(self.pipeline)
        # Output queue will be used to get the rgb frames from the output defined above
        self.q_rgb = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)
        
        # Read the a camera frame as a double check of the properties
        # > Read the frame
        in_rgb = self.q_rgb.get()  # Blocking call, will wait until a new data has arrived
        current_frame = in_rgb.getCvFrame()
        # > Get the dimensions of the frame
        dimensions = current_frame.shape
        # > Display the dimensions
        self.get_logger().info("[STREAM OAKD NODE] Check of resolution, a frame captured just now has dimensions = " + str(dimensions))


        # Create a CvBridge object for converting between OpenCV images and ROS Image messages
        self.cv_bridge = CvBridge()

        # For publishing the image 
        self.img_pub = self.create_publisher(
            # Image,"image",qos_profile_system_default
            Image,
            "image",
            qos_profile = custom_qos_profile
        )

        # Timer callback to publish the image
        # timer_period = (1.0/self.cap.get(cv2.CAP_PROP_FPS))
        timer_period = (1.0/30.0)
        self.timer = self.create_timer(timer_period,self.camera_callback)


    # Callback function for publishing the image
    def camera_callback(self):
        try:
            in_rgb = self.q_rgb.get()  # Blocking call, will wait until a new data has arrived
            frame = in_rgb.getCvFrame()
            
            # # Publish image straight from camera to the topic /image
            # self.img_pub.publish(self.cv_bridge.cv2_to_imgmsg(self.frame, "bgr8"))

            frame_small = cv2.resize(frame, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_AREA)

            # Publish the modified image to the topic /image
            frame_mono = cv2.cvtColor(frame_small, cv2.COLOR_BGR2GRAY)
            self.img_pub.publish(self.cv_bridge.cv2_to_imgmsg(frame_mono, "mono8"))

            # self.get_logger().error("[STREAM_OAKD_NODE] No frame received from camera.")
        
        except CvBridgeError as e:
            self.get_logger().error("[STREAM_OAKD_NODE] CvBridgeError: {0}".format(e))
 


def main(args=None):
    rclpy.init(args=args)
    node = OAKDPublishingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()