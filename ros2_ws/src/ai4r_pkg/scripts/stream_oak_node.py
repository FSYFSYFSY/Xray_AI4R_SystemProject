#!/usr/bin/env python3

# dependencies: pip install depthai; pip install opencv-python

import depthai as dai
import cv2 

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data, QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty

CAPTURE_FPS = 40
CAPTURE_FPS_DIVIDE_BY_STREAM_FPS = 4

class OAKDPublishingNode(Node):

    def __init__(self):
        super().__init__('stream_oak_node')

        custom_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.get_logger().info("[STREAM OAKD NODE] Now setting up the camera.")

        # Create a pipeline
        self.pipeline = dai.Pipeline()

        # Define a source - color camera
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.camRgb.setPreviewSize(640,640)  # Modified preview size to 640x640 for the Yolov8n model
        self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        self.camRgb.setImageOrientation(dai.CameraImageOrientation.VERTICAL_FLIP)    # Flip the image vertically
        self.camRgb.setPreviewKeepAspectRatio(False)
        self.camRgb.setInterleaved(False)
        self.camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        self.camRgb.setFps(CAPTURE_FPS)

        # Create an XLink output to send the RGB video to the host
        self.xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        self.xoutRgb.setStreamName("rgb")
        self.camRgb.preview.link(self.xoutRgb.input)

        # # Connect to device and start the pipeline
        # with dai.Device(self.pipeline) as self.device:
        self.device = dai.Device(self.pipeline)
        # Output queue will be used to get the rgb frames from the output defined above
        self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        # Read the a camera frame as a double check of the properties
        # > Read the frame
        inRgb = self.qRgb.get()  # Blocking call, will wait until a new data has arrived
        current_frame = inRgb.getCvFrame()
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

        # For storing the toggle state of the camera streaming
        self.cam_stream_on = True

        # For subscribing to the camera streaming toggle topic
        self.cam_stream_toggle_sub = self.create_subscription(
            Empty,
            "cam_stream_toggle",
            self.cam_stream_toggle_callback,
            10
        )

        # Timer callback to publish the image
        # timer_period = (1.0/self.cap.get(cv2.CAP_PROP_FPS))
        stream_fps = float(CAPTURE_FPS / CAPTURE_FPS_DIVIDE_BY_STREAM_FPS)
        timer_period = (1.0/stream_fps)
        self.timer = self.create_timer(timer_period,self.camera_callback)


    # Callback function for publishing the image
    def camera_callback(self):
        if self.cam_stream_on:
            try:
                inRgb = self.qRgb.get()  # Blocking call, will wait until a new data has arrived
                frame = inRgb.getCvFrame()
                
                # # Publish image straight from camera to the topic /image
                # self.img_pub.publish(self.cv_bridge.cv2_to_imgmsg(self.frame, "bgr8"))

                # frame_small = cv2.resize(frame, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_AREA) # Resize the frame
                frame_small = frame # Don't resize the frame

                # Publish the modified image to the topic /image
                frame_mono = cv2.cvtColor(frame_small, cv2.COLOR_BGR2GRAY)
                self.img_pub.publish(self.cv_bridge.cv2_to_imgmsg(frame_mono, "mono8"))

                # self.get_logger().error("[STREAM_OAKD_NODE] No frame received from camera.")
            
            except CvBridgeError as e:
                self.get_logger().error("[STREAM_OAKD_NODE] CvBridgeError: {0}".format(e))
 

    # Callback function for toggling the camera streaming
    def cam_stream_toggle_callback(self, msg):
        if self.cam_stream_on:
            self.cam_stream_on = False
            self.get_logger().info("[STREAM OAKD NODE] Camera streaming turned off.")
        else:
            self.cam_stream_on = True
            self.get_logger().info("[STREAM OAKD NODE] Camera streaming turned on.")


def main(args=None):
    rclpy.init(args=args)
    node = OAKDPublishingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()