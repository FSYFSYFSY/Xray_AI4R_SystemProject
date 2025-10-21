#!/usr/bin/env python3

from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
from types import SimpleNamespace

import os
import json
from array import array


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from ai4r_interfaces.msg import ConePointsArray
from transforms import CameraToWorld

import threading

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty, UInt8

CAMERA_FPS = 10
CAMERA_HEIGHT = 290    # mm
CAMERA_ALPHA = 22      # degrees
X_THRESHOLD = 4000     # mm ; only consider cones within 3m distance of the car
Z_THRESHOLD = 100      # mm ; ignore detections with height > 10 cm ; likely misdetections/noise
ENABLE_IRDOT = True     # Enables IR Dot Projection if True
ACCEPTABLE_NUMBER_OF_SECS_WITHOUT_CONES = 4     # Number of seconds without cones before cv status is not good

OPT_INFERENCE = True
# Settings for Optimizing Inference
NUM_INFERENCE_THREADS = 2
INFERENCE_IS_BLOCKING = False
DETECTOR_QUEUE_SIZE = 1

STRETCH = True

if STRETCH:
    NN_BLOB_PATH = '/home/asc/ai4r-system/ros2_ws/src/ai4r_pkg/scripts/models/yolov8n_cones_3510_yb_st_100_5s.blob'
    PREVIEW_KEEP_ASPECT_RATIO = False
else:
    NN_BLOB_PATH = '/home/asc/ai4r-system/ros2_ws/src/ai4r_pkg/scripts/models/yolov8n_det_3510_yb_5s.blob'
    PREVIEW_KEEP_ASPECT_RATIO = True

SYNCNN = True
LABELMAP = ["Yellow", "Blue"]

PREVIEW_STREAM = False
DEPTH_STREAM = False


class SpatialConeDetectorNode(Node):
    # def __init__(self, configs):
    def __init__(self, args):
        super().__init__('cone_detector_node')

        # QoS Profile for publishing images
        custom_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create a CvBridge object for converting between OpenCV images and ROS Image messages
        self.cv_bridge = CvBridge()

        # Log the command line arguments received by the node
        self.get_logger().info(f"[CONE DETECTOR NODE] Command line args: {args}") 

        # For publishing the bounding box image 
        self.bbox_img_pub = self.create_publisher(
            # Image,"image",qos_profile_system_default
            Image,
            "bbox_image",
            qos_profile = custom_qos_profile
        )
        # Whether the node should be initialised with this stream on or not
        if "--preview" in args:
            global PREVIEW_STREAM 
            PREVIEW_STREAM = True
            self.get_logger().info(f"[INFO] launched with --preview flag, hence variable PREVIEW_STREAM = {PREVIEW_STREAM}")
        # For storing the toggle state of the bounding box image streaming. Default is NOT streaming.
        self.bbox_img_stream_on = PREVIEW_STREAM
        # For subscribing to the bounding box streaming toggle topic
        self.bbox_img_stream_toggle_sub = self.create_subscription(
            Empty,
            "bbox_img_stream_toggle",
            self.bbox_img_stream_toggle_callback,
            10
        )

        # For publishing the depth image
        self.depth_img_pub = self.create_publisher(
            Image,
            "depth_image",
            qos_profile = custom_qos_profile
        )
        # Whether the node should be initialised with this stream on or not
        if "--depth" in args:
            global DEPTH_STREAM 
            DEPTH_STREAM = True
            self.get_logger().info(f"[INFO] launched with --depth flag, hence variable DEPTH_STREAM = {DEPTH_STREAM}")

        # For storing the toggle state of the depth image streaming. Default is NOT streaming.
        self.depth_img_stream_on = DEPTH_STREAM
        # For subscribing to the depth image streaming toggle topic
        self.depth_img_stream_toggle_sub = self.create_subscription(
            Empty,
            "depth_img_stream_toggle",
            self.depth_img_stream_toggle_callback,
            10
        )

        self.nnBlobPath = NN_BLOB_PATH
        self.camera_height = CAMERA_HEIGHT
        self.camera_alpha = CAMERA_ALPHA
        self.cam2world = CameraToWorld(self.camera_height, self.camera_alpha)
        self.x_threshold = X_THRESHOLD
        self.z_threshold = Z_THRESHOLD
        self.dot_projector = ENABLE_IRDOT
        self.label_map = LABELMAP
        self.pipeline = self.setup_spatial_detection_pipeline() 

        # self.detectionNNQueue = self.device.getOutputQueue(name="detections", maxSize=1, blocking=False)

        # For publishing the detected cones
        self.cone_publisher = self.create_publisher(ConePointsArray, 'cone_points', qos_profile_system_default)
        self.shutdown_flag = threading.Event()

        # For publishing the cv status
        self.cv_status_publisher = self.create_publisher(UInt8, 'cv_status', qos_profile_system_default)


    def bbox_img_stream_toggle_callback(self, msg):
        self.bbox_img_stream_on = not self.bbox_img_stream_on
        self.get_logger().info(f"[CONE DETECTOR NODE] Bounding Box Image Streaming: {self.bbox_img_stream_on}")

    def depth_img_stream_toggle_callback(self, msg):
        self.depth_img_stream_on = not self.depth_img_stream_on
        self.get_logger().info(f"[CONE DETECTOR NODE] Depth Image Streaming: {self.depth_img_stream_on}")

    
    @staticmethod
    def setup_camrgb(camRgb):
        # Camera properties
        camRgb.setPreviewSize(640, 640)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setImageOrientation(dai.CameraImageOrientation.VERTICAL_FLIP)    # Flip the image vertically due to reverse camera mount
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR) 
        camRgb.setPreviewKeepAspectRatio(PREVIEW_KEEP_ASPECT_RATIO)     # Stretch Images
        camRgb.setFps(CAMERA_FPS)  # Set Camera FPS to 10 to match NN
    
    @staticmethod
    def setup_stereo(monoLeft, monoRight, stereo):
        # Stereo Camra properties
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        monoLeft.setCamera("left")
        #monoLeft.setImageOrientation(dai.CameraImageOrientation.VERTICAL_FLIP)  # Flip the image vertically

        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        monoRight.setCamera("right")
        #monoRight.setImageOrientation(dai.CameraImageOrientation.VERTICAL_FLIP)  # Flip the image vertically

        # setting node configs
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Align depth map to the perspective of RGB camera, on which inference is done
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
        stereo.setSubpixel(True)
    
    def setup_sdn(self, spatialDetectionNetwork):
        # Spatial Detection Network Configs
        spatialDetectionNetwork.setBlobPath(self.nnBlobPath)
        spatialDetectionNetwork.setConfidenceThreshold(0.5)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        # Yolo specific parameters
        spatialDetectionNetwork.setNumClasses(2)
        spatialDetectionNetwork.setCoordinateSize(4)
        spatialDetectionNetwork.setIouThreshold(0.5)

        # Additional Settings
        if OPT_INFERENCE:
            spatialDetectionNetwork.setNumInferenceThreads(NUM_INFERENCE_THREADS)
            spatialDetectionNetwork.input.setBlocking(INFERENCE_IS_BLOCKING)
            spatialDetectionNetwork.input.setQueueSize(DETECTOR_QUEUE_SIZE)   # Makes sure the frames are real-time

    def setup_spatial_detection_pipeline(self):
        pipeline = dai.Pipeline()

        # Define sources and outputs
        camRgb = pipeline.create(dai.node.ColorCamera)
        spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        # Output Nodes
        xoutNN = pipeline.create(dai.node.XLinkOut)

        # Set output streams
        xoutNN.setStreamName("detections")

        # Setup Nodes
        self.setup_camrgb(camRgb)
        self.setup_stereo(monoLeft, monoRight, stereo)
        self.setup_sdn(spatialDetectionNetwork)

        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        camRgb.preview.link(spatialDetectionNetwork.input)

        spatialDetectionNetwork.out.link(xoutNN.input)

        # Manipulation to flip the depth by 180 degrees

        # Create ImageManip node
        manipDepth = pipeline.create(dai.node.ImageManip)
        manipDepth.initialConfig.setVerticalFlip(True)
        manipDepth.initialConfig.setFrameType(dai.ImgFrame.Type.RAW16)

        # Set the maximum output frame size to handle the larger frame
        manipDepth.setMaxOutputFrameSize(1843200)

        # Link setereo depth output to ImageManip input
        stereo.depth.link(manipDepth.inputImage)

        # Link ImageManip output to the spatial detection network depth input
        manipDepth.out.link(spatialDetectionNetwork.inputDepth)

        #stereo.depth.link(spatialDetectionNetwork.inputDepth)

        # Setup Node for RGB Stream
        if PREVIEW_STREAM:
            xoutRgb = pipeline.create(dai.node.XLinkOut)
            xoutRgb.setStreamName("rgb")
            if SYNCNN:
                spatialDetectionNetwork.passthrough.link(xoutRgb.input)
            else:
                camRgb.preview.link(xoutRgb.input)
        
        # Setup Node for Depth Stream
        if DEPTH_STREAM:
            xoutDepth = pipeline.create(dai.node.XLinkOut)
            xoutDepth.setStreamName("depth")
            spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

        return pipeline
    
    def run_detection(self):
        with dai.Device(self.pipeline) as device:
            # Enable IR Dot Projection
            if self.dot_projector:
                device.setIrLaserDotProjectorIntensity(1.0)
            detectionNNQueue = device.getOutputQueue(
                name="detections", maxSize=4, blocking=False)

            if PREVIEW_STREAM: previewQueue = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
            if DEPTH_STREAM: depthQueue = device.getOutputQueue(name="depth", maxSize=1, blocking=False)

            count_without_cones = 0

            while not self.shutdown_flag.is_set():
                inDet = detectionNNQueue.get()
                if PREVIEW_STREAM:
                    inPreview = previewQueue.get()
                    frame = inPreview.getCvFrame()
                    self.height = frame.shape[0]
                    self.width = frame.shape[1]
                else:
                    frame = None

                if DEPTH_STREAM:
                    inDepth = depthQueue.get()
                    depth_frame = inDepth.getFrame()
                else:
                    depth_frame = None

                detections = inDet.detections
                
                if detections:
                    count_without_cones = 0
                else:
                    count_without_cones += 1

                if count_without_cones >= ACCEPTABLE_NUMBER_OF_SECS_WITHOUT_CONES * CAMERA_FPS:
                    count_without_cones = 0
                    # Publish cv_status
                    msg = UInt8()
                    msg.data = 1
                    self.cv_status_publisher.publish(msg)

                # Publish even if no detections are made
                detection_data = self.process_detections(
                    detections, frame, depth_frame)
                self.publish_data(detection_data)
    
    def process_detections(self, detections, frame, depth_frame):
        detection_x_coordinates = []
        detection_y_coordinates = []
        detection_colors = []
        number_of_cones = 0

        for detection in detections:
            label = detection.label

            # Process Spatial Co-ordinates
            x_c = detection.spatialCoordinates.x
            y_c = detection.spatialCoordinates.y
            z_c = detection.spatialCoordinates.z

            # Transform coordinates to body frame
            x_w, y_w, z_w = self.cam2world.transform_to_body_frame(x_c, y_c, z_c)

            if x_w < self.x_threshold and z_w < self.z_threshold:
                detection_x_coordinates.append(x_w)
                detection_y_coordinates.append(y_w)
                detection_colors.append(label)
                number_of_cones += 1

            # Streaming Preview Frame
            if PREVIEW_STREAM:
                # Denormalize bounding box
                x1 = int(detection.xmin * self.width)
                x2 = int(detection.xmax * self.width)
                y1 = int(detection.ymin * self.height)
                y2 = int(detection.ymax * self.height)
                cv2.putText(frame, str(self.label_map[label]), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"X: {x_w/10:.2f} cm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Y: {y_w/10:.2f} cm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Z: {z_w/10:.2f} cm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), cv2.FONT_HERSHEY_SIMPLEX)
            
        # Streaming Depth Frame
        if DEPTH_STREAM: 
            depth_downscaled = depth_frame[::4]
            if np.all(depth_downscaled == 0):
                min_depth = 0  # Set a default minimum depth value when all elements are zero
            else:
                min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
            max_depth = np.percentile(depth_downscaled, 99)
            depthFrameColor = np.interp(depth_frame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
            depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

        if self.bbox_img_stream_on:
            if frame is None or getattr(frame, "size", 0) == 0:
                self.get_logger().warn(f"[WARN] frame is empty. Relaunch with flag: --preview")
            else:
                # Publish the grayscale bounding box (preview) image to the topic /bbox_image
                frame_mono = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                self.bbox_img_pub.publish(self.cv_bridge.cv2_to_imgmsg(frame_mono, "mono8"))
        if self.depth_img_stream_on:
            # Publish the grayscale depth image to the topic /depth_image
            # depth_mono = cv2.cvtColor(depthFrameColor, cv2.COLOR_BGR2GRAY)
            # self.depth_img_pub.publish(self.cv_bridge.cv2_to_imgmsg(depth_mono, "mono8"))
            # Publish the color depth image to the topic /depth_image
            if depthFrameColor is None or getattr(depthFrameColor, "size", 0) == 0:
                self.get_logger().warn(f"[WARN] depth frame is empty. Relaunch with flag: --depth")
            else:
                self.depth_img_pub.publish(self.cv_bridge.cv2_to_imgmsg(depthFrameColor, "bgr8"))
        
        # self.get_logger().info(f"{number_of_cones} cones detected!")

        return {
            'x': detection_x_coordinates,
            'y': detection_y_coordinates,
            'c': detection_colors,
            'n': number_of_cones
        }
    
    def publish_data(self, data):
        msg = ConePointsArray()
        msg.x = data['x']
        msg.y = data['y']
        msg.c = data['c']
        msg.n = data['n']

        filename = os.path.join(os.getcwd(), "detection_results.json")
        frame_data = {
            "x_list": [float(x) for x in msg.x],
            "y_list": [float(y) for y in msg.y],
            "c_list": [int(c) for c in msg.c],
            "n": int(msg.n)
        }
    

        # Append JSON object as a line to the file
        with open(filename, 'a', encoding='utf-8') as f:
            f.write(json.dumps(frame_data) + "\n")

        self.cone_publisher.publish(msg)


def main():
    rclpy.init()
    args = sys.argv[1:]
    detector = SpatialConeDetectorNode(args)
    detection_thread = threading.Thread(target = detector.run_detection)

    try:
        detection_thread.start()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.stop()
        detection_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()