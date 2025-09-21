#!/usr/bin/env python3

from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
from types import SimpleNamespace


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from ai4r_interfaces.msg import ConePointsArray


from transforms import CameraToWorld

CAMERA_FPS = 10
DETECT_FPS = 10
CAMERA_HEIGHT = 280    # mm
CAMERA_ALPHA = 20      # degrees
X_THRESHOLD = 3000     # mm ; only consider cones within 3m distance of the car
Z_THRESHOLD = 100      # mm ; ignore detections with height > 10 cm ; likely misdetections/noise
ENABLE_IRDOT = True     # Enables IR Dot Projection if True

OPT_INFERENCE = False
# Settings for Optimizing Inference
NUM_INFERENCE_THREADS = 1
INFERENCE_IS_BLOCKING = False
DETECTOR_QUEUE_SIZE = 1

STRETCH = True

if STRETCH:
    NN_BLOB_PATH = '/home/asc/ai4r-system/ros2_ws/src/ai4r_pkg/scripts/models/yolov8n_cones_3510_yb_st_100_5s.blob'
    PREVIEW_KEEP_ASPECT_RATIO = False
else:
    NN_BLOB_PATH = '/home/asc/ai4r-system/ros2_ws/src/ai4r_pkg/scripts/models/yolov8n_det_3510_yb_5s.blob'
    PREVIEW_KEEP_ASPECT_RATIO = True

STREAM_PREVIEW = False
STREAM_DEPTH = False
SYNCNN = True
LABELMAP = ["Yellow", "Blue"]

class SpatialConeDetectorNode(Node):
    # def __init__(self, configs):
    def __init__(self):
        super().__init__('cone_detector_node')

        self.nnBlobPath = NN_BLOB_PATH
        self.camera_height = CAMERA_HEIGHT
        self.camera_alpha = CAMERA_ALPHA
        self.cam2world = CameraToWorld(self.camera_height, self.camera_alpha)
        self.labelMap = ["Yellow", "Blue"]   # label map for detected objects
        self.x_threshold = X_THRESHOLD
        self.z_threshold = Z_THRESHOLD
        self.dot_projector = ENABLE_IRDOT
        self.label_map = LABELMAP
        self.pipeline = self.setup_spatial_detection_pipeline() 

        # # Connect to device and start the pipeline
        # with dai.Device(self.pipeline) as self.device:
        self.device = dai.Device(self.pipeline)
        # Output queues will be used to get the rgb frames and nn data from the outputs defined above

        # Enable IR Dot Projection
        if self.dot_projector: self.device.setIrLaserDotProjectorIntensity(1.0)

        # TODO: Increase Queue Size and Test
        self.detectionNNQueue = self.device.getOutputQueue(name="detections", maxSize=1, blocking=False)
        if STREAM_PREVIEW: self.previewQueue = self.device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
        if STREAM_DEPTH: self.depthQueue = self.device.getOutputQueue(name="depth", maxSize=1, blocking=False)

        # For publishing the detected cones
        self.cone_publisher = self.create_publisher(ConePointsArray, 'detected_cones', qos_profile_system_default)
        # TODO: Add publishers for rgb preview and depth
        if STREAM_PREVIEW: self.rgb_publisher = []
        if STREAM_DEPTH: self.depth_publisher = []

        # Timer callback to detect cones
        # detect_fps = 10
        timer_period = (1.0/DETECT_FPS)
        self.timer = self.create_timer(timer_period, self.cone_detection_callback)
    
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
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoLeft.setImageOrientation(dai.CameraImageOrientation.VERTICAL_FLIP)  # Flip the image vertically

        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")
        monoRight.setImageOrientation(dai.CameraImageOrientation.VERTICAL_FLIP)  # Flip the image vertically

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

        stereo.depth.link(spatialDetectionNetwork.inputDepth)

        # Setup Node for RGB Stream
        if STREAM_PREVIEW:
            xoutRgb = pipeline.create(dai.node.XLinkOut)
            xoutRgb.setStreamName("rgb")
            if SYNCNN:
                spatialDetectionNetwork.passthrough.link(xoutRgb.input)
            else:
                camRgb.preview.link(xoutRgb.input)
        
        # Setup Node for Depth Stream
        if STREAM_DEPTH:
            xoutDepth = pipeline.create(dai.node.XLinkOut)
            xoutDepth.setStreamName("depth")
            spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)
        
        return pipeline
    
    # Callback function for detecting cones
    def cone_detection_callback(self):

        inDet = self.detectionNNQueue.get()
        if STREAM_PREVIEW:
            inPreview = self.previewQueue.get()
            frame = inPreview.getCvFrame()
            # If the frame is available, draw bounding boxes on it and show the frame
            height = frame.shape[0]
            width  = frame.shape[1]

        detections = inDet.detections

        # detection_coordinates = []
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

            # Streaming Preview Frame
            if STREAM_PREVIEW:
                # Denormalize bounding box
                x1 = int(detection.xmin * width)
                x2 = int(detection.xmax * width)
                y1 = int(detection.ymin * height)
                y2 = int(detection.ymax * height)
                cv2.putText(frame, str(self.label_map[label]), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"X: {x_w/10:.2f} cm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Y: {y_w/10:.2f} cm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Z: {z_w/10:.2f} cm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), cv2.FONT_HERSHEY_SIMPLEX)
                # TODO: Publish Preview Frame to ROS
            
            # Streaming Depth Frame
            if STREAM_DEPTH: 
                depth = self.depthQueue.get()
                depthFrame = depth.getFrame() # depthFrame values are in millimeters

                depth_downscaled = depthFrame[::4]
                if np.all(depth_downscaled == 0):
                    min_depth = 0  # Set a default minimum depth value when all elements are zero
                else:
                    min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
                max_depth = np.percentile(depth_downscaled, 99)
                depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
                depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)
                # TODO: Publish Depth Frame to ROS


            if x_w < self.x_threshold and z_w < self.z_threshold:
                detection_x_coordinates.append(x_w)
                detection_y_coordinates.append(y_w)
                # detection_coordinates.append((int(x_w), int(y_w)))    # Converted to integers to reduce data; units: mm
                detection_colors.append(label)                        # Label 0: yellow 1: blue
                #print(f"Cone Detected at: {x_w/10:.2f}, {y_w/10:.2f}, {z_w/10:.2f}; Cone Color: {self.labelMap(label)}, Confidence: {detection.confidence*100:.2f}%")
                number_of_cones += 1

        # self.get_logger().info("[CONE DETECTOR NODE] Detection X Coordinates List: " + str(detection_x_coordinates))
        # self.get_logger().info("[CONE DETECTOR NODE] Detection Y Coordinates List: " + str(detection_y_coordinates))
        # self.get_logger().info("[CONE DETECTOR NODE] Detection Colors List: " + str(detection_colors))

        # Publish the detected cones to the topic /detected_cones
        msg = ConePointsArray()
        msg.x = detection_x_coordinates
        msg.y = detection_y_coordinates
        msg.c = detection_colors
        msg.n = number_of_cones
        self.cone_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpatialConeDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()