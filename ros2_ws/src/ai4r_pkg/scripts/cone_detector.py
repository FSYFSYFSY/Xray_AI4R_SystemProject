#!/usr/bin/env python3
from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
from types import SimpleNamespace

from transforms import CameraToWorld

class SpatialConeDetector:
    def __init__(self, configs):
        self.nnBlobPath = configs.nn_blob_path
        self.pipeline = self.setup_spatial_detection_pipeline(configs)
        self.cam2world = CameraToWorld(configs.camera_height, configs.camera_alpha)
        self.labelMap = configs.labelMap
        self.x_threshold = configs.x_threshold
        self.z_threshold = configs.z_threshold
    
    @staticmethod
    def setup_camrgb(camRgb):
        # Camera properties
        camRgb.setPreviewSize(640, 640)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setImageOrientation(dai.CameraImageOrientation.VERTICAL_FLIP)    # Flip the image vertically due to reverse camera mount
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        #camRgb.setFps(40)  # Set Camera FPS
    
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
        #spatialDetectionNetwork.setNumInferenceThreads(2)
        #spatialDetectionNetwork.input.setBlocking(False)

    def setup_spatial_detection_pipeline(self, configs):
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

        return pipeline
    
    def start_pipeline(self):
        # Connect to OAKD device and start pipeline
        with dai.Device(self.pipeline) as device:
            # Output queues will be used to get the rgb frames and nn data from the outputs defined above
            detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)

            while True:
                inDet = detectionNNQueue.get()
                detections = inDet.detections

                detection_coordinates = []
                detection_colors = []

                for detection in detections:
                    label = detection.label

                    # Process Spatial Co-ordinates
                    x_c = detection.spatialCoordinates.x
                    y_c = detection.spatialCoordinates.y
                    z_c = detection.spatialCoordinates.z

                    # Transform coordinates to body frame
                    x_w, y_w, z_w = self.cam2world.transform_to_body_frame(x_c, y_c, z_c)

                    if x_w < self.x_threshold and z_w < self.z_threshold:
                        detection_coordinates.append((int(x_w), int(y_w)))    # Converted to integers to reduce data; units: mm
                        detection_colors.append(label)                        # Label 0: yellow 1: blue
                        #print(f"Cone Detected at: {x_w/10:.2f}, {y_w/10:.2f}, {z_w/10:.2f}; Cone Color: {self.labelMap(label)}, Confidence: {detection.confidence*100:.2f}%")

                print("Detection Coordinates List: ", detection_coordinates)
                print("Detection Colors List: ", detection_colors)

                # These two lists `detection_coordinates` and `detection_colors` are meant to be published to ROS



detector_configs = SimpleNamespace(
    nn_blob_path = 'models/yolov8n_det_3510_yb_6shave.blob',
    camera_height = 290,    # mm
    camera_alpha = 20,      # degrees
    labelMap = ["Yellow", "Blue"],   # label map for detected objects
    x_threshold = 3000,     # mm ; only consider cones within 3m distance of the car
    z_threshold = 100,       # mm ; ignore detections with height > 10 cm ; likely misdetections/noise
)

cone_detector = SpatialConeDetector(detector_configs)
cone_detector.start_pipeline()