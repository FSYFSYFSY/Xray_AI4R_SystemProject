#!/usr/bin/env python3
import yaml
import os  
import cv2 
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from LineDetectorCircle import LineDetectorCircle
from LineDetectorEllipse import LineDetectorEllipse
from Utils import Util
#from ai4r_interfaces.msg import IntArray, ElementInt
from ai4r_interfaces.msg import ImagePointsArray, PixelCoordsFloat32
from std_msgs.msg import Bool as Bool1
from std_msgs.msg import UInt16 as UInt16_msg
from rclpy.parameter import Parameter

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

USB_PORT = 0

# Import any python modules or libraries here
#from ai4r_pkg.module_to_import import function


# Create a class which inherits from the rclpy Node class. Each ROS2 Python node that you’ll write will be a superset of “rclpy.node.Node”.
class LineDetectorNode(Node):
    # Create publishers, subscribers, and other ROS2 functionalities in the constructor
    def __init__(self):
        # Initialize the Node object, and we pass the node name to it.
        super().__init__('line_detector_node')

        # Log the namespace
        self.get_logger().info("[LINE DETECTOR NODE] starting __init__ with node namespace = " + self.get_namespace())

        # Declare the parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ("camera_resolution", rclpy.Parameter.Type.STRING),
                ("camera_frame_rate",rclpy.Parameter.Type.INTEGER),
                ("camera_auto_focus",rclpy.Parameter.Type.INTEGER),
                ("camera_focus",rclpy.Parameter.Type.INTEGER),
                ("camera_auto_exposure",rclpy.Parameter.Type.INTEGER),
                ("hls_select", rclpy.Parameter.Type.STRING_ARRAY),
                ("h_range", rclpy.Parameter.Type.INTEGER_ARRAY),
                ("l_range", rclpy.Parameter.Type.INTEGER_ARRAY),
                ("s_range", rclpy.Parameter.Type.INTEGER_ARRAY),
                ("morph", rclpy.Parameter.Type.BOOL),
                ("erosion", rclpy.Parameter.Type.BOOL),
                ("dilation", rclpy.Parameter.Type.BOOL),
                ("opening", rclpy.Parameter.Type.BOOL),
                ("closing", rclpy.Parameter.Type.BOOL),
                ("kernel_erode", rclpy.Parameter.Type.INTEGER_ARRAY),
                ("kernel_dilate",rclpy.Parameter.Type.INTEGER_ARRAY),
                ("kernel_open",rclpy.Parameter.Type.INTEGER_ARRAY),
                ("kernel_close",rclpy.Parameter.Type.INTEGER_ARRAY),
                ("largcon",rclpy.Parameter.Type.BOOL),
                ("blur",rclpy.Parameter.Type.BOOL),
                ("kernel_blur",rclpy.Parameter.Type.INTEGER_ARRAY),
                ("shape",rclpy.Parameter.Type.STRING),
                ("radius",rclpy.Parameter.Type.INTEGER),
                ("start_angle",rclpy.Parameter.Type.INTEGER),
                ("end_angle",rclpy.Parameter.Type.INTEGER),
                ("a",rclpy.Parameter.Type.INTEGER),
                ("b",rclpy.Parameter.Type.INTEGER),
                ("draw",rclpy.Parameter.Type.BOOL)
            ])
        self.res = self.get_parameter_or("camera_resolution",Parameter("default_camera_resolution",Parameter.Type.STRING,"480p"))
        self.frame_rate = self.get_parameter_or("camera_frame_rate",Parameter("default_camera_frame_rate",Parameter.Type.INTEGER,30))
        self.auto_focus = self.get_parameter_or("camera_auto_focus",Parameter("default_camera_auto_focus",Parameter.Type.INTEGER,0))
        self.focus = self.get_parameter_or("camera_focus",Parameter("default_camera_focus",Parameter.Type.INTEGER,0))
        self.auto_exposure = self.get_parameter_or("camera_auto_exposure",Parameter("default_camera_exposure",Parameter.Type.INTEGER,0))
        self.hls_select = self.get_parameter_or("hls_select",Parameter("default_hls",Parameter.Type.STRING_ARRAY,['Saturation']))
        self.h_range = self.get_parameter_or("h_range",Parameter("default_h_range",Parameter.Type.INTEGER_ARRAY,[]))
        self.l_range = self.get_parameter_or("l_range",Parameter("default_l_range",Parameter.Type.INTEGER_ARRAY,[]))
        self.s_range = self.get_parameter_or("s_range",Parameter("default_s_range",Parameter.Type.INTEGER_ARRAY,[100,255]))
        self.morph = self.get_parameter_or("morph",Parameter("default_morph",Parameter.Type.BOOL,False))
        self.erosion = self.get_parameter_or("erosion",Parameter("default_erosion",Parameter.Type.BOOL,False))
        self.dilation = self.get_parameter_or("dilation",Parameter("default_dilation",Parameter.Type.BOOL,False))
        self.opening = self.get_parameter_or("opening",Parameter("default_opening",Parameter.Type.BOOL,False))
        self.closing = self.get_parameter_or("closing",Parameter("default_closing",Parameter.Type.BOOL,False))
        self.kernel_erode = self.get_parameter_or("kernel_erode",Parameter("default_kernerode",Parameter.Type.INTEGER_ARRAY,[]))
        self.kernel_dilate = self.get_parameter_or("kernel_dilate",Parameter("default_kerndila",Parameter.Type.INTEGER_ARRAY,[]))
        self.kernel_open = self.get_parameter_or("kernel_open",Parameter("default_kernopen",Parameter.Type.INTEGER_ARRAY,[]))
        self.kernel_close = self.get_parameter_or("kernel_close",Parameter("default_kernclose",Parameter.Type.INTEGER_ARRAY,[]))
        self.largcon = self.get_parameter_or("largcon",Parameter("default_largcon",Parameter.Type.BOOL,False))
        self.blur = self.get_parameter_or("blur",Parameter("default_blur",Parameter.Type.BOOL,False))
        self.kernel_blur = self.get_parameter_or("kernel_blur",Parameter("default_kernblur",Parameter.Type.INTEGER_ARRAY,[]))
        self.shape = self.get_parameter_or("shape",Parameter("default_shape",Parameter.Type.STRING,"Circle"))
        self.radius = self.get_parameter_or("radius",Parameter("default_radius",Parameter.Type.INTEGER,50))
        self.start_angle = self.get_parameter_or("start_angle",Parameter("default_startangle",Parameter.Type.INTEGER,210))
        self.end_angle = self.get_parameter_or("end_angle",Parameter("default_endangle",Parameter.Type.INTEGER,330))
        self.a = self.get_parameter_or("a",Parameter("default_a",Parameter.Type.INTEGER,0))
        self.b = self.get_parameter_or("b",Parameter("default_b",Parameter.Type.INTEGER,0))
        self.draw = self.get_parameter_or("draw",Parameter("default_draw",Parameter.Type.BOOL,True))

        # Log all on the parameters values that were retrieved
        self.get_logger().info("camera_resolution = " + str(self.res.value))
        self.get_logger().info("camera_frame_rate = " + str(self.frame_rate.value))
        #self.get_logger().info("camera_auto_focus = " + str(self.auto_focus.value))
        #self.get_logger().info("camera_focus = " + str(self.focus.value))
        #self.get_logger().info("camera_auto_exposure = " + str(self.auto_exposure.value))
        self.get_logger().info("hls_select = " + str(self.hls_select.value))
        self.get_logger().info("h_range = " + str(self.h_range.value))
        self.get_logger().info("l_range = " + str(self.l_range.value))
        self.get_logger().info("s_range = " + str(self.s_range.value))
        self.get_logger().info("largcon = " + str(self.largcon.value))
        self.get_logger().info("morph = " + str(self.morph.value))
        if (self.erosion.value):
            self.get_logger().info("kernel_erode = " + str(self.kernel_erode.value))
        if (self.dilation.value):
            self.get_logger().info("kernel_dilate = " + str(self.kernel_dilate.value))
        if (self.opening.value):
            self.get_logger().info("kernel_open = " + str(self.kernel_open.value))
        if (self.closing.value):
            self.get_logger().info("kernel_close = " + str(self.kernel_close.value))
        if (self.blur.value):
            self.get_logger().info("kernel_blur = " + str(self.kernel_blur.value))
        self.get_logger().info("shape = " + str(self.shape.value))
        self.get_logger().info("radius = " + str(self.radius.value))
        self.get_logger().info("start_angle = " + str(self.start_angle.value))
        self.get_logger().info("end_angle = " + str(self.end_angle.value))
        #self.get_logger().info("a = " + str(self.a.value))
        #self.get_logger().info("b = " + str(self.b.value))
        self.get_logger().info("draw = " + str(self.draw.value))

        # Initialise the message for flagging line detection errors
        self.error_flag = Bool1()
        self.error_flag = False

        # Initialise the counter for image without sufficient line detection results
        self.update_count = 0

        # Initialise the "Utils" object
        self.Util1 = Util()
        
        # Open the camera
        self.get_logger().info("[LINE DETECTOR NODE] Now opening the camera capture.")
        self.cap = cv2.VideoCapture(USB_PORT)
        self.get_logger().info("[LINE DETECTOR NODE] Camera capture opened.")

        # Create a CvBridge object for converting between OpenCV images and ROS Image messages
        self.cv_bridge = CvBridge()

        # Set the resolution and frame rate capture properties of the camera
        self.get_logger().info("[LINE DETECTOR NODE] Now setting the camera capture properties of resolution and frame rate.")
        self.set_capture_properties(
            self.cap,
            self.res.value,
            self.frame_rate.value,
            auto_focus=None,
            focus=None,
            exposure=None
            )

        self.get_logger().info("[LINE DETECTOR NODE] Double check of resolution and frame rate (see next few log lines)")
        self.get_logger().info("[LINE DETECTOR NODE] CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        self.get_logger().info("[LINE DETECTOR NODE] CV_CAP_PROP_FRAME_WIDTH :  '{}'".format(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
        self.get_logger().info("[LINE DETECTOR NODE] CAP_PROP_FPS :             '{}'".format(self.cap.get(cv2.CAP_PROP_FPS)))

        # Read the a camera frame as a double check of the properties
        # > Read the frame
        return_flag , current_frame = self.cap.read()
        # > Get the dimensions of the frame
        dimensions = current_frame.shape
        # > Display the dimensions
        self.get_logger().info("[LINE DETECTOR NODE] Triple check of resolution, a frame captured just now has dimensions = " + str(dimensions))
        # > Also check the values
        expected_width, expected_height = self.get_resolution(self.res.value)
        if (not(dimensions[0]==expected_height) or not(dimensions[1]==expected_width)):
           self.get_logger().warn("[LINE DETECTOR NODE] ERROR: frame dimensions do NOT match the desired values.")
        #    # Update the variables
        #    self.camera_frame_height = dimensions[0]
        #    self.camera_frame_width  = dimensions[1]
        
        # Set the focus capture properties of the camera
        #self.get_logger().info("[LINE DETECTOR NODE] Now setting the camera capture properties of focus and auto focus.")
        #self.set_capture_properties(
        #    self.cap,
        #    resolution=None,
        #    frame_rate=None,
        #    auto_focus=self.auto_focus.value,
        #    focus=self.focus.value,
        #    exposure=None
        #    )

        # Initialise the Publishers
        # > For notifying of line detection errors
        self.error_pub = self.create_publisher(
            Bool1, "line_detector_timeout_flag", qos_profile_system_default
        )
        # > For publishing line detection points
        self.img_point_pub = self.create_publisher(
            ImagePointsArray,"image_points",qos_profile_system_default
        )
        # > For publishing the number of line detection points
        self.img_num_points_pub = self.create_publisher(
            UInt16_msg,"image_num_points",qos_profile_system_default
        )
        # > For publishing the image 
        self.img_pub = self.create_publisher(
            # Image,"image",qos_profile_system_default
            Image,"image",qos_profile_sensor_data
        )

        # Log the configuration
        #self.get_logger().info("[LINE DETECTOR NODE] Camera capture properties are:")
        #self.get_logger().info("[LINE DETECTOR NODE] CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        #self.get_logger().info("[LINE DETECTOR NODE] CV_CAP_PROP_FRAME_WIDTH :  '{}'".format(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
        #self.get_logger().info("[LINE DETECTOR NODE] CAP_PROP_FPS :             '{}'".format(self.cap.get(cv2.CAP_PROP_FPS)))
        #self.get_logger().info("[LINE DETECTOR NODE] CAP_PROP_FOCUS :           '{}'".format(self.cap.get(cv2.CAP_PROP_FOCUS)))
        #self.get_logger().info("[LINE DETECTOR NODE] CAP_PROP_AUTOFOCUS :       '{}'".format(self.cap.get(cv2.CAP_PROP_AUTOFOCUS)))
        #self.get_logger().info("[LINE DETECTOR NODE] CAP_PROP_BRIGHTNESS :      '{}'".format(self.cap.get(cv2.CAP_PROP_BRIGHTNESS)))
        #self.get_logger().info("[LINE DETECTOR NODE] CAP_PROP_CONTRAST :        '{}'".format(self.cap.get(cv2.CAP_PROP_CONTRAST)))
        #self.get_logger().info("[LINE DETECTOR NODE] CAP_PROP_SATURATION :      '{}'".format(self.cap.get(cv2.CAP_PROP_SATURATION)))
        #self.get_logger().info("[LINE DETECTOR NODE] CAP_PROP_AUTO_EXPOSURE :   '{}'".format(self.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)))
        #self.get_logger().info("[LINE DETECTOR NODE] CAP_PROP_EXPOSURE :        '{}'".format(self.cap.get(cv2.CAP_PROP_EXPOSURE)))
        #self.get_logger().info("[LINE DETECTOR NODE] CAP_PROP_EXPOSURE_TIME_ABSOLUTE :     '{}'".format(self.cap.get(cv2.CAP_PROP_EXPOSURE_TIME_ABSOLUTE)))
        #self.get_logger().info("[LINE DETECTOR NODE] CAP_PROP_EXPOSURE_DYNAMIC_FRAMERATE : '{}'".format(self.cap.get(cv2.CAP_PROP_EXPOSURE_DYNAMIC_FRAMERATE)))

        # Display the status
        self.get_logger().info("[LINE DETECTOR NODE] Initialisation complete")

        timer_period = (1.0/self.frame_rate.value)
        self.timer = self.create_timer(timer_period,self.camera_callback)

        """
        LOGITECH STREAM CAM PROPERTIES
        The command:
        >> v4l2-ctl --device=/dev/video0 --list-ctrls
        Lists the following
        User Control
                             brightness 0x00980900 (int)    : min=0 max=255 step=1 default=128 value=128
                               contrast 0x00980901 (int)    : min=0 max=255 step=1 default=128 value=128
                             saturation 0x00980902 (int)    : min=0 max=255 step=1 default=128 value=128
                white_balance_automatic 0x0098090c (bool)   : default=1 value=1
                                   gain 0x00980913 (int)    : min=0 max=255 step=1 default=0 value=255
                   power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=2 value=2 (60 Hz)
              white_balance_temperature 0x0098091a (int)    : min=2000 max=7500 step=1 default=4000 value=5904 flags=inactive
                              sharpness 0x0098091b (int)    : min=0 max=255 step=1 default=128 value=128
                 backlight_compensation 0x0098091c (int)    : min=0 max=1 step=1 default=0 value=1

        Camera Controls
                          auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=3 (Aperture Priority Mode)
                 exposure_time_absolute 0x009a0902 (int)    : min=3 max=2047 step=1 default=250 value=333 flags=inactive
             exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=0
                           pan_absolute 0x009a0908 (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                          tilt_absolute 0x009a0909 (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                         focus_absolute 0x009a090a (int)    : min=0 max=255 step=1 default=0 value=30 flags=inactive
             focus_automatic_continuous 0x009a090c (bool)   : default=1 value=1
                          zoom_absolute 0x009a090d (int)    : min=100 max=400 step=1 default=100 value=100
        """



    def camera_callback(self):
        self.ret,self.frame = self.cap.read()
        # # Publish image straight from camera
        # self.get_logger().info("[LINE_DETECTOR_NODE] Publishing video images to topic /image")
        # self.img_pub.publish(self.cv_bridge.cv2_to_imgmsg(self.frame, "bgr8"))
        
        #self.get_logger().info("[LINE DETECTOR NODE] self.frame.shape = " + str(self.frame.shape))
        points, self.img_det = self.process(self.frame)
        #self.get_logger().info("%s" %points)

        # Initialise the message to be sent
        #self.ImgPoint = IntArray()
        self.ImagePoints = ImagePointsArray()
        self.ImagePoints.image_width = self.frame.shape[1]
        self.ImagePoints.image_height = self.frame.shape[0]
        self.ImagePoints.time_since_previous_image = 0.0

        if (self.update_count > ((0.5)*self.frame_rate.value)):
            self.error_flag = True
            trial1 = Bool1()
            trial1.data = self.error_flag
            self.error_pub.publish(trial1)

        if points is None:
            #self.get_logger().info("No Line Found")
            self.update_count += 1
            return
        if points is not None:
            #self.get_logger().info("len(points) = " + str(len(points)) )
            if (len(points) < 4):
                self.update_count += 1
            else:
                self.error_flag = False
                trial1 = Bool1()
                trial1.data = self.error_flag
                self.error_pub.publish(trial1)
                self.update_count = 0

            for i in range(len(points)):
                #new_subpoint = ElementInt()
                #new_subpoint.elements.append(points[i][0])
                #new_subpoint.elements.append(points[i][1])
                #self.ImgPoint.lists.append(new_subpoint)
                new_subpoint = PixelCoordsFloat32()
                new_subpoint.u = float(points[i][0])
                new_subpoint.v = float(points[i][1])
                self.ImagePoints.points.append(new_subpoint)
            
            self.img_point_pub.publish(self.ImagePoints)

            num_points_msg = UInt16_msg()
            num_points_msg.data = len(points)
            self.img_num_points_pub.publish(num_points_msg)

    def load_image(self, image):
        """Converts the image to HLS and returns the HLS channels"""
        hls = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HLS)
        self.h, self.l, self.s = cv2.split(hls)
    
    def threshold_image(self):
        thresholded = []
        if not len(self.hls_select.value) == 0:
            for channel in self.hls_select.value:
                if channel == "Hue":
                    thresholded.append(self.Util1.thresh_image(self.h, self.h_range.value))
                elif channel == "Lightness":
                    thresholded.append(self.Util1.thresh_image(self.l, self.l_range.value))
                elif channel == "Saturation":
                    thresholded.append(self.Util1.thresh_image(self.s, self.s_range.value))

        self.im = self.Util1.combine_images(thresholded)

        return self.im

    def preprocess_image(self):
        """Applies pre-processing operations on the image such as - 
        - Morphological operations (optional)
        - Get Largest contour (optional)
        - Blur (optional)
        """
        # Morphological operations
        if self.morph.value:
            if self.erosion.value:
                self.im = self.Util1.erode(self.im, tuple(self.kernel_erode.value))
            if self.dilation.value:
                self.im = self.Util1.dilate(self.im, tuple(self.kernel_dilate.value))
            if self.opening.value:
                self.im = self.Util1.open_op(self.im, tuple(self.kernel_open.value))
            if self.closing.value:
                self.im = self.Util1.close_op(self.im, tuple(self.kernel_close.value))
        
        # Get largest contour
        if self.largcon.value:
            self.im = self.Util1.get_largest_contour(self.im)

        # Blur operation        
        if self.blur.value:
            self.im = self.Util1.gauss_blur(self.im, tuple(self.kernel_blur.value))

        return self.im

    def detect_line(self):
        """Detects the line using either of the specified methods"""
        if self.shape.value == "Circle":
            ld = LineDetectorCircle(self.radius.value, self.start_angle.value, self.end_angle.value, draw = self.draw.value)
        elif self.shape.value == "Ellipse":
            ld = LineDetectorEllipse(self.a.value, self.b.value, self.start_angle.value, self.end_angle.value, draw = self.draw.value)
        points = ld(self.im)
        img_det = ld.img_color if self.draw.value else None
        return points, img_det
    
    def process(self, image):
        """Processes the image and returns the detected line"""
        self.load_image(self.frame)
        threshold_image = self.threshold_image()
        preprocessed_image = self.preprocess_image()

        points, img_det = self.detect_line()

        # Publish the modified image to the topic /image
        # self.img_pub.publish(self.cv_bridge.cv2_to_imgmsg(preprocessed_image, "mono8"))
        self.img_pub.publish(self.cv_bridge.cv2_to_imgmsg(img_det, "bgr8"))

        return points, img_det

    def get_resolution(self,resolution):
        resolutions = {
            "480p": (640, 480),
            "540p": (960, 540),
            "720p": (1280, 720),
            "1080p": (1920, 1080)
        }
        return resolutions[resolution]

    def set_capture_properties(self,capture, resolution=None, frame_rate=None, auto_focus = None, focus = None, exposure = None):
        """Set video capture properties"""
        if resolution is not None:
            width, height = self.get_resolution(resolution)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if frame_rate is not None:
            capture.set(cv2.CAP_PROP_FPS, frame_rate)
        if auto_focus is not None:
            capture.set(cv2.CAP_PROP_AUTOFOCUS, auto_focus)
        if focus is not None:
            capture.set(cv2.CAP_PROP_FOCUS, focus)
        if exposure is not None:
            capture.set(cv2.CAP_PROP_EXPOSURE, exposure)




def main(args=None):
    rclpy.init(args=args)
    node = LineDetectorNode() # Instantiate the custom node we’ve written
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()