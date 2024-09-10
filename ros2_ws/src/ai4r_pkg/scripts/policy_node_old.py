#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ai4r_interfaces.msg import EscAndSteeringPercent
#from ai4r_interfaces.msg import IntArray
# from ai4r_interfaces.msg import ImagePointsArray
from ai4r_interfaces.msg import ConePointsArray
from std_msgs.msg import String, UInt16, Float32
from rclpy.parameter import Parameter

import numpy as np

FSM_STATE_NOT_PUBLISHING_ACTIONS   = 1
FSM_STATE_PUBLISHING_ZERO_ACTIONS  = 2
FSM_STATE_PUBLISHING_POLICY_ACTION = 3

LIST_OF_FSM_STATES = [FSM_STATE_NOT_PUBLISHING_ACTIONS, FSM_STATE_PUBLISHING_ZERO_ACTIONS, FSM_STATE_PUBLISHING_POLICY_ACTION]

TIME_WITHOUT_CV_THRESHOLD_IN_SECONDS = 2


# Create a class which inherits from the rclpy Node class (hence a superset of “rclpy.node.Node”).
class PolicyNode(Node):
    
    # CONSTRUCTOR FUNCTION
    def __init__(self):
        # Initialise the node object (passing the node name as an argument)
        super().__init__('policy_node')

        # Log the namespace
        self.get_logger().info("[POLICY NODE] starting __init__ with node namespace = " + self.get_namespace())

        # Initialise the FSM state
        self.fsm_state = FSM_STATE_PUBLISHING_ZERO_ACTIONS

        # Initialise a counter for timing a lack of CV data
        self.counts_of_timer_without_cv_data_ = 0

        # Declare the parameters of this node
        self.declare_parameters(
            namespace='',
            parameters=[
                ("timer_period", rclpy.Parameter.Type.DOUBLE),
                # ("u_optical_centre", rclpy.Parameter.Type.DOUBLE),
                # ("p_gain", rclpy.Parameter.Type.DOUBLE),
            ])
        self.timer_period = self.get_parameter_or("timer_period",Parameter("default_timer_period",Parameter.Type.DOUBLE,0.5)).value
        # self.u_optical_centre = self.get_parameter_or("u_optical_centre",Parameter("default_u_optical_centre",Parameter.Type.DOUBLE,640.0)).value
        # self.p_gain = self.get_parameter_or("p_gain",Parameter("default_p_gain",Parameter.Type.DOUBLE,0.1)).value

        self.baseline_esc_action = 0.0

        # Create ROS2 publishers
        # > For publishing the FSM state
        self.fsm_state_publisher_ = self.create_publisher(String, 'policy_fsm_state', 10)
        # > For publishing the policy actions
        self.policy_action_publisher_ = self.create_publisher(EscAndSteeringPercent, 'esc_and_steering_set_point_percent_action', 10)

        # Create ROS2 subscribers
        # > For subscribing to the CV line detection data
        #self.cv_subscription = self.create_subscription(IntArray, 'Image_Point', self.cv_line_dectection_callback, 10)
        #self.cv_subscription = self.create_subscription(ImagePointsArray, 'image_points', self.cv_line_dectection_callback, 10)
        # > For subscribing to the CV cone detection data
        self.cv_subscription = self.create_subscription(ConePointsArray, 'cone_points', self.cv_cone_detection_callback, 10)
        # > For subscribing to requests to transition the state
        self.fsm_transition_request_subscription = self.create_subscription(UInt16, 'policy_fsm_transition_request', self.fsm_transition_request_callback, 10)
        # # > For subscribing to requests for esc setpoint
        # self.esc_setpoint_request_subscription = self.create_subscription(Float32, 'policy_esc_setpoint_request', self.esc_setpoint_request_callback, 10)
        # > Prevent unused variable warning
        self.cv_subscription
        self.fsm_transition_request_subscription
        # self.esc_setpoint_request_subscription

        # Create a timer that is used for continually publishing the FSM state
        # > First argument is the duration between 2 callbacks (in seconds).
        # > Second argument is the callback function.
        self.create_timer(float(self.timer_period), self.timer_callback)



    # CALLBACK FUNCTION: for the CV line detection subscription 
    def cv_cone_detection_callback(self, msg):
        # Log the data received for debugging purposes:
        # self.get_logger().info("[POLICY NODE] Cone detection points: \"%s\"" % msg.n)

        # Set the counter to zero
        self.counts_of_timer_without_cv_data_ = 0

        # Return if in the "not publishing actions" state
        if (self.fsm_state == FSM_STATE_NOT_PUBLISHING_ACTIONS):
            return

        # Default the actions to zero
        esc_action = 0.0
        steering_action = 0.0

        # Run the policy, if in the "publishing policy" state
        if (self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION):

            # Extract the number of cones
            num_cones = msg.n

            # Exact the coordinates and colour of each cone
            # > NOTE: these are Python lists of length "num_cones"
            x_coords = msg.x
            y_coords = msg.y
            cone_colour = msg.c
            
            # =======================================
            # START OF: INSERT POLICY CODE BELOW HERE
            # =======================================

            # OBSERVATIONS:
            # > The "x_coords", "y_coords" and "cone_colour" variable are:
            #   - Lists with length equal to num_cones (the number of cones detected).
            #   - "x_coords" and "y_coords" give the x and y world coordinates of the cones respectively
            #   - "cone_colour" gives the colour of the cones (0 for yellow, 1 for blue)
            #   - A cone represents a single index of all three lists e.g. x_coords[i], y_coords[i], cone_colour[i] represent the ith cone

            # ACTIONS:
            # > The "esc_action" is:
            #   - The action for driving the main motor (esc := electronic speed contrller).
            #   - In units of signed percent, with valid range [-100,100]
            # > The "steering_action" is:
            #   - The action for changing the position of the steering servo.
            #   - In units of signed percent, with valid range [-100,100]

            # ACRONYMS:
            # "esc" := Electronic Speed Controller
            #   - This device on the Traxxas car does NOT control the speed.
            #   - The "esc" device set the voltage to the motor within the
            #     range [-(max voltage),+(max voltage)]


            # =====================================
            # START OF SIMPLE POLICY
            # =====================================
            ROAD_WIDTH = 1000           # Width of the Road (in mm)
            P_STEERING = 0.1            # P value for Steering Action
            STEER_OFFSET = 30           # Offset to adjust for biased steering
            FIXED_ESC_VALUE = 33.0      # Fixed ESC value for steering tests

            yellow_line = False         # Flag to signal yellow line detection 
            blue_line = False           # Flag to signal blue line detection

            # Convert to arrays for convenience
            x_array = np.array(x_coords)
            y_array = np.array(y_coords)
            c_array = np.array(cone_colour)

            # Create boolean masks based on c_np for yellow (0) and blue (1)
            yellow_mask = (c_array == 0)
            blue_mask = (c_array == 1)

            # Filter co-ordinates based on color
            bx = x_array[blue_mask]
            by = y_array[blue_mask]
            yx = x_array[yellow_mask]
            yy = y_array[yellow_mask]

            # Fit a line to yellow points if there are enough points (>=2 points)
            if len(yx) > 1:
                try:
                    yellow_fit = np.polyfit(yx, yy, 1)  # Linear fit (y = mx + b)
                    yellow_line = True
                    #self.get_logger().info("Yellow line fit: gradient = " + str(yellow_fit[0]) + "y-intercept + " + str(yellow_fit[1]))
                except:
                    self.get_logger().info("Could not fit yellow line")
            else:
                self.get_logger().info("Not enough yellow cones detected")

            # Fit a line to blue points if there are enough points
            if len(bx) > 1:
                try:
                    blue_fit = np.polyfit(bx, by, 1)  # Linear fit (y = mx + b)
                    blue_line = True
                    #self.get_logger().info("Blue line fit: gradient = " + str(blue_fit[0]) + "y-intercept + " + str(blue_fit[1]))
                except:
                    self.get_logger().info("Could not fit blue line")
            else:
                self.get_logger().info("Not enough blue cones detected")
            
            if yellow_line and blue_line:
                # Average the coefficients of the yellow and blue lines
                m = (yellow_fit[0] + blue_fit[0]) / 2
                b = (yellow_fit[1] + blue_fit[1]) / 2
            elif yellow_line:
                m = yellow_fit[0]
                b = yellow_fit[1] - ROAD_WIDTH / 2  # Offset calculation if only yellow line found
            elif blue_line:
                m = blue_fit[0]
                b = blue_fit[1] + ROAD_WIDTH / 2    # Offset calculation if only blue line found
            else:
                m, b = False, False

            if m and b:
                esc_action = FIXED_ESC_VALUE
                distance_to_target_line = m * 0 + b
                # Hack to guard against noise 
                # if abs(distance_to_target_line) < 15:
                #     distance_to_target_line = 0
                steering_action = P_STEERING * distance_to_target_line + STEER_OFFSET     # Steering Trim: 30
                self.get_logger().info(f"Steering action: {steering_action:.2f}")
            else:
                self.get_logger().info("Could not fit any line")
                esc_action = 0.0
                # LOGIC FOR TERMINATION

            # =====================================
            # END OF SIMPLE POLICY
            # =====================================
            


            # =====================================
            # END OF: INSERT POLICY CODE ABOVE HERE
            # =====================================

        # Prepare the message to send
        msg = EscAndSteeringPercent()
        msg.esc_percent = esc_action
        msg.steering_percent = steering_action

        # Publish the message
        self.policy_action_publisher_.publish(msg)

        # Log the string published for debugging purposes:
        #self.get_logger().info("[POLICY NODE] Published esc action = " + str(esc_action) + ", steering action = " + str(steering_action))


    # CALLBACK FUNCTION: for the FSM state transition request
    def fsm_transition_request_callback(self, msg):
        # Extract the requests FSM state from the message
        requested_state = msg.data

        # Check that the requested state is valid
        if requested_state not in LIST_OF_FSM_STATES:
            # Log that this occurred
            self.get_logger().info("[POLICY NODE] Received request to transition to an invalid state, requested_state = " + str(requested_state))

        # Transition the state
        self.fsm_state = requested_state

        # Log the data received for debugging purposes:
        self.get_logger().info("[POLICY NODE] Received request to transition to FSM state: " + str(msg.data))


    # CALLBACK FUNCTION: for the timer
    def timer_callback(self):
        # Compute the time without CV data, in seconds
        time_since_last_cv_data = self.counts_of_timer_without_cv_data_ * self.timer_period

        # Transition to zero action state if "too" long since last CV data
        if (time_since_last_cv_data > TIME_WITHOUT_CV_THRESHOLD_IN_SECONDS) and (self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION):
            self.fsm_state = FSM_STATE_PUBLISHING_ZERO_ACTIONS

        # Convert the FSM state to a string
        state_as_string = "Unknown state"

        if (self.fsm_state == FSM_STATE_NOT_PUBLISHING_ACTIONS):
            state_as_string = "Not publishing any actions"
        elif (self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION):
            state_as_string = "Publishing policy actions"
        elif (self.fsm_state == FSM_STATE_PUBLISHING_ZERO_ACTIONS):
            state_as_string = "Publishing zero actions"

        # Prepare the message to send
        msg = String()
        msg.data = state_as_string

        # Publish the message
        self.fsm_state_publisher_.publish(msg)

        # Increment the counter
        self.counts_of_timer_without_cv_data_ = self.counts_of_timer_without_cv_data_ + 1

        # # Publish actions if "too" long since last CV data
        # if (time_since_last_cv_data > TIME_WITHOUT_CV_THRESHOLD_IN_SECONDS) and ((self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION) or (self.fsm_state == FSM_STATE_PUBLISHING_ZERO_ACTIONS)):
        #     # Prepare the message to send
        #     msg = EscAndSteeringPercent()
        #     msg.esc_percent = 0.0
        #     msg.steering_percent = 0.0
        #     # Publish the message
        #     self.policy_action_publisher_.publish(msg)

        # # Append to the string if "too" long since last CV data
        # if (time_since_last_cv_data > TIME_WITHOUT_CV_THRESHOLD_IN_SECONDS):
        #     state_as_string = state_as_string + " (" + str(time_since_last_cv_data) + " seconds since last CV line detection data)"

        # Log the string published for debugging purposes:
        #self.get_logger().info("[POLICY NODE] Published FSM state = " + state_as_string)


    # # CALLBACK FUNCTION: for the CV line detection subscription 
    # def cv_line_dectection_callback(self, msg):
    #     # Log the data received for debugging purposes:
    #     #self.get_logger().info("[POLICY NODE] Line detection points: \"%s\"" % msg.points)

    #     # Set the counter to zero
    #     self.counts_of_timer_without_cv_data_ = 0

    #     # Return if in the "not publishing actions" state
    #     if (self.fsm_state == FSM_STATE_NOT_PUBLISHING_ACTIONS):
    #         return

    #     # Default the actions to zero
    #     esc_action = 0.0
    #     steering_action = 0.0

    #     # Run the policy, if in the "publishing policy" state
    #     if (self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION):

    #         # List of image coordinates
    #         list_of_points = msg.points

    #         # =======================================
    #         # START OF: INSERT POLICY CODE BELOW HERE
    #         # =======================================

    #         # OBSERVATIONS:
    #         # > The "list_of_points" variable is:
    #         #   - A list with length equal to the number of points detected.
    #         #   - Each element of the list has properties ".u" and ".v", which
    #         #     give the pixel coordinates of the respective point.
    #         #   - IMPORTANT: the (u,v) coordinates are relative to the top left
    #         #     corner of the image, i.e., they are NOT relative to the optical
    #         #     center of camera.

    #         # ACTIONS:
    #         # > The "esc_action" is:
    #         #   - The action for driving the main motor (esc := electronic speed contrller).
    #         #   - In units of signed percent, with valid range [-100,100]
    #         # > The "steering_action" is:
    #         #   - The action for changing the position of the steering servo.
    #         #   - In units of signed percent, with valid range [-100,100]

    #         # ACRONYMS:
    #         # "esc" := Electronic Speed Controller
    #         #   - This device on the Traxxas car does NOT control the speed.
    #         #   - The "esc" device set the voltage to the motor within the
    #         #     range [-(max voltage),+(max voltage)]
            
    #         # PERFORM POLICY COMPUTATIONS
    #         # > Set the ESC to a fixed value
    #         esc_action = self.baseline_esc_action
    #         # > Compute the average x of the line detection points
    #         steering_action = 0.0
    #         if (len(list_of_points) > 0):
    #             # Extract the first elements 
    #             u_coords = [tup.u for tup in list_of_points]
    #             # Calculate the average 
    #             u_average = sum(u_coords) / len(u_coords)
    #             # Set the steering action. Possibly needs to be negative depending on steering convention.
    #             steering_action = self.p_gain*(u_average - self.u_optical_centre) 
            
    #         # =====================================
    #         # END OF: INSERT POLICY CODE ABOVE HERE
    #         # =====================================


    #     # Prepare the message to send
    #     msg = EscAndSteeringPercent()
    #     msg.esc_percent = esc_action
    #     msg.steering_percent = steering_action

    #     # Publish the message
    #     self.policy_action_publisher_.publish(msg)

    #     # Log the string published for debugging purposes:
    #     #self.get_logger().info("[POLICY NODE] Published esc action = " + str(esc_action) + ", steering action = " + str(steering_action))



    # # CALLBACK FUNCTION: for the ESC setpoint request
    # def esc_setpoint_request_callback(self, msg):
    #     # Extract the ESC request from the message
    #     requested_setpoint = msg.data

    #     # Clip the setpoint
    #     if (requested_setpoint < -100.0):
    #         requested_setpoint = -100.0
    #     if (requested_setpoint >  100.0):
    #         requested_setpoint =  100.0

    #     # Update the setpoint
    #     self.baseline_esc_action = requested_setpoint

    #     # Log the data received for debugging purposes:
    #     self.get_logger().info("[POLICY NODE] Received request update ESC setpoint to " + str(msg.data) + " %")



def main(args=None):
    # Initialise ROS2 for this script
    rclpy.init(args=args)
    # Start as instance of the PolicyNode class
    node = PolicyNode()
    # Enter a ROS2 spin
    rclpy.spin(node)
    # Shutdown the nodes in this script
    rclpy.shutdown()

if __name__ == '__main__':
    main()
