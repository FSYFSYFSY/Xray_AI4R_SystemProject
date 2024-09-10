import numpy as np

class CameraToWorld:
    """
    A class to manage the transformation of coordinates from a camera frame (spatial detection node)
    to a world frame.

    Attributes:
        h_camera (float): The height of the camera from the base reference frame in millimeters.
        alpha_camera (float): The tilt angle of the camera from the vertical in degrees.
        T_CB (np.array): The translation vector from the camera frame to the body frame.
        BR_C (np.array): The rotation matrix from the camera frame to the world frame.
    """

    def __init__(self, h_camera = 290, alpha_camera = 20):
        """
        Initializes the CameraToWorldTransformer with specified camera height and tilt angle.
        
        Parameters:
            h_camera (float): The height of the camera from the base reference frame. (mm)
            alpha_camera (float): The tilt angle of the camera from the vertical. (degrees)
        """
        self.h_camera = h_camera
        self.alpha_camera = alpha_camera + 90  # Adjusting angle to include the 90 degree offset
        self.T_CB = np.array([0, 0, -h_camera])
        self.BR_C = self.calculate_rotation_matrix()

    def alpha_to_radians(self, alpha_degrees):
        """
        Converts an angle from degrees to radians.

        Parameters:
            alpha_degrees (float): The angle in degrees.

        Returns:
            float: The angle in radians.
        """
        return alpha_degrees * (np.pi / 180)
    
    def calculate_rotation_matrix(self):
        """
        Calculates the rotation matrix for transforming coordinates from the camera frame to the world frame.

        Returns:
            np.array: The rotation matrix BR_C.
        """
        alpha_rad = self.alpha_to_radians(self.alpha_camera)
        sin_alpha = np.sin(-0.5 * np.pi + alpha_rad)
        cos_alpha = np.cos(-0.5 * np.pi + alpha_rad)
        
        # Rotation matrix based on the camera tilt angle
        BR_C = np.array([
            [0, sin_alpha, cos_alpha],
            [-1, 0, 0],
            [0, cos_alpha, -sin_alpha]
        ])
        return BR_C
    
    def transform_to_body_frame(self, px_c, py_c, pz_c):
        """
        Transforms a point from camera coordinates (C) to world coordinates (W).

        Parameters:
            px_c (float): The x-coordinate in the camera frame. (mm)
            py_c (float): The y-coordinate in the camera frame. (mm)
            pz_c (float): The z-coordinate in the camera frame. (mm)

        Returns:
            tuple: The transformed coordinates (x, y, z) in the world frame. Units: mm
        """
        p_c = np.array([px_c, py_c, pz_c])
        p_b = self.BR_C @ p_c - self.T_CB
        x_w, y_w, z_w = p_b
        return x_w, (-1)*y_w, z_w

# Example usage:
# camera_transformer = CameraToWorldTransformer(h_camera=340, alpha_camera=20)
# transformed_coords = camera_transformer.transform_to_body_frame(px_c=100, py_c=50, pz_c=30)
# print(transformed_coords)