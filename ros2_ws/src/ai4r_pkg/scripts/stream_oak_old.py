# dependencies: pip install depthai; pip install opencv-python

import depthai as dai
import cv2

# Create a pipeline
pipeline = dai.Pipeline()

# Define a source - color camera
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam_rgb.setInterleaved(False)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# Adjust the camera to flip the image vertically
cam_rgb.setImageOrientation(dai.CameraImageOrientation.VERTICAL_FLIP)

# Create an XLink output to send the RGB video to the host
xout_video = pipeline.create(dai.node.XLinkOut)
xout_video.setStreamName("video")
cam_rgb.video.link(xout_video.input)

# Connect to device and start the pipeline
with dai.Device(pipeline) as device:
    # Output queue will be used to get the rgb frames from the output defined above
    q_rgb = device.getOutputQueue(name="video", maxSize=4, blocking=False)

    while True:
        in_rgb = q_rgb.get()  # Blocking call, will wait until a new data has arrived
        frame = in_rgb.getCvFrame()

        # Display the frame
        cv2.imshow("rgb", frame)

        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()