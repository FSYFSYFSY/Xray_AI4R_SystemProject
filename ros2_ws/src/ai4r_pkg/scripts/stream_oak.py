# dependencies: pip install depthai; pip install opencv-python

import depthai as dai
import cv2

# Create a pipeline
pipeline = dai.Pipeline()

# Define a source - color camera
camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setPreviewSize(640,640)  # Modified preview size to 640x640 for the Yolov8n model
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setImageOrientation(dai.CameraImageOrientation.VERTICAL_FLIP)    # Flip the image vertically
camRgb.setPreviewKeepAspectRatio(False)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setFps(40)


# Create an XLink output to send the RGB video to the host
xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutRgb.setStreamName("rgb")
camRgb.preview.link(xoutRgb.input)

# Connect to device and start the pipeline
with dai.Device(pipeline) as device:    
    # Output queues will be used to get the rgb frames from the outputs defined above
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    while True:
        inRgb = qRgb.get()  # Blocking call, will wait until a new data has arrived
        frame = inRgb.getCvFrame()

        # Display the frame
        cv2.imshow("rgb", frame)

        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()