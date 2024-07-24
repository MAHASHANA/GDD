import pyrealsense2 as rs
import numpy as np
import cv2

print("OpenCV version:", cv2.__version__)
print("Has aruco:", hasattr(cv2, 'aruco'))
print("Has Dictionary_get:", hasattr(cv2.aruco, 'Dictionary_get'))
print("Has DetectorParameters_create:", hasattr(cv2.aruco, 'DetectorParameters_create'))

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream

pipeline_profile = pipeline.start(config)
device = pipeline_profile.get_device()
depth_sensor = device.query_sensors()[0]
if depth_sensor.supports(rs.option.emitter_enabled):
    depth_sensor.set_option(rs.option.emitter_enabled, 0)

# Align depth frame to color frame
align_to = rs.stream.color
align = rs.align(align_to)

# Load the ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

# Set up ArUco detector parameters
parameters = cv2.aruco.DetectorParameters()

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)  # Align the depth frame to color frame
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Convert to grayscale
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Draw detected markers
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(color_image, corners, ids)

            for corner in corners:
                # Get the center of the AprilTag
                corner = corner[0]
                center_x = int((corner[0][0] + corner[3][0]) / 2) # change to postion at centre 
                center_y = int((corner[1][1] + corner[3][1]) / 2)
                
                # Get depth value at the center of the tag
                distance = 1000*(depth_frame.get_distance(center_x, center_y))
                print(distance)
                # Draw the depth value on the image
                cv2.putText(color_image, f"D: {distance:.2f}mm", (center_x, center_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Display the resulting frame
        cv2.imshow('Frame', color_image)

        # Exit the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Stop streaming
    pipeline.stop()
    # Close OpenCV window
    cv2.destroyAllWindows()

