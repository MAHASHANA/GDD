import pyrealsense2 as rs
import numpy as np
import cv2
import serial
import struct
import time
import csv

# Function to read IMU data
def read_msp_response(serial_conn):
    header = serial_conn.read(3)
    if header != b'$M>':
        raise Exception('Invalid header received')
    size = struct.unpack('B', serial_conn.read(1))[0]
    code = struct.unpack('B', serial_conn.read(1))[0]
    data = serial_conn.read(size)
    checksum = struct.unpack('B', serial_conn.read(1))[0]
    return code, data

def get_imu_data(serial_conn):
    command = b'$M<\x00\x66\x66'
    serial_conn.write(command)
    code, data = read_msp_response(serial_conn)
    if code != 0x66:
        raise Exception('Invalid response code received')
    if len(data) < 12:
        raise Exception(f'Unexpected data length: {len(data)}')
    imu_data = struct.unpack('<hhhhhh', data[:12])
    return imu_data

# Initialize the RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Initialize the IMU
serial_port = '/dev/ttyACM1'  # or 'COMx' on Windows
baud_rate = 115200
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Initialize lists to store data
timestamps = []
imu_data_list = []
depth_data_list = []
color_data_list = []

try:
    start_time = time.time()
    duration = 10  # seconds
    sample_rate = 0.1  # seconds

    while (time.time() - start_time) < duration:
        # Capture camera frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Get IMU data
        imu_data = get_imu_data(ser)
        if imu_data:
            accX, accY, accZ, gyrX, gyrY, gyrZ = imu_data
            timestamp = time.time() - start_time
            timestamps.append(timestamp)
            imu_data_list.append(imu_data)
            depth_data_list.append(depth_image)
            color_data_list.append(color_image)

        # Display the depth and color images
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((color_image, depth_colormap))
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

finally:
    # Save data
    np.save('timestamps.npy', timestamps)
    np.save('imu_data.npy', imu_data_list)
    np.save('depth_data.npy', depth_data_list)
    np.save('color_data.npy', color_data_list)
    pipeline.stop()
    ser.close()
    cv2.destroyAllWindows()
