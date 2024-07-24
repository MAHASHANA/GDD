import sys
import time
import serial
import struct
from yamspy import MSPy

# Define the serial port
serial_port = "/dev/ttyACM3"

class CustomMSPy(MSPy):
    def fast_read_imu(self):
        """Request, read and process RAW IMU"""

        # Request IMU values
        if self.send_RAW_msg(MSPy.MSPCodes['MSP_RAW_IMU']):
            # $ + M + < + data_length + msg_code + data + msg_crc
            # 6 bytes + data_length
            # data_length: 9 x 2 = 18 bytes
            data_length = 18
            msg = self.receive_raw_msg(size=(6 + data_length))[5:]
            converted_msg = struct.unpack('<%dh' % (data_length / 2), msg[:-1])

            # /512 for mpu6050, /256 for mma
            # currently we are unable to differentiate between the sensor types, so we are going with 512
            # And what about SENSOR_CONFIG???
            self.SENSOR_DATA['accelerometer'][0] = converted_msg[0]
            self.SENSOR_DATA['accelerometer'][1] = converted_msg[1]
            self.SENSOR_DATA['accelerometer'][2] = converted_msg[2]

            # properly scaled (INAV and BF use the same * (4 / 16.4))
            # but this is supposed to be RAW, so raw it is!
            self.SENSOR_DATA['gyroscope'][0] = converted_msg[3]
            self.SENSOR_DATA['gyroscope'][1] = converted_msg[4]
            self.SENSOR_DATA['gyroscope'][2] = converted_msg[5]

            # no clue about scaling factor (/1090), so raw
            self.SENSOR_DATA['magnetometer'][0] = converted_msg[6]
            self.SENSOR_DATA['magnetometer'][1] = converted_msg[7]
            self.SENSOR_DATA['magnetometer'][2] = converted_msg[8]

            return self.SENSOR_DATA
        else:
            print("Failed to send IMU read command")
            return None

    def fast_read_optical_flow(self):
        """Request, read and process optical flow data"""

        if self.send_RAW_msg(MSPy.MSPCodes['MSPV2_INAV_OPTICAL_FLOW']):
            # Assume a data length of 10 bytes for optical flow
            data_length = 10
            msg = self.receive_raw_msg(size=(6 + data_length))[5:]
            converted_msg = struct.unpack('<%dh' % (data_length / 2), msg[:-1])

            optical_flow_data = {
                'flowX': converted_msg[0],
                'flowY': converted_msg[1],
                'flowCompX': converted_msg[2],
                'flowCompY': converted_msg[3],
                'quality': converted_msg[4]
            }

            return optical_flow_data
        else:
            print("Failed to send optical flow read command")
            return None

def read_imu_data(board):
    """Read IMU data from the flight controller"""
    imu_data = board.fast_read_imu()
    if imu_data:
        print(f"IMU Data: {imu_data}")
    else:
        print("Failed to read IMU data")

def read_optical_flow_data(board):
    """Read optical flow data from the flight controller"""
    optical_flow_data = board.fast_read_optical_flow()
    if optical_flow_data:
        print(f"Optical Flow Data: {optical_flow_data}")
    else:
        print("Failed to read optical flow data")

# Initialize MSPy with detailed logging
with CustomMSPy(device=serial_port, logfilename='MSPy.log', logfilemode='a', loglevel='DEBUG') as board:
    if board == 1:
        print("An error occurred... probably the serial port is not available ;)")
        sys.exit(1)
    while True:
        # Read IMU data
        read_imu_data(board)
        time.sleep(1)
        #Read optical flow data
        read_optical_flow_data(board)
        time.sleep(1)

    

