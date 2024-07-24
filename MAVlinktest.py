from dronekit import connect, VehicleMode
import time
import logging

# Enable logging
logging.basicConfig(level=logging.DEBUG)

# Connection settings
connection_string = '/dev/ttyACM3'  # Adjust this if using a different port
baud_rate = 115200

print("Attempting to connect to flight controller...")

# Connect to the vehicle
try:
    vehicle = connect(connection_string, wait_ready=True, baud=baud_rate, heartbeat_timeout=30)
    print("Connection established.")
except Exception as e:
    print(f"Failed to connect: {e}")
    exit(1)

def read_imu_data(vehicle):
    """Read IMU data from the flight controller"""
    print("Started reading IMU data")
    try:
        # Access raw IMU data
        imu = vehicle.raw_imu
        imu_data = {
            'time_usec': imu.time_usec,
            'xacc': imu.xacc,
            'yacc': imu.yacc,
            'zacc': imu.zacc,
            'xgyro': imu.xgyro,
            'ygyro': imu.ygyro,
            'zgyro': imu.zgyro,
            'xmag': imu.xmag,
            'ymag': imu.ymag,
            'zmag': imu.zmag
        }
        print(f"IMU Data: {imu_data}")
    except Exception as e:
        print(f"Failed to read IMU data: {e}")

def read_optical_flow_data(vehicle):
    """Read optical flow data from the flight controller"""
    print("Started reading Optical Flow data")
    try:
        # Access optical flow data
        optical_flow = vehicle.optical_flow
        optical_flow_data = {
            'time_usec': optical_flow.time_usec,
            'sensor_id': optical_flow.sensor_id,
            'flow_x': optical_flow.flow_x,
            'flow_y': optical_flow.flow_y,
            'flow_comp_m_x': optical_flow.flow_comp_m_x,
            'flow_comp_m_y': optical_flow.flow_comp_m_y,
            'quality': optical_flow.quality,
            'ground_distance': optical_flow.ground_distance
        }
        print(f"Optical Flow Data: {optical_flow_data}")
    except Exception as e:
        print(f"Failed to read Optical Flow data: {e}")

while True:
    read_imu_data(vehicle)
    time.sleep(1)
    read_optical_flow_data(vehicle)
    time.sleep(1)
