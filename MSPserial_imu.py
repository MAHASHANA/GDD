import serial
import struct
import time
import matplotlib.pyplot as plt
import numpy as np

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

def collect_imu_data(serial_conn, duration=10, sample_rate=0.1):
    imu_data_list = []
    start_time = time.time()

    while (time.time() - start_time) < duration:
        imu_data = get_imu_data(serial_conn)
        imu_data_list.append(imu_data)
        time.sleep(sample_rate)

    return imu_data_list

def plot_imu_data(imu_data_list):
    imu_data_array = np.array(imu_data_list)
    timestamps = np.arange(len(imu_data_list)) * 0.1  # Assuming sample_rate of 0.1s

    plt.figure(figsize=(12, 8))

    # Plot accelerometer data
    plt.subplot(2, 1, 1)
    plt.plot(timestamps, imu_data_array[:, 0], label='accX')
    plt.plot(timestamps, imu_data_array[:, 1], label='accY')
    plt.plot(timestamps, imu_data_array[:, 2], label='accZ')
    plt.title('Accelerometer Data')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration')
    plt.legend()

    # Plot gyroscope data
    plt.subplot(2, 1, 2)
    plt.plot(timestamps, imu_data_array[:, 3], label='gyrX')
    plt.plot(timestamps, imu_data_array[:, 4], label='gyrY')
    plt.plot(timestamps, imu_data_array[:, 5], label='gyrZ')
    plt.title('Gyroscope Data')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    serial_port = '/dev/ttyACM0'  # or 'COMx' on Windows
    baud_rate = 115200

    try:
        with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
            duration = 10  # Duration in seconds
            sample_rate = 0.1  # Sample rate in seconds
            imu_data_list = collect_imu_data(ser, duration, sample_rate)
            plot_imu_data(imu_data_list)
    except Exception as e:
        print(f'Error: {e}')
