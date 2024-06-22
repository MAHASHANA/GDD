import serial
import struct
import time

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
    # Send MSP_RAW_IMU command
    command = b'$M<\x00\x66\x66'
    serial_conn.write(command)

    # Read the response
    code, data = read_msp_response(serial_conn)
    if code != 0x66:
        raise Exception('Invalid response code received')

    # Print the raw data for debugging
    print(f'Raw data: {data.hex()}')

    # Ensure the data received is the correct length (18 bytes for extended data)
    if len(data) < 12:
        raise Exception(f'Unexpected data length: {len(data)}')

    # Unpack IMU data (accX, accY, accZ, gyrX, gyrY, gyrZ)
    imu_data = struct.unpack('<hhhhhh', data[:12])
    return data, imu_data

def main():
    # Adjust the serial port name to match your setup
    serial_port = '/dev/ttyACM0'  # or 'COMx' on Windows
    baud_rate = 115200

    try:
        with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
            start_time = time.time()
            count = 0

            while count < 20 and (time.time() - start_time) < 10:
                raw_data, imu_data = get_imu_data(ser)
                print(f'IMU Data Point {count + 1}: Raw data: {raw_data.hex()} Modified data: {imu_data}')
                count += 1
                time.sleep(0.5)  # Small delay to avoid overwhelming the serial connection

    except Exception as e:
        print(f'Error: {e}')

if __name__ == '__main__':
    main()
