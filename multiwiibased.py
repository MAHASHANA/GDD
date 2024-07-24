import struct
import serial
import time

# Function to calculate checksum
def calculate_checksum(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

# Function to create and send the command
def send_msp_command(ser, command_id, payload):
    header = b'$M<'
    length = len(payload)
    checksum = calculate_checksum([length, command_id] + list(payload))
    command = header + bytes([length, command_id]) + payload + bytes([checksum])
    ser.write(command)

# Function to read MSP response
def read_msp_response(ser):
    header = ser.read(3)
    if header == b'$M>':
        length = ser.read(1)[0]
        command_id = ser.read(1)[0]
        data = ser.read(length)
        checksum = ser.read(1)[0]
        if checksum == calculate_checksum([length, command_id] + list(data)):
            return command_id, data
    return None, None

# Serial port setup
serial_port = "/dev/ttyUSB0"  # Adjust as necessary
ser = serial.Serial(serial_port, 115200, timeout=1)

# Arming command payload (example values)
# Channels: [Roll, Pitch, Yaw, Throttle, AUX1, AUX2, AUX3, AUX4]
arming_payload = struct.pack('<HHHHHHHH', 1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500)
arming_command_id = 200  # MSP_SET_RAW_RC

# Send arming command
send_msp_command(ser, arming_command_id, arming_payload)
print("Arming command sent")

# Allow some time for the drone to arm
time.sleep(1)

# Request status data to confirm arming
status_command_id = 108  # MSP_ATTITUDE
send_msp_command(ser, status_command_id, b'')

# Read and print the response
command_id, data = read_msp_response(ser)
if command_id == status_command_id:
    # Interpret the status data (example for attitude data)
    roll, pitch, yaw = struct.unpack('<HHH', data[:6])
    print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
else:
    print("Failed to read status data")

# Close the serial port
ser.close()
