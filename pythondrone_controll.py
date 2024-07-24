import struct
import serial
import time
import os

# Constants for device classes
DEV_CLASS_NONE = 0
DEV_CLASS_SERIAL = 1

# Device description structure
class DevDescription:
    def __init__(self, klass=DEV_CLASS_NONE, name='', param=0):
        self.klass = klass
        self.name = name
        self.param = param

# Function to check and determine the device
def check_device():
    device = "/dev/ttyACM1"  # Default device, change if necessary
    baud = 115200  # Default baud rate

    if os.path.exists(device):
        return DevDescription(DEV_CLASS_SERIAL, device, baud)
    else:
        raise ValueError("No device found")

# Function to initialize the serial connection
def init_serial(device_desc):
    if device_desc.klass == DEV_CLASS_SERIAL:
        return serial.Serial(device_desc.name, device_desc.param, timeout=1)
    raise ValueError("Unsupported device class")

# Function to calculate checksum
def calculate_checksum(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

# Function to send MSP command
def send_msp_command(ser, command_id, payload):
    header = b'$M<'
    length = len(payload)
    checksum = calculate_checksum([length, command_id] + list(payload))
    command = header + bytes([length, command_id]) + payload + bytes([checksum])
    ser.write(command)
    print(f"Sent command: {command.hex()}")

# Function to read MSP response
def read_msp_response(ser):
    header = ser.read(3)
    if header == b'$M>':
        length = ser.read(1)[0]
        command_id = ser.read(1)[0]
        data = ser.read(length)
        checksum = ser.read(1)[0]
        print(f"Received response: header={header}, length={length}, command_id={command_id}, data={data.hex()}, checksum={checksum}")
        if checksum == calculate_checksum([length, command_id] + list(data)):
            return command_id, data
    else:
        print(f"Invalid header: {header}")
    return None, None

# Function to arm the drone
def arm_drone(ser):
    # Arming command payload
    # Channels: [Roll, Pitch, Yaw, Throttle, AUX1, AUX2, AUX3, AUX4]
    arming_payload = struct.pack('<HHHHHHHH', 1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500)
    send_msp_command(ser, 200, arming_payload)
    print("Arming command sent")
    time.sleep(1)

# Function to set throttle
def set_throttle(ser, throttle_value):
    # Throttle command payload
    # Channels: [Roll, Pitch, Yaw, Throttle, AUX1, AUX2, AUX3, AUX4]
    throttle_payload = struct.pack('<HHHHHHHH', 1500, 1500, 1000, throttle_value, 1500, 1500, 1500, 1500)
    send_msp_command(ser, 200, throttle_payload)
    print(f"Throttle set to {throttle_value}")
    time.sleep(1)

# Function to check status
def check_status(ser):
    send_msp_command(ser, 101, b'')
    command_id, data = read_msp_response(ser)
    if command_id == 101:
        if len(data) >= 10:
            flags, cycleTime, i2cErrors, sensor, mode = struct.unpack('<HHHHH', data[:10])
            print(f"Status: flags={flags}, cycleTime={cycleTime}, i2cErrors={i2cErrors}, sensor={sensor}, mode={mode}")
            return flags
    return None

# Main function to run the motors
def main():
    # Check and initialize device
    device_desc = check_device()
    ser = init_serial(device_desc)

    # Arm the drone
    arm_drone(ser)

    # Check status to confirm arming
    flags = check_status(ser)
    if flags is not None and (flags & (1 << 0)) == 0:
        print("Drone is armed, setting throttle")
        set_throttle(ser, 1200)  # Example throttle value to spin motors
    else:
        print("Drone is not armed. Please check transmitter and arming conditions.")
    
    # Close the serial port
    ser.close()

if __name__ == "__main__":
    main()
