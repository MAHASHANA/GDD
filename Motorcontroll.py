import sys
import time
import serial as ser
from yamspy import MSPy

# Define the serial port
serial_port = "/dev/ttyACM3"

# Avoid dangerous commands
avoid_list = ['MSP_DATAFLASH_ERASE', 'MSP_DATAFLASH_READ', 'MSP_EEPROM_WRITE']

# Initialize MSPy with detailed logging
with MSPy(device=serial_port, logfilename='MSPy.log', logfilemode='a', loglevel='DEBUG') as board:
    if board == 1:
        print("An error occurred, probably the serial port is not available")
        sys.exit(1)

    def arm_drone():
        """Arm the drone using RC channels"""
        rc_data = [1500, 1500, 1500, 1000, 2000, 1000, 1000, 1000]  # Example RC data to arm
        if board.send_RAW_RC(rc_data):
            print("Arming command sent")
        else:
            print("Failed to send arming command")

    def disarm_drone():
        """Disarm the drone using RC channels"""
        rc_data = [1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000]  # Example RC data to disarm
        if board.send_RAW_RC(rc_data):
            print("Disarming command sent")
        else:
            print("Failed to send disarming command")

    def set_motor_speeds(motor_speeds):
        """Set the speeds for the motors"""
        if len(motor_speeds) != 4:
            raise ValueError("Motor speeds should be a list of four values")
        # Send motor speeds (pad with zeros for remaining motors)
        if board.send_RAW_MOTORS(motor_speeds + [0, 0, 0, 0]):
            print(f"Motor speeds set to: {motor_speeds}")
        else:
            print("Failed to set motor speeds")

    try:
        # Arm the drone
        arm_drone()
        time.sleep(2)  # Wait for the command to be processed

        # Set motor speeds (values between 1000 and 2000)
        motor_speeds = [1500, 1500, 1500, 1500]  # Example motor speeds
        set_motor_speeds(motor_speeds)
        
        # Keep motors running until interrupted
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        # Disarm the drone on keyboard interrupt
        print("Keyboard interrupt received. Disarming the drone...")
        disarm_drone()
        time.sleep(2)  # Wait for the command to be processed
    finally:
        # Ensure the serial connection is closed on exit
        ser.close()
        print("Serial connection closed.")

    # Print arming disable flags
    print("armingDisableFlags: {}".format(board.process_armingDisableFlags(board.CONFIG['armingDisableFlags'])))

    # Reboot the flight controller (if needed)
    if board.reboot():
        try:
            dataHandler = board.receive_msg()
            board.process_recv_data(dataHandler)
        except ser.SerialException:
            print("Board is rebooting")
