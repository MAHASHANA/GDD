from pymultiwii import MultiWii
import time

# Initialize the MultiWii object with the serial port
serialPort = "/dev/ttyACM0"  # Adjust the serial port as needed
board = MultiWii(serialPort)

# Function to arm the drone
def arm_drone(board):
    board.sendCMD(8, MultiWii.SET_RAW_RC, [1500, 1500, 1000, 989, 2100, 1500, 1500, 1500], '8H')
    print("Arming command sent")
    time.sleep(1)  # Allow some time for the drone to arm

# Function to run motors
def run_motors(board, duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        board.sendCMD(8, MultiWii.SET_RAW_RC, [1500, 1500, 1000, 2000, 2100, 1500, 1500, 1500], '8H')
        print("Running motors command sent")
        time.sleep(0.5)

# Function to disarm the drone
def disarm_drone(board):
    board.sendCMD(8, MultiWii.SET_RAW_RC, [1500, 1500, 1000, 989, 1300, 1500, 1500, 1500], '8H')
    print("Disarming command sent")

# Main logic to arm, run motors, and disarm
if __name__ == "__main__":
    # Arm the drone
    arm_drone(board)

    # Check if the drone is armed
    data = board.getData(MultiWii.ATTITUDE)
    print(f"Status after arming: {data}")

    # Run motors for 10 seconds
    run_motors(board, 30)

    # Disarm the drone
    disarm_drone(board)

    # Continuously read and print incoming data
    while True:
        data = board.getData(MultiWii.ATTITUDE)
        print(data)
        time.sleep(0.1)  # Adjust the sleep time as needed to control the rate of data reading
