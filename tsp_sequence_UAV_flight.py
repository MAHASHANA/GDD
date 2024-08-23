# -*- coding: utf-8 -*-
import time
from threading import Thread
import motioncapture
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from TSP_experiment import generate_points, solve_tsp_with_fixed_start_end, create_cycles, create_X_matrix
import random
import time
import numpy as np

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# The host name or IP address of the mocap system
host_name = '192.168.0.100'

# The type of the mocap system
mocap_system_type = 'vicon'

# The name of the rigid body that represents the Crazyflie
rigid_body_name = 'cf_new'

# True: send position and orientation; False: send position only
send_full_pose = True

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
orientation_std_dev = 8.0e-3  # Adjusted for 120 Hz mocap system
# Scaling factor
SCALE_FACTOR = 1.5 / 100
# scaled down for the vicon space 
# previous space was 100x100x100, scaled down to match 1.8x1.8x1.8

# Original constants
UAV_SPEED = 10
# UAV_BATTERY_TIME = 20                       # Time in seconds the UAV can fly on a single charge
FIXED_Z = 100                               # Fixed Z for the operation
TAKEOFF_LANDING_TIME = FIXED_Z / UAV_SPEED  # Time in seconds for takeoff or landing
UGV_SPEED = 1                               # Speed of the UGV
N = 10                                      # Number of points to sample
seed = 42                                   # Fixed seed
START_POINT = (0, 0)
END_POINT = (100, 100)
SCALED_FIXED_Z = FIXED_Z * SCALE_FACTOR
SCALED_TAKEOFF_LANDING_TIME = TAKEOFF_LANDING_TIME * SCALE_FACTOR
SCALED_START_POINT = (START_POINT[0] * SCALE_FACTOR, START_POINT[1] * SCALE_FACTOR)
SCALED_END_POINT = (END_POINT[0] * SCALE_FACTOR, END_POINT[1] * SCALE_FACTOR)

def get_uav_sequences(points, cycles):
    uav_sequences = []
    for cycle in cycles:
        # Generate the sequence of points with constant altitude
        sequence = [(points[point_index][0], points[point_index][1], SCALED_FIXED_Z, 0) for point_index in cycle]
        # Get the starting point
        start_point = sequence[0]
        # Append the landing point at the same x, y but with z = 0
        landing_point = (start_point[0], start_point[1], 0.0, 0)
        sequence.append(landing_point)
        uav_sequences.append(sequence)
    return uav_sequences

tsp_start_time = time.time()
points = generate_points(N, seed=seed)
tsp_tour, tsp_air_points, reordered_points = solve_tsp_with_fixed_start_end(points, SCALED_START_POINT, SCALED_END_POINT)

cycles, cycle_times = create_cycles(tsp_tour, points, reordered_points)
uav_sequences = get_uav_sequences(points, cycles)

# Now uav_sequences contains the sequences you can feed to your seq_mocap.py script
for i, sequence in enumerate(uav_sequences):
    print(f"Cycle {i + 1}:")
    for point in sequence:
        print(point)


# cycle1 = uav_sequences[0]
# print(cycle1)

class MocapWrapper(Thread):
    def __init__(self, body_name):
        Thread.__init__(self)
        self.body_name = body_name
        self.on_pose = None
        self._stay_open = True
        self.start()

    def close(self):
        self._stay_open = False

    def run(self):
        mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
        while self._stay_open:
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name == self.body_name:
                    if self.on_pose:
                        pos = obj.position
                        self.on_pose([pos[0], pos[1], pos[2], obj.rotation])

def send_extpose_quat(cf, x, y, z, quat):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
    This is going to be forwarded to the Crazyflie's position estimator.
    """
    if send_full_pose:
        cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)
    else:
        cf.extpos.send_extpos(x, y, z)

def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)

def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)

def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')

def run_sequence(cf, sequence):
    commander = cf.high_level_commander

    # Take off to an initial height
    commander.takeoff(0.5, 4.0)
    time.sleep(3.0)  # Allow time for takeoff

    # Visit each waypoint in the sequence
    for (x, y, z, yaw) in sequence:
        commander.go_to(x, y, z, yaw, 4.0)  # Move to each waypoint over 3 seconds
        time.sleep(3.5)  # Wait for the movement to complete
    
    # Land after visiting all waypoints
    commander.land(0.0, 2.0)
    time.sleep(2.0)
    commander.stop()

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Connect to the mocap system
    mocap_wrapper = MocapWrapper(rigid_body_name)
    print("System up")
    
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # Set up a callback to handle data from the mocap system
        mocap_wrapper.on_pose = lambda pose: send_extpose_quat(cf, pose[0], pose[1], pose[2], pose[3])
        print("Mocap callback set")

        adjust_orientation_sensitivity(cf)
        activate_kalman_estimator(cf)
        activate_mellinger_controller(cf)
        
        
        
        for i, cycle in enumerate(uav_sequences, start=1):
            print(f"Performing Cycle {i}")
            reset_estimator(cf)
            run_sequence(cf, cycle)
            print(f"Cycle {i} complete, landing at {cycle[-1]}")

    mocap_wrapper.close()
