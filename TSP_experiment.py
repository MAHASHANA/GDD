# TSP_experiment.py

import random
from scipy.spatial.distance import euclidean
import time
import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# Scaling factor
SCALE_FACTOR = 1.5 / 100

# Original constants
UAV_SPEED = 10
UAV_BATTERY_TIME = 20                       # Time in seconds the UAV can fly on a single charge
FIXED_Z = 100                               # Fixed Z for the operation
TAKEOFF_LANDING_TIME = FIXED_Z / UAV_SPEED  # Time in seconds for takeoff or landing
UGV_SPEED = 1                               # Speed of the UGV
N = 10                                      # Number of points to sample
seed = 42                                   # Fixed seed
START_POINT = (0, 0)
END_POINT = (100, 100)

# Scaled constants
SCALED_UAV_SPEED = UAV_SPEED * SCALE_FACTOR
SCALED_UGV_SPEED = UGV_SPEED * SCALE_FACTOR
SCALED_UAV_BATTERY_TIME = UAV_BATTERY_TIME  # not scaled, change manually for the mission
SCALED_FIXED_Z = FIXED_Z * SCALE_FACTOR
SCALED_TAKEOFF_LANDING_TIME = TAKEOFF_LANDING_TIME * SCALE_FACTOR
SCALED_START_POINT = (START_POINT[0] * SCALE_FACTOR, START_POINT[1] * SCALE_FACTOR)
SCALED_END_POINT = (END_POINT[0] * SCALE_FACTOR, END_POINT[1] * SCALE_FACTOR)

def generate_points(n, x_range=(0, 100), y_range=(0, 100), FIXED_Z=100, decimals=2, seed=None):
    if seed is not None:
        random.seed(seed)
    points = [
        (
            round(random.uniform(*x_range) * SCALE_FACTOR, decimals),
            round(random.uniform(*y_range) * SCALE_FACTOR, decimals),
            FIXED_Z * SCALE_FACTOR
        )
        for _ in range(n)
    ]
    return points

def create_distance_matrix(points):
    num_points = len(points)
    distance_matrix = np.zeros((num_points, num_points))

    for i in range(num_points):
        for j in range(num_points):
            distance_matrix[i][j] = euclidean(points[i], points[j])
    
    return distance_matrix

def find_closest_point(points, reference_point):
    """Find the index of the closest point to a given reference point."""
    return min(range(len(points)), key=lambda i: euclidean(points[i][:2], reference_point))

def reorder_points(points, start_index, end_index):
    """Reorder points with the start and end points in the correct positions."""
    reordered_points = [points[start_index]] + [
        p for i, p in enumerate(points) if i != start_index and i != end_index
    ] + [points[end_index]]
    return reordered_points

def solve_tsp_with_fixed_start_end(points, start_point, end_point):
    start_index = find_closest_point(points, start_point)
    end_index = find_closest_point(points, end_point)
    reordered_points = reorder_points(points, start_index, end_index)
    distance_matrix = create_distance_matrix(reordered_points)
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, [0], [len(distance_matrix) - 1])
    routing = pywrapcp.RoutingModel(manager)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    solution = routing.SolveWithParameters(search_parameters)

    tsp_tour = []
    tsp_air_points = []
    if solution:
        index = routing.Start(0)
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            tsp_tour.append(node_index)
            tsp_air_points.append(reordered_points[node_index])
            index = solution.Value(routing.NextVar(index))
        node_index = manager.IndexToNode(index)
        tsp_tour.append(node_index)
        tsp_air_points.append(reordered_points[node_index])

    return tsp_tour, tsp_air_points, reordered_points

def create_cycles(tsp_tour, points, reordered_points):
    cycles = []
    cycle_times = []
    current_cycle = []
    current_time = 0
    for i in range(len(tsp_tour)):
        actual_point = reordered_points[tsp_tour[i]]
        actual_index = points.index(actual_point)
        if not current_cycle:
            current_cycle.append(actual_index)
            takeoff_landing_time = 2 * SCALED_TAKEOFF_LANDING_TIME
            current_time += takeoff_landing_time
        else:
            travel_time = euclidean(points[current_cycle[-1]], points[actual_index]) / SCALED_UAV_SPEED
            round_trip_time = travel_time
            if current_time + round_trip_time + (euclidean(points[actual_index], points[current_cycle[0]]) / SCALED_UAV_SPEED) > SCALED_UAV_BATTERY_TIME:
                return_time = euclidean(points[current_cycle[-1]], points[current_cycle[0]]) / SCALED_UAV_SPEED
                current_time += return_time
                cycles.append(current_cycle)
                cycle_times.append(current_time)
                current_cycle = [actual_index]
                current_time = takeoff_landing_time
            else:
                current_cycle.append(actual_index)
                current_time += travel_time

    if current_cycle:
        return_time = euclidean(points[current_cycle[-1]], points[current_cycle[0]]) / SCALED_UAV_SPEED
        current_time += return_time
        cycles.append(current_cycle)
        cycle_times.append(current_time)

    return cycles, cycle_times

def create_X_matrix(cycles, points):
    max_cycle_length = max(len(cycle) for cycle in cycles)
    X = np.zeros((len(cycles), max_cycle_length + 1, 3))
    for i, cycle in enumerate(cycles):
        X[i, 0] = [points[cycle[0]][0], points[cycle[0]][1], 0]  # Ground point at the start
        for j in range(1, len(cycle) + 1):
            X[i, j] = [points[cycle[j - 1]][0], points[cycle[j - 1]][1], SCALED_FIXED_Z]  # UAV points with correct z-value
    return X

def calculate_total_mission_time(cycles, cycle_times, ugv_points, points):
    total_time = 0
    current_battery = SCALED_UAV_BATTERY_TIME
    travel_time_to_first_point = euclidean(SCALED_START_POINT, ugv_points[0]) / SCALED_UGV_SPEED
    total_time += travel_time_to_first_point
    for i, cycle_time in enumerate(cycle_times):
        total_time += cycle_time
        current_battery -= cycle_time
        if i < len(cycle_times) - 1:
            travel_time_to_next_point = euclidean(ugv_points[i], ugv_points[i + 1]) / SCALED_UGV_SPEED
            recharge_time = SCALED_UAV_BATTERY_TIME - current_battery
            waiting_time = max(travel_time_to_next_point, recharge_time)
            total_time += waiting_time
            current_battery = SCALED_UAV_BATTERY_TIME
    travel_time_to_end_point = euclidean(ugv_points[-1], SCALED_END_POINT) / SCALED_UGV_SPEED
    total_time += travel_time_to_end_point
    return total_time

# The functions above can now be imported and used in another script
