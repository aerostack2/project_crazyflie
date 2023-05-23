#!/bin/python3

import sys
import argparse
import time
import threading
from typing import List
import math
import rclpy
from gates import Gates
from copy import deepcopy
from tf2_ros.buffer_interface import TransformRegistration
from tf2_geometry_msgs import PointStamped, TransformStamped
from as2_python_api.drone_interface import DroneInterface

# pos= [[1,0,1],[-1,1,1.5],[-1,-1,2.0]]
parser=argparse.ArgumentParser(description="Starts gates mission for crazyswarm in either simulation or real environment")
parser.add_argument('-s', '--simulated', action='store_true', default=False)

rclpy.init()

speed = 1.0
ingore_yaw = True
height = 2.2
desp_gates = 0.5

if parser.parse_args().simulated:
    print ("Running mission in simulation mode")
    drones_ns = [
        'drone_sim_0', 'drone_sim_1']
    position_gate_0 = [0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 1.57] # position and orientation
    position_gate_1 = [3.0, 1.0, 2.0, 0.0, 0.0, 0.0, 1.57] # position and orientation
else:
    print ("Running mission in real mode")
    drones_ns = [
        'cf0', 'cf1']
    gates_node = Gates()

    position_gate_0 = gates_node.get_gate_0_pose()
    position_gate_1 = gates_node.get_gate_1_pose()

    position_gate_0[2] += 0.6
    position_gate_1[2] += 0.7

    print(position_gate_0)
    print(position_gate_1)

    gates_node.shutdown()

drone_turn = 0

t_gate_0 = TransformStamped()
t_gate_1 = TransformStamped()

h_dist = math.sqrt((position_gate_0[0] - position_gate_1[0])
                   ** 2 + (position_gate_0[1] - position_gate_1[1])**2)

v_dist = 4.0

initial_point_rel_gate_0 = [h_dist/2,
                            -v_dist/2, 0.0]

initial_point_rel_gate_1 = [-h_dist/2,
                            v_dist/2, 0.0]
if desp_gates != 0.0:

    poses_rel_gate_0 = [[0.0, -desp_gates,
                        0.0], [0.0, desp_gates, 0.0]]
    poses_rel_gate_1 = [[0.0, desp_gates,
                        0.0], [0.0, -desp_gates, 0.0]]
else:

    poses_rel_gate_0 = [[0.0, 0.0, 0.0]]
    poses_rel_gate_1 = [[0.0, 0.0, 0.0]]


poses_rel_gate_0.append(initial_point_rel_gate_1)
poses_rel_gate_1.append(initial_point_rel_gate_0)


def shutdown_all(uavs):
    print("Exiting...")
    for uav in uavs:
        uav.shutdown()
    sys.exit(1)

# create decorator for creating a thread for each drone


def takeoff(uav: DroneInterface):
    uav.arm()
    uav.offboard()
    uav.takeoff(2.0, 0.7)
    time.sleep(1)


def land(drone_interface: DroneInterface):
    # position = drone_interface.position
    # land_position = [
    #     position[0],
    #     position[1],
    #     0.2
    # ]
    drone_interface.land(0.5)


def follow_path(drone_interface: DroneInterface):
    path = []
    if drone_interface.drone_id == drones_ns[0]:
        if drone_turn == 0:
            path = poses_rel_gate_0
            frame = "gate_0"
        else:
            path = poses_rel_gate_1
            frame = "gate_1"
    elif drone_interface.drone_id == drones_ns[1]:
        if drone_turn == 0:
            path = poses_rel_gate_1
            frame = "gate_1"
        else:
            path = poses_rel_gate_0
            frame = "gate_0"

    drone_interface.go_to.go_to_point_path_facing(
        path=path, speed=speed, frame=frame)
    # drone_interface.goto.go_to_point_path_facing(o
    #     pose_generator(drone_interface), speed=speed)


def go_to(drone_interface: DroneInterface):
    if (drone_interface.drone_id == drones_ns[0]):
        point = initial_point_rel_gate_0
    elif (drone_interface.drone_id == drones_ns[1]):
        point = initial_point_rel_gate_1
    drone_interface.go_to.go_to_point_path_facing(
        point=point, speed=speed
    )


def confirm(uavs: List[DroneInterface], msg: str = 'Continue') -> bool:
    confirmation = input(f"{msg}? (y/n): ")
    if confirmation == "y":
        return True
    elif confirmation == "n":
        return False
    else:
        shutdown_all(uavs)


def run_func(uavs: List[DroneInterface], func, *args):
    threads = []
    for uav in uavs:
        t = threading.Thread(target=func, args=(uav, *args))
        threads.append(t)
        t.start()
    print("Waiting for threads to finish...")
    for t in threads:
        t.join()
    print("all done")


def follow_path_uavs(uavs):
    run_func(uavs, follow_path)
    return


def go_to_uavs(uavs):
    run_func(uavs, go_to)
    return


def join_paths(uavs):
    global poses_rel_gate_0, poses_rel_gate_1
    path_gate_0_tmp = deepcopy(poses_rel_gate_0)
    poses_rel_gate_0.extend(poses_rel_gate_1)
    poses_rel_gate_1.extend(path_gate_0_tmp)
    print(poses_rel_gate_0)
    print(poses_rel_gate_1)


def print_status(drone_interface: DroneInterface):
    while (True):
        drone_interface.get_logger().info(str(drone_interface.go_to.status))

if __name__ == '__main__':

    rclpy.init()
    uavs = []
    for ns in drones_ns:
        uavs.append(DroneInterface(ns, verbose=False))

    print("Takeoff")
    if confirm(uavs, "Takeoff"):
        run_func(uavs, takeoff)
    print("Initial Go To")
    if confirm(uavs, "Go To"):
        go_to_uavs(uavs)
    print("Follow Path")
    if confirm(uavs, "Follow Path"):
        follow_path_uavs(uavs)
    drone_turn = 1
    join_counter = 0
    while confirm(uavs, "Replay"):
        join_counter += 1
        if join_counter == 2:
            join_paths(uavs)

        follow_path_uavs(uavs)
        if join_counter < 2:
            drone_turn = abs(drone_turn) - 1

    print("Land")
    if confirm(uavs, "Land"):
        run_func(uavs, land)

    print("Shutdown")
    rclpy.shutdown()

    exit(0)
