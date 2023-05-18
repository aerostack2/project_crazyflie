import sys
import argparse
import time
import threading
from typing import List
import math
import rclpy
from copy import deepcopy
from tf2_ros.buffer_interface import TransformRegistration
from tf2_geometry_msgs import PointStamped, TransformStamped
from as2_python_api.drone_interface_teleop import DroneInterface
from as2_python_api.modules.follow_reference_module import FollowReferenceModule

drones_ns = ['cf0']

def takeoff(uav: DroneInterface):
    uav.arm()
    uav.offboard()
    uav.takeoff(1.0, 0.7)
    time.sleep(1)

def land(drone_interface: DroneInterface):
    # position = drone_interface.position
    # land_position = [
    #     position[0],
    #     position[1],
    #     0.2
    # ]
    drone_interface.land(0.5)

def follow_reference(drone_interface: DroneInterface):
    drone_interface.follow_reference = FollowReferenceModule(drone_interface)
    if drone_interface.drone_id == drones_ns[0]:
        drone_interface.follow_reference.follow_reference_with_yaw(0.0, -1.0, 0, 'object', 1.0, 1.0, 1.0, 0.0)
    elif drone_interface.drone_id == drones_ns[1]:
        drone_interface.follow_reference.follow_reference_with_yaw(0.0, 1.0, 0, 'object', 1.0, 1.0, 1.0, 3.14)

def stop_follow_reference(drone_interface: DroneInterface):
    if drone_interface.drone_id == drones_ns[0]:
        drone_interface.follow_reference.stop()
    elif drone_interface.drone_id == drones_ns[1]:
        drone_interface.follow_reference.stop()

def follow_reference_uavs(uavs):
    run_func(uavs, follow_reference)
    return

def stop_follow_reference_uavs(uavs):
    run_func(uavs, stop_follow_reference)
    return

def go_to(drone_interface: DroneInterface):
    if (drone_interface.drone_id == drones_ns[0]):
        point = [0, -1, 1.0]
    elif (drone_interface.drone_id == drones_ns[1]):
        point = [0, 4, 1.0]
    drone_interface.go_to.go_to_point(
        point=point, speed=1.0
    )

def go_to_uavs(uavs):
    run_func(uavs, go_to)
    return

def shutdown_all(uavs):
    print("Exiting...")
    for uav in uavs:
        uav.shutdown()
    sys.exit(1)

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

def print_status(drone_interface: DroneInterface):
    while (True):
        drone_interface.get_logger().info(str(drone_interface.go_to.status))


def drone_run(drone_interface0: DroneInterface, drone_interface1: DroneInterface):
    """ Run the mission """

    sleep_time = 2.0
    drone_interface0.follow_reference = FollowReferenceModule(drone_interface0)
    drone_interface1.follow_reference = FollowReferenceModule(drone_interface1)
    print("Start mission")

    if confirm([drone_interface0], "Takeoff"):
    ##### ARM OFFBOARD #####
        drone_interface0.offboard()
        drone_interface0.arm()
        drone_interface1.offboard()
        drone_interface1.arm()

        ##### TAKE OFF #####
        print("Take Off")
        drone_interface0.takeoff(1.0, 0.5)
        drone_interface1.takeoff(1.0, 0.5)
        print("Take Off done")

    if confirm([drone_interface0], "go to initial position"):
        drone_interface0.go_to.go_to_point(point=[0.0, -1.0, 3.0], speed=0.5)
        drone_interface1.go_to.go_to_point(point=[0.0, 1.0, 3.0], speed=0.5)

    if confirm([drone_interface0], "reference"):
        print('going to reference')
        drone_interface0.follow_reference.follow_reference_with_yaw(1.0, 0, 1.0, 'point_0', 0.5, 0.5, 0.5, 3.14)
        drone_interface1.follow_reference.follow_reference_with_yaw(-1.0, 0, 1.0, 'point_0', 0.5, 0.5, 0.5, 0.0)
    
    if confirm([drone_interface0], "stop reference"):
        drone_interface0.follow_reference.stop()
        drone_interface1.follow_reference.stop()

    if confirm([drone_interface0], "land"):
        drone_interface0.land(speed=0.5)
        drone_interface1.land(speed=0.5)


if __name__ == '__main__':
    # for i, ns in enumerate(drones_ns):
    #     uavs.append(DroneInterface(ns, verbose=False))

    # print("Takeoff")
    # if confirm(uavs, "Takeoff"):
    #     run_func(uavs, takeoff)
    # if confirm(uavs, "Follow Reference"):
    #     follow_reference_uavs(uavs)
    # if confirm(uavs, "Stop Follow Reference"):
    #     follow_reference_uavs(uavs)
    # if confirm(uavs, "go to initial position"):
    #     go_to_uavs(uavs)
    # if confirm(uavs, "land"):
    #     run_func(uavs, land)

    rclpy.init()

    uav_name = ["cf0", "cf1"]
    uav_0 = DroneInterface(uav_name[0], verbose=False, use_sim_time=False)
    uav_1 = DroneInterface(uav_name[1], verbose=False, use_sim_time=False)
    drone_run(uav_0, uav_1)

    uav_0.shutdown()
    uav_1.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
