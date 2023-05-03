#!/bin/python3

from time import sleep
import rclpy
import argparse
from as2_python_api.drone_interface import DroneInterface

parser = argparse.ArgumentParser(
    description="Starts gates mission for crazyswarm in either simulation or real environment")
parser.add_argument('-s', '--simulated', action='store_true', default=True)

def drone_run(drone_interface: DroneInterface):
    """ Run the mission """

    speed = 0.5
    takeoff_height = 1.0
    height = 1.0

    sleep_time = 2.0

    dim = 1.0
    path = [
        [dim, dim, height],
        [dim, -dim, height],
        [-dim, dim, height],
        [-dim, -dim, height],
        [0.0, 0.0, takeoff_height],
    ]

    print("Start mission")

    ##### ARM OFFBOARD #####
    drone_interface.offboard()
    drone_interface.arm()

    ##### TAKE OFF #####
    print("Take Off")
    drone_interface.takeoff(takeoff_height, speed=1.0)
    print("Take Off done")
    sleep(sleep_time)

    ##### GO TO #####
    for goal in path:
        print(f"Go to with path facing {goal}")
        drone_interface.go_to.go_to_point_path_facing(goal, speed=speed)
        print("Go to done")
    sleep(sleep_time)

    ##### LAND #####
    print("Landing")
    drone_interface.land(speed=0.5)
    print("Land done")

    drone_interface.disarm()

if __name__ == '__main__':

    if parser.parse_args().simulated:
        print("Mission running in simulation mode")
        uav_name = "drone_sim_0"
        sim = True
    else:
        print("Mission running in real mode")
        uav_name = "cf0"
        sim = False        

    rclpy.init()
    
    uav = DroneInterface(uav_name, verbose=False, use_sim_time=sim)

    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
