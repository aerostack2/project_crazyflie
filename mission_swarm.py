#!/bin/python3

import rclpy,sys
import numpy as np
from time import sleep
import threading
from typing import List
from python_interface.drone_interface import DroneInterface

pos= [[1,0,1],[-1,1,1.5],[-1,-1,2.0]]
# drones_ns=['cf0','cf1','cf2']
drones_ns=['cf0','cf1']
# drones_ns=['cf1']

n_point=0
def pose_generator():
    ret =pos[n_point]
    n_point+=1
    return ret

# def create_circle(radius, centre, n_uav):
#     circle = []
#     for i in range(n_uav):
#         circle.append(np.array([radius*np.cos(2*np.pi*i/n_uav) + centre[0], radius*np.sin(2*np.pi*i/n_uav) + centre [1], centre[2]]))
#     return circle

# def drone_run(drone_interface:DroneInterface , n_uav ):
#     drone_interface.offboard()
#     drone_interface.arm()
#     drone_interface.takeoff(10, 10)
#     sleep(5)
#     circle_1 = create_circle(10 , [0,0,10], n_uavs)
#     pose = circle_1[n_uav]
#     pose[2] += 2*(n_uav/n_uavs)
#     drone_interface.go_to(pose)
#     sleep(2)
#     circle_2 = create_circle(5, [0,0,10], n_uavs)
#     if n_uav-1 < 0 :
#         drone_interface.go_to(circle_2[n_uavs-1])
#     else:
#         drone_interface.go_to(circle_2[n_uav-1])
#     sleep(2)

    # drone_interface.land(0.2)
# create decorator for creating a thread for each drone
def takeoff(uav:DroneInterface):
    uav.arm()
    uav.offboard()
    uav.takeoff(1, 0.7)
    # sleep(1)

def land(drone_interface:DroneInterface):
    drone_interface.land(0.4)

def go_to(drone_interface:DroneInterface):
    drone_interface.go_to_point(pose_generator(),0.5)

def confirm(uavs:List[DroneInterface]):
    confirmation = input("Continue? (y/n): ")
    if confirmation != "y":
        print("Exiting...")
        for uav in uavs:
            uav.shutdown()
        sys.exit(1)


def run_func(uavs:List[DroneInterface], func, *args):
    threads = []
    for uav in uavs:
        t = threading.Thread(target=func, args=(uav, *args))
        threads.append(t)
        t.start()
    print("Waiting for threads to finish...")
    for t in threads:
        t.join()
    print ("all done")

if __name__ == '__main__':

    rclpy.init()
    uavs = []
    for i in range(len(drones_ns)):
        uavs.append(DroneInterface(drones_ns[i],verbose=True))

    print("Takeoff")
    confirm(uavs)
    run_func(uavs, takeoff)


    print("Go to")
    confirm(uavs)
    run_func(uavs, go_to)

    print("Land")
    confirm(uavs)
    run_func(uavs, land)

    
    print("Shutdown")
    confirm(uavs)
    rclpy.shutdown()

    exit(0)

