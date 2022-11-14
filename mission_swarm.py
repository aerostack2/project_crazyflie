#!/bin/python3

import rclpy,sys
import numpy as np
from time import sleep
import threading
from typing import List
from python_interface.drone_interface import DroneInterface

# pos= [[1,0,1],[-1,1,1.5],[-1,-1,2.0]]

h1=1.0
h2=1.5
h3=2.0


v0=[-2,-1,h1]
v1=[0,2,h1]
v2=[2,-1,h1]

v3=[2,1,h2]
v4=[-2,1,h2]
v5=[0,-2,h2]


v6=v0
v6[2]=h3
v7=v1
v7[2]=h3
v8=v2
v8[2]=h3


l0=[2,0,1.0]
l1=[-2,0,1.0]
l2=[0,0,1.0]


# pos0= [[1,1,1],[1,-1,1.5],[-1,-1,1.0],[-1,0,1]]
# pos1= [[-1,-1,1],[-1,1,1.5],[1,1,2.0],[1,0,1]]
# pos2= [[-1,-1,1],[-1,1,1.5],[1,1,2.0],[1,0,1]]

# pos0= [[1,1,1],[1,-1,1.5],[-1,-1,1.0],[-1,0,1]]

pos0= [v2,v1,v0,v2,v5,v4,v3,v5,v8,v7,v6,v8,v2,l0]
pos1= [v0,v2,v1,v0,v4,v3,v5,v4,v6,v8,v7,v6,v0,l1]
pos2= [v1,v0,v2,v1,v3,v5,v4,v3,v7,v6,v8,v7,v1,l2]

drones_ns=['cf0','cf1','cf2']
# drones_ns=['cf0','cf1']
# drones_ns=['cf1']

n_point_0=0
n_point_1=0
n_point_2=0
def pose_generator(uav:DroneInterface):
    global n_point_0,n_point_1,n_point_2
    if uav.get_namespace()[-1]=='0':
        ret = pos0[n_point_0]
        n_point_0+=1
    elif uav.get_namespace()[-1]=='1':
        ret = pos1[n_point_1]
        n_point_1+=1
    elif uav.get_namespace()[-1]=='2':
        ret = pos2[n_point_2]
        n_point_2+=1
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
    drone_interface.go_to_point(pose_generator(drone_interface),2.0)

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
    for i in range(pos0):
        run_func(uavs, go_to)
        n_point_0=0
        n_point_1=0
        n_point_2=0

    print("Land")
    confirm(uavs)
    run_func(uavs, land)

    
    print("Shutdown")
    confirm(uavs)
    rclpy.shutdown()

    exit(0)

