#!/bin/python3

import os
import sys
import rclpy
import numpy as np
from time import sleep
from python_interface.drone_interface import DroneInterface

n_uavs = 0

def setupDrone(drone_interface):
    drone_interface.offboard()
    drone_interface.arm()

def logPos(di):
    di.offboard()
    di.arm()
    while(True):
        final_pose = shiftPos(di,np.array([100,-100,300]))
        if(not (True in np.isnan(final_pose))):
            print(final_pose)
        else:
            print("Waiting for init")
        sleep(0.5)

def shiftPos(di, disp, pos=None):
    if pos==None:
        pos = di.get_position()

    return pos + disp

def create_circle(radius, centre, n_uav):
    circle = []
    for i in range(n_uav):
        circle.append(np.array([radius*np.cos(2*np.pi*i/n_uav) + centre[0], radius*np.sin(2*np.pi*i/n_uav) + centre [1], centre[2]]))
    return circle

def simpleTest(drone_interface):
    drone_interface.offboard()
    drone_interface.arm()
    drone_interface.takeoff(0.5,speed=0.5)
    sleep(3)

    drone_interface.land(speed=0.5)
    sleep(3)

    drone_interface.disarm()
    print("FINISHED")

def simpleTest2(drone_interface):
    drone_interface.offboard()
    drone_interface.arm()
    drone_interface.go_to(shiftPos(drone_interface,np.array([1.0,0,0.0])),speed=0.1)
    sleep(3)

    drone_interface.go_to(shiftPos(drone_interface,np.array([0,1.0,0.0])),speed=0.1)
    sleep(1)

    drone_interface.disarm()
    print("FINISHED")

def test1(drone_interface):
    drone_interface.offboard()
    drone_interface.arm()
    print("Taking off")
    drone_interface.takeoff(0.5,0.1)
    sleep(3)
    objective = shiftPos(drone_interface,np.array([0.2,0.2,0.0]))
    print("Going to: ", objective)
    drone_interface.go_to_point(objective,0.1)
    sleep(2)
    print("Landing")
    drone_interface.land(speed=0.5)
    sleep(1)

    drone_interface.disarm()
    print("FINISHED")

def makeSquare(w,h):
    points = [[0,0,0],[w,0,0],[w,h,0],[0,h,0],[0,0,0]]

    return points

def squareTest(drone_interface,w,h, vel=0.5):
    drone_interface.offboard()
    drone_interface.arm()
    print("Taking off")
    drone_interface.takeoff(0.7,vel)
    sleep(2)

    points = makeSquare(w,h)
    init_pos = drone_interface.get_position()

    for p in points:
        objective = shiftPos(drone_interface,np.array(p),pos=init_pos)
        print("Going to: ", objective)
        drone_interface.go_to(objective,vel)
        sleep(2)


    print("Landing")
    drone_interface.land(speed=vel)
    sleep(1)

    drone_interface.disarm()
    print("FINISHED")

def test2(drone_interface):
    drone_interface.offboard()
    drone_interface.arm()
    print("Taking off")
    drone_interface.takeoff(0.5,0.3)
    sleep(3)
    objective = shiftPos(drone_interface,np.array([0,0,0.5]))
    print("Going to: ", objective)
    drone_interface.go_to(objective,0.3)
    sleep(2)
    print("Landing")
    drone_interface.land(speed=0.3)
    sleep(1)

    drone_interface.disarm()
    print("FINISHED")

def height_test(drone_interface):
    drone_interface.offboard()
    drone_interface.arm()
    print("Taking off")
    drone_interface.takeoff(0.3,0.1)
    sleep(3)
    objective = shiftPos(drone_interface,np.array([0,0,-0.2]))
    print("Going to: ", objective)
    drone_interface.go_to_point(objective,0.1)
    sleep(2)
    print("Landing")
    drone_interface.land(speed=0.3)
    sleep(1)

    drone_interface.disarm()
    print("FINISHED")

def test_L(drone_interface):
    drone_interface.offboard()
    drone_interface.arm()
    drone_interface.takeoff(1.0,speed=0.5)
    # sleep(1)
    # drone_interface.go_to_point([1.0, 0.0, 1.0], speed=0.5, ignore_yaw=False)
    # sleep(1)
    # drone_interface.go_to_point([1.0, -1.0, 1.0], speed=0.5, ignore_yaw=False)
    # sleep(1)
    # drone_interface.go_to_point([0.0, -1.0, 1.0], speed=0.5, ignore_yaw=False)
    # sleep(1)
    # drone_interface.go_to_point([0.0, 0.0, 1.0], speed=0.5, ignore_yaw=False)
    # sleep(1)
    # drone_interface.land(speed=0.5)
    # sleep(3)

    # drone_interface.disarm()
    print("FINISHED")


if __name__ == '__main__':

    rclpy.init()

    drone_id = os.getenv('AEROSTACK2_SIMULATION_DRONE_ID')

    if drone_id is None:
        sys.exit("$AEROSTACK2_SIMULATION_DRONE_ID not set!!")

    drone_id = drone_id[:-1]
    print("Connecting to: ", drone_id)

    uav = DroneInterface('cf', True)
    test_L(uav)

    uav.shutdown()

    rclpy.shutdown()

    exit(0)
