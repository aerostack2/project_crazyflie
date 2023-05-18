#!/bin/python3

from time import sleep, time
import rclpy
from as2_python_api.drone_interface_teleop import DroneInterface
from as2_python_api.modules.follow_reference_module import FollowReferenceModule


def drone_run(drone_interface: DroneInterface):
    """ Run the mission """

    sleep_time = 2.0
    drone_interface.follow_reference = FollowReferenceModule(drone_interface)

    print("Start mission")

    ##### ARM OFFBOARD #####
    drone_interface.offboard()
    drone_interface.arm()

    ##### TAKE OFF #####
    print("Take Off")
    drone_interface.takeoff(1.0, 1.0)
    print("Take Off done")
    sleep(sleep_time)

    print('going to reference')

    t_end = time() + 30.0
    drone_interface.follow_reference.follow_reference_with_yaw(
        5, 0, 0, '/object_0', 0.0, 0.0, 1.0, 3.14)

    while (time() < t_end):
        pass

    print('increase height')
    drone_interface.follow_reference.stop()
    ##### LAND #####
    print("Landing")
    drone_interface.land(speed=0.5)
    print("Land done")

    drone_interface.disarm()


if __name__ == '__main__':
    rclpy.init()

    uav_name = "drone_sim_0"
    uav = DroneInterface(uav_name, verbose=False, use_sim_time=True)

    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
