#!/bin/python3

from time import sleep
import argparse
import rclpy
from as2_python_api.drone_interface import DroneInterface
from as2_msgs.msg import AlertEvent
from as2_python_api.modules.motion_reference_handler_module import MotionReferenceHandlerModule


class DroneTest(DroneInterface):
    """Drone interface node with PointGimbalModule."""

    def __init__(self, name, verbose=False, use_sim_time=False):
        super().__init__(name, verbose, use_sim_time)

        self.create_subscription(
            AlertEvent, '/cf0/alert_event', self.alert_cbk, 10)

        self.motion_ref_handler = MotionReferenceHandlerModule(drone=self)

        # self.current_bh = None

    def alert_cbk(self, msg: AlertEvent):
        """Alert cbk"""
        if msg.alert == 1:
            self.go_to.stop()
            # if self.current_bh is not None:
            #     self.current_bh.stop()
            self.motion_ref_handler.hover()
            self.get_logger().info("alert detected")

    def run_test(self):
        """ Run the mission """
        self.offboard()
        self.arm()
        self.takeoff(1.0, wait=True)
        sleep(1.0)
        self.go_to(10.0, 0.0, 1.0, 0.5, wait=True)

        self.go_to(0.0, 0.0, 1.0, 0.5, wait=True)
        self.land(0.5)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description="Starts gates mission for crazyswarm in either simulation or real environment")
    parser.add_argument('-s', '--simulated',
                        action='store_true', default=False)

    args = parser.parse_args()

    if args.simulated:
        print("Mission running in simulation mode")
    else:
        print("Mission running in real mode")

    rclpy.init()

    uav = DroneTest(name="cf0", verbose=False,
                    use_sim_time=args.simulated)

    uav.run_test()

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
