#!/bin/bash

# Arguments
drone_namespace='cf'

# cf_uri="radio://0/40/2M/E7E7E7E7E7"
# cf_uri="radio://0/66/2M/E7E7E7E744"
cf_uri="radio://0/60/250K/E7E7E7E700"
# aideck_ip="192.168.0.140"
# aideck_port="5000"

source ./launch_tools.bash

new_session $drone_namespace
new_window 'crazyflie_interface' "ros2 launch crazyflie_platform crazyflie_platform_launch.py \
    drone_id:=$drone_namespace \
    drone_URI:=$cf_uri \
    estimator_type:=2 \
    controller_type:=1"

# new_window 'state_estimator' "ros2 launch basic_state_estimator mocap_state_estimator_launch.py \
#     namespace:=$drone_namespace"

new_window 'state_estimator' "ros2 launch basic_state_estimator basic_state_estimator_launch.py \
    namespace:=$drone_namespace \
    odom_only:=true "

new_window 'controller_manager' "ros2 launch controller_manager controller_manager_launch.py \
    namespace:=$drone_namespace  \
    use_bypass:=False \
    config:=drone_config/controller.yaml"

# new_window 'traj_generator' "ros2 launch trajectory_generator trajectory_generator_launch.py  \
#     drone_id:=$drone_namespace "

new_window 'basic_behaviours' "ros2 launch as2_basic_behaviours all_basic_behaviours_launch.py \
    drone_id:=$drone_namespace \
    config_goto:=drone_config/goto.yaml \
    config_takeoff:=drone_config/takeoff.yaml \
    config_land:=drone_config/land.yaml"

new_window 'teleop' "ros2 launch keyboard_teleoperation keyboard_teleoperation.launch.py \
    drone_id:=$drone_namespace \
    verbose:=true"

new_window 'viewer' "gnome-terminal -x bash -c '\
    ros2 run alphanumeric_viewer alphanumeric_viewer_node --ros-args -r  __ns:=/$drone_namespace'"

# echo -e "Launched drone $drone_namespace. For attaching to the session, run: \n  \t $ tmux a -t $drone_namespace"
tmux a -t :0
