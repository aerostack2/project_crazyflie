#!/bin/bash

# this file could have 3 arguments passed to it 

# Arguments
drone_namespace=$1
if [ -z "$drone_namespace" ]; then
    drone_namespace="cf"
fi
cf_uri=$2
if [ -z "$cf_uri" ]; then
    cf_uri="radio://0/79/2M/E7E7E7E7E7"
fi
run_mocap=$3
if [ -z "$run_mocap" ]; then
    run_mocap="false"
fi

# aideck_ip="192.168.0.140"
# aideck_port="5000"

source ./launch_tools.bash

new_session $drone_namespace

if [ "$run_mocap" = "true" ]; then
new_window 'crazyflie_interface' "ros2 launch crazyflie_platform crazyflie_swarm_launch.py \
    external_odom:=true \
    external_odom_topic:=self_localization/pose \
    estimator_type:=2 \
    controller_type:=1"
fi

new_window 'state_estimator' "ros2 launch basic_state_estimator mocap_state_estimator_launch.py \
    namespace:=$drone_namespace"

# new_window 'state_estimator' "ros2 launch basic_state_estimator basic_state_estimator_launch.py \
#     namespace:=$drone_namespace \
#     odom_only:=True"

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

if [ "$run_mocap" = "true" ]; then
  new_window 'mocap' "ros2 launch mocap_optitrack mocap.launch.py  namespace:=$drone_namespace"
fi

# echo -e "Launched drone $drone_namespace. For attaching to the session, run: \n  \t $ tmux a -t $drone_namespace"
# tmux a -t :0
