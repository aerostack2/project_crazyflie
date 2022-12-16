#!/bin/bash

# Arguments
drone_namespace=$1
if [ -z "$drone_namespace" ]; then
    drone_namespace="cf"
fi
cf_uri=$2
if [ -z "$cf_uri" ]; then
    cf_uri="radio://0/80/2M/E7E7E7E7E7"
fi
run_mocap=$3
if [ -z "$run_mocap" ]; then
    run_mocap="false"
fi

behavior_type="position" # "position" or "trajectory"
launch_bt="false" # "true" or "false"
using_optitrack="false" # "true" or "false"
cf_uri="radio://0/80/2M/E7E7E7E703"
aideck_ip="192.168.0.140"
aideck_port="5000"

source ./utils/launch_tools.bash

new_session $drone_namespace

new_window 'crazyflie_platform' "ros2 launch as2_crazyflie_platform crazyflie_platform_launch.py \
    drone_id:=$drone_namespace \
    drone_URI:=$cf_uri \
    estimator_type:=2 \
    controller_type:=1"

new_window 'as2_controller_manager' "ros2 launch as2_controller_manager controller_manager_launch.py \
    namespace:=$drone_namespace \
    cmd_freq:=100.0 \
    info_freq:=10.0 \
    use_bypass:=false \
    plugin_name:=controller_plugin_speed_controller \
    plugin_config_file:=config/controller.yaml"

if [[ "$using_optitrack" == "true" ]]
then
    if [ "$run_mocap" = "true" ]; then
        new_window 'mocap' "ros2 launch mocap_optitrack mocap.launch.py  namespace:=$drone_namespace"
    fi

    new_window 'as2_state_estimator' "ros2 launch as2_state_estimator state_estimator_launch.py \
        namespace:=$drone_namespace \
        plugin_name:=as2_state_estimator_plugin_mocap"
else
    new_window 'as2_state_estimator' "ros2 launch as2_state_estimator state_estimator_launch.py \
        namespace:=$drone_namespace \
        plugin_name:=as2_state_estimator_plugin_external_odom"
fi

new_window 'as2_platform_behaviors' "ros2 launch as2_platform_behaviors as2_platform_behaviors_launch.py \
    namespace:=$drone_namespace \
    follow_path_plugin_name:=follow_path_plugin_$behavior_type \
    goto_plugin_name:=goto_plugin_$behavior_type \
    takeoff_plugin_name:=takeoff_plugin_$behavior_type \
    land_plugin_name:=land_plugin_speed"

if [[ "$behavior_type" == "trajectory" ]]
then
    new_window 'traj_generator' "ros2 launch trajectory_generator trajectory_generator_launch.py  \
        namespace:=$drone_namespace"
fi

new_window 'teleop' "ros2 launch keyboard_teleoperation keyboard_teleoperation.launch.py \
    drone_id:=$drone_namespace \
    verbose:=true"

new_window 'viewer' "gnome-terminal -x bash -c '\
    ros2 run alphanumeric_viewer alphanumeric_viewer_node --ros-args -r  __ns:=/$drone_namespace'"

if [[ "$launch_bt" == "true" ]] 
then
    new_window 'mission_planner' "ros2 launch behaviour_trees behaviour_trees.launch.py \
    drone_id:=$drone_namespace \
    groot_logger:=true \
    tree:=trees/go.xml"

    new_window 'groot' "$AEROSTACK2_WORKSPACE/build/groot/Groot --mode monitor"
fi

# echo -e "Launched drone $drone_namespace. For attaching to the session, run: \n  \t $ tmux a -t $drone_namespace"
tmux a -t :0
