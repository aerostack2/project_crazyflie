#!/bin/bash

# Arguments
drone_namespace='cf'
behavior_type="position" # "position" or "trajectory"

# cf_uri="radio://0/40/2M/E7E7E7E7E7"
cf_uri="radio://0/66/2M/E7E7E7E744"
aideck_ip="192.168.0.140"
aideck_port="5000"

source ./launch_tools.bash

new_session $drone_namespace

new_window 'crazyflie_interface' "ros2 launch crazyflie_platform crazyflie_platform_launch.py \
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

new_window 'as2_state_estimator' "ros2 launch as2_state_estimator state_estimator_launch.py \
    namespace:=$drone_namespace \
    plugin_name:=as2_state_estimator_plugin_external_odom" 

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

# echo -e "Launched drone $drone_namespace. For attaching to the session, run: \n  \t $ tmux a -t $drone_namespace"
tmux a -t :0
