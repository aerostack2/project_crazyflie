#!/bin/bash

usage() {
    echo "usage: $0 [-p <ign_gz|dji_osdk>] [-r] [-t] [drone_namespace]"
    echo ""
    echo "  options:"
    echo "      -s: simulated, choices: [true | false]"
    echo "      -w: swarm, choices: [true | false]"
    echo "      -e: estimator_type, choices: [ground_truth, raw_odometry, mocap]"
    echo "      -r: record rosbag"
    echo "      -t: launch keyboard teleoperation"
    echo "      drone_namespace: [drone_sim_0 | drone_0]"
}

# Arg parser
while getopts ":s:w:e:r:t" opt; do
  case ${opt} in
    s )
      simulated=${OPTARG}
      ;;
    w )
      swarm=${OPTARG}
      ;;
    e )
      estimator_plugin=${OPTARG}
      ;;
    r )
      record_rosbag=${OPTARG}
      ;;
    t )
      launch_keyboard_teleop=${OPTARG}
      ;;
    \? )
      echo "Invalid option: -$OPTARG" 1>&2
      usage
      exit 1
      ;;
    : )
      echo "Option -$OPTARG requires an argument" 1>&2
      usage
      exit 1
      ;;
  esac
done

# Shift optional args
shift $((OPTIND -1))

## DEFAULTS
simulated=${simulated:="false"}  # default ign_gz
if [[ ${simulated} == "false" && -z ${estimator_plugin} ]]; then
  echo "Error: when -s is false, -e argument must be set" 1>&2
  usage
  exit 1
fi
swarm=${swarm:="false"}
estimator_plugin=${estimator_plugin:="ground_truth"}  # default ign_gz
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}

if [[ ${simulated} == "true" ]]; then
  if [[ ${swarm} == "true" ]]; then
    simulation_config="sim_config/world_swarm.json"
    drone_ns=('drone_sim_0' 'drone_sim_1' 'drone_sim_2')
  else
    simulation_config="sim_config/world.json" 
    drone_ns=('drone_sim_0')
  fi
else
  if [[ ${swarm} == "true" ]]; then
    drone_ns=('cf0' 'cf1' 'cf2')
  else 
    drone_ns=('cf0')
  fi  
fi

for ns in "${drone_ns[@]}"
do
  if [[ ${ns} == ${drone_ns[0]} ]]; then
    base_launch="true"
  else
    base_launch="false"
  fi 

  tmuxinator start -n ${ns} -p utils/session.yml drone_namespace=${ns} base_launch=${base_launch} simulated=${simulated} estimator_plugin=${estimator_plugin} record_rosbag=${record_rosbag} launch_keyboard_teleop=${launch_keyboard_teleop} simulation_config=${simulation_config} &
  wait
done

if [[ ${simulated} == "true" ]]; then
  tmuxinator start -n gazebo -p utils/gazebo.yml simulation_config=${simulation_config} &
  wait
fi

tmux a
pkill -9 -f "gazebo"