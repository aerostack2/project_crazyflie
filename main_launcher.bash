#!/bin/bash

usage() {
    echo "usage: $0 [-p <ign_gz|dji_osdk>] [-r] [-t] [drone_namespace]"
    echo ""
    echo "  options:"
    echo "      -s: simulated, choices: [true | false]"
    echo "      -e: estimator_type, choices: [ground_truth, raw_odometry, mocap]"
    echo "      -r: record rosbag"
    echo "      -t: launch keyboard teleoperation"
    echo "      drone_namespace: [drone_sim_0 | drone_0]"
}

# Arg parser
while getopts ":s:e:r:t" opt; do
  case ${opt} in
    s )
      simulated=${OPTARG}
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

## POSITIONAL ARGS
# drone_namespace=$1

## DEFAULTS
simulated=${simulated:="true"}  # default ign_gz
estimator_plugin=${estimator_plugin:="ground_truth"}  # default ign_gz
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}

tmuxinator start -n drone_sim_0 -p utils/session.yml drone_namespace="drone_sim_0" base_launch="true" simulated=${simulated} estimator_plugin=${estimator_plugin} record_rosbag=${record_rosbag} launch_keyboard_teleop=${launch_keyboard_teleop}
