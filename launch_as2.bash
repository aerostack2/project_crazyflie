#!/bin/bash

usage() {
    echo "  options:"
    echo "      -e: estimator_type for real fly, choices: [raw_odometry, mocap_pose]"
    echo "      -r: record rosbag"
    echo "      -t: launch keyboard teleoperation"
    echo "      -s: simulated, choices: [true | false]"
    echo "      -m: multi agent, choices: [true | false]"
    echo "      -v: open rviz, choices: [true | false]"
}

# Arg parser
while getopts "e:rtsmv" opt; do
  case ${opt} in
    e )
      estimator_plugin="${OPTARG}"
      ;;
    r )
      record_rosbag="true"
      ;;
    t )
      launch_keyboard_teleop="true"
      ;;
    s )
      simulated="true"
      ;;
    m )
      swarm="true"
      ;;
    v )
      rviz="true"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[swrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# Shift optional args
shift $((OPTIND -1))

## DEFAULTS
estimator_plugin=${estimator_plugin:="raw_odometry"}
simulated=${simulated:="false"}
# If simulated, estimator_plugin is ground_truth
if [[ ${simulated} == "true" ]]; then
  estimator_plugin="ground_truth"
fi
swarm=${swarm:="false"}
rviz=${rviz:="false"}
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}

swarm_config="real_config/swarm_config_file.yaml"
if [[ ${simulated} == "true" ]]; then
  swarm_config="sim_config/world.json"
  if [[ ${swarm} == "true" ]]; then
    swarm_config="sim_config/world_swarm.json"
  fi
fi

# Fron swarm config, get the list of drone namespaces
drone_namespaces=$(python3 utils/get_drones.py ${swarm_config})
drone_namespaces_list=($(echo $drone_namespaces | tr ':' ' '))

for namespace in "${drone_namespaces_list[@]}"
do
  base_launch="false"
  if [[ ${namespace} == ${drone_namespaces_list[0]} ]]; then
    base_launch="true"
  fi

  tmuxinator start -n ${namespace} -p tmuxinator/aerostack2.yml \
      drone_namespace=${namespace} \
      base_launch=${base_launch} \
      estimator_plugin=${estimator_plugin} \
      simulation=${simulated} \
      swarm_config=${swarm_config} &
  wait
done

if [[ ${estimator_plugin} == "mocap_pose" ]]; then
  tmuxinator start -n mocap -p tmuxinator/mocap4ros2.yml &
  wait
fi

if [[ ${record_rosbag} == "true" ]]; then
  tmuxinator start -n rosbag -p tmuxinator/rosbag.yml \
      drone_namespaces=${drone_namespaces} &
  wait
fi

if [[ ${launch_keyboard_teleop} == "true" ]]; then
  drone_namespaces_comma=$(echo $drone_namespaces | tr ':' ',')
  tmuxinator start -n keyboard_teleop -p tmuxinator/keyboard_teleop.yml \
      simulation=${simulated} \
      drone_namespaces=${drone_namespaces_comma} &
  wait
fi

if [[ ${simulated} == "true" ]]; then
  tmuxinator start -n gazebo -p tmuxinator/gazebo.yml \
      simulation_config=${swarm_config} &
  wait
fi

if [[ ${rviz} == "true" ]]; then
  tmuxinator start -p tmuxinator/rviz.yml \
      simulation=${simulated} \
      drone_list=$(python3 utils/get_drones.py ${swarm_config} --sep ',') &
  wait
fi

# Attach to tmux session
tmux attach-session -t ${drone_namespaces_list[0]}:mission
