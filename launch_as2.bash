#!/bin/bash

usage() {
    echo "  options:"
    echo "      -b: launch behavior tree"
    echo "      -n: drone namespaces, comma separated. Default get from config file"
    echo "      -g: launch using gnome-terminal instead of tmux"
}

# Initialize variables with default values
behavior_tree="false"
drones_namespace_comma=""
use_gnome="false"

# Arg parser
while getopts "bn:g" opt; do
  case ${opt} in
    b )
      behavior_tree="true"
      ;;
    n )
      drones_namespace_comma="${OPTARG}"
      ;;
    g )
      use_gnome="true"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[wrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# If no drone namespaces are provided, get them from the world config file
if [ -z "$drones_namespace_comma" ]; then
  drones_namespace_comma=$(python3 utils/get_drones.py -p config/config.yaml --sep ',')
fi
IFS=',' read -r -a drone_namespaces <<< "$drones_namespace_comma"

# Select between tmux and gnome-terminal
tmuxinator_mode="start"
tmuxinator_end="wait"
tmp_file="/tmp/as2_project_launch_${drone_namespaces[@]}.txt"
if [[ ${use_gnome} == "true" ]]; then
  tmuxinator_mode="debug"
  tmuxinator_end="> ${tmp_file} && python3 utils/tmuxinator_to_genome.py -p ${tmp_file} && wait"
fi

# Launch aerostack2 for each drone namespace
for namespace in ${drone_namespaces[@]}; do
  base_launch="false"
  if [[ ${namespace} == ${drone_namespaces[0]} ]]; then
    base_launch="true"
  fi
  eval "tmuxinator ${tmuxinator_mode} -n ${namespace} -p tmuxinator/aerostack2.yaml \
    drone_namespace=${namespace} \
    base_launch=${base_launch} \
    behavior_tree=${behavior_tree} \
    ${tmuxinator_end}"

  sleep 0.1 # Wait for tmuxinator to finish
done

# Attach to tmux session
if [[ ${use_gnome} == "false" ]]; then
  tmux attach-session -t ${drone_namespaces[0]}
# If tmp_file exists, remove it
elif [[ -f ${tmp_file} ]]; then
  rm ${tmp_file}
fi
