# CRAZYFLIE PROJECT

# cf_exploration
Crazyflie area exploration.

## How to launch
### Mocap setup
```bash
source ~/mocap_ws/install/setup.bash
tmuxinator start -n mocap -p tmuxinator/mocap4ros2.yml
```

### Launch Aerostack2
```bash
# Try ./launch_as2.sh -h for help
./launch_as2.bash -e mocap_pose -r -t
```
