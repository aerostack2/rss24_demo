# RSS'24 Aerostack2 demonstration
Repository contained the demonstration used in the Aerostack2 Tutorial in RSS24 Workshop on "Aerial Swarm Tools and Applications"

## Installation steps.
For this demo we are going to use Aerostack2 v1.1 which is not available throught apt in the moment.
We provide two ways of setting everything up.

### Setup using Docker (RECOMENDED)

All the demo is designed to be easily used and modify using docker.

```
# clone this Repository
git clone https://github.com/aerostack2/rss24_demo
cd rss24_demo
# build and run the container
xhost + # this will enable gazebo visualization
docker compose up -d # use the -d for keep the container alive in background
```

With this there shall be a running instance of the container with this project mounted in ```/root/project_rss24_demo```.
Now you can run as much terminals as you need by running: 

```
docker exec -it aerostack2_rss /bin/bash
```

> For stopping the container run ```xhost - ; docker compose down ``` command on the repo root folder. This will also remove the access to the XServer from the container.

### Setup from source 

This demo is tested under ROS 2 Humble in Ubuntu 22.04, please refer to official installation guide for setting everything up before continuing with these steps.

You can follow [Aerostack2 setup guide](https://aerostack2.github.io/_00_getting_started/source_install.html) for installing and setting up aerostack2.

After this you shall also install and compile the crazyflie platform if you are willing to perform real flight testing.

```
cd ~/aerostack2_ws/src/
git clone https://github.com/aerostack2/as2_platform_crazyflie.git
```

For compiling you can use AS2 CLI just by running

```
as2 build 
```

otherwise you can run 

```
cd ~/aerostack2_ws/ && colcon build --symlink-install 
```

> Remind to ``` source ~/aerostack2_ws/install/setup.bash ``` on each terminal you are going to use for runnning the demo.


## Launching the demo.

For running the demo you need to launch the followings Aerostack2 nodes in different terminals for each drone agent.

1. Gazebo simulator (once)
```
ros2 launch as2_gazebo_assets launch_simulation.py use_sim_time:=true simulation_config_file:=configs/gazebo/world.yaml
```

2. Aerostack2 platform
```
ros2 launch as2_platform_gazebo platform_gazebo_launch.py namespace:=drone0 platform_config_file:=configs/gazebo/config_file.yaml simulation_config_file:=configs/gazebo/world.yaml
```

3. Aerostack2 basic robotics functions

- State Estimation
```
ros2 launch as2_state_estimator state_estimator_launch.py namespace:=drone0 config_file:=configs/gazebo/config_file.yaml
```

- Motion Controller
```
ros2 launch as2_motion_controller controller_launch.py namespace:=drone0 config_file:=configs/gazebo/config_file.yaml plugin_name:=pid_speed_controller plugin_config_file:=configs/gazebo/pid_speed_controller.yaml
```

4. Aerostack2 Motion Behaviors
```
ros2 launch as2_behaviors_motion motion_behaviors_launch.py namespace:=drone0 config_file:=configs/gazebo/config_file.yaml
```

We have automated this using tmux. For that, you can run the following command:

```
./launch_as2.bash
```

Also, you can run ground station utilities for monitoring the drone status and send commands to it.

- Keyboard Teleoperation
```
ros2 launch as2_keyboard_teleoperation as2_keyboard_teleoperation_launch.py namespace:=drone0,drone1 config_file:=ground_station/keyboard_teleop.yaml use_sim_time:=true
```

- RVIZ2 visualization
```
ros2 launch as2_visualization swarm_viz.launch.py namespace_list:=drone0,drone1 rviz_config:=ground_station/rviz2_config.rviz
```

We have also automated this using tmux. For that, you can run the following command:

```
./launch_ground_station.bash
```

To close all tmux sessions, you can run the following command:

```
./stop.bash
```

For running leader mission you can run the following command:

```
python3 mission_leader.py -l drone0 -f drone1
```

For running follower mission you can run the following command:

```
python3 mission_follower.py -l drone1 -f drone0
```