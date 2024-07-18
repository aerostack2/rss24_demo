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


## Launching Aerostack2 components for crazyflie

For launching the demo using **Crazyflie**, we automated the process using tmuxinator. You can run the following commands:

- Launch leader drone:
```
tmuxinator start -p tmuxinator/crazyflie_launch.yaml namespace=drone0
```

- Launch follower drone 1:
```
tmuxinator start -p tmuxinator/crazyflie_launch.yaml namespace=drone1
```

- Launch follower drone 2:
```
tmuxinator start -p tmuxinator/crazyflie_launch.yaml namespace=drone2
```

## Launching Aerostack2 components for simulation

For launching the demo, usign **Gazebo**, we automated the process using tmuxinator. You can run the following commands:

- Launch the gazebo simulator:
```
ros2 launch as2_gazebo_assets launch_simulation.py simulation_config_file:=platforms_config/gazebo/world.yaml
```

- Launch leader drone:
```
tmuxinator start -p tmuxinator/gazebo_launch.yaml namespace=drone0
```

- Launch follower drone 1:
```
tmuxinator start -p tmuxinator/gazebo_launch.yaml namespace=drone1
```

- Launch follower drone 2:
```
tmuxinator start -p tmuxinator/gazebo_launch.yaml namespace=drone2
```

## Launching the ground station utilities

For monitoring the drone status using RViz and send commands to it using Aerostack2 Keyboard Teleoperation, you can run the following command:

- Launch the ground station:
```
tmuxinator start -p tmuxinator/ground_station.yaml namespace=drone0,drone1,drone2 rviz=true keyboard_teleop=true use_sim_time=true
```

*Note: If not using Gazebo, set use_sim_time to False.*

## Launching the follow-drone mission

For running the follow-drone mission you can run the following commands:

- Launch leader mission:
```
python3 mission_leader.py
```

- Launch follower mission 1:
```
python3 mission_follower.py -f drone1 -l drone0
```

- Launch follower mission 2:
```
python3 mission_follower.py -f drone2 -l drone1
```

*Note: If not using Gazebo, add '-r' flag to the mission scripts to set use_sim_time to False.*

## Stop everything

For stopping the demo you can run the following commands:

```
./stop.bash
```