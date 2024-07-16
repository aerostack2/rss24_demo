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





