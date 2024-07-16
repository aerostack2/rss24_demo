FROM aerostack2/nightly-humble:latest 
# TODO(miferco97): fix the tag

# Clone crazyflie project
WORKDIR /root/aerostack2_ws/src
RUN git clone https://github.com/aerostack2/as2_platform_crazyflie
# TODO(miferco97): fix the tag of the platform
WORKDIR /root/aerostack2_ws/
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to as2_platform_crazyflie


WORKDIR /root

# Crazyflie dependencies 
RUN apt-get install git python3-pip libxcb-xinerama0 libxcb-cursor0 -y
RUN pip3 install --upgrade pip
RUN pip3 install cfclient
RUN apt install udev usbutils -y

# RUN groupadd plugdev
RUN usermod -aG plugdev root
COPY ./99-bitcraze.rules /etc/udev/rules.d/99-bitcraze.rules

