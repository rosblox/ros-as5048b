FROM ros:humble-ros-core

RUN apt-get update && apt-get install -y --no-install-recommends \
    i2c-tools \
    python3-pip python3-colcon-common-extensions python3-smbus \
    && rm -rf /var/lib/apt/lists/*

RUN groupmod -g 998 i2c

COPY ros_entrypoint.sh .

WORKDIR /colcon_ws
COPY ros_as5048b src/.

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install --event-handlers console_direct+ 

RUN echo 'alias build="colcon build --symlink-install  --event-handlers console_direct+"' >> /etc/bash.bashrc
RUN echo 'source /colcon_ws/install/setup.bash; ros2 run ros_as5048b ros_as5048b_publisher --ros-args -p i2c_device:=1  -p i2c_address:=0x42' >> /run.sh && chmod +x /run.sh
RUN echo 'alias run="usermod -aG i2c ros; su - ros /run.sh"' >> /etc/bash.bashrc
