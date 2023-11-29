FROM ros:humble-ros-core

RUN apt-get update && apt-get install -y --no-install-recommends \
    i2c-tools \
    python3-pip python3-colcon-common-extensions python3-smbus \
    && rm -rf /var/lib/apt/lists/*

COPY ros_entrypoint.sh .

WORKDIR /colcon_ws
COPY ros_as5048b src/ros_as5048b

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install --event-handlers console_direct+ 

RUN echo 'alias build="colcon build --symlink-install  --event-handlers console_direct+"' >> /etc/bash.bashrc && \
    echo 'source /colcon_ws/install/setup.bash; ros2 launch ros_as5048b ros_as5048b.launch.py i2c_device:=${I2C_DEVICE} i2c_address:=${I2C_ADDRESS}' >> /run.sh && chmod +x /run.sh && \
    echo 'alias run="su - ros --whitelist-environment=\"I2C_DEVICE,I2C_ADDRESS\" /run.sh"' >> /etc/bash.bashrc
