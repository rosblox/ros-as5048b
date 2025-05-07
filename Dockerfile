FROM ros:humble-ros-core

RUN apt-get update && apt-get install -y --no-install-recommends \
    i2c-tools \
    python3-pip python3-colcon-common-extensions python3-smbus \
    && rm -rf /var/lib/apt/lists/*

COPY ros_entrypoint.sh .

WORKDIR /colcon_ws
COPY ros_as5048b src/ros_as5048b

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install --event-handlers console_direct+ 

ENV LAUNCH_COMMAND='ros2 launch ros_as5048b ros_as5048b.launch.py i2c_device:=${I2C_DEVICE} i2c_address:=${I2C_ADDRESS} encoder_topic:=/encoder/at_${I2C_ADDRESS}/deg'

# Create build and run aliases
RUN echo 'alias build="colcon build --symlink-install  --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+ "' >> /etc/bash.bashrc && \
    echo 'alias run="su - ros --whitelist-environment=\"I2C_DEVICE,I2C_ADDRESS,ROS_DOMAIN_ID\" /run.sh"' >> /etc/bash.bashrc && \
    echo "source /colcon_ws/install/setup.bash; echo UID: $UID; echo ROS_DOMAIN_ID: $ROS_DOMAIN_ID; $LAUNCH_COMMAND" >> /run.sh && chmod +x /run.sh
