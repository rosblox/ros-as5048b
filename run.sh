docker run -it --rm --name=ros-as5048b \
--volume $(pwd)/ros_as5048b:/colcon_ws/src/. \
--device=/dev/i2c-1 \
--ipc=host --pid=host \
--network=host \
--env UID=$(id -u) \
--env GID=$(id -g) \
ghcr.io/rosblox/ros-as5048b:humble


