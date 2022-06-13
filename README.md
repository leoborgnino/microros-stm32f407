# microros-stm32f407

Steps:

1. clone the repo
2. in repo directory: git clone https://github.com/micro-ROS/micro_ros_stm32cubemx_utils
3. docker pull microros/micro_ros_static_library_builder:foxy && docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:foxy
4. make in root folder generates .elf
5. I use CubeIDE to import elf and debug
6. docker run -it --rm --net=host -v /dev:/dev --privileged microros/micro-ros-agent:foxy serial --dev /dev/ttyUSB0 -b 115200 -v6
