#!/bin/zsh
source /opt/ros/humble/setup.zsh
ros2 launch psdk_wrapper wrapper.launch.py link_config_file_path:=/home/ivaq-raspi/Development/psdk_ros2/psdk_wrapper/cfg/link_config.json psdk_params_file_path:=/home/ivaq-raspi/Development/psdk_ros2/psdk_wrapper/cfg/psdk_params.yaml
