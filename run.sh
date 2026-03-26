##!/bin/bash
source /opt/ros/humble/setup.bash
colcon build
source /home/testeo/install/setup.bash
#export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/home/testeo/install/plugins/lib
#gz sim -v 4 src/puzzlebot_description/models/plugin_test.sdf