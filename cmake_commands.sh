#!/bin/bash
source /opt/ros/foxy/local_setup.sh
/usr/bin/cmake /home/revofu/ros2_ws/src/ros2_driver \
-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
-DCMAKE_INSTALL_PREFIX=$HOME/ros2_ws/build/structure_core_ros2_driver
/usr/bin/cmake --build $HOME/ros2_ws/build/structure_core_ros2_driver -- -j4 -l4
/usr/bin/cmake --install $HOME/ros2_ws/build/structure_core_ros2_driver