# Installation of the ROS2 Driver 

<!-- more -->

# 1) Prerequisites 

 - Ubuntu 18.04
 - ROS Crystal Clemmys
Structure Core SDK = 0.7.2 (firmware version 0.9.7)

 All the dependencies for ROS and Structure Core SDK must also be installed.

 The ROS driver is available for both x86_64 and arm64 (aarch64) architectures.


# 2) Package Installation 

Open a bash terminal and initialize new **ROS2** workspace, for example:

~~~{.sh}
cd 
mkdir -p ros_ws/src 
cd ~/ros_ws/src
~~~

Please copy SDK Cross-platform  folder with Strucrure Core ROS and ROS2 drivers using the following command:

~~~
cp StructureSDK-CrossPlatform-0.7.2-ROS .
~~~

Download image_common package for ROS2:

~~~
sudo apt install ros-foxy-image-common
~~~

Set the ROS 2 version to use:

~~~
cd StructureSDK-CrossPlatform-0.7.2-ROS/ROS
~~~

Open **CMakeLists.txt** and on the first line set the version:

~~~
set(ROS_VERSION 2)
~~~

After that close the file and run **to_ros2.sh** script:

~~~
chmod 755 to_ros2.sh
source to_ros2.sh
~~~

Build the ROS2 driver using colcon:

~~~
cp -r ros2_driver/ ../..
cd ../../..
colcon build --symlink-install
~~~
Colcon supports the option **--symlink-install**. This allows the installed files to be changed by changing the files in the source space (e.g. Python files or other not compiled resourced) for faster iteration.

And finally initialize the ROS environment:

~~~
source ./install/setup.bash
~~~
