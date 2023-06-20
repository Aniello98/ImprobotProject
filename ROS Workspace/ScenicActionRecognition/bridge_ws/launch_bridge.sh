. /opt/ros/noetic/setup.bash
. /opt/ros/foxy/setup.bash
. ../noetic/devel_isolated/setup.bash
. ../foxy/install/local_setup.bash
. install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
