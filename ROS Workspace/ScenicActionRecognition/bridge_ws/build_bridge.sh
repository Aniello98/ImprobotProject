. /opt/ros/noetic/setup.bash
. /opt/ros/foxy/setup.bash
. ../noetic/install_isolated/setup.bash
. ../foxy/install/local_setup.bash

colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
