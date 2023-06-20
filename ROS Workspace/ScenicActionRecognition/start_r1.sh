cd bridge_ws
bash launch_bridge.sh &
cd ..
cd noetic
source /opt/ros/noetic/setup.bash
source devel_isolated/setup.bash
roslaunch leg_tracker joint_leg_tracker.launch