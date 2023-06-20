#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

void callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    print("ciao")
    ROS_INFO("I heard: [%s]", msg);
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "hokuyo_tester");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan_right", 1000, callback);

    ros::spin();

    return 0;
}