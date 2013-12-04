#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;

void controlCallback(const std_msgs::String::ConstPtr& msg)
 {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
 }

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "TestListener");
    ros::NodeHandle node;
    int const queueSize = 1000;
    ros::Rate loopRate(4);  // publish messages at 4 Hz

    ros::Subscriber sub = node.subscribe("simple", 1000, controlCallback);

    ros::spin();

    return 0;
}
