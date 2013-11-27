#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PoseEstimator");
    ros::NodeHandle node;

    int const queueSize = 1000;
    ros::Publisher chatterPub = node.advertise<std_msgs::String>("chatter", queueSize);

    ros::Rate loopRate(4);  // publish messages at 4 Hz

    int count = 0;
    while(ros::ok()) {
        std_msgs::String msg;
        stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        chatterPub.publish(msg);

        ros::spinOnce();
        loopRate.sleep();
        count++;
    }
}
