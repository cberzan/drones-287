#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cstdio>

using namespace std;

ros::Publisher simplePub;

std_msgs::String makeSimpleMsg(int count) 
{
	std_msgs::String simpleMsg;
	std::stringstream stringstream;
	stringstream << "hello world " << count;
	simpleMsg.data = stringstream.str();
	ROS_INFO("%s", simpleMsg.data.c_str());
	return simpleMsg;
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "TestPublisher");
    ros::NodeHandle node;
    int const queueSize = 1000;
    ros::Rate loopRate(4);  // publish messages at 4 Hz

    int count = 0;
    // Set up publisher for RC output
    rcPub = \
    	node.advertise<roscopter::RC>("simple", queueSize);
        std_msgs::String simpleMsg = makeSimpleMsg(count);
        simplePub.publish(simpleMsg);
        ROS_INFO("Tick.");
        ros::spinOnce();
        loopRate.sleep();
        ++count;
    }

    return 0;
}