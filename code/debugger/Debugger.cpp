#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"

using namespace std;

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "Debugger");
    ros::NodeHandle node;
    int const queueSize = 1000;
    ros::Rate loopRate(4);  // publish messages at 4 Hz

    ros::Subscriber sub = node.subscribe("control", 1000, controlCallback);

	//Debugger posts output to command line
	//Log info messages too?

    ros::spin();

    return 0;
}

/* Messages of interest-
* 
* ==pose_estimator==
* corners
* simplePose
*
* ==lander==
* control
* controlOutput
* controlInput
* states
*
* ==roscopter==
* rc
* send_rc
*/