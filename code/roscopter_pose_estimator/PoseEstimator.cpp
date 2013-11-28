#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/String.h"

#include <sstream>
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PoseEstimator");
    ros::NodeHandle node;

    int const queueSize = 1000;
    ros::Publisher chatterPub = node.advertise<std_msgs::String>("chatter", queueSize);
    ros::Publisher cornerImagePtsPub = \
        node.advertise<std_msgs::Float64MultiArray>(
            "cornerImagePts", queueSize);
    ros::Publisher simplePosePub = \
        node.advertise<std_msgs::Float64MultiArray>(
            "simplePose", queueSize);

    ros::Rate loopRate(4);  // publish messages at 4 Hz

    int count = 0;
    while(ros::ok()) {
        std_msgs::String msg;
        stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        chatterPub.publish(msg);

        std_msgs::Float64MultiArray cornerImagePts;
        std_msgs::MultiArrayDimension dim1;
        dim1.label = "corner";
        dim1.size = 24;
        dim1.stride = 2;
        std_msgs::MultiArrayDimension dim2;
        dim2.label = "xy";
        dim2.size = 2;
        dim2.stride = 1;
        cornerImagePts.layout.dim.push_back(dim1);
        cornerImagePts.layout.dim.push_back(dim2);
        for(int i = 0; i < 24; i++) {
            cornerImagePts.data.push_back(i);
            cornerImagePts.data.push_back(-i);
        }
        cornerImagePtsPub.publish(cornerImagePts);

        std_msgs::Float64MultiArray simplePose;
        simplePose.data.push_back(1);
        simplePose.data.push_back(2);
        simplePose.data.push_back(3);
        simplePose.data.push_back(4);
        simplePosePub.publish(simplePose);

        ros::spinOnce();
        loopRate.sleep();
        count++;
    }
}
