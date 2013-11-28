#include "Corners.h"
#include "Geometry.h"

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include <sstream>
using namespace std;

std_msgs::Float64MultiArray makeCornersMsg(Mat_<double> const imagePts)
{
    assert(imagePts.rows == 24);
    assert(imagePts.cols == 2);
    std_msgs::Float64MultiArray cornersMsg;
    for(int i = 0; i < 24; i++) {
        cornersMsg.data.push_back(imagePts(i, 0));
        cornersMsg.data.push_back(imagePts(i, 1));
    }
    return cornersMsg;
}

std_msgs::Float64MultiArray makeSimplePoseMsg(Mat_<double> const simplePose)
{
    std_msgs::Float64MultiArray simplePoseMsg;
    simplePoseMsg.data.push_back(simplePose(0));
    simplePoseMsg.data.push_back(simplePose(1));
    simplePoseMsg.data.push_back(simplePose(2));
    simplePoseMsg.data.push_back(simplePose(3));
    return simplePoseMsg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PoseEstimator");
    ros::NodeHandle node;
    int const queueSize = 1000;
    ros::Rate loopRate(4);  // publish messages at 4 Hz

    // "corners" message: [x1, y1, ..., x24, y24] -- contains the image
    // coordinates of the 24 corners, in the order described in the paper
    ros::Publisher cornersPub = \
        node.advertise<std_msgs::Float64MultiArray>("corners", queueSize);

    // "simplePose" message: [x, y, z, yaw] -- contains the estimate
    // of the camera pose w.r.t. the landing pad
    ros::Publisher simplePosePub = \
        node.advertise<std_msgs::Float64MultiArray>("simplePose", queueSize);

    Mat_<double> imagePts;
    Mat_<double> simplePose;
    while(ros::ok()) {
        bool ok = captureAndDetectCorners(imagePts);
        if(ok) {
            std_msgs::Float64MultiArray cornersMsg = makeCornersMsg(imagePts);
            cornersPub.publish(cornersMsg);
            simplePose = estimatePose(imagePts);
            std_msgs::Float64MultiArray simplePoseMsg = \
                makeSimplePoseMsg(simplePose);
            simplePosePub.publish(simplePoseMsg);
            ROS_INFO("Tick.");
        } else {
            // Don't publish anything this tick.
            ROS_INFO("Could not detect all corners!");
        }

        ros::spinOnce();
        loopRate.sleep();
    }
}
