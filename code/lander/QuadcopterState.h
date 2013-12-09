#ifndef QUADCOPTER_STATE_H
#define QUADCOPTER_STATE_H

#include "std_msgs/Float64MultiArray.h"

#include "ros/ros.h"
#include "roscopter/Attitude.h"
#include "roscopter/VFR_HUD.h"

struct QuadcopterState {
    double roll;
    double pitch;
    double yaw;
    double airspeed;
    double groundspeed;
    int heading;
    int throttle;
    double alt;
    double climb;
};


struct QuadcopterPose {
    double x;
    double y;
    double z;
    double yaw;
};

/**
* Updates the latest attitude values from the autopilot
*
* Input:
* attitudeMsg = Attitude message posted by roscopter
*/
void updateAttitude(const roscopter::Attitude::ConstPtr& attitudeMsg);

/**
* Updates the latest telemetry values from the autopilot
*
* Input:
* hudMsg = VFR_HUD message posted by roscopter
*/
void updateTelemetry (const roscopter::VFR_HUD::ConstPtr& hudMsg);

/**
* Updates the latest pose values from the pose_estimator
*
* Input:
* poseMsg = simplePose message posted by pose_estimator
*/
void updatePose (const std_msgs::Float64MultiArray::ConstPtr& poseMsg);


/**
* Evaluates whether the quadcopter is on the ground or not.
*
* Output:
* boolean value, true if quadcopter is at ground level.
*/
bool onGround();


/**
* Evaluates whether the quadcopter is below the camera's field
* of view or not.
*
* Output:
* boolean value, true if quadcopter is below minimum field of 
* view altitude.
*/
bool belowFieldOfView();

/**
* Evaluates whether the quadcopter is flying stable or not.
* 
* Output:
* boolean value, true if quadcopter is flying stable.
*/
bool isStable();

/**
* Evaluates whether the quadcopter is on the ground or not.
*
* Output:
* boolean value, true if quadcopter is at ground level.
*/
QuadcopterState getQuadcopterState();


/**
* Evaluates whether the quadcopter is on the ground or not.
*
* Output:
* boolean value, true if quadcopter is at ground level.
*/
QuadcopterPose getQuadcopterPose();

#endif
