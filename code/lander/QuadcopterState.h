#include "std_msgs/Float64MultiArray.h"

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


bool onGround();

QuadcopterState getQuadcopterState();

QuadcopterPose getQuadcopterPose();
