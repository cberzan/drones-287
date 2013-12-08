#include "QuadcopterState.h"

const float NO_CLIMB_RATE = 0.01;
const float GROUND_ALT = 0.1; 
const float MIN_FIELD_OF_VIEW_ALT = 0.75;

QuadcopterState latestState;
QuadcopterPose latestPose;

void updateAttitude(const roscopter::Attitude::ConstPtr& attitudeMsg) {
	ROS_DEBUG("Updated attitude");
	latestState.roll = attitudeMsg->roll;
	latestState.pitch = attitudeMsg->pitch;
	latestState.yaw = attitudeMsg->yaw;
}

void updateTelemetry (const roscopter::VFR_HUD::ConstPtr& hudMsg) {
	ROS_DEBUG("Updated telemetry");
 	latestState.airspeed  = hudMsg->airspeed;
	latestState.groundspeed = hudMsg->groundspeed;
	latestState.heading = hudMsg->heading;
	latestState.throttle = hudMsg->throttle;
	latestState.alt = hudMsg->alt;
	latestState.climb = hudMsg->climb;
}

void updatePose (const std_msgs::Float64MultiArray::ConstPtr& poseMsg) {
	//[x, y, z, yaw]
	ROS_DEBUG("Updated pose");
	latestPose.x = poseMsg->data[0];
	latestPose.y = poseMsg->data[1];
	latestPose.z = poseMsg->data[2];
	latestPose.yaw = poseMsg->data[3];
}

bool onGround () {
	return (latestState.climb <= NO_CLIMB_RATE && latestState.alt <= GROUND_ALT);
}

bool belowFieldOfView () {
	return (latestState.alt <= MIN_FIELD_OF_VIEW_ALT);
}

QuadcopterState getQuadcopterState () {
	return latestState;
}

QuadcopterPose getQuadcopterPose () {
	return latestPose;
}