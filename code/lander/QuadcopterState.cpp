#include "QuadcopterState.h"

const float NO_CLIMB_RATE = 0.01;
const float GROUND_ALT = 0.1; 
const float MIN_FIELD_OF_VIEW_ALT = 0.75;
const float STABILITY_LIMIT = 0.2;
const float PI_F=3.14159265358979f;
const float CAMERA_ROTATION = PI_F / 2; // Camera is 90 degrees clockwise of the front

QuadcopterState latestState = QuadcopterState();
QuadcopterPose latestPose = QuadcopterPose();

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

double normaliseYaw(double yaw) {
	double normalisedYaw = yaw - CAMERA_ROTATION;
	if (normalisedYaw < (-1 * PI_F)) {
		normalisedYaw += 2 * PI_F;
	}
	return normalisedYaw;
}

void updatePose (const std_msgs::Float64MultiArray::ConstPtr& poseMsg) {
	//[x, y, z, yaw]
	ROS_DEBUG("Updated pose");
	latestPose.x = poseMsg->data[0];
	latestPose.y = poseMsg->data[1];
	latestPose.z = poseMsg->data[2];
	latestPose.yaw = normaliseYaw(poseMsg->data[3]);
}

bool onGround () {
	double highestAltitudeEstimate = std::max(latestState.alt, latestPose.z / 100);
	return (latestState.climb <= NO_CLIMB_RATE && highestAltitudeEstimate <= GROUND_ALT);
}

bool belowFieldOfView () {
	double highestAltitudeEstimate = std::max(latestState.alt, latestPose.z / 100);
	return (highestAltitudeEstimate <= MIN_FIELD_OF_VIEW_ALT);
}

bool isStable () {
	return (latestState.roll >= (-1 * STABILITY_LIMIT) && latestState.roll <= STABILITY_LIMIT \
		&& latestState.pitch >= (-1 * STABILITY_LIMIT) && latestState.pitch <= STABILITY_LIMIT);
}

QuadcopterState getQuadcopterState () {
	return latestState;
}

QuadcopterPose getQuadcopterPose () {
	return latestPose;
}