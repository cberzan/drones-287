#include "QuadcopterState.h"

#include "std_msgs/Float64MultiArray.h"

#include "roscopter/Attitude.h"
#include "roscopter/VFR_HUD.h"

struct QuadcopterState {
	float32 roll;
	float32 pitch;
	float32 yaw; 
	float32 airspeed;
	float32 groundspeed; 
	int16 heading;
	uint16 throttle;
	float32 alt;
	float32 climb;
};


struct QuadcopterPose {
	float32 x;
	float32 y;
	float32 z;
	float32 yaw;
};

QuadcopterState latestState;
QuadcopterPose latestPose;

void updateAttitude(const roscopter::Attitude::ConstPtr& attitudeMsg) {
	latestState.roll = attitudeMsg.roll;
	latestState.pitch = attitudeMsg.pitch;
	latestState.yaw = attitudeMsg.yaw;
}

void updateTelemetry (const roscopter::VFR_HUD::ConstPtr& hudMsg) {
 	latestState.airspeed  = hudMsg.airspeed;
	latestState.groundspeed = hudMsg.groundspeed;
	latestState.heading = hudMsg.heading;
	latestState.throttle = latestState.throttle;
	latestState.alt = latestState.alt;
	latestState.climb = latestState.climb;
}

void updatePose (const std_msgs::Float64MultiArray::ConstPtr& poseMsg) {
	//[x, y, z, yaw] 
	latestPose.x = poseMsg[0];
	latestPose.y = poseMsg[1];
	latestPose.z = poseMsg[2];
	latestPose.yaw = poseMsg[3];
}

bool onGround () {
	// Check altitude close to 0, orientation level 
	// and climb rate is 0
	return false;
}
