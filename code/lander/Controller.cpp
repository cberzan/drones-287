#include "Controller.h"
#include "LanderStates.h"
#include "QuadcopterState.h"

#include <vector>
#include "std_msgs/Int32MultiArray.h"


// values from radio calibration
// 1: aileron, neutral = 1527, low = 1128, high = 1925
// 2: elevator, neutral = 1528, low = 1921, high = 1124
// 3: throttle, neutral ?, low = 1124, high = 1927
// 4: yaw, neutral = 1530, low = 1131, high = 1928
// 5: mode, low = 1128, med = 1528, high = 1928

enum Aileron {
	AILERON_LOW = 1128,
	AILERON_HIGH = 1925,
	AILERON_RANGE = 797
};

enum Elevator {
	ELEVATOR_LOW = 1124,
	ELEVATOR_HIGH = 1921,
	ELEVATOR_RANGE = 797 
};

enum Throttle {
	THROTTLE_LOW = 1124,
	THROTTLE_HIGH = 1927
};

enum Yaw {
	YAW_LOW = 1128,
	YAW_HIGH = 1928,
	YAW_RANGE = 800
};

const int NEUTRAL = 1528;
int AILERON_NEUTRAL = NEUTRAL;
int ELEVATOR_NEUTRAL = NEUTRAL;
int THROTTLE_NEUTRAL = 0;
int YAW_NEUTRAL = NEUTRAL;

// TODO: Add +pi/2 offset for webcam being rotated
const float YAW_CORRECTION = 0.08; // ~= 5 degrees 
const float	THROTTLE_GAIN = 0.1;

roscopter::RC buildRCMsg(int aileron, int elevator, int throttle, int yaw) {
	roscopter::RC msg;
	msg.channel[0] = aileron;
	msg.channel[1] = elevator;
	msg.channel[2] = throttle;
	msg.channel[3] = yaw;

	//We never override channel 4, this will let us recover to manual control.
	msg.channel[4] = 0;

	//Channels 5-7 are not in use, so we set them = 0.
	msg.channel[5] = 0;
	msg.channel[6] = 0;
	msg.channel[7] = 0;

	return msg;
}

roscopter::RC getNeutralControlMsg () {
	return buildRCMsg(AILERON_NEUTRAL, ELEVATOR_NEUTRAL, THROTTLE_NEUTRAL, \
		YAW_NEUTRAL);
}

roscopter::RC getRTLControlMsg () {
	return buildRCMsg(AILERON_NEUTRAL, ELEVATOR_NEUTRAL, THROTTLE_NEUTRAL, \
		YAW_NEUTRAL);
}

// Max error for x,y dimensions in mm
const int MAX_DISPLACEMENT_ERROR = 10000; 

roscopter::RC getRotationControlMsg () {
	double yaw_error =  getQuadcopterPose().yaw; //radians
	float yaw_gain = -1 * (yaw_error / 180);
	int yaw = (int) (yaw_gain * YAW_RANGE) / 2;
	return buildRCMsg(AILERON_NEUTRAL, ELEVATOR_NEUTRAL, THROTTLE_NEUTRAL, yaw);
}

roscopter::RC getTranslateAndDescendControlMsg () {

	if (getQuadcopterPose().yaw > YAW_CORRECTION || getQuadcopterPose().yaw < (-1*YAW_CORRECTION)) {
		return getRotationControlMsg();
	}

	// Assume we are always pointing forwards and therefore
	// can map x and y onto aileron and elevator inputs
	double x_error = getQuadcopterPose().x; //mm
				double y_error = getQuadcopterPose().y; //mm

	float aileron_gain = -1 * (x_error / MAX_DISPLACEMENT_ERROR);
	float elevator_gain = -1 * (y_error / MAX_DISPLACEMENT_ERROR);

	int aileron = (int) (aileron_gain * AILERON_RANGE) / 2;
	int elevator = (int) (elevator_gain * ELEVATOR_RANGE) / 2;

	// Calculate z error
	// FIX THROTTLE FOR NOW (always descending)
	double z_error =  getQuadcopterPose().z; //mm
	int throttle = (1 - THROTTLE_GAIN) * THROTTLE_NEUTRAL;

	return buildRCMsg(aileron, elevator, throttle, YAW_NEUTRAL);
}

roscopter::RC getDescendOnlyControlMsg () {
	int throttle = (1 - THROTTLE_GAIN) * THROTTLE_NEUTRAL;
	return buildRCMsg(AILERON_NEUTRAL, ELEVATOR_NEUTRAL, throttle, YAW_NEUTRAL);
}

roscopter::RC getManualControlMsg () {
	return buildRCMsg(0,0,0,0);
}

roscopter::RC getPowerOffControlMsg () {
	return buildRCMsg(AILERON_NEUTRAL, ELEVATOR_NEUTRAL, THROTTLE_LOW, \
		YAW_NEUTRAL);
}

void updateRC(const roscopter::RC::ConstPtr& rcMsg) {
	if (!isLanderActive()) {
		AILERON_NEUTRAL = rcMsg->channel[0];
		ELEVATOR_NEUTRAL = rcMsg->channel[1];
		THROTTLE_NEUTRAL = rcMsg->channel[2];
		YAW_NEUTRAL = rcMsg->channel[3];
	}
}

