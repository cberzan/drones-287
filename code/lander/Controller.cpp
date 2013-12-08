#include "Controller.h"
#include "QuadcopterState.h"


#include <vector>
#include "std_msgs/Int32MultiArray.h"

enum Aileron {
	AILERON_LOW = 1128,
	AILERON_HIGH = 1925
};

enum Elevator {
	ELEVATOR_LOW = 1124,
	ELEVATOR_HIGH = 1921 
};

enum Throttle {
	THROTTLE_LOW = 1124,
	THROTTLE_HIGH = 1927
};

enum Yaw {
	YAW_LOW = 1128,
	YAW_HIGH = 1928
};

const int NEUTRAL = 1528;
int AILERON_NEUTRAL = NEUTRAL;
int ELEVATOR_NEUTRAL = NEUTRAL;
int THROTTLE_NEUTRAL = 0;
int YAW_NEUTRAL = NEUTRAL;

const float	AILERON_GAIN = 0.2;
const float	ELEVATOR_GAIN = 0.2;
const float	THROTTLE_GAIN = 0.2;
const float	YAW_GAIN = 0.2;

roscopter::RC buildRCMsg(int aileron, int elevator, int throttle, int yaw,  int mode, bool neutral_5_to_8 = true) {
	roscopter::RC msg;
	msg.channel[0] = aileron;
	msg.channel[1] = elevator;
	msg.channel[2] = throttle;
	msg.channel[3] = yaw;
	msg.channel[4] = mode;
	if (neutral_5_to_8) {
		//Channels 5-7 are not in use, so we set them = NEUTRAL.
		msg.channel[5] = NEUTRAL;
		msg.channel[6] = NEUTRAL;
		msg.channel[7] = NEUTRAL;
	} else {
		msg.channel[5] = 0;
		msg.channel[6] = 0;
		msg.channel[7] = 0;
	}
	return msg;
}

roscopter::RC getNeutralControlMsg () {
	return buildRCMsg(AILERON_NEUTRAL, ELEVATOR_NEUTRAL, THROTTLE_NEUTRAL, \
		YAW_NEUTRAL, MODE_LOITER);
}

roscopter::RC getRTLControlMsg () {
	return buildRCMsg(AILERON_NEUTRAL, ELEVATOR_NEUTRAL, THROTTLE_NEUTRAL, \
		YAW_NEUTRAL, MODE_RTL);
}

bool isQuadcopterStable() {
//QuadcopterState::getQuadcopterState().roll
	//TODO: do we want this check?
	return true;
}

roscopter::RC getTranslateAndDescendControlMsg () {
	if (isQuadcopterStable())
	{	
		// TODO: calculate control input relative to latest pose
		int aileron;
		int elevator; 
		int throttle = (1 - THROTTLE_GAIN) * THROTTLE_NEUTRAL;
		int yaw;
	// Iff attitude of quadcopter is stable
				// (i.e., we're not moving to left or right)	
					// Calculate control input WRT pose estimate
					// Send control input
		return buildRCMsg(aileron, elevator, throttle, yaw, MODE_LOITER);
	}
	
	return getNeutralControlMsg();
}

roscopter::RC getDescendOnlyControlMsg () {
	int throttle = (1 - THROTTLE_GAIN) * THROTTLE_NEUTRAL;
	return buildRCMsg(AILERON_NEUTRAL, ELEVATOR_NEUTRAL, throttle, YAW_NEUTRAL, MODE_LOITER);
}

roscopter::RC getManualControlMsg () {
	return buildRCMsg(0,0,0,0,0, false);
}

roscopter::RC getPowerOffControlMsg () {
	return buildRCMsg(AILERON_NEUTRAL, ELEVATOR_NEUTRAL, THROTTLE_LOW, \
		YAW_NEUTRAL, MODE_LOITER);
}

void updateRC(const roscopter::RC::ConstPtr& rcMsg) {
	AILERON_NEUTRAL = rcMsg->channel[0];
	ELEVATOR_NEUTRAL = rcMsg->channel[1];
	THROTTLE_NEUTRAL = rcMsg->channel[2];
	YAW_NEUTRAL = rcMsg->channel[3];
}

