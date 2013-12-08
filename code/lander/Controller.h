#include "roscopter/RC.h"

// 1: aileron, neutral = 1527, low = 1128, high = 1925
// 2: elevator, neutral = 1528, low = 1921, high = 1124
// 3: throttle, neutral ?, low = 1124, high = 1927
// 4: yaw, neutral = 1530, low = 1131, high = 1928
// 5: mode, low = 1128, med = 1528, high = 1928


// TODO: Do we want to set third mode to RTL or
// just to loiter?
enum Mode {
	MODE_RTL = 1128,
	MODE_LOITER = 1528,
	MODE_STABILISE = 1928
};

/**
* Responsible for generating all control input messages
*/

roscopter::RC getRTLControlMsg ();

roscopter::RC getNeutralControlMsg ();

roscopter::RC getTranslateAndDescendControlMsg ();

roscopter::RC getDescentOnlyControlMsg ();

/**
* Creates a message that will return the drone to manual control.
*/
roscopter::RC getManualControlMsg ();

roscopter::RC getPowerOffControlMsg ();

void updateRC(const roscopter::RC::ConstPtr& rcMsg) ;
