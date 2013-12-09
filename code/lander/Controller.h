#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "roscopter/RC.h"

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

/**
* Responsible for generating all control input messages
*/
roscopter::RC getRTLControlMsg ();

roscopter::RC getNeutralControlMsg ();

roscopter::RC getTranslateAndDescendControlMsg ();

roscopter::RC getDescendOnlyControlMsg ();

/**
* Creates a message that will return the drone to manual control.
*/
roscopter::RC getManualControlMsg ();

roscopter::RC getPowerOffControlMsg ();

void updateRC(const roscopter::RC::ConstPtr& rcMsg) ;

#endif
