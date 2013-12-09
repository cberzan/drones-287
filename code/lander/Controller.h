#include "roscopter/RC.h"

/* User defined in MissionPlanner */
enum Mode {
	MODE_RTL = 1128,
	MODE_LOITER = 1528,
	MODE_STABILISE = 1928
};

/**
* Responsible for generating all control input messages
*/
roscopter::RC getNeutralControlMsg ();

roscopter::RC getTranslateAndDescendControlMsg ();

roscopter::RC getDescendOnlyControlMsg ();

/**
* Creates a message that will return the drone to manual control.
*/
roscopter::RC getManualControlMsg ();

roscopter::RC getPowerOffControlMsg ();

void updateRC(const roscopter::RC::ConstPtr& rcMsg) ;
