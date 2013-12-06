#include "roscopter/RC.h"


/**
* Responsible for generating all control input messages
*/

roscopter::RC getRTLControlMsg ();

roscopter::RC getTranslateAndDescendControlMsg ();

roscopter::RC getDescendOnlyControlMsg ();

roscopter::RC getManualControlMsg ();

roscopter::RC getPowerOffControlMsg ();

roscopter::RC getDescentOnlyControlMsg();
