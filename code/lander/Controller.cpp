#include "Controller.h"
#include "QuadcopterState.h"

#include "roscopter/RC.h"

/*
* User Modes:
* 1 (low) - stabilise
* 2 (med) - loiter
* 3 (high) - loiter (we're in control)
*/

roscopter::RC getRTLControlMsg () {
	// Set mode to RTL if we're not already there
	// Stick inputs should be neutral
}

roscopter::RC getTranslateAndDescendControlMsg () {
	// Set mode to Loiter if we're not already there
			// Iff attitude of quadcopter is stable
				// (i.e., we're not moving to left or right)	
					// Calculate control input WRT pose estimate
					// Send control input
}

roscopter::RC getDescendOnlyControlMsg () {
	// Roll and pitch should be neutral
	// Throttle should be some amount below neutral
}

roscopter::RC getManualControlMsg () {
	// Return to manual control == OVERRIDE
	// send [0, 0, 0, 0, 0, 0, 0, 0]
}

roscopter::RC getPowerOffControlMsg () {
	// Change mode to stabilise, set throttle to low 
}