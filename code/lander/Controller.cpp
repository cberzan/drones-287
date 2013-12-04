#include "Controller.h"
#include "QuadcopterState.h"

#include "roscopter/RC.h"

/*
* User Modes:
* 1 (low) - stabilise
* 2 (med) - loiter
* 3 (high) - loiter (we're in control)
*/

roscopter::RC getControlMsg () {
	/*
	* 1. roll (right thumb left/right)
2. pitch (right thumb up/down)
3. throttle (left thumb up/down)
4. yaw (left thumb left/right)
5. mode switch (some switch, e.g. gear or flap)
	*/
}

roscopter::RC returnControl () {
	// send [0, 0, 0, 0, 0, 0, 0, 0]
}