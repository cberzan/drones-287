#include "Controller.h"
#include "LanderStates.h"
#include "QuadcopterState.h"

#include "roscopter/RC.h"

bool landerActive = false;

bool isLanderActive() 
{
	return landerActive;
}

bool updateLanderActive(int32 controlChannelValue) {
	bool oldLanderActive = false;
	if (controlChannelValue >= 1000 && controlChannelValue <= 2000) //CHECK ME
		landerActive = true;
	else 
		landerActive = false;

	return (oldLanderActive == landerActive);
}

enum class States : char 
{
	FLYING,
	SEEK_HOME,
	LAND_HIGH,
	LAND_LOW,
	POWER_OFF
};

States currentState;

States getState () 
{
	return currentState;
}

bool setState (States newState) 
{
	bool changed = true;
	if (newState == currentState) 
	{
		changed = false;
	} 
	else
	{
		currentState = newState;
	}
	return changed;
}

roscopter::RC performStateAction() 
{
	switch (currentState)
	case (States.FLYING):
		// Return to manual control == OVERRIDE
		return Controller::returnControl();
	case(States.SEEK_HOME):
		// if current mode is not RTL, set mode to RTL
	case(States.LAND_HIGH):
		// Iff attitude of quadcopter is stable
			// (i.e., we're not moving to left or right)	
				// Calculate control input WRT pose estimate
				// Send control input
		
	case(States.LAND_LOW):
		// Descend an orderly pace
	case(States.POWER_OFF):
		// Check altitude close to 0, orientation level 
		// and climb rate is 0
		// Change mode to stabilise, set throttle to low
}	