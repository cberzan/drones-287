#include "roscopter/RC.h"

/**
* Gets the current active state of the Lander
* 
* Returns:
* landerActive = true if the lander is currently active
*/
bool isLanderActive();

/**
* Updates the current active state of the Lander.
*
* Inputs:
* controlChannelValue = the current integer value of the 
* RC control channel.
* 
* Returns:
* changed = true if the lander state has changed
*/
bool updateLanderActive(int32 controlChannelValue);

/**
* Enum of all possible Lander states:
* FLYING (stabilise) 
* SEEK_HOME (return to launch) <- user sets mode to return to launch, we detect this and begin overriding inputs
* LAND_HIGH (loiter) 
* LAND_LOW (loiter) 
* POWER_OFF (stabilise)
*/
enum class States;

/**
* Get the current state.
*
* Returns:
* currentState = the current state
*/
States getState ();

/**
* Set the current state to a new state.
*
* Inputs:
* newState = new state
* 
* Returns:
* changed = true if the new state is different
*/
bool setState(States newState);

/**
* Gets the appropriate control input for 
* this state, at this time.
* 
* Returns:
* controlMsg = roscopter::RC control input
*/
roscopter::RC performStateAction ();
