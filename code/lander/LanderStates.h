#include <vector>
#include "roscopter/RC.h"


/**
* Enum of all possible Lander states:
* FLYING (stabilise)
* LAND_HIGH (loiter)
* LAND_LOW (loiter)
* POWER_OFF (loiter)
*/
enum States {
    FLYING,
    LAND_HIGH,
    LAND_LOW,
    POWER_OFF
};

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
bool updateLanderActive(int controlChannelValue);

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
std::vector<roscopter::RC> getStateAction ();


/**
* Returns a neutral input that will reset values 
* to neutral to maintain straight and level flight.
*
* Returns:
* controlMsg = roscopter::RC control input
*/
std::vector<roscopter::RC> getNeutralAction ();
