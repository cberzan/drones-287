#include "Controller.h"
#include "LanderStates.h"
#include "QuadcopterState.h"

bool landerActive = false;

bool isLanderActive() {
    return landerActive;
}

bool updateLanderActive(int controlChannelValue) {
    bool oldLanderActive = landerActive;

    //Add 10 either side to account for inaccuracy in signal
    if (controlChannelValue <= (MODE_LOITER + 10) && \
    	controlChannelValue >= (MODE_LOITER - 10)) 
        landerActive = true;
    else
        landerActive = false;

    return (oldLanderActive == landerActive);
}

States currentState;

States getState () {
    return currentState;
}

bool setState (States newState) {
    bool changed = true;
    if (newState == currentState) {
        changed = false;
    }
    else {
        currentState = newState;
    }
    return changed;
}

std::vector<roscopter::RC> getStateAction() {
	std::vector<roscopter::RC> controlMsgs;
    switch (currentState) {
        case FLYING:
        	controlMsgs.push_back(getManualControlMsg());
            break;
        case LAND_HIGH:
        	controlMsgs.push_back(getTranslateAndDescendControlMsg());
        	break;
        case LAND_LOW:
        	controlMsgs.push_back(getDescendOnlyControlMsg());
        	break;
        case POWER_OFF:
            controlMsgs.push_back(getPowerOffControlMsg());
            break;
    }
    return controlMsgs;
}

std::vector<roscopter::RC> getNeutralAction() {
	std::vector<roscopter::RC> controlMsgs;
	controlMsgs.push_back(getNeutralControlMsg());
    return controlMsgs;
}
