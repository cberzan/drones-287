#include "Controller.h"
#include "LanderStates.h"
#include "QuadcopterState.h"

#include "roscopter/RC.h"

bool landerActive = false;

bool isLanderActive() {
    return landerActive;
}

bool updateLanderActive(int controlChannelValue) {
    bool oldLanderActive = false;
    if (controlChannelValue >= 1000 && controlChannelValue <= 2000) //CHECK ME
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

roscopter::RC performStateAction() {
    switch (currentState) {
        case FLYING:
            return getManualControlMsg();
        case SEEK_HOME:
            return getRTLControlMsg();
        case LAND_HIGH:
            return getTranslateAndDescendControlMsg();
        case LAND_LOW:
            return getDescentOnlyControlMsg();
        case POWER_OFF:
            if (onGround()) {
                return getPowerOffControlMsg();
            }
            break;
    }
}
