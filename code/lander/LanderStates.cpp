#include "Controller.h"
#include "LanderStates.h"
#include "QuadcopterState.h"

bool landerActive = false;

bool isLanderActive() {
    return landerActive;
}

bool updateLanderActive(int controlChannelValue) {
    bool oldLanderActive = landerActive;

    //Add 10 to account for inaccuracy in signal
    if (controlChannelValue <= (MODE_LOITER - 10)) 
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
            return getPowerOffControlMsg();
    }
}
