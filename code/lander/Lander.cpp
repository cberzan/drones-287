#include <string>

#include "Controller.h"
#include "LanderStates.h"
#include "QuadcopterState.h"

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include "roscopter/Attitude.h"
#include "roscopter/RC.h"
#include "roscopter/VFR_HUD.h"

ros::Publisher rcPub;

// default = SEEK_HOME
const States START_STATE = LAND_HIGH;
const std::string START_STATE_NAME = "LAND_HIGH";
const int MAX_CONTROL_CYCLES = 2;
int cyclesSinceControlInput = 0;

void publishControlMsgs(std::vector<roscopter::RC> controlMsgs) {
	for (int i=0; i<controlMsgs.size(); i++) {
		roscopter::RC controlMsg = controlMsgs[i];
		ROS_INFO("%d %d %d %d", controlMsg.channel[0], controlMsg.channel[1], controlMsg.channel[2], controlMsg.channel[3]); 
		rcPub.publish(controlMsg);	
	}
}

void performNeutralAction() {
	ROS_INFO("Sending neutral control message");
	publishControlMsgs(getNeutralAction());	
}

void performStateAction() {
	ROS_INFO("Sending control message");
	publishControlMsgs(getStateAction());
}

void setStateAndPerformAction(States newState) {
	setState(newState);
	performStateAction();
}


void attitudeCallback(const roscopter::Attitude::ConstPtr& attitudeMsg) {
	updateAttitude(attitudeMsg);
}

/**
* State transitions happen when we receive one of the following:
* New quadcopter state estimate
* New pose estimate
* New RC input
*/

void hudCallback(const roscopter::VFR_HUD::ConstPtr& hudMsg) {
	updateTelemetry(hudMsg);

	if (isLanderActive()) {
		if (getState() == LAND_HIGH) {
			if (belowFieldOfView()) {
				ROS_INFO("Setting state and performing action: LAND_LOW");
				setStateAndPerformAction(LAND_LOW);
			}
		} else if (getState() == LAND_LOW) {
			if (onGround()) {
				ROS_INFO("Setting state and performing action: POWER_OFF");
				setStateAndPerformAction(POWER_OFF);
			} else {
				ROS_INFO("Performing action: LAND_LOW");
				performStateAction();	
			}
		}
		// we don't care about the other states at the moment
	}
}

void poseCallback(const std_msgs::Float64MultiArray::ConstPtr& poseMsg) {
	updatePose(poseMsg);

	if (isLanderActive()) {
		if (getState() == SEEK_HOME) {
			// We must be above the landing pad (valid pose_estimate)
			ROS_INFO("Setting state and performing action: LAND_HIGH");
			setStateAndPerformAction(LAND_HIGH);
		} else if (getState() == LAND_HIGH) {
			// We need the quad to be stable to get a valid pose estimate.
			if (isStable() && (cyclesSinceControlInput > MAX_CONTROL_CYCLES)) {
				// Correct course, cap'n
				cyclesSinceControlInput = 0;
				ROS_INFO("Performing action: LAND_HIGH");
				performStateAction();
			} else {
				if (cyclesSinceControlInput == MAX_CONTROL_CYCLES) {
					// Give it a control input if a certain number
					// of estimates have passed
					performNeutralAction();
				} 
				// Stay the course, cap'n
				cyclesSinceControlInput++;
				ROS_INFO("Staying the course %d cycles", cyclesSinceControlInput);
			}
		} else if (getState() == LAND_LOW) {
			// Somehow we can see the landing pad again
			ROS_INFO("Setting state and performing action: LAND_HIGH");
			setStateAndPerformAction(LAND_HIGH);
		}
	} 
	// Not active, don't do anything
}

void rcCallback(const roscopter::RC::ConstPtr& rcMsg) {
	updateRC(rcMsg);

	if (updateLanderActive(rcMsg->channel[4])) {  // CHECK CHANNEL
		if (isLanderActive() && getState() == FLYING) {
			// Activate lander
			ROS_INFO("ACTIVATING LANDER");
			ROS_INFO_STREAM("Setting state and performing action: " << START_STATE_NAME);
			setStateAndPerformAction(START_STATE);
		} else if (!isLanderActive() && getState() != FLYING) {
			// Revert to manual control
			ROS_INFO("Setting state and performing action: FLYING");
			setStateAndPerformAction(FLYING);
		}
	}
	// Else no change
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Lander");
    ros::NodeHandle node;
    int const queueSize = 1000;

	ROS_INFO("Setting up subscriptions");
    // Subscribe to pose estimates
    ros::Subscriber simplePoseSub = \
    	node.subscribe("simplePose", queueSize, poseCallback);

    // Subscribe to RC inputs
    ros::Subscriber rcSub = \
    	node.subscribe("rc", queueSize, rcCallback);

    // Subscribe to quadcopter state
    ros::Subscriber hudSub = \
    	node.subscribe("vfr_hud", queueSize, hudCallback);

    ros::Subscriber attitudeSub = \
    	node.subscribe("attitude", queueSize, attitudeCallback);

    // Set up publisher for RC output
    rcPub = \
    	node.advertise<roscopter::RC>("send_rc", queueSize);

    ros::spin();

    return 0;
}
