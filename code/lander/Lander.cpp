#include "Controller.h"
#include "LanderStates.h"
#include "QuadcopterState.h"

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include "roscopter/Attitude.h"
#include "roscopter/RC.h"
#include "roscopter/VFR_HUD.h"

ros::Publisher rcPub;


void performAction() {
	ROS_INFO("Sending control message");
	roscopter::RC controlMsg = performStateAction();
	rcPub.publish(controlMsg);
}

void setStateAndPerformAction(States newState) {
	setState(newState);
	performAction();
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
				performAction();	
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
			// Stay the course, cap'n
			ROS_INFO("Performing action: LAND_HIGH");
			performAction();
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
			ROS_INFO("Setting state and performing action: SEEK_HOME");
			setStateAndPerformAction(SEEK_HOME);
		} else if (!isLanderActive() && getState() != FLYING) {
			// Revert to manual control
			ROS_INFO("Setting state and performing action: FLYING");
			setStateAndPerformAction(FLYING);
		}
	}
	// else no change
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Lander");
    ros::NodeHandle node;
    int const queueSize = 1000;
    ros::Rate loopRate(4);  // publish messages at 4 Hz
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
