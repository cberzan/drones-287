#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include <stdint.h>

#include "roscopter/Attitude.h"
#include "roscopter/VFR_HUD.h"

namespace QuadcopterState {

	struct QuadcopterState {
		float32_t roll;
		float32_t pitch;
		float32_t yaw; 
		float32_t airspeed;
		float32_t groundspeed; 
		int16_t heading;
		uint16_t throttle;
		float32_t alt;
		float32_t climb;
	};


	struct QuadcopterPose {
		float32 x;
		float32 y;
		float32 z;
		float32 yaw;
	};

	/**
	* Updates the latest attitude values from the autopilot
	* 
	* Input:
	* attitudeMsg = Attitude message posted by roscopter
	*/
	void updateAttitude(const roscopter::Attitude::ConstPtr& attitudeMsg);

	/**
	* Updates the latest telemetry values from the autopilot
	* 
	* Input:
	* hudMsg = VFR_HUD message posted by roscopter
	*/
	void updateTelemetry (const roscopter::VFR_HUD::ConstPtr& hudMsg);

	/**
	* Updates the latest pose values from the pose_estimator
	* 
	* Input:
	* poseMsg = simplePose message posted by pose_estimator
	*/
	void updatePose (const std_msgs::Float64MultiArray::ConstPtr& poseMsg);


	bool onGround();

	QuadcopterState getQuadcopterState();

	QuadcopterPose getQuadcopterPose();
}