/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


/* ==================================================
FUNCTIONALITY: This function uses PID controller to generate desired Voltages, sends them to motors.
INPUT: Desired angular velocity (using SYSTEM_TIMER) / feedback angular velocity (using MOTOR_TIMER).
OUTPUT: pwm_duty_cycle of 2 motors.
================================================== */



// ==================================================
// Constants/Variables/Functions 
// ==================================================
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt16.h"

#include <bitset>
#include "amr/amr.h"

using namespace asclinic_pkg;

AmrPose pre_pose = AmrPose();
AmrPose est_pose = AmrPose();

void subscriberCallbackPoseEst(const AmrPose& msg);
void subscriberCallbackPoseRef(const AmrPose& msg);

// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
	ros::init(argc, argv, amr_node::POSE_CONTROLLER);
	ros::NodeHandle nd;
	
	ros::Subscriber subscriber_pose_est = nd.subscribe(amr_topic::POSE_EST, 1, subscriberCallbackPoseEst);
	ros::Subscriber subscriber_pose_ref = nd.subscribe(amr_topic::POSE_REF, 1, subscriberCallbackPoseRef);

	ros::Publisher publisher_motor_pwm_duty_cycle = nd.advertise<MotorAngularVelocity>(amr_topic::MOTOR_ANGULAR_VELOCITY_REF, 100, false);

	ros::spin();

	return 0;
}


// ==================================================
// CALLBACK Functions
// ==================================================


// ==================================================
// Other Functions
// ==================================================