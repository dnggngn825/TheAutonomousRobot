/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


/* ==================================================
FUNCTIONALITY: Estimate the new pose of amr base on "odometry" technique.
INPUT: Wheel angular displacements
OUTPUT: New pose (x,y,phi)
================================================== */



// ==================================================
// Including library
// ==================================================
#include <math.h>
#include <stdlib.h>
#include "amr/amr.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace asclinic_pkg;


// ==================================================
// Constants/Variables/Functions 
// ==================================================
AmrPose pose = AmrPose();
ros::Publisher publisher_4_pose_fusion;

void subscriberCallback4MotorRotationAngle(const MotorRotationAngle& msg);
void estimate_new_pose(float delta_theta_left, float delta_theta_right);


// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
    ros::init(argc, argv, amr_node::ODOMETRY_ESTIMATOR);
    ros::NodeHandle nd;

	ros::Subscriber subscriber_4_encoder_data_odometry_ctl = nd.subscribe(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL, 1000, subscriberCallback4MotorRotationAngle);
    publisher_4_pose_fusion = nd.advertise<AmrPose>(amr_topic::POSE_DATA_FROM_ODOMETRY, 10, false);
    
    ros::spin();

	return 0;
}

// ==================================================
// Sub-functions
// ==================================================
/**
 * @brief Estimate the new position of the robot based on the angle displacement of two wheels
 * @param delta_theta_left angle displacement of left wheel 
 * @param delta_theta_right angle displacement of right wheel 
 */
void estimate_new_pose(float delta_theta_left, float delta_theta_right)
{
	// Odometry Calculation
	float delta_s = (AMR_BASE_RADIUS_WHEEL_L*delta_theta_left + AMR_BASE_RADIUS_WHEEL_R*delta_theta_right)/2;
	float delta_phi = (AMR_BASE_RADIUS_WHEEL_L*delta_theta_left - AMR_BASE_RADIUS_WHEEL_R*delta_theta_right)/(2*AMR_BASE_HALF_WHEEL_BASE);

	pose.pose_x = (pose.pose_x + delta_s*cos(pose.pose_phi + delta_phi/2));
	pose.pose_y = (pose.pose_y + delta_s*sin(pose.pose_phi + delta_phi/2));
	pose.pose_phi = (pose.pose_phi + delta_phi);
}

// ==================================================
// Subscriber Callbacks
// ==================================================
/**
 * @brief [Subscriber Callback] Subscribe to topic of rotation angles sent from ENCODER.
 * @param msg sent message.
 */
void subscriberCallback4MotorRotationAngle(const MotorRotationAngle& msg)
{
	float delta_theta_left = msg.rotation_angle_motor_l;
	float delta_theta_right = msg.rotation_angle_motor_r;

	ROS_INFO_STREAM("[ODOMETRY CALCULATOR] Rotation angles of motor (left:right) = " << delta_theta_left << ";" << delta_theta_right);
	
	estimate_new_pose(delta_theta_left, delta_theta_right);

	ROS_INFO_STREAM("[ODOMETRY CALCULATOR] New pose: ("<<pose.pose_x<<";"<<pose.pose_y<<";"<<pose.pose_phi<<")");
	publisher_4_pose_fusion.publish(pose);
}
