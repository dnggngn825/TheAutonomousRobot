/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


// Include some useful libraries
#include <math.h>
#include <stdlib.h>
#include "amr/amr.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

// Include the asclinic message types
#include "asclinic_pkg/AmrPose.h"
#include "asclinic_pkg/MotorRotationAngle.h"

// Namespacing the package
using namespace asclinic_pkg;


// Initialize global variables
AmrPose pose = AmrPose();
// double pose_x = 0, pose_y = 0, pose_phi = 0;
ros::Publisher publisher_4_pose_fusion;


// Publishing and subscribing functions
void subscriberCallback(const AmrPose& msg);
void subscriberCallback4MotorRotationAngle(const MotorRotationAngle& msg);
bool calculate_new_pose(float delta_theta_left, float delta_theta_right);


// just for TESTING
void subscriberCallback(const AmrPose& msg)
{
	// Display that a message was received
	ROS_INFO_STREAM("[ODOMETRY CALCULATOR] Message receieved with data: ("<<pose.pose_x<<";"<<pose.pose_y<<";"<<pose.pose_phi<<")");
}

// Subscribe to topic sent from ENCODER
void subscriberCallback4MotorRotationAngle(const MotorRotationAngle& msg)
{
	float delta_theta_left = msg.rotation_angle_motor_l;
	float delta_theta_right = msg.rotation_angle_motor_r;

	ROS_INFO_STREAM("[ODOMETRY CALCULATOR] Rotation angles of motor (left:right) = " << 
					delta_theta_left << ";" << delta_theta_right);
	
	calculate_new_pose(delta_theta_left, delta_theta_right);

	ROS_INFO_STREAM("[ODOMETRY CALCULATOR] New pose: ("<<pose.pose_x<<";"<<pose.pose_y<<";"<<pose.pose_phi<<")");
	publisher_4_pose_fusion.publish(pose);
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "odometry_calculator");
    ros::NodeHandle nodeHandle;

	ros::Subscriber subscriber_4_encoder_data_odometry_ctl = nodeHandle.subscribe("encoder_data_odometry_ctl_topic", 1000, subscriberCallback4MotorRotationAngle);

    publisher_4_pose_fusion = nodeHandle.advertise<AmrPose>("pose_est_odometry", 10, false);
    
	ROS_INFO("[ODOMETRY CALCULATOR] Ready!");
    ros::spin();

	return 0;
}


bool calculate_new_pose(float delta_theta_left, float delta_theta_right)
{
	// Odometry Calculation
	float delta_s = (AMR_BASE_RADIUS_WHEEL_L*delta_theta_left + AMR_BASE_RADIUS_WHEEL_R*delta_theta_right)/2;
	float delta_phi = (AMR_BASE_RADIUS_WHEEL_L*delta_theta_left - AMR_BASE_RADIUS_WHEEL_R*delta_theta_right)/(2*AMR_BASE_HALF_WHEEL_BASE);

	// pose_x = pose_x + delta_s*cos(pose_phi + delta_phi/2);
	// pose_y = pose_y + delta_s*sin(pose_phi + delta_phi/2);
	// pose_phi = pose_phi + delta_phi;

	pose.pose_x = (pose.pose_x + delta_s*cos(pose.pose_phi + delta_phi/2));
	pose.pose_y = (pose.pose_y + delta_s*sin(pose.pose_phi + delta_phi/2));
	pose.pose_phi = (pose.pose_phi + delta_phi);

	return true;
}