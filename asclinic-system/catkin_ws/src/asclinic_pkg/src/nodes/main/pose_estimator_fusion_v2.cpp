/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


// ==================================================
// Including library
// ==================================================
#include <math.h>
#include <stdlib.h>
#include "amr/amr.h"
#include "ros/ros.h"
#include <math.h>
using namespace asclinic_pkg;


bool is_cam_data_subscribed     = false;
bool is_odom_data_subscribed    = false;
bool is_pause_states_activated  = false;

AmrPose estimated_pose;
AmrPose estimated_pose_cam;
AmrPose estimated_pose_odom;


void subscriberCallback4MotorRotationAngle(const MotorRotationAngle& msg);
void subscriberCallback4EstimatedPoseFromCam(const AmrPose& msg);
void subscriberCallback4PauseStates(const std_msgs::UInt32& msg);
void estimate_pose_odom(float delta_theta_left, float delta_theta_right);
float clip_heading(float phi);


// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
    ros::init(argc, argv, amr_node::POSE_ESTIMATOR_FUSION);
    ros::NodeHandle nd;

    ros::Publisher publisher_4_localisation                 = nd.advertise<AmrPose>(amr_topic::POSE_EST, 1, false);
    ros::Subscriber subscriber_4_encoder_data_odometry_ctl  = nd.subscribe(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL, 1, subscriberCallback4MotorRotationAngle);
	ros::Subscriber subscriber_4_estimated_pose_from_cam    = nd.subscribe(amr_topic::POSE_DATA_FROM_CAM, 1, subscriberCallback4EstimatedPoseFromCam);

    ros::Subscriber subscriber_4_pause_states               = nd.subscribe(amr_topic::CAMERA_TRIGGER, 1, subscriberCallback4PauseStates);
	
    // Starting point
	estimated_pose.pose_x = 1.2f;
	estimated_pose.pose_y = 1.78f;
	// estimated_pose.pose_phi = 0.0f;
	estimated_pose.pose_phi = -M_PI/2;

	// estimated_pose.pose_x 	= 0.0f;
	// estimated_pose.pose_y 	= 0.0f;
	// estimated_pose.pose_phi = 0.0f;

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        // is_cam_data_subscribed = false;
        if (is_pause_states_activated && is_cam_data_subscribed)
        {
            estimated_pose = estimated_pose_cam;
            ROS_INFO_STREAM("[POSE_ESTIMATOR_FUSION] Read from cam: "<< estimated_pose.pose_x<< ";" << estimated_pose.pose_y << ";" << estimated_pose.pose_phi);
            // is_pause_states_activated = false;
        }
        else if (is_odom_data_subscribed)
        {
            estimated_pose = estimated_pose_odom;
            ROS_INFO_STREAM("[POSE_ESTIMATOR_FUSION] Read from odom: "<< estimated_pose.pose_x<< ";" << estimated_pose.pose_y << ";" << estimated_pose.pose_phi);
        }
        

        if ((is_pause_states_activated && is_cam_data_subscribed) || is_odom_data_subscribed)
        {
            publisher_4_localisation.publish(estimated_pose);
            // reset flags
            is_odom_data_subscribed = false;
            is_cam_data_subscribed = false;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
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
	float delta_theta_left    = msg.rotation_angle_motor_l;
	float delta_theta_right   = msg.rotation_angle_motor_r;
	
	estimate_pose_odom(delta_theta_left, delta_theta_right);
    is_odom_data_subscribed = true;

	// ROS_INFO_STREAM("[ODOMETRY_ESTIMATOR] New estimated_pose: ("<<estimated_pose.pose_x<<";"<<estimated_pose.pose_y<<";"<<estimated_pose.pose_phi<<")");
	// publisher_4_pose_fusion.publish(estimated_pose);
}


void subscriberCallback4EstimatedPoseFromCam(const AmrPose& msg)
{
    estimated_pose_cam = msg;
    
    if (msg.pose_x >= 5000)
        is_cam_data_subscribed = false;
    else 
        is_cam_data_subscribed = true;

}

void subscriberCallback4PauseStates(const std_msgs::UInt32& msg)
{
    is_pause_states_activated = !is_pause_states_activated;
}


/**
 * @brief Estimate the new position of the robot based on the angle displacement of two wheels
 * @param delta_theta_left angle displacement of left wheel 
 * @param delta_theta_right angle displacement of right wheel 
 */
void estimate_pose_odom(float delta_theta_left, float delta_theta_right)
{
	// Odometry Calculation
	float delta_s 	= (AMR_BASE_RADIUS_WHEEL_L*delta_theta_left + AMR_BASE_RADIUS_WHEEL_R*delta_theta_right)/2;
	float delta_phi = (AMR_BASE_RADIUS_WHEEL_L*delta_theta_right - AMR_BASE_RADIUS_WHEEL_R*delta_theta_left)/(2*AMR_BASE_HALF_WHEEL_BASE);

	estimated_pose_odom.pose_x 	    = (estimated_pose.pose_x   + delta_s*cos(estimated_pose.pose_phi + delta_phi/2));
	estimated_pose_odom.pose_y 	    = (estimated_pose.pose_y   + delta_s*sin(estimated_pose.pose_phi + delta_phi/2));
	estimated_pose_odom.pose_phi 	= (estimated_pose.pose_phi + delta_phi);
    estimated_pose_odom.pose_phi    = clip_heading(estimated_pose_odom.pose_phi);

}

float clip_heading(float phi)
{
    if (phi>M_PI)
        phi = phi - 2*M_PI;
    else if (phi <= -M_PI)
        phi = phi + 2*M_PI;
    return phi;
}
