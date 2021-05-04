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
#define _USE_MATH_DEFINES // for C
#include <math.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <gpiod.h>

// Message packages
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"

#include "amr/amr.h"


// Namespacing the package
using namespace asclinic_pkg;


ros::Publisher publisher_encoder_data_4_motor_control;
ros::Publisher publisher_encoder_data_4_odometry_control;
int incoming_encoder_data_4_motor_control_tracker = 0;
int incoming_encoder_data_4_odometry_control_tracker = 0;


int encoder_counter_4_motor_control_l = 0;
int encoder_counter_4_odometry_control_l = 0;
int encoder_counter_4_motor_control_r = 0;
int encoder_counter_4_odometry_control_r = 0;


// Functions
void subscriberCallbackForMotorControl(const MotorAngularVelocity& msg);
void subscriberCallbackForOdometryControl(const MotorRotationAngle& msg);


int main(int argc, char* argv[])
{
    ros::init(argc, argv, amr_node::ENCODER_MONITOR);
    ros::NodeHandle nd;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    publisher_encoder_data_4_motor_control = nd.advertise<MotorAngularVelocity>(amr_topic::ENCODER_DATA_4_MOTOR_CONTROL, 10, false);
    publisher_encoder_data_4_odometry_control = nd.advertise<MotorRotationAngle>(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL, 10, false);

    ros::Subscriber subscriber_4_motor_control_l = nd.subscribe(amr_topic::ENCODER_DATA_4_MOTOR_CONTROL_L, 1, subscriberCallbackForMotorControl);
    ros::Subscriber subscriber_4_motor_control_r = nd.subscribe(amr_topic::ENCODER_DATA_4_MOTOR_CONTROL_R, 1, subscriberCallbackForMotorControl);
    ros::Subscriber subscriber_4_odometry_control_l = nd.subscribe(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL_L, 1, subscriberCallbackForOdometryControl);
	ros::Subscriber subscriber_4_odometry_control_r = nd.subscribe(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL_R, 1, subscriberCallbackForOdometryControl);

	// ros::Subscriber subscriber_4_odometry_control_l = nd.subscribe(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL_L, 1, subscriberCallbackForOdometryControlLeft);
	// ros::Subscriber subscriber_4_odometry_control_r = nd.subscribe(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL_R, 1, subscriberCallbackForOdometryControlRight);

    // ros::spin();
    ros::waitForShutdown();
}


void subscriberCallbackForMotorControl(const MotorAngularVelocity& msg)
{
    incoming_encoder_data_4_motor_control_tracker++;
    
    float angular_velocity_motor_l = msg.angular_velocity_motor_l;
    float angular_velocity_motor_r = msg.angular_velocity_motor_r;

    MotorAngularVelocity angular_velocities = MotorAngularVelocity();
    if (angular_velocity_motor_l != 0)
        angular_velocities.angular_velocity_motor_l = msg.angular_velocity_motor_l;
    else 
        angular_velocities.angular_velocity_motor_r = msg.angular_velocity_motor_r;

    if (incoming_encoder_data_4_motor_control_tracker == 2)
    {
        publisher_encoder_data_4_motor_control.publish(angular_velocities);
        // ROS_INFO_STREAM("[ENCODER MONITOR] Angular Velocity in Odom(L/R): "<< angular_velocities.angular_velocity_motor_l
        //                                                                 << ";"<< angular_velocities.angular_velocity_motor_r);
        incoming_encoder_data_4_motor_control_tracker = 0;
    }
}

void subscriberCallbackForOdometryControl(const MotorRotationAngle& msg)
{
    incoming_encoder_data_4_odometry_control_tracker++;

    float rotation_angle_motor_l = msg.rotation_angle_motor_l;
    float rotation_angle_motor_r = msg.rotation_angle_motor_r;
    
    MotorRotationAngle rotation_angles = MotorRotationAngle();
    if (rotation_angle_motor_l != 0)
        rotation_angles.rotation_angle_motor_l = rotation_angle_motor_l;
    else 
        rotation_angles.rotation_angle_motor_r = rotation_angle_motor_r;

    if (incoming_encoder_data_4_odometry_control_tracker == 2)
    {
        publisher_encoder_data_4_odometry_control.publish(rotation_angles);
        ROS_INFO_STREAM("[ENCODER MONITOR] Rotation Angle in Odom (L/R): "<< rotation_angles.rotation_angle_motor_l
                                                                << ";"<< rotation_angles.rotation_angle_motor_r);
        incoming_encoder_data_4_odometry_control_tracker = 0;
    }
}
