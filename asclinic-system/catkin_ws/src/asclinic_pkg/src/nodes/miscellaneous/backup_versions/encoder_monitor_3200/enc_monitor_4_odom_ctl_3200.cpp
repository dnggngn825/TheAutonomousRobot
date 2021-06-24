/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


/* ==================================================
FUNCTIONALITY: This node subscribes to the encoder counters from 2 wheels and publishes the angular velocities to the motor controller.
INPUT: Encoder counter left/right
OUTPUT: Motor angular velocity 
================================================== */


// ==================================================
// Including library
// ==================================================
#include "ros/ros.h"
#include <ros/package.h>
#include "amr/amr.h"
using namespace asclinic_pkg;


// ==================================================
// Constants/Variables/Functions 
// ==================================================
ros::Publisher publisher_encoder_monitor_data;
float sampling_time_in_sec = AMR_TIMER_4_SYSTEM_CONTROL;

Pololu_SMC_G2_Encoder encoder_l = Pololu_SMC_G2_Encoder();
Pololu_SMC_G2_Encoder encoder_r = Pololu_SMC_G2_Encoder();

bool received_data_encoder_l = false;
bool received_data_encoder_r = false;

int encoder_counter_4_motor_l = 0;
int encoder_counter_4_motor_r = 0;


// Functions
void subscriberCallbackForMotorLeft(const amr_msgs::EncCounter& msg);
void subscriberCallbackForMotorRight(const amr_msgs::EncCounter& msg);
void publishRotationAngleForOdometryControl();


// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
    ros::init(argc, argv, amr_node::ENCODER_MONITOR_4_ODOM_CONTROL_3200);
    ros::NodeHandle nd;

    publisher_encoder_monitor_data = nd.advertise<MotorRotationAngle>(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL, 1, false);

    ros::Subscriber subscriber_4_motor_l = nd.subscribe(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL_L, 1, subscriberCallbackForMotorLeft);
    ros::Subscriber subscriber_4_motor_r = nd.subscribe(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL_R, 1, subscriberCallbackForMotorRight);

    while (ros::ok())
    {
        publishRotationAngleForOdometryControl();
        ros::spinOnce();
    }
}


// ==================================================
// CALLBACK Functions
// ==================================================
/**
 * @brief Callback function that subscribes to Encoder counter from left wheel encoder
 * @param msg Encoder counter
 */
void subscriberCallbackForMotorLeft(const amr_msgs::EncCounter& msg)
{
    encoder_counter_4_motor_l = msg.data;
    received_data_encoder_l = true;
}

/**
 * @brief Callback function that subscribes to Encoder counter from right wheel encoder
 * @param msg Encoder counter
 */
void subscriberCallbackForMotorRight(const amr_msgs::EncCounter& msg)
{
    encoder_counter_4_motor_r = msg.data;
    received_data_encoder_r = true;
}


// ==================================================
// Other Functions
// ==================================================
/**
 * @brief Publish angular velocity of 2 wheels, if fully received data from both encoders.
 */
void publishRotationAngleForOdometryControl()
{
    if (received_data_encoder_l && received_data_encoder_r)
    {
        encoder_l.set_angular_velocity_rpm(encoder_counter_4_motor_l, sampling_time_in_sec);
        encoder_r.set_angular_velocity_rpm(encoder_counter_4_motor_r, sampling_time_in_sec);

        float rotation_angle_rad_l = encoder_l.get_rotation_angle_rad(sampling_time_in_sec);
        float rotation_angle_rad_r = encoder_r.get_rotation_angle_rad(sampling_time_in_sec);

        MotorRotationAngle motor_rotation_angle = MotorRotationAngle();
        motor_rotation_angle.rotation_angle_motor_l = rotation_angle_rad_l;
        motor_rotation_angle.rotation_angle_motor_r = rotation_angle_rad_r;

        publisher_encoder_monitor_data.publish(motor_rotation_angle);

        // // Display data
        // static int counter = 0;
        // counter++;
        // if (counter == 1)
        // {
        //     ROS_INFO_STREAM("[ENCODER_MONITOR_4_ODOM_CONTROL] Encoder counters (L/R): " << encoder_counter_4_motor_l << ";" << encoder_counter_4_motor_r);
        //     counter = 0;
        // }


        received_data_encoder_l = false;
        received_data_encoder_r = false;
        encoder_counter_4_motor_l = 0;
        encoder_counter_4_motor_r = 0;

    }
}
