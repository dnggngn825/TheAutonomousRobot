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

// Namespacing the package
using namespace asclinic_pkg;


// ==================================================
// Constants/Variables/Functions 
// ==================================================
ros::Publisher publisher_encoder_monitor_data;
float sampling_time_in_sec = AMR_TIMER_4_MOTOR_CONTROL;

Pololu_SMC_G2_Encoder encoder_l = Pololu_SMC_G2_Encoder();
Pololu_SMC_G2_Encoder encoder_r = Pololu_SMC_G2_Encoder();

bool received_data_encoder_l = false;
bool received_data_encoder_r = false;

// Functions
void subscriberCallbackForMotorLeft(const amr_msgs::EncCounter& msg);
void subscriberCallbackForMotorRight(const amr_msgs::EncCounter& msg);
void publishAngularVelocityForMotorControl();


// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
    ros::init(argc, argv, amr_node::ENCODER_MONITOR_4_MOTOR_CONTROL);
    ros::NodeHandle nd;

    publisher_encoder_monitor_data = nd.advertise<MotorAngularVelocity>(amr_topic::MOTOR_ANGULAR_VELOCITY_EST, 10, false);

    ros::Subscriber subscriber_4_motor_l = nd.subscribe(amr_topic::ENCODER_DATA_4_MOTOR_CONTROL_L, 1, subscriberCallbackForMotorLeft);
    ros::Subscriber subscriber_4_motor_r = nd.subscribe(amr_topic::ENCODER_DATA_4_MOTOR_CONTROL_R, 1, subscriberCallbackForMotorRight);

    while (ros::ok())
    {
        publishAngularVelocityForMotorControl();
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
    int encoder_counter_4_motor_l = msg.data;

    encoder_l.set_angular_velocity_rpm(encoder_counter_4_motor_l, sampling_time_in_sec);
    received_data_encoder_l = true;
}

/**
 * @brief Callback function that subscribes to Encoder counter from right wheel encoder
 * @param msg Encoder counter
 */
void subscriberCallbackForMotorRight(const amr_msgs::EncCounter& msg)
{
    int encoder_counter_4_motor_r = msg.data;

    encoder_r.set_angular_velocity_rpm(encoder_counter_4_motor_r, sampling_time_in_sec);
    received_data_encoder_r = true;
}


// ==================================================
// Other Functions
// ==================================================
/**
 * @brief Publish angular velocity of 2 wheels, if fully received data from both encoders.
 */
void publishAngularVelocityForMotorControl()
{
    if (received_data_encoder_l && received_data_encoder_r)
    {
        float angular_velocity_l = encoder_l.get_angular_velocity_rdps();
        float angular_velocity_r = encoder_r.get_angular_velocity_rdps();

        MotorAngularVelocity motor_angular_velocity = MotorAngularVelocity();
        motor_angular_velocity.angular_velocity_motor_l = angular_velocity_l;
        motor_angular_velocity.angular_velocity_motor_r = angular_velocity_r;

        publisher_encoder_monitor_data.publish(motor_angular_velocity);
        received_data_encoder_l = false;
        received_data_encoder_r = false;
    }
}
