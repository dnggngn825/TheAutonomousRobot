/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


/* ==================================================
FUNCTIONALITY: This node subscribes to the encoder counters from 2 wheels and saves the rotation angles into a csv file (for motor parameters estimation).
INPUT: Encoder counter left/right
OUTPUT: csv file
================================================== */


// ==================================================
// Including library
// ==================================================
#include "ros/ros.h"
#include <ros/package.h>
#include "amr/amr.h"

#include <iostream>
#include <fstream>

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

float enc_data_storage[3000][3]; //[time, angle_l, angle_r]
int enc_data_storage_length = 0;

float rotation_angle_l = 0;
float rotation_angle_r = 0;

bool is_motors_stopped = false;

// Functions
void subscriberCallbackForMotorLeft(const amr_msgs::EncCounter& msg);
void subscriberCallbackForMotorRight(const amr_msgs::EncCounter& msg);
void subscriberCallbackForMotorStop(const amr_msgs::EncCounter& msg);
void publishRotationAngleForOdometryControl();
void write_data();


// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "encoder_subscriber");
    ros::NodeHandle nd;

    // publisher_encoder_monitor_data = nd.advertise<MotorRotationAngle>(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL, 10, false);

    ros::Subscriber subscriber_4_motor_l = nd.subscribe(amr_topic::ENCODER_DATA_4_MOTOR_CONTROL_L, 1, subscriberCallbackForMotorLeft);
    ros::Subscriber subscriber_4_motor_r = nd.subscribe(amr_topic::ENCODER_DATA_4_MOTOR_CONTROL_R, 1, subscriberCallbackForMotorRight);

    ros::Subscriber subscriber_4_motor_stop = nd.subscribe("motor_stop", 1, subscriberCallbackForMotorStop);

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

void subscriberCallbackForMotorStop(const amr_msgs::EncCounter& msg)
{
    is_motors_stopped = true;
    write_data();
}


// ==================================================
// Other Functions
// ==================================================
/**
 * @brief Publish angular velocity of 2 wheels, if fully received data from both encoders.
 */
void publishRotationAngleForOdometryControl()
{
    if ((~is_motors_stopped) && (received_data_encoder_l && received_data_encoder_r))
    {
        rotation_angle_l += encoder_l.get_rotation_angle_rad(sampling_time_in_sec);
        rotation_angle_r += encoder_r.get_rotation_angle_rad(sampling_time_in_sec);

        enc_data_storage[enc_data_storage_length][0] = sampling_time_in_sec*(enc_data_storage_length+1);
        enc_data_storage[enc_data_storage_length][1] = rotation_angle_l;
        enc_data_storage[enc_data_storage_length][2] = rotation_angle_r;

        enc_data_storage_length ++;

        received_data_encoder_l = false;
        received_data_encoder_r = false;
    }
}


void write_data()
{
	std::ofstream myfile;
	myfile.open("/home/asc05/asclinic-system/catkin_ws/src/asclinic_pkg/src/example.csv");
	for (int i=0; i<enc_data_storage_length; i++)
		myfile <<   (std::to_string(enc_data_storage[i][0]) 
                    + "," + std::to_string(enc_data_storage[i][1]) 
                    + "," + std::to_string(enc_data_storage[i][2])
                    + "\n");
	
	myfile.close();  	
}

