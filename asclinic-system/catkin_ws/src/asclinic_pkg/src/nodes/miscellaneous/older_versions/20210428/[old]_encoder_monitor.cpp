/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */

/* 
This node reads the Encoder triggered, publishes the encoder counter to INNER loop (motor control) and OUTER loop (system control)
 */


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
#include "std_msgs/Int32.h"

#include "asclinic_pkg/MotorRotationAngle.h"
#include "asclinic_pkg/MotorAngularVelocity.h"
#include "asclinic_pkg/Encoder.h"

#include "amr/amr.h"

#include <iostream>
#include <fstream>

// Namespacing the package
using namespace asclinic_pkg;



// ==================================================
// Constants/Variables/Functions 
// ==================================================
// DEVICE INFO // Note: for the 40-pin header of the Jetson SBCs -> "/dev/gpiochip0"
const char * gpio_chip_name = "/dev/gpiochip0"; 
struct gpiod_chip *gpiod_chip = gpiod_chip_open(gpio_chip_name); // Open the GPIO chip


// Variables and Functions
ros::Publisher publisher_4_motor_control;
ros::Publisher publisher_4_odometry_control;


Pololu_SMC_G2_Encoder encoder_motor_l = Pololu_SMC_G2_Encoder(AMR_CW_DIRECTION_MOTOR_L);
Pololu_SMC_G2_Encoder encoder_motor_r = Pololu_SMC_G2_Encoder(AMR_CW_DIRECTION_MOTOR_R);


int encoder_counter_4_motor_control_motor_l = 0;
int encoder_counter_4_motor_control_motor_r = 0;
int encoder_counter_4_odometry_control_motor_l = 0;
int encoder_counter_4_odometry_control_motor_r = 0;


// Functions
void subscriberCallbackForMotorControl(const std_msgs::UInt32& msg);
void subscriberCallbackForOdometryControl(const std_msgs::UInt32& msg);

bool reset_encoder_counter_4_motor_control();
bool reset_encoder_counter_4_odometry_control();
void update_encoder_counters(int encoder_trigger_l, int encoder_trigger_r);


// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, amr_node::ENCODER_MONITOR);
	ros::NodeHandle nodeHandle;


    publisher_4_motor_control = nodeHandle.advertise<MotorAngularVelocity>(amr_topic::ENCODER_DATA_4_MOTOR_CONTROL, 10, false);    
    publisher_4_odometry_control = nodeHandle.advertise<MotorRotationAngle>(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL, 10, false);
    
	ros::Subscriber subscriber_4_motor_control = nodeHandle.subscribe(amr_topic::TIMER_4_MOTOR_CONTROL, 1, subscriberCallbackForMotorControl);
	ros::Subscriber subscriber_4_odometry_control = nodeHandle.subscribe(amr_topic::TIMER_4_SYSTEM_CONTROL, 1, subscriberCallbackForOdometryControl);



	encoder_motor_l.set_line_channel(AMR_ENCODER_PIN_CHANNEL_A_MOTOR_L, AMR_ENCODER_PIN_CHANNEL_B_MOTOR_L);
	encoder_motor_r.set_line_channel(AMR_ENCODER_PIN_CHANNEL_A_MOTOR_R, AMR_ENCODER_PIN_CHANNEL_B_MOTOR_R);

	encoder_motor_l.set_gpiod_chip(gpio_chip_name, gpiod_chip);
	encoder_motor_r.set_gpiod_chip(gpio_chip_name, gpiod_chip);

	encoder_motor_l.set_gpiod_line_request_events(Pololu_SMC_G2_Encoder::Line_Request_Event::FALLING_EDGE); // falling edge 
	encoder_motor_r.set_gpiod_line_request_events(Pololu_SMC_G2_Encoder::Line_Request_Event::FALLING_EDGE);


	struct gpiod_line_event event_motor_l;
	struct gpiod_line_event event_motor_r;

	int encoder_trigger_l = 0, encoder_trigger_r = 0;
	while (ros::ok())
	{
		// Monitor for the requested events on the GPIO line
		encoder_trigger_l = encoder_motor_l.monitor_encoder_trigger(event_motor_l);
		encoder_trigger_r = encoder_motor_r.monitor_encoder_trigger(event_motor_r);

		update_encoder_counters(encoder_trigger_l, encoder_trigger_r);

		ros::spinOnce();
	} // END OF: "while (ros::ok())"

	// Close the GPIO chip
	gpiod_chip_close(gpiod_chip);

	return 0;
}


// ==================================================
// CALLBACK Functions
// ==================================================
// Functionality: Subscribe to INNER LOOP timer call, in order to publish "angular_velocity" of 2 motors (rad)
void subscriberCallbackForMotorControl(const std_msgs::UInt32& msg)
{
	// static uint trajectory_counter = 0;
	// static bool is_motor_stop = false;

	encoder_motor_l.set_angular_velocity_rpm(encoder_counter_4_motor_control_motor_l, AMR_TIMER_4_MOTOR_CONTROL);
	encoder_motor_r.set_angular_velocity_rpm(encoder_counter_4_motor_control_motor_r, AMR_TIMER_4_MOTOR_CONTROL);

	MotorAngularVelocity motor_angular_velocity = MotorAngularVelocity();
	motor_angular_velocity.angular_velocity_motor_l = encoder_motor_l.get_angular_velocity_rdps();
	motor_angular_velocity.angular_velocity_motor_r = encoder_motor_r.get_angular_velocity_rdps();

	publisher_4_motor_control.publish(motor_angular_velocity);

	// ROS_INFO_STREAM("[ENCODER READER] Angular velocity (L/R) (rad/s): "<< encoder_motor_l.get_angular_velocity_rdps()<<";"<<encoder_motor_r.get_angular_velocity_rdps());

    // reset_encoder_counter_4_motor_control();

	// trajectory_counter++;
}

// Functionality: Subscribe to OUTER LOOP timer call, in order to publish "rotation_angle" of 2 motors (rad)
void subscriberCallbackForOdometryControl(const std_msgs::UInt32& msg)
{
	encoder_motor_l.set_angular_velocity_rpm(encoder_counter_4_odometry_control_motor_l, AMR_TIMER_4_SYSTEM_CONTROL);
	encoder_motor_r.set_angular_velocity_rpm(encoder_counter_4_odometry_control_motor_r, AMR_TIMER_4_SYSTEM_CONTROL);	

	MotorRotationAngle motor_rotation_angle = MotorRotationAngle();
	motor_rotation_angle.rotation_angle_motor_l = encoder_motor_l.get_rotation_angle_rad(AMR_TIMER_4_SYSTEM_CONTROL);
	motor_rotation_angle.rotation_angle_motor_r = encoder_motor_r.get_rotation_angle_rad(AMR_TIMER_4_SYSTEM_CONTROL);

	publisher_4_odometry_control.publish(motor_rotation_angle);

	ROS_INFO_STREAM("[ENCODER READER] Encoder counter (L/R): "<<encoder_counter_4_odometry_control_motor_l<<";"<<encoder_counter_4_odometry_control_motor_r);
	// ROS_INFO_STREAM("[ENCODER READER] Rotation angles (L/R): "<< motor_rotation_angle.rotation_angle_motor_l
	// 														  <<";"<<motor_rotation_angle.rotation_angle_motor_r);

    reset_encoder_counter_4_odometry_control();
}


// ==================================================
// Other Functions
// ==================================================
// Functionality: reset counter for motor control loop
bool reset_encoder_counter_4_motor_control()
{
    encoder_counter_4_motor_control_motor_l = 0;
	encoder_counter_4_motor_control_motor_r = 0;

	return true;
}

// Functionality: reset counter for system control loop
bool reset_encoder_counter_4_odometry_control()
{
    encoder_counter_4_odometry_control_motor_l = 0;
	encoder_counter_4_odometry_control_motor_r = 0;

	return true;
}

void update_encoder_counters(int encoder_trigger_l, int encoder_trigger_r)
{
	encoder_counter_4_motor_control_motor_l += encoder_trigger_l;
	encoder_counter_4_odometry_control_motor_l += encoder_trigger_l;

	encoder_counter_4_motor_control_motor_r += encoder_trigger_r;
	encoder_counter_4_odometry_control_motor_r += encoder_trigger_r;
}


// void write_data()
// {
// 	std::ofstream myfile;
// 	myfile.open ("src/asclinic_pkg/src/example.csv");
// 	for (int i=0; i<states_list_length; i++)
// 		myfile << (std::to_string(states_list[i][0]) + "," + std::to_string(states_list[i][1]) + "\n");
	
// 	myfile.close();  	
// }

