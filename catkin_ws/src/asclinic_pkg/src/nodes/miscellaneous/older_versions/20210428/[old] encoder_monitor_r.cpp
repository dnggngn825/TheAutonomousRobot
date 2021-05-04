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

#include "amr/amr.h"


// Namespacing the package
using namespace asclinic_pkg;



// ==================================================
// Constants/Variables/Functions 
// ==================================================
// DEVICE INFO // Note: for the 40-pin header of the Jetson SBCs -> "/dev/gpiochip0"
// const char * gpio_chip_name = "/dev/gpiochip0"; 
struct gpiod_chip *gpiod_chip = gpiod_chip_open(AMR_GPIO_CHIP_NAME); // Open the GPIO chip
struct gpiod_line_event event_motor_r;


// Variables and Functions
ros::Publisher publisher_4_motor_control;
ros::Publisher publisher_4_odometry_control;

Pololu_SMC_G2_Encoder encoder_motor_r = Pololu_SMC_G2_Encoder(AMR_CW_DIRECTION_MOTOR_R);

int encoder_counter_4_motor_control_motor_r = 0;
int encoder_counter_4_odometry_control_motor_r = 0;


// Functions
void subscriberCallbackForMotorControl(const std_msgs::UInt32& msg);
void subscriberCallbackForOdometryControl(const std_msgs::UInt32& msg);

bool reset_encoder_counter_4_motor_control();
bool reset_encoder_counter_4_odometry_control();
void update_encoder_counters(int encoder_trigger_r);


// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, amr_node::ENCODER_MONITOR_R);
	ros::NodeHandle nd;


    publisher_4_motor_control = nd.advertise<MotorAngularVelocity>(amr_topic::ENCODER_DATA_4_MOTOR_CONTROL_R, 10, false);    
    publisher_4_odometry_control = nd.advertise<MotorRotationAngle>(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL_R, 10, false);
    
	ros::Subscriber subscriber_4_motor_control = nd.subscribe(amr_topic::TIMER_4_MOTOR_CONTROL, 1, subscriberCallbackForMotorControl);
	ros::Subscriber subscriber_4_odometry_control = nd.subscribe(amr_topic::TIMER_4_SYSTEM_CONTROL, 1, subscriberCallbackForOdometryControl);


	// encoder_motor_r.set_gpiod_chip(AMR_GPIO_CHIP_NAME, gpiod_chip);
	encoder_motor_r.set_gpiod_chip();
	encoder_motor_r.set_gpiod_line_channel(AMR_ENCODER_PIN_CHANNEL_A_MOTOR_R, AMR_ENCODER_PIN_CHANNEL_B_MOTOR_R);
	encoder_motor_r.set_gpiod_line_request_events(Pololu_SMC_G2_Encoder::Line_Request_Event::FALLING_EDGE);
	// encoder_motor_r.set_gpiod_line_request_events(Pololu_SMC_G2_Encoder::Line_Request_Event::BOTH_EDGES);


	
	int encoder_trigger_r = 0;
	// ros::Rate loop_rate(200000);
	while (ros::ok())
	{
		encoder_trigger_r = encoder_motor_r.monitor_encoder_trigger_falling_edge_wo_debounce(event_motor_r);
		// encoder_trigger_r = encoder_motor_r.monitor_encoder_trigger_falling_edge_w_time_debounce(event_motor_r);
		// encoder_trigger_r = encoder_motor_r.monitor_encoder_polling_w_sw_debounce();

		update_encoder_counters(encoder_trigger_r);

		ros::spinOnce();
		// loop_rate.sleep();
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
	encoder_motor_r.set_angular_velocity_rpm(encoder_counter_4_motor_control_motor_r, AMR_TIMER_4_MOTOR_CONTROL);

	MotorAngularVelocity motor_angular_velocity = MotorAngularVelocity();
	motor_angular_velocity.angular_velocity_motor_l = 0;
	motor_angular_velocity.angular_velocity_motor_r = encoder_motor_r.get_angular_velocity_rdps();

	publisher_4_motor_control.publish(motor_angular_velocity);

	reset_encoder_counter_4_motor_control();
}

// Functionality: Subscribe to OUTER LOOP timer call, in order to publish "rotation_angle" of 2 motors (rad)
void subscriberCallbackForOdometryControl(const std_msgs::UInt32& msg)
{
	encoder_motor_r.set_angular_velocity_rpm(encoder_counter_4_odometry_control_motor_r, AMR_TIMER_4_SYSTEM_CONTROL);	

	MotorRotationAngle motor_rotation_angle = MotorRotationAngle();
	motor_rotation_angle.rotation_angle_motor_l = 0;
	motor_rotation_angle.rotation_angle_motor_r = encoder_motor_r.get_rotation_angle_rad(AMR_TIMER_4_SYSTEM_CONTROL);

	publisher_4_odometry_control.publish(motor_rotation_angle);

	ROS_INFO_STREAM("[ENCODER MONITOR R] Encoder counter in Odom (R): "<<encoder_counter_4_odometry_control_motor_r);

    reset_encoder_counter_4_odometry_control();
}


// ==================================================
// Other Functions
// ==================================================
// Functionality: reset counter for motor control loop
bool reset_encoder_counter_4_motor_control()
{
	encoder_counter_4_motor_control_motor_r = 0;
	return true;
}

// Functionality: reset counter for system control loop
bool reset_encoder_counter_4_odometry_control()
{
	encoder_counter_4_odometry_control_motor_r = 0;
	return true;
}

void update_encoder_counters(int encoder_trigger_r)
{
	encoder_counter_4_motor_control_motor_r += encoder_trigger_r;
	encoder_counter_4_odometry_control_motor_r += encoder_trigger_r;
}


