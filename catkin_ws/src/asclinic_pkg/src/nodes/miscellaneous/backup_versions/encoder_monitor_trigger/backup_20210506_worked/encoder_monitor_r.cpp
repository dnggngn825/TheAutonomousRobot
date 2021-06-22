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
#include "amr/amr.h"


// Namespacing the package
using namespace asclinic_pkg;



// ==================================================
// Constants/Variables/Functions 
// ==================================================
// DEVICE INFO // Note: for the 40-pin header of the Jetson SBCs -> "/dev/gpiochip0"
// const char * gpio_chip_name = "/dev/gpiochip0"; 
struct gpiod_chip *gpiod_chip = gpiod_chip_open(AMR_GPIO_CHIP_NAME); // Open the GPIO chip
struct gpiod_line_event gpiod_line_event;


// Variables and Functions
ros::Publisher publisher_4_motor_control;
ros::Publisher publisher_4_odometry_control;

Pololu_SMC_G2_Encoder encoder = Pololu_SMC_G2_Encoder(AMR_CW_DIRECTION_MOTOR_R);

int encoder_counter_4_motor_control = 0;
int encoder_counter_4_odometry_control = 0;
int encoder_trigger = 0;


// Functions
void subscriberCallbackForMotorControl(const amr_msgs::TimerID& msg);
void subscriberCallbackForOdometryControl(const amr_msgs::TimerID& msg);
void timerCallbackForEncoderMonitorPolling(const ros::TimerEvent&);

void reset_encoder_counter_4_motor_control();
void reset_encoder_counter_4_odometry_control();
void update_encoder_counter(int encoder_trigger);
void monitor_encoder_counter_trigger_falling_edge();
void monitor_encoder_counter_polling(ros::NodeHandle nd);


// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, amr_node::ENCODER_MONITOR_R);
	ros::NodeHandle nd;

	// publisher_4_motor_control 		= nd.advertise<std_msgs::Int16>(amr_topic::ENCODER_DATA_4_MOTOR_CONTROL_R, 10, false);
    // publisher_4_odometry_control 	= nd.advertise<std_msgs::Int16>(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL_R, 10, false);

	publisher_4_motor_control 		= nd.advertise<amr_msgs::EncCounter>(amr_topic::ENCODER_DATA_4_MOTOR_CONTROL_R, 10, false);
    publisher_4_odometry_control 	= nd.advertise<amr_msgs::EncCounter>(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL_R, 10, false);
    
	ros::Subscriber subscriber_4_motor_control 		= nd.subscribe(amr_topic::TIMER_4_MOTOR_CONTROL, 1, subscriberCallbackForMotorControl);
	ros::Subscriber subscriber_4_odometry_control 	= nd.subscribe(amr_topic::TIMER_4_SYSTEM_CONTROL, 1, subscriberCallbackForOdometryControl);


	// encoder.set_gpiod_chip(AMR_GPIO_CHIP_NAME, gpiod_chip);
	encoder.set_gpiod_chip();
	encoder.set_gpiod_line_channel(AMR_ENCODER_PIN_CHANNEL_A_MOTOR_R, AMR_ENCODER_PIN_CHANNEL_B_MOTOR_R);
	encoder.set_gpiod_line_request_events(Pololu_SMC_G2_Encoder::Line_Request_Event::FALLING_EDGE);
	// encoder.set_gpiod_line_request_events(Pololu_SMC_G2_Encoder::Line_Request_Event::BOTH_EDGES);

	monitor_encoder_counter_trigger_falling_edge();
	// monitor_encoder_counter_polling(nd);
	
	gpiod_chip_close(gpiod_chip); // Close the GPIO chip

	return 0;
}


// ==================================================
// CALLBACK Functions
// ==================================================
// Functionality: Subscribe to INNER LOOP timer call, in order to publish "angular_velocity" of 2 motors (rad)
/**
 * @brief Subscribe to INNER LOOP timer call, in order to publish "encoder counter" of motor (pulses).
 */
void subscriberCallbackForMotorControl(const amr_msgs::TimerID& msg)
{
	amr_msgs::EncCounter encoder_counter;
	encoder_counter.data = encoder_counter_4_motor_control;
	
	publisher_4_motor_control.publish(encoder_counter);
	reset_encoder_counter_4_motor_control();
}

/**
 * @brief Subscribe to OUTER LOOP timer call, in order to publish "encoder counter" of motor (pulses).
 */
void subscriberCallbackForOdometryControl(const amr_msgs::TimerID& msg)
{
	// ROS_INFO_STREAM("[ENCODER MONITOR R] Encoder counter in Odom (R): "<<encoder_counter_4_odometry_control);
	amr_msgs::EncCounter encoder_counter;
	encoder_counter.data = encoder_counter_4_odometry_control;

	publisher_4_odometry_control.publish(encoder_counter);
    reset_encoder_counter_4_odometry_control();
}


// ==================================================
// Other Functions
// ==================================================
/**
 * @brief Reset counter for motor control loop.
 */
void reset_encoder_counter_4_motor_control()
{
	encoder_counter_4_motor_control = 0;
}

/**
 * @brief Reset counter for system control loop.
 */
void reset_encoder_counter_4_odometry_control()
{
	encoder_counter_4_odometry_control = 0;
}

/**
 * @brief Update encoder counter after detecting encoder trigger.
 * @param encoder_trigger Encoder trigger direction (+1 forward / -1 backward)
 */
void update_encoder_counter(int encoder_trigger)
{
	encoder_counter_4_motor_control 	+= encoder_trigger;
	encoder_counter_4_odometry_control 	+= encoder_trigger;
}

/**
 * @brief Monitor encoder to detect trigger on falling edge and update the encoder counter.
 */
void monitor_encoder_counter_trigger_falling_edge()
{
	ros::AsyncSpinner spinner(0);
    spinner.start();

	while (ros::ok())
	{
		encoder_trigger = encoder.monitor_encoder_trigger_falling_edge_wo_debounce(gpiod_line_event);
		update_encoder_counter(encoder_trigger);
		ros::spinOnce();
		

	}
	ros::waitForShutdown();
}


void monitor_encoder_counter_polling(ros::NodeHandle nd)
{
    ros::AsyncSpinner spinner(0);
    spinner.start();
	ros::Timer timer_4_encoder_monitor = nd.createTimer(ros::Duration(AMR_TIMER_4_ENC_MONITOR), timerCallbackForEncoderMonitorPolling, true);

	ros::waitForShutdown();
}

void timerCallbackForEncoderMonitorPolling(const ros::TimerEvent&)
{
	encoder_trigger = encoder.monitor_encoder_polling_w_sw_debounce();
	update_encoder_counter(encoder_trigger);
}


