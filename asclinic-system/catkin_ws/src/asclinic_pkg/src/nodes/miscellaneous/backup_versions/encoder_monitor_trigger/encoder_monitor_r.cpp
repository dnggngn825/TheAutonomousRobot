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
using namespace asclinic_pkg;


// ==================================================
// Constants/Variables/Functions 
// ==================================================
// Control variables
int encoder_trigger_mode 	= Pololu_SMC_G2_Encoder::Line_Request_Event::FALLING_EDGE; // default
int encoder_monitor_mode 	= Pololu_SMC_G2_Encoder::Monitor_Mode::TRIGGER; // default


// Variables and Functions
struct gpiod_chip *gpiod_chip = gpiod_chip_open(AMR_GPIO_CHIP_NAME); // Open the GPIO chip
struct gpiod_line_event gpiod_line_event;

ros::Publisher publisher_4_motor_control;
ros::Publisher publisher_4_odometry_control;

Pololu_SMC_G2_Encoder encoder = Pololu_SMC_G2_Encoder(POLOLU_SMC_G2_ENCODER_DIR_CHANNEL_A2B_MOTOR_R);

int encoder_counter_4_motor_control 	= 0;
int encoder_counter_4_odometry_control 	= 0;
int encoder_trigger = 0;


// Functions
void subscriberCallbackForMotorControl(const amr_msgs::TimerID& msg);
void subscriberCallbackForOdometryControl(const amr_msgs::TimerID& msg);

void reset_encoder_counter_4_motor_control();
void reset_encoder_counter_4_odometry_control();
void update_encoder_counter(int encoder_trigger);
void monitor_encoder_counter_trigger();
void monitor_encoder_counter_polling();


// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, amr_node::ENCODER_MONITOR_R);
	ros::NodeHandle nd;

	publisher_4_motor_control 		= nd.advertise<amr_msgs::EncCounter>(amr_topic::ENCODER_DATA_4_MOTOR_CONTROL_R, 10, false);
    publisher_4_odometry_control 	= nd.advertise<amr_msgs::EncCounter>(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL_R, 10, false);
    
	ros::Subscriber subscriber_4_motor_control 		= nd.subscribe(amr_topic::TIMER_4_MOTOR_CONTROL, 1, subscriberCallbackForMotorControl);
	ros::Subscriber subscriber_4_odometry_control 	= nd.subscribe(amr_topic::TIMER_4_SYSTEM_CONTROL, 1, subscriberCallbackForOdometryControl);

	nd.getParam("encoder_trigger_mode", encoder_trigger_mode);
	nd.getParam("encoder_monitor_mode", encoder_monitor_mode);


	// ENC setup
	encoder.set_gpiod_chip();
	encoder.set_gpiod_line_channel(AMR_ENCODER_PIN_CHANNEL_A_MOTOR_R, AMR_ENCODER_PIN_CHANNEL_B_MOTOR_R);
	if (encoder_trigger_mode == 1)
		encoder.set_gpiod_line_request_events(Pololu_SMC_G2_Encoder::Line_Request_Event::FALLING_EDGE);
	else if (encoder_trigger_mode == 2)
		encoder.set_gpiod_line_request_events(Pololu_SMC_G2_Encoder::Line_Request_Event::BOTH_EDGES);


	// Loop
    ros::AsyncSpinner spinner(0);
    spinner.start();
	while (ros::ok())
	{
		if (encoder_monitor_mode == Pololu_SMC_G2_Encoder::Monitor_Mode::TRIGGER)
			// encoder_trigger = encoder.monitor_encoder_trigger_falling_edge_wo_debounce(gpiod_line_event);
			encoder_trigger = encoder.monitor_encoder_trigger_both_edges_wo_debounce(gpiod_line_event);
		else
			encoder_trigger = encoder.monitor_encoder_polling_w_sw_debounce();
		
		update_encoder_counter(encoder_trigger);
		ros::spinOnce();
	}
	ros::waitForShutdown();
	
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
	encoder_counter.data = encoder_counter_4_motor_control * AMR_DIRECTION_CW_MOTOR_R;
	
	publisher_4_motor_control.publish(encoder_counter);
	reset_encoder_counter_4_motor_control();
}

/**
 * @brief Subscribe to OUTER LOOP timer call, in order to publish "encoder counter" of motor (pulses).
 */
void subscriberCallbackForOdometryControl(const amr_msgs::TimerID& msg)
{
	amr_msgs::EncCounter encoder_counter;
	encoder_counter.data = encoder_counter_4_odometry_control * AMR_DIRECTION_CW_MOTOR_R;

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


void monitor_encoder_counter_trigger()
{
	ros::AsyncSpinner spinner(0);
    spinner.start();

	while (ros::ok())
	{
		// encoder_trigger = encoder.monitor_encoder_trigger_falling_edge_wo_debounce(gpiod_line_event);
		encoder_trigger = encoder.monitor_encoder_trigger_both_edges_wo_debounce(gpiod_line_event);
		update_encoder_counter(encoder_trigger);
		ros::spinOnce(); 
	}
	ros::waitForShutdown();
}

void monitor_encoder_counter_polling()
{
    ros::AsyncSpinner spinner(0);
    spinner.start();
	while (ros::ok())
	{
		encoder_trigger = encoder.monitor_encoder_polling_w_sw_debounce();
		update_encoder_counter(encoder_trigger);
		ros::spinOnce();
	}

	ros::waitForShutdown();
}

