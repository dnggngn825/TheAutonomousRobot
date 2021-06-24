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
#include "ros/ros.h"
#include <ros/package.h>
#include <gpiod.h>
#include "amr/amr.h"
using namespace asclinic_pkg;


// ==================================================
// Constants/Variables/Functions 
// ==================================================
// Variables and Functions
struct gpiod_chip *gpiod_chip = gpiod_chip_open(AMR_GPIO_CHIP_NAME); // Open the GPIO chip
struct gpiod_line_event gpiod_line_event;
Pololu_SMC_G2_Encoder encoder = Pololu_SMC_G2_Encoder(POLOLU_SMC_G2_ENCODER_DIR_CHANNEL_A2B_MOTOR_L);

ros::Publisher publisher_4_motor_control;
ros::Publisher publisher_4_odometry_control;


int encoder_counter_4_motor_control 	= 0;
int encoder_counter_4_odometry_control 	= 0;
int encoder_trigger = 0;

int state_channel_a = 0;
int state_channel_b = 0;


// Functions
void subscriberCallbackForMotorControl(const amr_msgs::TimerID& msg);
void subscriberCallbackForOdometryControl(const amr_msgs::TimerID& msg);

void subscriberCallback4ChannelATriggerEventtype(const amr_msgs::EncLineState& msg);
void subscriberCallback4ChannelBTriggerEventtype(const amr_msgs::EncLineState& msg);

void reset_encoder_counter_4_motor_control();
void reset_encoder_counter_4_odometry_control();
void update_encoder_counter(int encoder_trigger);



// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, amr_node::ENC_MONITOR_L_3200);
	ros::NodeHandle nd;

	publisher_4_motor_control 		= nd.advertise<amr_msgs::EncCounter>(amr_topic::ENCODER_DATA_4_MOTOR_CONTROL_L, 10, false);
    publisher_4_odometry_control 	= nd.advertise<amr_msgs::EncCounter>(amr_topic::ENCODER_DATA_4_ODOMETRY_CONTROL_L, 10, false);
    
	ros::Subscriber subscriber_4_motor_control 		= nd.subscribe(amr_topic::TIMER_4_MOTOR_CONTROL, 1, subscriberCallbackForMotorControl);
	ros::Subscriber subscriber_4_odometry_control 	= nd.subscribe(amr_topic::TIMER_4_SYSTEM_CONTROL, 1, subscriberCallbackForOdometryControl);


	ros::Subscriber subsciber_channel_a_trigger_eventtype = nd.subscribe(amr_topic::ENC_L_CHANNEL_A_STATE, 1, subscriberCallback4ChannelATriggerEventtype); 
	ros::Subscriber subsciber_channel_b_trigger_eventtype = nd.subscribe(amr_topic::ENC_L_CHANNEL_B_STATE, 1, subscriberCallback4ChannelBTriggerEventtype); 

	// ENC setup
	encoder.set_gpiod_chip();
	encoder.set_gpiod_line_channel(AMR_ENCODER_PIN_CHANNEL_A_MOTOR_L, AMR_ENCODER_PIN_CHANNEL_B_MOTOR_L);


    ros::AsyncSpinner spinner(0);
    spinner.start();

	ros::spin();
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
	encoder_counter.data = encoder_counter_4_motor_control * AMR_DIRECTION_CW_MOTOR_L;
	
	publisher_4_motor_control.publish(encoder_counter);
	reset_encoder_counter_4_motor_control();
}

/**
 * @brief Subscribe to OUTER LOOP timer call, in order to publish "encoder counter" of motor (pulses).
 */
void subscriberCallbackForOdometryControl(const amr_msgs::TimerID& msg)
{
	amr_msgs::EncCounter encoder_counter;
	encoder_counter.data = encoder_counter_4_odometry_control * AMR_DIRECTION_CW_MOTOR_L;

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


void subscriberCallback4ChannelATriggerEventtype(const amr_msgs::EncLineState& msg)
{
	static bool init_state_channel_a = false;

	if (!init_state_channel_a)
	{
		state_channel_a = msg.data;
		init_state_channel_a = true;
	}
	else
	{
		int event_type = msg.data;
		// Identify rotation direction
		if (((event_type == GPIOD_LINE_EVENT_RISING_EDGE) && (state_channel_b == Pololu_SMC_G2_Encoder::Line_State::LOW)) ||
			((event_type == GPIOD_LINE_EVENT_FALLING_EDGE) && (state_channel_b == Pololu_SMC_G2_Encoder::Line_State::HIGH)))  // CW
			encoder_trigger = encoder.dir_channel_a2b;
		else if (((event_type == GPIOD_LINE_EVENT_FALLING_EDGE) && (state_channel_b == Pololu_SMC_G2_Encoder::Line_State::LOW)) ||
				((event_type == GPIOD_LINE_EVENT_RISING_EDGE) && (state_channel_b == Pololu_SMC_G2_Encoder::Line_State::HIGH)))  // CCW
			encoder_trigger = encoder.dir_channel_b2a;

		update_encoder_counter(encoder_trigger);
		state_channel_a = 2-msg.data;
	}
}


void subscriberCallback4ChannelBTriggerEventtype(const amr_msgs::EncLineState& msg)
{
	static bool init_state_channel_b = false;

	if (!init_state_channel_b)
	{
		state_channel_b = msg.data;
		init_state_channel_b = true;
	}
	else
	{
		int event_type = msg.data;
		// Identify rotation direction
		if (((event_type == GPIOD_LINE_EVENT_RISING_EDGE) && (state_channel_a == Pololu_SMC_G2_Encoder::Line_State::HIGH)) ||
			((event_type == GPIOD_LINE_EVENT_FALLING_EDGE) && (state_channel_a == Pololu_SMC_G2_Encoder::Line_State::LOW)))  // CW
			encoder_trigger = encoder.get_dir_channel_a2b();
		else if (((event_type == GPIOD_LINE_EVENT_FALLING_EDGE) && (state_channel_a == Pololu_SMC_G2_Encoder::Line_State::HIGH)) ||
				((event_type == GPIOD_LINE_EVENT_RISING_EDGE) && (state_channel_a == Pololu_SMC_G2_Encoder::Line_State::LOW)))  // CCW
			encoder_trigger = encoder.get_dir_channel_b2a();

		update_encoder_counter(encoder_trigger);

		state_channel_b = 2-msg.data;
	}
}


