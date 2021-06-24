/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


// Include some useful libraries
#define _USE_MATH_DEFINES // for C
#include <math.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <gpiod.h>

#include "amr/amr.h"
#include "pololu_smc_g2_encoder/pololu_smc_g2_encoder.h"

#include "std_msgs/Int32.h"
#include "asclinic_pkg/MotorRotationAngle.h"

// Namespacing the package
using namespace asclinic_pkg;


// DEVICE INFO
const char * gpio_chip_name = "/dev/gpiochip0"; // Note: for the 40-pin header of the Jetson SBCs -> "/dev/gpiochip0"


// Identify motor direction coefficient. Edit after experiment
const int CW_MOTOR_L = -1;
const int CCW_MOTOR_L = CW_MOTOR_L*(-1);
const int CW_MOTOR_R = +1;
const int CCW_MOTOR_R = CW_MOTOR_R*(-1);

// Timer Constants
const int TIME_OUT_NS = 1e8;
const float TIMER_4_ODOMETRY_CONTROL_PUBLISHING = 1;



// Variables and Functions
ros::Publisher publisher_4_odometry_control;
ros::Timer timer_4_odometry_control_publishing;

int encoder_counter_4_odometry_control_motor_l = 0;
int encoder_counter_4_odometry_control_motor_r = 0;
bool encoder_direction_motor_l = false;
bool encoder_direction_motor_r = false;


void timerCallbackForOdometryControlPublishing(const ros::TimerEvent&);

bool are_encoder_pins_ready(ros::NodeHandle nodeHandle, int encoder_pins[]);
bool is_pin_ready(ros::NodeHandle nodeHandle, int pin_number);
bool reset_encoder_counter_4_odometry_control();
bool count_encoder_pulse(int returned_wait_flag, struct gpiod_line * line_wheel_channel_a, int pin_wheel_channel_b, struct gpiod_line_event event);
float calculate_rotation_angle(int encoder_counter, float timer_duration);


void timerCallbackForOdometryControlPublishing(const ros::TimerEvent&)
{
	float rotation_angle_motor_l = calculate_rotation_angle(encoder_counter_4_odometry_control_motor_l, TIMER_4_ODOMETRY_CONTROL_PUBLISHING);
	float rotation_angle_motor_r = calculate_rotation_angle(encoder_counter_4_odometry_control_motor_r, TIMER_4_ODOMETRY_CONTROL_PUBLISHING);

    ROS_INFO_STREAM("[ENCODER TEST] <Left/Right wheel> Encoder pulses: "<< encoder_counter_4_odometry_control_motor_l <<";"<< encoder_counter_4_odometry_control_motor_r);
    ROS_INFO_STREAM("[ENCODER TEST] <Left/Right wheel> Rotational Angles: "<< rotation_angle_motor_l <<";"<< rotation_angle_motor_r);

    reset_encoder_counter_4_odometry_control();

	// START THE TIMER AGAIN
	// > Stop any previous instance that might still be running
	timer_4_odometry_control_publishing.stop();
	// > Set the period again (second argument is reset)
	timer_4_odometry_control_publishing.setPeriod( ros::Duration(TIMER_4_ODOMETRY_CONTROL_PUBLISHING), true);
	// > Start the timer again
	timer_4_odometry_control_publishing.start();
}


int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "encoder_reader");
	ros::NodeHandle nodeHandle("~");


    // Publisher and timer for ODOMETRY CONTROL 
    timer_4_odometry_control_publishing = nodeHandle.createTimer(ros::Duration(TIMER_4_ODOMETRY_CONTROL_PUBLISHING), timerCallbackForOdometryControlPublishing, true);
    

	// ros::Subscriber subscriber_motor_control = nodeHandle.subscribe("encoder_data_motor_ctl_topic", 1, subscriberCallback);
	
    are_encoder_pins_ready(nodeHandle, ENCODER_PINS);


	// Initialise a GPIO chip, line, and event object
	struct gpiod_chip *chip;
	struct gpiod_line *line_motor_l_channel_a;
	struct gpiod_line *line_motor_r_channel_a;
	struct gpiod_line_event event;

	// Specify the timeout specifications
	struct timespec time_out = { 0, TIME_OUT_NS };
	
	// Intialise a variable for the flags returned by GPIO calls
	int returned_wait_flag_motor_l, returned_wait_flag_motor_r;

	// // Get and print the value of the GPIO line on startup
	// int value = gpiod_ctxless_get_value(gpio_chip_name, PIN_CHANNEL_A_MOTOR_L, false, "foobar");
	// ROS_INFO_STREAM("[ENCODER READER] On startup of node, chip " << gpio_chip_name << " line " << PIN_CHANNEL_A_MOTOR_L << " returned value = " << value);

	
	chip = gpiod_chip_open(gpio_chip_name); // Open the GPIO chip
	line_motor_l_channel_a = gpiod_chip_get_line(chip, AMR_ENCODER_PIN_CHANNEL_A_MOTOR_L); // Retrieve the GPIO line
	line_motor_r_channel_a = gpiod_chip_get_line(chip, AMR_ENCODER_PIN_CHANNEL_A_MOTOR_R); // Retrieve the GPIO line

	// Display the status
	ROS_INFO_STREAM("[ENCODER READER] Chip " << gpio_chip_name << 
					" opened and line " << AMR_ENCODER_PIN_CHANNEL_A_MOTOR_L << "and" << 
					AMR_ENCODER_PIN_CHANNEL_A_MOTOR_R << " retrieved");

	/* Request the line events to be mointored
	> Note: only one of these should be uncommented
	gpiod_line_request_rising_edge_events(line, "foobar");
	gpiod_line_request_falling_edge_events(line, "foobar");
	gpiod_line_request_both_edges_events(line, "foobar"); */
	gpiod_line_request_falling_edge_events(line_motor_l_channel_a, "foobar"); 
	gpiod_line_request_falling_edge_events(line_motor_r_channel_a, "foobar"); 


	while (ros::ok())
	{
		// Monitor for the requested events on the GPIO line
		returned_wait_flag_motor_l = gpiod_line_event_wait(line_motor_l_channel_a,&time_out);
		returned_wait_flag_motor_r = gpiod_line_event_wait(line_motor_r_channel_a,&time_out);

		count_encoder_pulse(returned_wait_flag_motor_l, line_motor_l_channel_a, AMR_ENCODER_PIN_CHANNEL_B_MOTOR_L, event);
		count_encoder_pulse(returned_wait_flag_motor_r, line_motor_r_channel_a, AMR_ENCODER_PIN_CHANNEL_B_MOTOR_R, event);

		ros::spinOnce();
	} // END OF: "while (ros::ok())"

	// Close the GPIO chip
	gpiod_chip_close(chip);

	return 0;
}


bool are_encoder_pins_ready(ros::NodeHandle nodeHandle, int encoder_pins[])
{
	int encoder_pins_array_size = 4;

	for (int index = 0; index < encoder_pins_array_size; index++)
	{
		if (!is_pin_ready(nodeHandle, encoder_pins[index]))
		{
			ROS_INFO("[GPIO EVENT TRIG. ENC] All encoder pins are ready!");
			return false;
		}
			
	}

	ROS_INFO("[GPIO EVENT TRIG. ENC] NOT all encoder pins are ready!");
	return true;
}



bool is_pin_ready(ros::NodeHandle nodeHandle, int line_number)
{
	if ( !nodeHandle.getParam("line_number", line_number) )
	{
		ROS_INFO("[GPIO EVENT TRIG. ENC] FAILED to get \"line_number\" parameter. Using default value instead.");
		line_number = AMR_ENCODER_PIN_DEFAULT; // Set the line number to a default value
	}
	// > Display the line number being monitored
	ROS_INFO_STREAM("[GPIO EVENT TRIG. ENC] Will monitor \"line_number\" = " << line_number);

    return (line_number == AMR_ENCODER_PIN_DEFAULT);
}


bool reset_encoder_counter_4_odometry_control()
{
    encoder_counter_4_odometry_control_motor_l = 0;
	encoder_counter_4_odometry_control_motor_r = 0;

	return true;
}

// This function count number of encoder pulses. Input: channel A, B (of motor left/right). Output: update total number of pulses.
bool count_encoder_pulse(int returned_wait_flag, struct gpiod_line * line_wheel_channel_a, int pin_wheel_channel_b, struct gpiod_line_event event)
{
	switch (returned_wait_flag) // Respond based on the the return flag
	{
		case 1: // Event occurred:
		{
			int returned_read_flag = gpiod_line_event_read(line_wheel_channel_a,&event); // Read the pending event on the GPIO line

			switch (returned_read_flag)
			{
				case 0: // Event read correctly
				{
					int motor_direction_cw, motor_direction_ccw;
					int motor_direction = 0;

					int state_channel_b = gpiod_ctxless_get_value(gpio_chip_name, pin_wheel_channel_b, false, "foobar");

					// Motor Left/Right
					if (pin_wheel_channel_b == AMR_ENCODER_PIN_CHANNEL_B_MOTOR_L)
					{
						motor_direction_cw = CW_MOTOR_L;
						motor_direction_ccw = CCW_MOTOR_L;
					}
					else
					{
						motor_direction_cw = CW_MOTOR_R;
						motor_direction_ccw = CCW_MOTOR_R;
					}

					// Identify rotation direction
					if (state_channel_b == 0)
						motor_direction = motor_direction_cw;
					else 
						motor_direction = motor_direction_ccw;

					// Update encoder counters
					if (pin_wheel_channel_b == AMR_ENCODER_PIN_CHANNEL_B_MOTOR_L)
					{
						// encoder_counter_4_motor_control_motor_l += motor_direction;
						encoder_counter_4_odometry_control_motor_l += motor_direction;
					}
					else
					{
						// encoder_counter_4_motor_control_motor_r += motor_direction;
						encoder_counter_4_odometry_control_motor_r += motor_direction;
					}

					break;
				}

				case -1: // Error occurred
				{
					// Display the status
					ROS_INFO("[ENCODER READER] gpiod_line_event_wait returned the status that an error occurred");
					break;
				}

				default:
				{
					// Display the status
					ROS_INFO_STREAM("[ENCODER READER] gpiod_line_event_read returned an unrecognised status, return_flag =  " << returned_read_flag );
					break;
				}
			} // END OF: "switch (returned_read_flag)"
			break;
		}
		
		case 0: // Time out occurred
		{
			// ros::spinOnce();
			break;
		}

		case -1: // Error occurred
		{
			// Display the status
			ROS_INFO("[ENCODER READER] gpiod_line_event_wait returned the status that an error occurred");
			break;
		}

		default:
		{
			// Display the status
			ROS_INFO_STREAM("[ENCODER READER] gpiod_line_event_wait returned an unrecognised status, return_flag =  " << returned_wait_flag );
			break;
		}

		// ros::spinOnce();
	} // END OF: "switch (returned_wait_flag)"

	return true;
}

float calculate_rotation_angle(int encoder_counter, float timer_duration)
{
	return 2*M_PI*(encoder_counter/AMR_ENCODER_RESOLUTION)/timer_duration;
}
