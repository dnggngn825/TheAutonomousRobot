// Copyright (C) 2021, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
//
// This file is part of ASClinic-System.
//    
// See the root of the repository for license details.
//
// ----------------------------------------------------------------------------
//     _    ____   ____ _ _       _          ____            _                 
//    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________  
//   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \ 
//  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
// /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
//                                                 |___/                       
//
// DESCRIPTION:
// Node for running a proximity sensor through GPIO Polling pin. 
// Sensor has been digitally debounced
// ----------------------------------------------------------------------------


#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Int32.h"
#include "amr/amr.h"

#include <gpiod.h>

#define PROX_SENSOR_POLL_RATE 250 // Polling rate of Proximity Sensor
#define DEBOUNCE_THRESHOLD    20  // Count threshold at which sensor debounces
#define DEBOUNCE_MARGIN       5   // Margin at which counter keeps previous state
#define MAX_DEBOUNCE_COUNTS   30  // Maximum value for debouncer counter
#define OUTPUT_TIMER          25  // Timer to output sensor state 

std::int32_t sensor_state;
bool is_requested = false;

// Respond to subscriber receiving a message
void subscribeCallback4GettingRequest(const amr_msgs::GeneralMsg& msg)
{
	ROS_INFO_STREAM("[TEMPLATE GPIO POLLING] Message receieved with data = " << msg.data);
	is_requested = true;
}

int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "proximity_sensor");
	ros::NodeHandle nodeHandle;

	// Initialise a publisher
	ros::Publisher gpio_event_publisher = nodeHandle.advertise<amr_msgs::ProxSensorState>(amr_topic::PROX_SENSOR_SIGNAL, 1, false);

	// Initialise a subscriber
	// > Note that the subscriber is included only for the purpose of demonstrating this template node running stand-alone
	ros::Subscriber gpio_event_subscriber = nodeHandle.subscribe(amr_topic::REQUEST_PROX_SS_SIGNAL, 1, subscribeCallback4GettingRequest);


	// Initialise a variable with loop rate for polling the GPIO pin
	// > Input argument is the frequency in hertz, as a double
	// ros::Rate loop_rate(PROX_SENSOR_POLL_RATE);
	ros::Rate loop_rate(50);

	// Specify the chip name of the GPIO interface
	// > Note: for the 40-pin header of the Jetson SBCs, this is "/dev/gpiochip0"
	const char * gpio_chip_name = "/dev/gpiochip0";

	// Get the GPIO line number to monitor
	// Notes:
	// > If you look at the "template_gpio.launch" file located in
	//   the "launch" folder, you see the following lines of code:
	//       <param
	//           name   = "line_number"
	//           value  = 148
	//       />
	// > These lines of code add a parameter named "line_number"
	//   to the this node.
	// > Thus, to access this "line_number" parameter, we first
	//   get a handle to this node within the namespace that it
	//   was launched.
	//std::string namespace = ros::this_node::getNamespace();
	int line_number = 0;
	if ( !nodeHandle.getParam("line_number", line_number) )
	{
		ROS_INFO("[PROXIMITY_SENSOR_DB] FAILED to get \"line_number\" parameter. Using default value instead.");
		// Set the line number to a default value
		line_number = 148;
	}
	// > Display the line number being monitored
	ROS_INFO_STREAM("[PROXIMITY_SENSOR_DB] Will monitor \"line_number\" = " << line_number);

	// Initialise a GPIO chip, line, and event object
	struct gpiod_chip *chip;
	struct gpiod_line *line;

	// Get and print the value of the GPIO line
	// > Note: the third argument to "gpiod_ctxless_get_value"
	//   is an "active_low" boolean input argument.
	//   If true, this indicate to the function that active state
	//   of this line is low.
	int value;
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number, false, "foobar");
	ROS_INFO_STREAM("[PROXIMITY_SENSOR_DB] On startup of node, chip " << gpio_chip_name << " line " << line_number << " returned value = " << value);

	// Open the GPIO chip
	chip = gpiod_chip_open(gpio_chip_name);
	// Retrieve the GPIO line
	line = gpiod_chip_get_line(chip,line_number);
	// Display the status
	ROS_INFO_STREAM("[PROXIMITY_SENSOR_DB] Chip " << gpio_chip_name << " opened and line " << line_number << " retrieved.");


	// Initialize a debounce counter 
	int debounce_count = 0; 
	int previous_state = 0; 
	sensor_state = 0;
	

	// Enter a loop that continues while ROS is still running
	while (ros::ok())
	{

		// Get the current value of the line
		// > Note: the third argument to "gpiod_ctxless_get_value"
		//   is an "active_low" boolean input argument.
		//   If true, this indicate to the function that active state
		//   of this line is low.
		int current_value;
		current_value = gpiod_ctxless_get_value(gpio_chip_name, line_number, false, "foobar");

		// Respond only if the value is 0 or 1
		
		if (is_requested && (current_value==0 || current_value==1))
		{
			// Display the value
			//ROS_INFO_STREAM("[TEMPLATE GPIO POLLING] gpiod_ctxless_get_value returned \"current_value\" = " << current_value);
			
			// If statement: if current_value is 1 (active-low), no object detected
			if (current_value == 1) {
				// If counter is 0 (or even lower in case of error)...
				if (debounce_count <= 0) {
					// Keep it at 0
					debounce_count = 0; 
				} else { 
					// Else, decrease counter 
					debounce_count--; 								
				}
			// If current_value is 0: an object is detected... 	
			} else if (current_value == 0) {
				// If counter already at maximum, do not increment
				if (debounce_count >= MAX_DEBOUNCE_COUNTS) {
					debounce_count = MAX_DEBOUNCE_COUNTS; 
				// Else, increment the counter	
				} else {
					debounce_count++;
				}
			}

			// Set status of sensor
			// If higher than threshold, sensor debounced: detected object
			if (debounce_count >= (DEBOUNCE_THRESHOLD+DEBOUNCE_MARGIN)) {
				
				// Output logic for previous_state and current state
				if (previous_state == 0) { 
					previous_state = 0; 
					sensor_state = 1;
				} else if (previous_state == 1) {
					previous_state = 1;
					sensor_state = 1;
				}
				
			} else if (debounce_count < DEBOUNCE_THRESHOLD) {

				// Else, sensor debounced: no object 
				if (previous_state == 0) { 
					previous_state = 0; 
					sensor_state = 0;
				} else if (previous_state == 1) {
					previous_state = 1;
					sensor_state = 0;
				} 
				
			} else {
				// Else, current state is same as previous state 
				sensor_state = previous_state; 
			}

				// Publish a message
				amr_msgs::ProxSensorState sens_state;
				// Message is the sensor state (1 for something detected, 0 if nothing detected)
				sens_state.data = sensor_state;
				ROS_INFO_STREAM("[PROX. SS] OUTPUT:  "<< sensor_state<< std::endl);
				
				gpio_event_publisher.publish(sens_state);
		}
		// else
		// {
		// 	// Display the status
		// 	// // ROS_INFO_STREAM("[TEMPLATE GPIO POLLING] gpiod_ctxless_get_value returned unexpected value, \"current_value\" = " << current_value);
		// 	ROS_INFO_STREAM("[PROX REQUESTED] YES OR NAH:??? "<< is_requested << std::endl );
			
		// }

		// Spin once so that this node can service any
		// callbacks that this node has queued.
		ros::spinOnce();

		// Sleep for the specified loop rate
		loop_rate.sleep();
	} // END OF: "while (ros::ok())"

	// Close the GPIO chip	
	gpiod_chip_close(chip);

	return 0;
}
