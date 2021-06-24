/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */

// FUNCTIONALITY: This function generates 2 timers of inner loop and outer loop. The frequencies/timers are set in CONSTANTS section.
// INPUT: None
// OUTPUT: Timer of inner & outer loop



// Include some useful libraries
#include <math.h>
#include <stdlib.h>
#include "amr/amr.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"

// Namespacing the package
// using namespace asclinic_pkg;


// CONSTANTS
const int MESSAGE_ID_UPPER_LIMIT = 1E6;

// Initialize variables for node handling
ros::Timer timer_4_system_control;
ros::Timer timer_4_motor_control;
ros::Publisher publisher_4_system_control_timer;
ros::Publisher publisher_4_motor_control_timer;
ros::Subscriber subscriber_test;


void timerCallbackForSystemControl(const ros::TimerEvent&);
void timerCallbackForMotorControl(const ros::TimerEvent&);
void selfSubscriberCallback(const std_msgs::UInt32& msg); // for testing


// Functions
void timerCallbackForSystemControl(const ros::TimerEvent&)
{
    static int message_id_system_control = 0x00;
    message_id_system_control++;

    if (message_id_system_control>MESSAGE_ID_UPPER_LIMIT)
        message_id_system_control = 0x00;
    
    // Publish a message
    std_msgs::UInt32 msg;
    msg.data = message_id_system_control;
    publisher_4_system_control_timer.publish(msg);

	// START THE TIMER AGAIN
	// > Stop any previous instance that might still be running
	timer_4_system_control.stop();
	// > Set the period again (second argument is reset)
	timer_4_system_control.setPeriod( ros::Duration(AMR_TIMER_4_SYSTEM_CONTROL), true);
	// > Start the timer again
	timer_4_system_control.start();
}

void timerCallbackForMotorControl(const ros::TimerEvent&)
{
    static int message_id_motor_control = 0x00;
    message_id_motor_control++;

    if (message_id_motor_control>MESSAGE_ID_UPPER_LIMIT)
        message_id_motor_control = 0x00;

    // Publish a message
    std_msgs::UInt32 msg;
    msg.data = message_id_motor_control;
    publisher_4_motor_control_timer.publish(msg);

	// START THE TIMER AGAIN
	// > Stop any previous instance that might still be running
	timer_4_motor_control.stop();
	// > Set the period again (second argument is reset)
	timer_4_motor_control.setPeriod( ros::Duration(AMR_TIMER_4_MOTOR_CONTROL), true);
	// > Start the timer again
	timer_4_motor_control.start();
}

void selfSubscriberCallback(const std_msgs::UInt32& msg)
{
    ROS_INFO_STREAM("[TIMER MONITOR] Message receieved with data: " << msg.data);
}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "timer_monitor");
    ros::NodeHandle nodeHandle;

    timer_4_system_control = nodeHandle.createTimer(ros::Duration(AMR_TIMER_4_SYSTEM_CONTROL), timerCallbackForSystemControl, true);
    timer_4_motor_control = nodeHandle.createTimer(ros::Duration(AMR_TIMER_4_MOTOR_CONTROL), timerCallbackForMotorControl, true);

    publisher_4_system_control_timer = nodeHandle.advertise<std_msgs::UInt32>("timer_4_system_control", 10, false);
    publisher_4_motor_control_timer = nodeHandle.advertise<std_msgs::UInt32>("timer_4_motor_control", 10, false);
    
    subscriber_test = nodeHandle.subscribe("timer_4_system_control", 1, selfSubscriberCallback);
    subscriber_test = nodeHandle.subscribe("timer_4_motor_control", 1, selfSubscriberCallback);
    
    ros::spin();
}
