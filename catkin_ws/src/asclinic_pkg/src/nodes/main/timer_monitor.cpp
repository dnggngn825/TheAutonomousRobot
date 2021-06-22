/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


/* ==================================================
FUNCTIONALITY: This function generates 2 timers of inner loop and outer loop. The frequencies/timers are set in CONSTANTS section.
INPUT: None
OUTPUT: Timer of inner & outer loop
================================================== */


// ==================================================
// Constants/Variables/Functions 
// ==================================================
// Include some useful libraries
#include <math.h>
#include <stdlib.h>
#include "amr/amr.h"
#include "ros/ros.h"
using namespace asclinic_pkg;


// CONSTANTS
const int MESSAGE_ID_UPPER_LIMIT = 1E6;

// Initialize variables for node handling
ros::Timer timer_4_system_control;
ros::Timer timer_4_motor_control;
ros::Timer timer_4_camera_control;

ros::Publisher publisher_4_system_control_timer;
ros::Publisher publisher_4_motor_control_timer;
ros::Publisher publisher_4_camera_control_timer;

void timerCallbackForSystemControl(const ros::TimerEvent&);
void timerCallbackForMotorControl(const ros::TimerEvent&);
void timerCallbackForCameraControl(const ros::TimerEvent&);


// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
    ros::init(argc, argv, amr_node::TIMER_MONITOR);
    ros::NodeHandle nd;

    publisher_4_system_control_timer    = nd.advertise<amr_msgs::TimerID>(amr_topic::TIMER_4_SYSTEM_CONTROL, 10, false);
    publisher_4_motor_control_timer     = nd.advertise<amr_msgs::TimerID>(amr_topic::TIMER_4_MOTOR_CONTROL, 10, false);
    publisher_4_camera_control_timer    = nd.advertise<amr_msgs::TimerID>(amr_topic::TIMER_4_CAMERA_CONTROL, 10, false);

    ros::AsyncSpinner spinner(0);
    spinner.start();

    timer_4_system_control  = nd.createTimer(ros::Duration(AMR_TIMER_4_SYSTEM_CONTROL), timerCallbackForSystemControl, true);
    timer_4_motor_control   = nd.createTimer(ros::Duration(AMR_TIMER_4_MOTOR_CONTROL), timerCallbackForMotorControl, true);
    timer_4_camera_control  = nd.createTimer(ros::Duration(AMR_TIMER_4_CAMERA_CONTROL), timerCallbackForCameraControl, true);
    
    ros::waitForShutdown();
    // ros::spin();
}



// ==================================================
// CALLBACK Functions
// ==================================================
/**
 * @brief Functionality as a timer for the OUTER loop control
 * It is called after the timer & publishes a message as a timer callback.
 */
void timerCallbackForSystemControl(const ros::TimerEvent&)
{
    static int message_id_system_control = 0x00;
    message_id_system_control++;

    if (message_id_system_control>MESSAGE_ID_UPPER_LIMIT)
        message_id_system_control = 0x00;
    
    amr_msgs::TimerID msg;
    msg.data = message_id_system_control;
    publisher_4_system_control_timer.publish(msg);

	// START THE TIMER AGAIN
	timer_4_system_control.stop(); // > Stop any previous instance that might still be running
	timer_4_system_control.setPeriod(ros::Duration(AMR_TIMER_4_SYSTEM_CONTROL), true); // > Set the period again (second argument is reset)
	timer_4_system_control.start(); // > Start the timer again
}

/**
 * @brief Functionality as a timer for the INNER loop control. 
 * It is called after the timer & publishes a message as a timer callback.
 */
void timerCallbackForMotorControl(const ros::TimerEvent&)
{
    static int message_id_motor_control = 0x00;
    message_id_motor_control++;

    if (message_id_motor_control>MESSAGE_ID_UPPER_LIMIT)
        message_id_motor_control = 0x00;

    // Publish a message
    amr_msgs::TimerID msg;
    msg.data = message_id_motor_control;
    publisher_4_motor_control_timer.publish(msg);

	// START THE TIMER AGAIN
	timer_4_motor_control.stop(); // > Stop any previous instance that might still be running
	timer_4_motor_control.setPeriod(ros::Duration(AMR_TIMER_4_MOTOR_CONTROL), true); // > Set the period again (second argument is reset)
	timer_4_motor_control.start(); // > Start the timer again
}

/**
 * @brief Functionality as a timer for the INNER loop control. 
 * It is called after the timer & publishes a message as a timer callback.
 */
void timerCallbackForCameraControl(const ros::TimerEvent&)
{
    static int message_id_camera_control = 0x00;
    message_id_camera_control++;

    if (message_id_camera_control>MESSAGE_ID_UPPER_LIMIT)
        message_id_camera_control = 0x00;

    // Publish a message
    amr_msgs::TimerID msg;
    msg.data = message_id_camera_control;
    publisher_4_camera_control_timer.publish(msg);

	// START THE TIMER AGAIN
	timer_4_camera_control.stop(); // > Stop any previous instance that might still be running
	timer_4_camera_control.setPeriod(ros::Duration(AMR_TIMER_4_CAMERA_CONTROL), true); // > Set the period again (second argument is reset)
	timer_4_camera_control.start(); // > Start the timer again
}


