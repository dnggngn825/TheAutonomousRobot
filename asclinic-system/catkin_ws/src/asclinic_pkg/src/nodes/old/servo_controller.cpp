/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */

#include "ros/ros.h"
#include <ros/package.h>
#include <bitset>
#include "amr/amr.h"
#include <math.h>
// #include "std_msgs/UInt32.h"
using namespace asclinic_pkg;



int robot_state = -1;
amr_msgs::GeneralMsg state_completed;
amr_msgs::GeneralMsg prox_ss_signal_request;

ros::Publisher publisher_4_state_change;
ros::Publisher publisher_4_servo_ctl_signal;
ros::Publisher publisher_4_request_prox_ss_signal;


void subscriberCallback4ProxSsSignal(const amr_msgs::ProxSensorState& msg);
void subscriberCallback4LoadUnloadStateSignal(const amr_msgs::RobotState& msg);

void publish_servo_ctl_signal(int pulse);
void reset_pca9685();

enum FSM_State: int
{
    DEFAULT = 0,
    LOAD    = 1,
    UNLOAD  = 2,
};

enum Sensor_State: int
{
    OBJECT_MISSING = 0,
    OBJECT_PRESENT = 1, 
};


int main(int argc, char* argv[])
{
	ros::init(argc, argv, amr_node::SERVO_CONTROLLER);
	ros::NodeHandle nd;

    publisher_4_state_change            = nd.advertise<amr_msgs::GeneralMsg>(amr_topic::STATE_COMPLETED_SIGNAL, 10, false);
    publisher_4_servo_ctl_signal        = nd.advertise<ServoPulseWidth>(amr_topic::SERVO_CONTROL_SIGNAL, 10, false);
    publisher_4_request_prox_ss_signal  = nd.advertise<amr_msgs::GeneralMsg>(amr_topic::REQUEST_PROX_SS_SIGNAL, 10, false);

	ros::Subscriber subscriber_4_prox_ss_signal = nd.subscribe(amr_topic::PROX_SENSOR_SIGNAL, 1, subscriberCallback4ProxSsSignal);
	ros::Subscriber subscriber_4_servo_ctl      = nd.subscribe(amr_topic::LOAD_UNLOAD_STATE_SIGNAL, 1, subscriberCallback4LoadUnloadStateSignal);

    reset_pca9685();

    ros::spin();

    return 0;
}


void subscriberCallback4LoadUnloadStateSignal(const amr_msgs::RobotState& msg)
{
    robot_state = msg.data; // load: 1, unload: 2

    if (robot_state == FSM_State::LOAD)
    {
        ROS_INFO_STREAM("[SCREEN] Request for item ... ");

        publisher_4_request_prox_ss_signal.publish(prox_ss_signal_request);
    }

    else if (robot_state == FSM_State::UNLOAD)
    {
        prox_ss_signal_request.data = Sensor_State::OBJECT_MISSING;
        publisher_4_request_prox_ss_signal.publish(prox_ss_signal_request);

        publish_servo_ctl_signal(PCA9685_PULSE_POS_180_DEG);
    }
}

void subscriberCallback4ProxSsSignal(const amr_msgs::ProxSensorState& object_detection)
{
    if ((robot_state == FSM_State::LOAD) && (object_detection.data == Sensor_State::OBJECT_PRESENT))
    {
        publisher_4_state_change.publish(state_completed);
        robot_state = FSM_State::DEFAULT;
    }
    else if ((robot_state == FSM_State::UNLOAD) && (object_detection.data == Sensor_State::OBJECT_MISSING))
    {
        ros::Duration(2).sleep();

        reset_pca9685();
        publisher_4_state_change.publish(state_completed);
        robot_state = FSM_State::DEFAULT;
    }
}

void publish_servo_ctl_signal(int pulse)
{
    ServoPulseWidth servo_msg;
    servo_msg.channel                       = AMR_PCA9685_CHANNEL;
    servo_msg.pulse_width_in_microseconds   = pulse;

    publisher_4_servo_ctl_signal.publish(servo_msg);
}

void reset_pca9685()
{
    publish_servo_ctl_signal(PCA9685_PULSE_POS_0_DEG);
}

