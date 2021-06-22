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
#include "std_msgs/Int32.h"
using namespace asclinic_pkg;


bool is_load_unload_state_activated     = false;
bool is_prox_ss_config                  = false;
const int SLEEP_DURAION = 5;

int rb_state = amr_state::FSM::DEFAULT;
amr_msgs::GeneralMsg state_completed;
amr_msgs::GeneralMsg prox_ss_signal_request;

ros::Publisher publisher_4_state_change;
ros::Publisher publisher_4_servo_ctl_signal;
ros::Publisher publisher_4_request_prox_ss_signal;


void subscriberCallback4ProxSsSignal(const amr_msgs::ProxSensorState& msg);
void subscriberCallback4LoadUnloadStateSignal(const amr_msgs::RobotState& msg);

void publish_servo_ctl_signal(int pulse);
void reset_pca9685();
void publish_state_completed(const amr_msgs::ProxSensorState& object_detection);
void publish_state_completed();


int main(int argc, char* argv[])
{
	ros::init(argc, argv, amr_node::SERVO_CONTROLLER);
	ros::NodeHandle nd;

    publisher_4_state_change            = nd.advertise<amr_msgs::GeneralMsg>(amr_topic::STATE_COMPLETED_SIGNAL, 1, false);
    publisher_4_servo_ctl_signal        = nd.advertise<ServoPulseWidth>(amr_topic::SERVO_CONTROL_SIGNAL, 1, false);
    publisher_4_request_prox_ss_signal  = nd.advertise<amr_msgs::GeneralMsg>(amr_topic::REQUEST_PROX_SS_SIGNAL, 1, false);

	ros::Subscriber subscriber_4_prox_ss_signal     = nd.subscribe(amr_topic::PROX_SENSOR_SIGNAL, 1, subscriberCallback4ProxSsSignal);
	ros::Subscriber subscriber_4_load_unload_state  = nd.subscribe(amr_topic::LOAD_UNLOAD_STATE_SIGNAL, 1, subscriberCallback4LoadUnloadStateSignal);

    reset_pca9685();

    int timer_counter = 0;
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        if ((is_load_unload_state_activated) && (!is_prox_ss_config))
        {
            timer_counter ++;

            if (timer_counter == SLEEP_DURAION)
            {
                publish_state_completed();
                timer_counter = 0;
                is_load_unload_state_activated = false;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


void subscriberCallback4LoadUnloadStateSignal(const amr_msgs::RobotState& msg)
{
    rb_state = msg.data;

    if (rb_state == amr_state::FSM::ST_ITEM_PICKUP)
    {
        ROS_INFO_STREAM("[SCREEN] Request for item ... ... ... ");
        publish_servo_ctl_signal(PCA9685_PULSE_POS_0_DEG);

        prox_ss_signal_request.data = amr_state::Sensor_State::OBJECT_PRESENT;
        publisher_4_request_prox_ss_signal.publish(prox_ss_signal_request);
    }
    else if (rb_state == amr_state::FSM::ST_ITEM_UNLOADING)
    {
        ROS_INFO_STREAM("[SCREEN] Unloading item");
        prox_ss_signal_request.data = amr_state::Sensor_State::OBJECT_MISSING;
        publisher_4_request_prox_ss_signal.publish(prox_ss_signal_request);

        publish_servo_ctl_signal(PCA9685_PULSE_POS_180_DEG);
    }
    else
        ROS_INFO_STREAM("[SCREEN] Wrong signal ........");


    if (~is_prox_ss_config)
        is_load_unload_state_activated = true;
}

void subscriberCallback4ProxSsSignal(const amr_msgs::ProxSensorState& object_detection)
{
    publish_state_completed(object_detection);
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

void publish_state_completed(const amr_msgs::ProxSensorState& object_detection)
{
    if ((rb_state == amr_state::FSM::ST_ITEM_PICKUP) && (object_detection.data == amr_state::Sensor_State::OBJECT_PRESENT))
    {
        publisher_4_state_change.publish(state_completed);
        rb_state = amr_state::FSM::DEFAULT;
    }
    else if ((rb_state == amr_state::FSM::ST_ITEM_UNLOADING) && (object_detection.data == amr_state::Sensor_State::OBJECT_MISSING))
    {
        ros::Duration(2).sleep();

        reset_pca9685();
        publisher_4_state_change.publish(state_completed);
        rb_state = amr_state::FSM::DEFAULT;
    }
}

void publish_state_completed()
{
    if (rb_state == amr_state::FSM::ST_ITEM_PICKUP)
    {
        publisher_4_state_change.publish(state_completed);
        rb_state = amr_state::FSM::DEFAULT;
    }
    else if (rb_state == amr_state::FSM::ST_ITEM_UNLOADING)
    {
        reset_pca9685();
        publisher_4_state_change.publish(state_completed);
        rb_state = amr_state::FSM::DEFAULT;
    }
}

