/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Hai Dang Nguyen (860308)
================================================== */

/**
 * @brief The node subscribes to get traj ref signal from motion planning
 * control path following and purely rotation and publish the desired robot movement/command
 * to the internal controller of the robot.
 * 
 */

#include "ros/ros.h"
#include <math.h>
#include <stdlib.h>
#include "amr/amr.h"
#include "amr/amr_constants.h"

#include "asclinic_pkg/Polygon.h"
#include "asclinic_pkg/Point32.h"
#include "asclinic_pkg/AmrPose.h"
#include "asclinic_pkg/MotorAngularVelocity.h"
#include "MotionPlanHeader.h"
#include "std_msgs/Bool.h"
#include "asclinic_pkg/ServoPulseWidth.h"
#include "std_msgs/Int32.h"

using namespace asclinic_pkg;

subscriberCallback4ProximitySS(const std::Int32& msg)
{
    bool is_object = msg.data;
    if (is_object)
    {
        
        publisher_4_servo_motor.publish(servoMsg);
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, amr_node::TRAJ_TRACK_CONTROLLER);
    ros::NodeHandle nodeHandle;

    ros::Subscriber subscribe_proximity_ss = nodeHandle.subscribe(amr_topic::PROXIMITY_DATA, 1, subscriberCallback4ProximitySS);
	// ros::Subscriber subscribe_traj_ref_signal = nodeHandle.subscribe(amr_topic::TRAJECTORY_REF_SIGNAL, 1, subscriberCallback4TrajRefSignal);
    // ros::Subscriber subscribe_4_movement_type = nodeHandle.subscribe(amr_topic::MOVEMENT_TYPE, 1, subscriberCallback4MovementType);
    // ros::Subscriber subscribe_4_dynamic_state = nodeHandle.subscribe("dynamic_state", 1, subscriberCallback4DynamicState);

    publisher_4_servo_motor = nodeHandle.advertise<ServoPulseWidth>(amr_topic::SERVO_MOTOR_CMD, 1, false);
}