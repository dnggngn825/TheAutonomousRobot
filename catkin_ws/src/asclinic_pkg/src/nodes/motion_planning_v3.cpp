/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Hai Dang Nguyen (860308)
================================================== */

/**
 * @brief The node subscribes to get planned path from matlab, make decision and 
 * publish the reference node to the trajectory tracking controller.
 * 
 */

#include "ros/ros.h"
#include "amr/amr.h"
#include <math.h>
#include <stdlib.h>

#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "asclinic_pkg/AmrPose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt32.h"
#include "MotionPlanHeader.h"

using namespace asclinic_pkg;

ros::Publisher publisher_4_trajectory_tracking;
ros::Publisher publisher_state_changePath;
ros::Publisher publisher_4_testing_Matlab;
ros::Publisher publisher_4_emergency_stop;
ros::Publisher publisher_4_movement_type;
ros::Publisher publisher_4_currentPose;
ros::Publisher publisher_4_dynamic_state;
ros::Publisher publisher_4_load_unload_state;
ros::Publisher publisher_4_next_cycle;
ros::Publisher publisher_4_pause_states;
ros::Publisher publisher_4_target_pose;

std::string stateSentRequest;
// amr_msgs::GeneralMsg state_change_request;

std_msgs::Bool is_pure_rotation;
int rb_history_state    = amr_state::FSM::ST_IDLE;
int rb_state            = amr_state::FSM::ST_IDLE;

bool is_load_unload_state_completed = false;
bool is_camera_triggered            = false;
bool is_compensated_pr_completed    = false;

const int loop_rate     = 1;
// const int PAUSE_INDEX   = AMR_TIMER_4_POSE_REF_SIGNAL*loop_rate;
const int REF_CYCLE     = AMR_TIMER_4_POSE_REF_SIGNAL*loop_rate;
const int PAUSE_CYCLE   = 2*loop_rate;
int available_cycle = 0;

pathStruct transitPath      = pathStruct();
pathStruct plannedPath      = pathStruct();
pathStruct testPath         = pathStruct();
pathStruct Back2StartPath   = pathStruct();



void subscriberCallback4TransitPath(const geometry_msgs::Polygon& msg);
void subscriberCallback4PlannedPath(const geometry_msgs::Polygon& msg);
void subscriberCallback4TestPath(const geometry_msgs::Polygon& msg);
void subscriberCallback4Back2StartPath(const geometry_msgs::Polygon& msg);
// void testingMatlab(const std_msgs::Float64& msg);
void getStateIndicator(const std_msgs::String& msg);
AmrPose toAmrPose(geometry_msgs::Point32 data);
void subscriberCallback4SwitchState(const amr_msgs::GeneralMsg& msg);
void subscriberCallback4notifyCycle(const std_msgs::UInt32& msg);
// void subscriberCallback4CameraReturn(const std_msgs::UInt32& msg);


int get_movement_type(int index, pathStruct planned_path, int pre_movement_type);



int main(int argc, char* argv[])
{
    ros::init(argc, argv, amr_node::MOTION_PLANNING);
    ros::NodeHandle nodeHandle;

	ros::Subscriber subscribe_transit_path  = nodeHandle.subscribe(amr_topic::TRANSIT_PATH_MATLAB, 1, subscriberCallback4TransitPath);
    ros::Subscriber subscribe_planned_path  = nodeHandle.subscribe(amr_topic::PLANNED_PATH_MATLAB, 1, subscriberCallback4PlannedPath);
    ros::Subscriber subscribe_test_path     = nodeHandle.subscribe("test_path", 1, subscriberCallback4TestPath);
    ros::Subscriber subscribe_back2start_path = nodeHandle.subscribe("back2start_path", 1, subscriberCallback4Back2StartPath);
    
    ros::Subscriber state_indicator         = nodeHandle.subscribe(amr_topic::STATE, 1, getStateIndicator);
    ros::Subscriber subscribe_4_switch_state= nodeHandle.subscribe(amr_topic::STATE_COMPLETED_SIGNAL, 1, subscriberCallback4SwitchState);
    ros::Subscriber subscribe_4_notify_cycle= nodeHandle.subscribe("notify_available_cycle",1, subscriberCallback4notifyCycle);
    // ros::Subscriber subscribe_4_camera_trigger = nodeHandle.subscribe(amr_topic::CAMERA_TRIGGER_RETURN, 1, subscriberCallback4CameraReturn);

    publisher_4_trajectory_tracking         = nodeHandle.advertise<AmrPose>(amr_topic::TRAJECTORY_REF_SIGNAL, 1, false);
    publisher_4_emergency_stop              = nodeHandle.advertise<MotorAngularVelocity>(amr_topic::MOTOR_ANGULAR_VELOCITY_REF, 1, false);
    publisher_4_movement_type               = nodeHandle.advertise<std_msgs::Bool>(amr_topic::MOVEMENT_TYPE, 1, false);
    publisher_4_currentPose                 = nodeHandle.advertise<AmrPose>(amr_topic::POSE_DATA_FROM_ODOMETRY, 1, true);
    publisher_4_dynamic_state               = nodeHandle.advertise<std_msgs::Bool>("dynamic_state", 1, true);
    publisher_4_target_pose                 = nodeHandle.advertise<AmrPose>(amr_topic::TRAJECTORY_TARGET_SIGNAL,1, true);
    
    
    publisher_4_load_unload_state           = nodeHandle.advertise<amr_msgs::RobotState>(amr_topic::LOAD_UNLOAD_STATE_SIGNAL, 1, false);
    publisher_4_next_cycle                  = nodeHandle.advertise<std_msgs::UInt32>("get_next_cycle", 1, true);
    ros::Publisher publisher_4_in_movement  = nodeHandle.advertise<std_msgs::Bool>("in_movement", 1, true);
    publisher_4_pause_states                = nodeHandle.advertise<std_msgs::UInt32>(amr_topic::CAMERA_TRIGGER, 1, true);
    
    // need to convert point 32 to AmrPose
    // freq = 2Hz, T = 0.5s
    
    
    std_msgs::Bool in_moving_msg;
    // ros::Rate rate(1/AMR_TIMER_4_POSE_REF_SIGNAL);
    ros::Rate rate(loop_rate);
    
    // index of the path list
    int index = 0;
    int counter_camera_trigger = 0;
    int counter_compensated_pr_completed = 0;
    int counter_ref_point_publish = 0;
    bool is_pose_init = true;
    std_msgs::UInt32 pause_state_msg;
    

    while (ros::ok())
    {

        // STATE TRANSITION =======================================================
        if ((rb_state == amr_state::FSM::ST_IDLE) && (transitPath.noPoints) && available_cycle)
        {
            rb_state = amr_state::FSM::DR_TRANSIT;
            index = 0;
        }
        else if ((rb_state == amr_state::FSM::DR_TRANSIT) && (index >= transitPath.noPoints))
        {
            rb_state = amr_state::FSM::ST_ITEM_PICKUP;
            is_load_unload_state_completed = false;
        }
        else if ((rb_state == amr_state::FSM::ST_ITEM_PICKUP) && (is_load_unload_state_completed))
        {
            rb_state = amr_state::FSM::DR_DELIVERY;
            index = 0;
        }
        else if ((rb_state == amr_state::FSM::DR_DELIVERY) && (index >= plannedPath.noPoints))
        {
            rb_state = amr_state::FSM::ST_ITEM_UNLOADING;
            is_load_unload_state_completed = false;
        }
        else if ((rb_state == amr_state::FSM::ST_ITEM_UNLOADING) && (is_load_unload_state_completed))
        {
            rb_state = amr_state::FSM::DR_RETURN;
            index = 0;
        }
        else if ((rb_state == amr_state::FSM::DR_RETURN) && (index >= Back2StartPath.noPoints))
        {
            rb_state = amr_state::FSM::ST_IDLE;
            available_cycle--;
            transitPath = pathStruct();
            plannedPath = pathStruct();
            Back2StartPath = pathStruct();
        }
        

        // PAUSE TRANSITION =======================================================
        else if ((rb_state == amr_state::FSM::ST_IDLE) && (is_pose_init))
        {
            rb_state = amr_state::FSM::PAUSE_ST_IDLE;
            is_camera_triggered = false;
            counter_camera_trigger = 0;

            is_pose_init = false;
            publisher_4_pause_states.publish(pause_state_msg);
        }
        else if ((rb_state == amr_state::FSM::PAUSE_ST_IDLE) && (is_camera_triggered))
        {
            publisher_4_pause_states.publish(pause_state_msg);
            rb_state = amr_state::FSM::ST_IDLE;
        }


        // PAUSE ON TRANSIT STATE
        else if ((rb_state == amr_state::FSM::DR_TRANSIT) && (counter_ref_point_publish == 0) && (index != 0))
        {
            rb_state = amr_state::FSM::PAUSE_DR_TRANSIT;
            is_camera_triggered = false;
            counter_camera_trigger = 0;
            publisher_4_pause_states.publish(pause_state_msg);
        }
        else if ((rb_state == amr_state::FSM::PAUSE_DR_TRANSIT) && (is_camera_triggered))
        {
            publisher_4_pause_states.publish(pause_state_msg);
            rb_state = amr_state::FSM::COMP_PR_TRANSIT;
            is_compensated_pr_completed = false;
        }
        else if ((rb_state == amr_state::FSM::COMP_PR_TRANSIT) && (is_compensated_pr_completed))
        {
            rb_state == amr_state::FSM::PAUSE_DR_TRANSIT_AT;
            is_camera_triggered = false;
            counter_camera_trigger = 0;
            publisher_4_pause_states.publish(pause_state_msg);
        }
        else if ((rb_state == amr_state::FSM::PAUSE_DR_TRANSIT_AT) && (is_camera_triggered))
        {
            publisher_4_pause_states.publish(pause_state_msg);
            rb_state = amr_state::FSM::DR_TRANSIT;
        }


        // PAUSE ON DELIVERY STATE
        else if ((rb_state == amr_state::FSM::DR_DELIVERY) && (counter_ref_point_publish == 0) && (index != 0))
        {
            rb_state = amr_state::FSM::PAUSE_DR_DELIVERY;
            is_camera_triggered = false;
            counter_camera_trigger = 0;
            publisher_4_pause_states.publish(pause_state_msg);
        }
        else if ((rb_state == amr_state::FSM::PAUSE_DR_DELIVERY) && (is_camera_triggered))
        {
            publisher_4_pause_states.publish(pause_state_msg);
            rb_state = amr_state::FSM::COMP_PR_DELIVERY;
            is_compensated_pr_completed = false;
            counter_compensated_pr_completed = 0;
        }
        else if ((rb_state == amr_state::FSM::COMP_PR_DELIVERY) && (is_compensated_pr_completed))
        {
            rb_state == amr_state::FSM::PAUSE_DR_DELIVERY_AT;
            is_camera_triggered = false;
            counter_camera_trigger = 0;
            publisher_4_pause_states.publish(pause_state_msg);
        }
        else if ((rb_state == amr_state::FSM::PAUSE_DR_DELIVERY_AT) && (is_camera_triggered))
        {
            publisher_4_pause_states.publish(pause_state_msg);
            rb_state = amr_state::FSM::DR_DELIVERY;
        }

        // PAUSE ON DELIVERY STATE
        else if ((rb_state == amr_state::FSM::DR_RETURN) && (counter_ref_point_publish == 0) && (index != 0))
        {
            rb_state = amr_state::FSM::PAUSE_DR_RETURN;
            is_camera_triggered = false;
            counter_camera_trigger = 0;
            publisher_4_pause_states.publish(pause_state_msg);
        }
        else if ((rb_state == amr_state::FSM::PAUSE_DR_RETURN) && (is_camera_triggered))
        {
            publisher_4_pause_states.publish(pause_state_msg);
            rb_state = amr_state::FSM::COMP_PR_RETURN;
            is_compensated_pr_completed = false;
            counter_compensated_pr_completed = 0;
        }
        else if ((rb_state == amr_state::FSM::COMP_PR_RETURN) && (is_compensated_pr_completed))
        {
            rb_state == amr_state::FSM::PAUSE_DR_RETURN_AT;
            is_camera_triggered = false;
            counter_camera_trigger = 0;
            publisher_4_pause_states.publish(pause_state_msg);
        }
        else if ((rb_state == amr_state::FSM::PAUSE_DR_RETURN_AT) && (is_camera_triggered))
        {
            publisher_4_pause_states.publish(pause_state_msg);
            rb_state = amr_state::FSM::DR_RETURN;
        }

        ROS_INFO_STREAM("[MOTION_PLANNING] FSM State: " << rb_state);



        // STATE BEHAVIOR ==========================================================
        // MOVEMENT FLAG
        if (((rb_state == amr_state::FSM::DR_TRANSIT) || (rb_state == amr_state::FSM::DR_DELIVERY) || (rb_state == amr_state::FSM::DR_RETURN)) || ((rb_state == amr_state::FSM::COMP_PR_TRANSIT) || (rb_state == amr_state::FSM::COMP_PR_DELIVERY) || (rb_state == amr_state::FSM::COMP_PR_RETURN)))
        {
            in_moving_msg.data = true;
            publisher_4_in_movement.publish(in_moving_msg);
        }
        else
        {
            in_moving_msg.data = false;
            publisher_4_in_movement.publish(in_moving_msg);
        }


        // EMERGENCY STOP
        if (rb_state == amr_state::FSM::ST_EMERGENCY_STOP)
        {
            ;
        }

        // IDLE
        if (rb_state == amr_state::FSM::ST_IDLE)
        {
            if (available_cycle)
            {
                std_msgs::UInt32 msg;
	            msg.data = 1;

                // publish for requesting process the list of cycle
                ROS_INFO_STREAM("[MOTION_PLANNING] [IDLE] REQUESTING PROCESSING CYCLE"<< std::endl);
                publisher_4_next_cycle.publish(msg);
            }
            else
            {
                ROS_INFO_STREAM("[MOTION_PLANNING] [IDLE] UNAVAILABLE CYCLE"<< std::endl);
            }
        }

 /*        // MOVING TO THE PICK UP POINT
        if (rb_state == amr_state::FSM::DR_TRANSIT)
        {
            if (index <= transitPath.noPoints-1)
            {
                ROS_INFO_STREAM(std::endl<<"[MOTION_PLANNING] [IN TRANSIT] Index: "   << index);

                if (index == 0)
                    is_pure_rotation.data = 1;
                else
                {
                    if ((transitPath.path.points[index].x != transitPath.path.points[index-1].x) || (transitPath.path.points[index].y != transitPath.path.points[index-1].y) || (transitPath.path.points[index].z != transitPath.path.points[index-1].z))
                        is_pure_rotation.data = (transitPath.path.points[index].z != transitPath.path.points[index-1].z);
                }
 
                publisher_4_movement_type.publish(is_pure_rotation);

                // increment index
                publisher_4_trajectory_tracking.publish(toAmrPose(transitPath.path.points[index]));
                index++;
            }
        }

        // MOVING TO THE DELIVERY POINT
        if (rb_state == amr_state::FSM::DR_DELIVERY)
        {
            if (index <= plannedPath.noPoints-1)
            {
                // ROS_INFO_STREAM("[State: " << Robot_State << "]" << std::endl);
                ROS_INFO_STREAM("[IN DELIVERY] Index: "<< index << std::endl);
                // follow the transit path to move to the transit point
                // send the reference point for very 0.5s

                if (index == 0)
                    is_pure_rotation.data = 1;
                else
                {
                    if ((plannedPath.path.points[index].x != plannedPath.path.points[index-1].x) || (plannedPath.path.points[index].y != plannedPath.path.points[index-1].y) || (plannedPath.path.points[index].z != plannedPath.path.points[index-1].z))
                        is_pure_rotation.data = (plannedPath.path.points[index].z != plannedPath.path.points[index-1].z);
                }

                publisher_4_movement_type.publish(is_pure_rotation);

                // reset index
                publisher_4_trajectory_tracking.publish(toAmrPose(plannedPath.path.points[index]));
                index++;
            }
        }

        // GET BACK TO START
        if (rb_state == amr_state::FSM::DR_RETURN)
        {            
            if (index <= Back2StartPath.noPoints-1)
            {
                ROS_INFO_STREAM("[IN DELIVERY] Index: "<< index << std::endl);

                if (index == 0)
                    is_pure_rotation.data = 1;
                else
                {
                    if ((Back2StartPath.path.points[index].x != Back2StartPath.path.points[index-1].x) || (Back2StartPath.path.points[index].y != Back2StartPath.path.points[index-1].y) || (Back2StartPath.path.points[index].z != Back2StartPath.path.points[index-1].z))
                        is_pure_rotation.data = (Back2StartPath.path.points[index].z != Back2StartPath.path.points[index-1].z);
                }

                publisher_4_movement_type.publish(is_pure_rotation);

                // reset index
                publisher_4_trajectory_tracking.publish(toAmrPose(Back2StartPath.path.points[index]));
                index++;
            }
        } */

        if ((rb_state == amr_state::FSM::DR_TRANSIT) || (rb_state == amr_state::FSM::DR_DELIVERY) || (rb_state == amr_state::FSM::DR_RETURN))
        {   
            pathStruct path  = pathStruct();

            switch (rb_state){
                case (amr_state::FSM::DR_TRANSIT):
                    path = transitPath;
                    break;
                case (amr_state::FSM::DR_DELIVERY):
                    path = plannedPath;
                    break;
                case (amr_state::FSM::DR_RETURN):
                default:
                    path = Back2StartPath;
                    break;
            }
            
            
            if ((counter_ref_point_publish % REF_CYCLE) == 0)
            {
                counter_ref_point_publish = 0;

                if (index <= path.noPoints-1)
                {
                    ROS_INFO_STREAM("[IN DELIVERY] Index: "<< index << std::endl);

                    if (index == 0)
                        is_pure_rotation.data = 1;
                    else
                    {
                        if ((path.path.points[index].x != path.path.points[index-1].x) || (path.path.points[index].y != path.path.points[index-1].y) || (path.path.points[index].z != path.path.points[index-1].z))
                            is_pure_rotation.data = (path.path.points[index].z != path.path.points[index-1].z);
                    }

                    publisher_4_movement_type.publish(is_pure_rotation);

                    // reset index
                    publisher_4_trajectory_tracking.publish(toAmrPose(path.path.points[index]));
                    index++;
                }
            }
            counter_ref_point_publish++;
        }


        if ((rb_state == amr_state::FSM::ST_ITEM_UNLOADING) || (rb_state == amr_state::FSM::ST_ITEM_PICKUP))
        {
            amr_msgs::RobotState LoadUnloadSignal;

            LoadUnloadSignal.data = rb_state;
            publisher_4_load_unload_state.publish(LoadUnloadSignal);
        }

        if (((rb_state == amr_state::FSM::PAUSE_DR_TRANSIT) || (rb_state == amr_state::FSM::PAUSE_DR_DELIVERY) || (rb_state == amr_state::FSM::PAUSE_DR_RETURN)) || ((rb_state == amr_state::FSM::PAUSE_DR_TRANSIT_AT) || (rb_state == amr_state::FSM::PAUSE_DR_DELIVERY_AT) || (rb_state == amr_state::FSM::PAUSE_DR_RETURN_AT)) || (rb_state == amr_state::FSM::PAUSE_ST_IDLE))
        {
            counter_camera_trigger++;
            if (counter_camera_trigger == PAUSE_CYCLE)
            {
                is_camera_triggered = true;
                counter_camera_trigger = 0;
            }
        }

        
        ros::spinOnce();
        rate.sleep();
    }

	return 0;
}


// ---------- ADDITIONAL FUNCTIONS FOR GETTING PATHS ---------------

void subscriberCallback4TransitPath(const geometry_msgs::Polygon& msg)
{
    ROS_INFO_STREAM("[GETTING TRANSIT PATH] transit Path: " << std::endl);
    transitPath.importPolygonMSG(msg);
    transitPath.isAvailable("transit Path");  
}

void subscriberCallback4PlannedPath(const geometry_msgs::Polygon& msg)
{
    ROS_INFO_STREAM("[GETTING PLANNED PATH] planned Path: " << std::endl);
    plannedPath.importPolygonMSG(msg);
    plannedPath.isAvailable("planned Path");
}

void subscriberCallback4TestPath(const geometry_msgs::Polygon& msg)
{
    ROS_INFO_STREAM("[GETTING TEST PATH] test Path: " << std::endl);
    testPath.importPolygonMSG(msg);
    ROS_INFO_STREAM("Last point: " << testPath.path.points[23].x);
    ROS_INFO_STREAM("Last point: " << testPath.path.points[23].y);
    ROS_INFO_STREAM("POINTS: " << testPath.noPoints << std::endl);
    testPath.isAvailable("test Path");
}


void subscriberCallback4Back2StartPath(const geometry_msgs::Polygon& msg)
{
    ROS_INFO_STREAM("[GETTING back2start PATH] back2start Path: " << std::endl);
    Back2StartPath.importPolygonMSG(msg);
    Back2StartPath.isAvailable("back 2 start Path");
}
// --------------------------------------------------------------------


AmrPose toAmrPose(geometry_msgs::Point32 data)
{
    AmrPose msgPose;
    msgPose.pose_x = data.x;
    msgPose.pose_y = data.y;
    msgPose.pose_phi = data.z;
    return msgPose;
}

void getStateIndicator(const std_msgs::String& msg)
{
    stateSentRequest = msg.data;

    if (!stateSentRequest.compare("IDLE") || !stateSentRequest.compare("idle"))
    {
        rb_state = amr_state::FSM::ST_IDLE;
    }
    else if (!stateSentRequest.compare("STOP") || !stateSentRequest.compare("stop"))
    {
        rb_state = amr_state::FSM::ST_EMERGENCY_STOP;
    }
    else if (!stateSentRequest.compare("start_transit") || !stateSentRequest.compare("START_TRANSIT"))
    {
        rb_state = amr_state::FSM::DR_TRANSIT;
    }
    else if (!stateSentRequest.compare("arrived at transit") || !stateSentRequest.compare("ARRIVED AT TRANSIT"))
    {
        rb_state = amr_state::FSM::ST_ITEM_PICKUP;
    }
    else if (!stateSentRequest.compare("start_delivery") || !stateSentRequest.compare("START_DELIVERY"))
    {
        rb_state = amr_state::FSM::DR_DELIVERY;
    }
    else if (!stateSentRequest.compare("arrived at final") || !stateSentRequest.compare("ARRIVED AT FINAL"))
    {
        rb_state = amr_state::FSM::ST_ITEM_UNLOADING;
    }
    else if (!stateSentRequest.compare("get back to start") || !stateSentRequest.compare("GET BACK TO START"))
    {
        rb_state = amr_state::FSM::DR_RETURN;
    }
    else
    {
        ROS_INFO_STREAM("-- INVALID STATE INDICATOR --" << std::endl);
    }
    
}

void subscriberCallback4SwitchState(const amr_msgs::GeneralMsg& msg)
{
    is_load_unload_state_completed = true;
}

void subscriberCallback4notifyCycle(const std_msgs::UInt32& msg)
{
    if (msg.data)
    {
        ROS_INFO_STREAM("[MOTION PLANNING] --- There is(are) "<< msg.data <<" cycle available ---" << std::endl);
        available_cycle = msg.data;
    }
    else
    {
        ROS_INFO_STREAM("[MOTION PLANNING] --- NO CYCLE YET ---"<< std::endl);
    }
}

int get_movement_type(int index, pathStruct planned_path, int pre_movement_type)
{
    int movement_type = pre_movement_type;

    if (index == 0)
        movement_type = 1;
    else
    {
        if ((planned_path.path.points[index].x != planned_path.path.points[index-1].x) || (planned_path.path.points[index].y != planned_path.path.points[index-1].y) || (planned_path.path.points[index].z != planned_path.path.points[index-1].z))
            movement_type = (transitPath.path.points[index].z != transitPath.path.points[index-1].z);
    }

    return movement_type;
}
