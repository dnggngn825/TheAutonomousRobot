/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


#ifndef AMR_ARCHITECTURE_H
#define AMR_ARCHITECTURE_H


namespace amr_topic 
{
    const char* TIMER_4_MOTOR_CONTROL               = "timer_4_motor_control";
    const char* TIMER_4_SYSTEM_CONTROL              = "timer_4_system_control";
    const char* TIMER_4_CAMERA_CONTROL              = "timer_4_camera_control";
    
    const char* POSE_REF                            = "pose_reference";
    const char* POSE_EST                            = "pose_estimated";

    const char* MOTOR_ANGULAR_VELOCITY_REF          = "motor_angular_velocity_reference";
    const char* MOTOR_ANGULAR_VELOCITY_EST          = "motor_angular_velocity_estimated";
    
    const char* MOTOR_PWM_DUTY_CYCLE                = "motor_pwm_duty_cycle";

    const char* ENCODER_DATA_4_MOTOR_CONTROL        = "encoder_data_motor_control";
    const char* ENCODER_DATA_4_MOTOR_CONTROL_L      = "encoder_data_motor_control_l";
    const char* ENCODER_DATA_4_MOTOR_CONTROL_R      = "encoder_data_motor_control_r";
    const char* ENCODER_DATA_4_ODOMETRY_CONTROL     = "encoder_data_odometry_control";
    const char* ENCODER_DATA_4_ODOMETRY_CONTROL_L   = "encoder_data_odometry_control_l";
    const char* ENCODER_DATA_4_ODOMETRY_CONTROL_R   = "encoder_data_odometry_control_r";
    
    // const char* ENC_L_CHANNEL_A_STATE               = "encoder_l_channel_a_state";
    // const char* ENC_L_CHANNEL_B_STATE               = "encoder_l_channel_b_state";
    // const char* ENC_R_CHANNEL_A_STATE               = "encoder_r_channel_a_state";
    // const char* ENC_R_CHANNEL_B_STATE               = "encoder_r_channel_b_state";

    // Inside servo control node
    const char* STATE_COMPLETED_SIGNAL              = "state_completed_signal";
    const char* SERVO_CONTROL_SIGNAL                = "servo_control_signal";
    const char* LOAD_UNLOAD_STATE_SIGNAL            = "load_unload_state_signal";
    const char* REQUEST_PROX_SS_SIGNAL              = "request_prox_ss_signal";
    const char* PROX_SENSOR_SIGNAL                  = "proximity_sensor_signal";


    const char* POSE_DATA_FROM_ODOMETRY             = "estimated_pose_from_odometry"; //old
    // const char* POSE_DATA_FROM_ODOM                 = "estimated_pose_from_odom";
    const char* POSE_DATA_FROM_CAM                  = "estimated_pose_from_cam";
    const char* TRANSFORMATION_MAT_CAM_2_MARKER     = "transformation_mat_cam_to_marker";

    const char* TRANSIT_PATH_MATLAB                 = "transit_path_Matlab";
    const char* PLANNED_PATH_MATLAB                 = "planned_path_Matlab";
    const char* TRAJECTORY_REF_SIGNAL               = "trajectory_ref_signal";
    const char* TRAJECTORY_TARGET_SIGNAL            = "trajectory_target_signal";
    const char* STATE                               = "state_indicator";

    const char* STOP                                = "stop";
    const char* MOVEMENT_TYPE                       = "movement_type";
    const char* PROXIMITY_DATA                      = "proximity_data";
    const char* SERVO_MOTOR_CMD                     = "servo_motor_cmd";
    const char* CAMERA_TRIGGER                      = "camera_trigger";
    const char* CAMERA_TRIGGER_RETURN               = "camera_trigger_return";
    
}

namespace amr_node 
{
    const char* TIMER_MONITOR                       = "timer_monitor";
    const char* POSE_CONTROLLER                     = "pose_controller";

    const char* MOTOR_CONTROLLER                    = "motor_controller";
    const char* MOTOR_DYNAMICS                      = "motor_dynamics";

    // const char* ENCODER_MONITOR                     = "encoder_monitor"; // OLD
    const char* ENCODER_MONITOR_4_MOTOR_CONTROL     = "enc_monitor_4_motor_ctl";
    const char* ENCODER_MONITOR_4_ODOM_CONTROL      = "enc_monitor_4_odom_ctl";
    const char* ENCODER_MONITOR_L                   = "encoder_monitor_l";
    const char* ENCODER_MONITOR_R                   = "encoder_monitor_r";

    // const char* ODOMETRY_ESTIMATOR                  = "odometry_estimator";  // OLD
    // const char* POSE_ESTIMATOR_ODOM                 = "pose_estimator_odom"; // OLD
    const char* POSE_ESTIMATOR_CAM                  = "pose_estimator_cam"; 
    const char* POSE_ESTIMATOR_FUSION               = "pose_estimator_fusion"; 
    const char* CAMERA_MONITOR                      = "camera_monitor"; 

    // const char* ENC_MONITOR_L_CHANNEL_A             = "enc_monitor_l_channel_a";
    // const char* ENC_MONITOR_L_CHANNEL_B             = "enc_monitor_l_channel_b";
    // const char* ENC_MONITOR_R_CHANNEL_A             = "enc_monitor_r_channel_a";
    // const char* ENC_MONITOR_R_CHANNEL_B             = "enc_monitor_r_channel_b";
    // const char* ENC_MONITOR_L_3200                  = "enc_monitor_l_3200";
    // const char* ENC_MONITOR_R_3200                  = "enc_monitor_r_3200";
    // const char* ENCODER_MONITOR_4_MOTOR_CONTROL_3200     = "enc_monitor_4_motor_ctl_3200";
    // const char* ENCODER_MONITOR_4_ODOM_CONTROL_3200      = "enc_monitor_4_odom_ctl_3200";

    const char* SERVO_CONTROLLER                    = "servo_controller";
    // const char* SERVO_DYNAMICS                      = "servo_dynamics";


    const char* MOTION_PLANNING                     = "motion_planning";
    const char* TRAJ_TRACK_CONTROLLER               = "traj_track_controller";

    // Testing 
    const char* REF_PWM_PUBLISH     = "ref_pwm_publish";
    const char* REF_ANG_VEL_PUBLISH = "ref_velocity_publish";
    const char* REF_POSE_PUBLISH    = "ref_pose_publish";
}

namespace amr_state
{
    const char* REQUEST_STATE       = "Request for state";
    const char* IDLE                = "Idle";
    const char* EMERGENCY_STOP      = "EMG Stop";
    const char* IN_TRANSIT          = "In transit";
    const char* ARRIVE_AT_TRANSIT   = "Arrive at transit";
    const char* IN_DELIVERY         = "In delivery";
    const char* ARRIVE_AT_FINAL     = "Arrive at final";
    const char* GET_BACK_TO_START   = "Get back to the start position";

    enum FSM:int
    {
        ST_EMERGENCY_STOP   = 0,
        ST_IDLE,
        
        DR_TRANSIT = 2,
        DR_DELIVERY,
        DR_RETURN,
        ST_ITEM_PICKUP,
        ST_ITEM_UNLOADING,
        
        ROTATION_AFTER_PAUSE_TRANSIT = 100,
        ROTATION_AFTER_PAUSE_DELIVERY,
        ROTATION_AFTER_PAUSE_RETURN,

        PAUSE_ST_IDLE = 10,
        PAUSE_DR_TRANSIT,
        PAUSE_DR_DELIVERY,
        PAUSE_DR_RETURN,

        PAUSE_DR_TRANSIT_AT,
        PAUSE_DR_DELIVERY_AT,
        PAUSE_DR_RETURN_AT,

        COMP_PR_TRANSIT = 20,
        COMP_PR_DELIVERY,
        COMP_PR_RETURN,

        // DEfined by Quang Trung LE
        DR_PR_TRANSIT,
        DR_LT_TRANSIT,        

        DR_PR_DELIVERY,
        DR_LT_DELIVERY,

        DR_PR_RETURN,
        DR_LT_RETURN,
        

        DEFAULT,
    };

    enum Sensor_State: int
    {
        OBJECT_MISSING = 0,
        OBJECT_PRESENT = 1, 
    };

    enum Movement_Type: int
    {
        ROTATION = 0,
        LINEAR  = 1,
    };

}



#endif

