/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


#ifndef AMR_CONSTANTS_H
#define AMR_CONSTANTS_H


// ROBOT FRAME INFO
#define AMR_BASE_RADIUS_WHEEL_L                 0.05f
#define AMR_BASE_RADIUS_WHEEL_R                 0.05f
#define AMR_BASE_HALF_WHEEL_BASE                0.112f

// SAMPLING TIME SETTING
#define AMR_TIMER_4_MOTOR_CONTROL_IN_SEC        0.02f
#define AMR_TIMER_4_MOTOR_CONTROL_IN_MIN        (AMR_TIMER_4_MOTOR_CONTROL_IN_SEC/60.0f)
#define AMR_TIMER_4_SYSTEM_CONTROL_IN_SEC       0.5f
#define AMR_TIMER_4_SYSTEM_CONTROL_IN_MIN       (AMR_TIMER_4_SYSTEM_CONTROL_IN_SEC/60.0f)
// #define AMR_TIMER_4_CAMERA_CONTROL_IN_SEC       1.0f
#define AMR_TIMER_4_CAMERA_CONTROL_IN_SEC       0.5f
#define AMR_TIMER_4_CAMERA_CONTROL_IN_MIN       (AMR_TIMER_4_SYSTEM_CONTROL_IN_SEC/60.0f)
#define AMR_TIMER_4_MOTOR_CONTROL               (AMR_TIMER_4_MOTOR_CONTROL_IN_SEC)
#define AMR_TIMER_4_SYSTEM_CONTROL              (AMR_TIMER_4_SYSTEM_CONTROL_IN_SEC)
#define AMR_TIMER_4_CAMERA_CONTROL              (AMR_TIMER_4_CAMERA_CONTROL_IN_SEC)
#define AMR_TIMER_4_POSE_REF_SIGNAL             6.0f //4.0f

// #define AMR_TIMER_4_CAMERA_CONTROL_V3           0.5f
// #define AMR_TIMER_4_POSE_REF_SIGNAL_V3          6.0f



#define AMR_TIMER_4_ENC_MONITOR                 4e-6 // <7.8125e-6

// MOTOR CONFIG         
// #define AMR_FORWARD_DIRECTION                   +1
// #define AMR_CW_DIRECTION_MOTOR_L                (AMR_FORWARD_DIRECTION)
// #define AMR_CW_DIRECTION_MOTOR_R                ((-1)*AMR_FORWARD_DIRECTION)
// #define AMR_CCW_DIRECTION_MOTOR_L               ((-1)*AMR_FORWARD_DIRECTION)
// #define AMR_CCW_DIRECTION_MOTOR_R               (AMR_FORWARD_DIRECTION)

#define AMR_DIRECTION_CW_MOTOR_R                -1 // +1: FW / -1: BW
#define AMR_DIRECTION_CW_MOTOR_L                +1 // +1: FW / -1: BW

#define AMR_MOTOR_VOLTAGE                       12.0f

#endif 