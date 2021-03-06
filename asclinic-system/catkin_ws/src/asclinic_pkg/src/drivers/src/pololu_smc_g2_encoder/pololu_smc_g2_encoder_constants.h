/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


#ifndef POLOLU_SMC_G2_ENCODER_CONSTANTS_H
#define POLOLU_SMC_G2_ENCODER_CONSTANTS_H


// ROBOT MOTOR INFO
#define POLOLU_SMC_G2_MOTOR_GEARBOX                                         50

// ROBOT ENCODER INFO
#define POLOLU_SMC_G2_ENCODER_RESOLUTION_SESC_NO_GEARBOX                    16 // Single edge single channel
#define POLOLU_SMC_G2_ENCODER_RESOLUTION_MESC_NO_GEARBOX                    32 // Multi edges single channel
#define POLOLU_SMC_G2_ENCODER_RESOLUTION_MEMC_NO_GEARBOX                    64 // Multi edges multi channels
#define POLOLU_SMC_G2_ENCODER_RESOLUTION_SESC_W_GEARBOX                     (POLOLU_SMC_G2_ENCODER_RESOLUTION_SESC_NO_GEARBOX*POLOLU_SMC_G2_MOTOR_GEARBOX) // 800
#define POLOLU_SMC_G2_ENCODER_RESOLUTION_MESC_W_GEARBOX                     (POLOLU_SMC_G2_ENCODER_RESOLUTION_MESC_NO_GEARBOX*POLOLU_SMC_G2_MOTOR_GEARBOX) // 1600
#define POLOLU_SMC_G2_ENCODER_RESOLUTION_MEMC_W_GEARBOX                     (POLOLU_SMC_G2_ENCODER_RESOLUTION_MEMC_NO_GEARBOX*POLOLU_SMC_G2_MOTOR_GEARBOX) // 3200

      
#define POLOLU_SMC_G2_MOTOR_MAX_ANGULAR_VELOCITY_RPM_NO_GEARBOX             10000.0f
#define POLOLU_SMC_G2_MOTOR_MAX_ANGULAR_VELOCITY_RPM_W_GEARBOX              (POLOLU_SMC_G2_MOTOR_MAX_ANGULAR_VELOCITY_RPM_NO_GEARBOX/POLOLU_SMC_G2_MOTOR_GEARBOX)


// ROBOT CONFIGURATIONS
#define POLOLU_SMC_G2_ENCODER_RESOLUTION                                    (POLOLU_SMC_G2_ENCODER_RESOLUTION_SESC_W_GEARBOX)
#define POLOLU_SMC_G2_ENCODER_CHANNEL_MAX_FREQUENCY_PPS_SESC_W_GEARBOX      (POLOLU_SMC_G2_MOTOR_MAX_ANGULAR_VELOCITY_RPM_W_GEARBOX*POLOLU_SMC_G2_ENCODER_RESOLUTION_SESC_W_GEARBOX/60.0f)  // 1/4 channel period
#define POLOLU_SMC_G2_ENCODER_CHANNEL_MIN_DURATION_SPP_SESC_W_GEARBOX       (1.0f/POLOLU_SMC_G2_ENCODER_CHANNEL_MAX_FREQUENCY_PPS_SESC_W_GEARBOX)
#define POLOLU_SMC_G2_ENCODER_MAX_DEBOUNCE_DURATION_SESC_W_GEARBOX          (POLOLU_SMC_G2_ENCODER_CHANNEL_MIN_DURATION_SPP_SESC_W_GEARBOX/4.0f) // 1/4 channel period = 93.75us


// PARAMETER SELECTIONS
#define POLOLU_SMC_G2_ENCODER_DEBOUNCE_INTERVAL_SESC_W_GEARBOX              10 // us
#define POLOLU_SMC_G2_ENCODER_POLLING_SW_FREQUENCY_MEMC_W_GEARBOX           20000                                
#define POLOLU_SMC_G2_ENCODER_POLLING_DF_FREQUENCY_MEMC_W_GEARBOX           240000                                


// #define POLOLU_SMC_G2_ENCODER_CHANNEL_STATE_HIGH                            1
// #define POLOLU_SMC_G2_ENCODER_CHANNEL_STATE_LOW                             0

// ENCODER DIRECTION
#define POLOLU_SMC_G2_ENCODER_DIR_CHANNEL_A2B_MOTOR_R                       -1 // +1: equal cw; -1: equal ccw 
#define POLOLU_SMC_G2_ENCODER_DIR_CHANNEL_A2B_MOTOR_L                       -1 // +1: equal cw; -1: equal ccw 

#endif
