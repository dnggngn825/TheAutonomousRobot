/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


#ifndef AMR_CONNECTION_CONSTANTS_H
#define AMR_CONNECTION_CONSTANTS_H


// ROBOT ENCODER CONNECTION INFO
// #define AMR_ENCODER_PIN_CHANNEL_A_MOTOR_L       133
// #define AMR_ENCODER_PIN_CHANNEL_B_MOTOR_L       134
// #define AMR_ENCODER_PIN_CHANNEL_A_MOTOR_R       105
// #define AMR_ENCODER_PIN_CHANNEL_B_MOTOR_R       160
#define AMR_ENCODER_PIN_CHANNEL_A_MOTOR_L       105
#define AMR_ENCODER_PIN_CHANNEL_B_MOTOR_L       160
#define AMR_ENCODER_PIN_CHANNEL_A_MOTOR_R       133
#define AMR_ENCODER_PIN_CHANNEL_B_MOTOR_R       134
#define AMR_ENCODER_PIN_DEFAULT                 0 
#define AMR_GPIO_CHIP_NAME                      "/dev/gpiochip0" // DEVICE INFO // Note: for the 40-pin header of the Jetson SBCs -> "/dev/gpiochip0"

// ROBOT MOTOR CONNECTION INFO 
#define AMR_POLOLU_SMC_ADDRESS_L                14
#define AMR_POLOLU_SMC_ADDRESS_R                13
#define AMR_I2C_DEVICE_NAME                     "/dev/i2c-1"

#define AMR_PCA9685_ADDRESS                     0x42
#define AMR_PCA9685_CHANNEL                     15
#define AMR_PCA9685_FREQUENCY                   50.0f


#endif 
