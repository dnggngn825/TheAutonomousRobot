/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


#ifndef POLOLU_SMC_G2_ENCODER_H
#define POLOLU_SMC_G2_ENCODER_H

#include "pololu_smc_g2_encoder/pololu_smc_g2_encoder_constants.h"
#include <gpiod.h>
#include "ros/ros.h"



class Pololu_SMC_G2_Encoder
{
    public:
        enum Line_Request_Event : int
        {
            RISING_EDGE     = 0,
            FALLING_EDGE    = 1,
            BOTH_EDGES      = 2,
        };

        enum Line_event
        {
            GPIOD_LINE_EVENT_RISING_EDGE = 1,
            GPIOD_LINE_EVENT_FALLING_EDGE,
        };

        enum Line_State 
        {
            LOW     = 0,
            HIGH    = 1,
        };

        enum Monitor_Mode: int
        {
            POLLING = 0,
            TRIGGER = 1,
        };

    private:
        // Motor specification
        float angular_velocity_rpm = 0; 

        struct timespec TIME_OUT = {0, 10000000}; 

        // Debounce: polling_digital filter 
        uint16_t df_debounce_state = 0x0000;

        // Debounce: polling_state_waves
        uint16_t sw_pre_state = 0x0000;
        uint16_t sw_new_state = 0x0000;
        int sw_states_tracker = 0;
        
        int NUM_CHANNEL_STATE_TRACKER = 2;
        int STATE_DIRECTION_DEFAULT[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
        int state_directions[16];


    public:
        int dir_channel_a2b = 0; // +1: equal cw; -1: equal ccw 
        int dir_channel_b2a = 0; // +1: equal cw; -1: equal ccw 
        

    private:
        const char * chip;
        int line_channel_a;
        int line_channel_b;

        struct gpiod_chip *gpiod_chip;
        struct gpiod_line *gpiod_line_channel_a;
        struct gpiod_line *gpiod_line_channel_b;
        struct gpiod_line_event gpiod_line_event_channel_a;
        struct gpiod_line_event gpiod_line_event_channel_b;

    public: 
        Pololu_SMC_G2_Encoder();
        Pololu_SMC_G2_Encoder(int dir_channel_a2b);

    public: // getters and setters
        void set_angular_velocity_rpm(float angular_velocity_rpm);
        float get_angular_velocity_rpm(); // revolution per minute
    
    public: 
        // Rotation Angle and Angular Velocity
        void set_angular_velocity_rpm(int encoder_counter, float sampling_time_in_sec);
        float get_angular_velocity_rps(); // revolution per second
        float get_angular_velocity_rdps(); // radian per second
        float get_angular_velocity_pwm(); // pwm
        float get_rotation_angle_rad(float sampling_period_in_sec); 
        float get_rotation_angle_rev(float sampling_period_in_sec); 

        // Getters & Setters
        // int get_cw_direction();
        // int get_ccw_direction();
        // void set_cw_direction(int cw_direction);
        // void set_ccw_direction(int ccw_direction);

        int get_dir_channel_a2b();
        int get_dir_channel_b2a();

        int get_line_channel_a();
        int get_line_channel_b();

        void set_dir_channel_a2b(int dir_channel_a2b);
        void set_dir_channel_b2a(int dir_channel_b2a);

        void set_line_channel_a(int line_channel_a);
        void set_line_channel_b(int line_channel_b);

        // Connection gpiod
        void set_gpiod_chip();
        void set_gpiod_chip(const char* chip, struct gpiod_chip *gpiod_chip);
        void set_gpiod_line_channel(int line_channel_a, int line_channel_b);
        
        void set_gpiod_line_request_events(Pololu_SMC_G2_Encoder::Line_Request_Event line_request_event);

        int monitor_encoder_trigger_falling_edge_wo_debounce(struct gpiod_line_event event);
        int monitor_encoder_trigger_falling_edge_w_time_debounce(struct gpiod_line_event event);
        int monitor_encoder_trigger_both_edges_wo_debounce(struct gpiod_line_event event);
        int monitor_encoder_polling_w_df_debounce();
        int monitor_encoder_polling_w_sw_debounce();

        int get_channel_event_type(struct gpiod_line *gpiod_line_channel, struct gpiod_line_event event);
        int get_channel_a_event_type(struct gpiod_line_event event);
        int get_channel_b_event_type(struct gpiod_line_event event);
};

#endif
