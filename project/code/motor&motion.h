

#ifndef __MM__H__
#define __MM__H__
#include "pid.h"
#define ENCODER_PER_dM  (6)//12000/m
#define ENCODER_PER_cm   (120)
#define mo1		PWM2_MODULE3_CHA_D2//PWM2_MODULE3_CHA_D2
#define mo2		PWM2_MODULE3_CHB_D3//
#define mo3		PWM2_MODULE1_CHB_C9//PWM2_MODULE1_CHA_C8
#define mo4		PWM2_MODULE0_CHB_C7
#define PI_F       3.14159265358979323846f
#define HALF_PI_F  1.57079632679489661923f
#define MAX_TRANSLATION_ACCEL_DMPS2    12.0f
#define TRANSLATION_STOP_EPS_DMPS      0.05f
typedef struct {
    float x;        
    float y;       
    float angle;    
} Odometry_t;
extern int16 M1,M2,M3,M4;
extern Odometry_t robot_pos;
extern float roll,pitch;
extern float vel_x_robot,vel_y_robot,vel_w_cmd;

void encoder_init(void);
int16 get_encoder(void);
void Motor_Init(void);
void mecanum_basic(float y,float x,float w);
void mecanum_cascade_control(float raw_target_y, float raw_target_x, float raw_target_angle, 
                             float curr_y, float curr_x, float curr_angle);
void mecanum_spin_move(float target_y, float target_x, float rotate_speed, 
                       float curr_y, float curr_x, float curr_angle);
#endif 
