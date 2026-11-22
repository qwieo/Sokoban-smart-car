

#include "position_fusion.h" 
#include <math.h> 
#include <string.h> 
 
void PositionFusion_Init(PositionFusion_t *fusion) 
{ 
    if (fusion == 0) 
    { 
        return; 
    } 
 
    memset(fusion, 0, sizeof(PositionFusion_t)); 
		fusion->camera_enable = 1; 
    fusion->alpha_normal = 0.707f; 
    fusion->alpha_slip = 1.00f; 
    fusion->slip_threshold = 3.f; 
    fusion->max_camera_jump = 3.0f; 
    fusion->max_reject_count = 3; 
    fusion->initialized = 0; 
} 
 


    float residual_x; 
    float residual_y; 
 
uint8_t PositionFusion_CorrectCamera( 
    PositionFusion_t *fusion, 
    int16_t camera_x, 
    int16_t camera_y) 
{ 
	 
		if(!fusion->camera_enable) 
    { 
        return 0; 
    } 
		 
    float measurement_x; 
    float measurement_y; 
 
    float residual_distance; 
 
    float alpha; 
 
    if (fusion == 0) 
    { 
        return 0; 
    } 
 
    measurement_x = (float)camera_x; 
    measurement_y = (float)camera_y; 
 
    if (!fusion->initialized) 
    { 
        fusion->x = measurement_x; 
        fusion->y = measurement_y; 
 
        fusion->reject_count = 0; 
        fusion->initialized = 1; 
 
        return 1; 
    } 
 
    residual_x = measurement_x - fusion->x; 
    residual_y = measurement_y - fusion->y; 
 
    residual_distance = sqrtf( 
        residual_x * residual_x + 
        residual_y * residual_y); 
 
    

    if (residual_distance > fusion->max_camera_jump) 
    { 
        fusion->reject_count++; 
 
        if (fusion->reject_count >= fusion->max_reject_count) 
        { 
            fusion->x = measurement_x; 
            fusion->y = measurement_y; 
 
            fusion->reject_count = 0; 
 
            return 1; 
        } 
 
        return 0; 
    } 
 
    fusion->reject_count = 0; 
 
    if (residual_distance > fusion->slip_threshold) 
    { 
        alpha = fusion->alpha_slip; 
    } 
    else 
    { 
        alpha = fusion->alpha_normal; 
    } 
 
    fusion->x += alpha * residual_x; 
    fusion->y += alpha * residual_y; 
 
    return 1; 
} 
 
 
 
 
void PositionFusion_UpdateOdometry( 
    PositionFusion_t *fusion, 
    int16_t cnt1, 
    int16_t cnt2, 
    int16_t cnt3, 
    int16_t cnt4, 
    float gyro) 
{ 
    float d1; 
    float d2; 
    float d3; 
    float d4; 
 
    float dx_robot; 
    float dy_robot; 
 
 
    float delta_x_world; 
    float delta_y_world; 
 
    float cos_gyro; 
    float sin_gyro; 
 
    if (fusion == 0) 
    { 
        return; 
    } 
 
    

    d1 = (float)cnt1 / ENCODER_COUNT_PER_PIXEL; 
    d2 = (float)cnt2 / ENCODER_COUNT_PER_PIXEL; 
    d3 = (float)cnt3 / ENCODER_COUNT_PER_PIXEL; 
    d4 = (float)cnt4 / ENCODER_COUNT_PER_PIXEL; 
 
    dy_robot = 
        (d1 + d2 + d3 + d4) * 0.25f; 
 
    dx_robot = 
        (-d1 + d2 - d3 + d4) * 0.25f; 
		 
		dx_robot*=ODOM_SCALE_X; 
		dy_robot*=ODOM_SCALE_Y; 
    cos_gyro = cosf(gyro); 
    sin_gyro = sinf(gyro); 
 
    delta_x_world = 
        dx_robot * cos_gyro - 
        dy_robot * sin_gyro; 
 
    delta_y_world = 
        dx_robot * sin_gyro + 
        dy_robot * cos_gyro; 
				 
				 
		    if (!fusion->initialized) 
    { 
        return; 
    } 
 
    fusion->x += delta_x_world; 
    fusion->y -= delta_y_world; 

} 