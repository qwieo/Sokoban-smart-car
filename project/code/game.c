

#include <math.h>
#include <stdbool.h>
#include "zf_common_headfile.h"
#include "motor&motion.h"
#define MOVE_STEP_CM  20.0f                                
#define ROT_STEP_RAD  PI/2.f       
#define POS_TOLERANCE 1.0f

float auto_target_y;
float auto_target_x;
float target_angle;
extern float gyroz;
      
