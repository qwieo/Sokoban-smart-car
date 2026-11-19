/* ======================================================================
 * >> 关卡与阶段定义
 * 【面试问答】
 * Q1: Game_level / Game_phase 的作用？
 * A: 两级状态枚举：关卡 1/2/3，每关内 Idle/Explore/Feedback/Upload/Start_push/
 * A: Completed 六个阶段，主循环 switch 驱动，状态显式、便于扩展新关卡。
 * ----------------------------------------------------------------------
 * Q1: MOVE_STEP_CM / ROT_STEP_RAD 的含义？
 * A: 场地网格步距 20cm、单次转向 90 度，是动作指令的基本单位，与地图格子一一对应。
 * ----------------------------------------------------------------------
 * ====================================================================== */
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
      
