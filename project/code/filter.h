/* ======================================================================
 * >> 滤波器与位姿历史（filter.h）
 * 【面试问答】
 * Q: GyroLPF2_t 为什么要保存 x1/x2/y1/y2？
 * A: 二阶 IIR 滤波器需要两级历史输入(x1/x2)和两级历史输出(y1/y2)，按直接 II 型
 * A: 差分递推。
 * ----------------------------------------------------------------------
 * Q: 位姿历史相关常量的含义？
 * A: MAX_RECORD 最多记录 128 个位姿；POSE_DISTANCE_TOLERANCE=15 为同一点判定半径；
 * A: ANGLE_TOL=0.05rad 为正航向判定容差。
 * ----------------------------------------------------------------------
 * ====================================================================== */
#ifndef __FILTER_H__
#define __FILTER_H__
#include <math.h>
#include "zf_common_headfile.h"
#define PI_2 (PI * 0.5f)
#define ANGLE_TOL 0.05f
#define MAX_RECORD 128
#define POSE_DISTANCE_TOLERANCE 15  
extern float gyrox_bias;
extern float gyroy_bias;
extern float gyroz_bias;

typedef struct
{
    float x1;
    float x2;
    float y1;
    float y2;
} GyroLPF2_t;

extern GyroLPF2_t gyro_lpf_x;
extern GyroLPF2_t gyro_lpf_y;
extern GyroLPF2_t gyro_lpf_z;

typedef struct 
{
	float x;
	float y;
	float z;
}Vector3f;

typedef struct 
{
	uint16_t x;
	uint16_t y;
	uint16_t z;
}Vector3u16;

extern	Vector3f Processed_gyro;
extern	Vector3u16 history[MAX_RECORD];

void GyroLPF2_Init(GyroLPF2_t *f);
float GyroLPF2_Update(GyroLPF2_t *f, float x);

void	Posture_Transition(Vector3f *vector);
void	calibrate(void);
void	Quaternion_Update(float gx, float gy, float gz, float dt);
bool 	allow_recognition(uint16_t x,uint16_t y,int8_t *dir_out);
void 	record_pose(uint16_t gx, uint16_t gy, int8_t dir);

void 	clear_pose_history(void);

#endif