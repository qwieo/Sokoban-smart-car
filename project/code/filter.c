#include "zf_common_headfile.h"
/* ======================================================================
 * >> 陀螺滤波与数据链路
 * 【面试问答】
 * Q1: 陀螺仪数据处理的完整链路？
 * A: 原始角速度 -> 安装角坐标变换 -> 扣零偏 -> 二阶巴特沃斯低通 -> 死区置零 -> 四元数
 * A: 积分更新姿态 -> 输出 roll/pitch/yaw。
 * ----------------------------------------------------------------------
 * Q1: 为什么选二阶巴特沃斯、截止 20Hz？
 * A: 车体真实运动是低频、陀螺噪声主要在高频；巴特沃斯通带最平坦，二阶兼顾衰减斜率与
 * A: 相位滞后。系数按采样 200Hz、截止 20Hz 用 MATLAB butter(2,20/100) 生成，写成
 * A: 常数按直接 II 型差分实现。
 * ----------------------------------------------------------------------
 * ====================================================================== */
#include "filter.h"
#include <float.h> 
#include <string.h>

Vector3u16 history[MAX_RECORD];

Vector3f Processed_gyro;

extern float gyrox,gyroy,gyroz;

uint8_t hist_cnt = 0;
float Gx,Gy,Gz;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float	gyrox_bias,gyroy_bias,gyroz_bias;
float install_roll,install_pitch;

void GyroLPF2_Init(GyroLPF2_t *f)
{
    f->x1 = 0.0f;
    f->x2 = 0.0f;
    f->y1 = 0.0f;
    f->y2 = 0.0f;
}

GyroLPF2_t gyro_lpf_x;
GyroLPF2_t gyro_lpf_y;
GyroLPF2_t gyro_lpf_z;
/*
 * Butterworth 
 * Fs = 200 Hz
 * Fc = 20 Hz
 */
float GyroLPF2_Update(GyroLPF2_t *f, float x)
{
    const float b0 = 0.06745527f;
    const float b1 = 0.13491055f;
    const float b2 = 0.06745527f;
    const float a1 = -1.14298050f;
    const float a2 = 0.41280160f;
    float y =
          b0 * x
        + b1 * f->x1
        + b2 * f->x2
        - a1 * f->y1
        - a2 * f->y2;
    f->x2 = f->x1;
    f->x1 = x;
    f->y2 = f->y1;
    f->y1 = y;
    return y;
}

/* ======================================================================
 * >> 快速逆平方根
 * 【面试问答】
 * Q1: 快速逆平方根的原理？
 * A: 利用 IEEE754 浮点数位布局，用魔法数 0x5f3759df 得到很接近的初值，再做一次牛顿
 * A: 迭代精化，比当年标准库开方快很多（Quake 经典算法）。M7 本身有硬件开方指令，
 * A: 这里属于沿用，1/sqrtf 也足够快。
 * ----------------------------------------------------------------------
 * ====================================================================== */
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Quaternion_Init(float roll, float pitch, float yaw) 
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;
}

/* ======================================================================
 * >> 四元数姿态解算
 * 【面试问答】
 * Q1: 四元数姿态解算的原理？
 * A: 用一阶方法积分四元数微分方程 dq/dt = 0.5*(q 叉乘 ω)（角速度先换算成 rad/s），再由
 * A: 四元数反解欧拉角。
 * ----------------------------------------------------------------------
 * Q1: 为什么每步都要归一化？
 * A: 单位四元数才表示纯旋转，积分截断误差会让模长漂移，不补偿姿态会缓慢失真；归一化
 * A: 用快速逆平方根、开销很小。
 * ----------------------------------------------------------------------
 * Q1: yaw 为什么要连续化？
 * A: 与航向 unroll 同理，把每拍 yaw 增量归一化到正负 PI 内累加，消除正负 PI 跳变，
 * A: 输出连续航向供控制和里程计使用。
 * ----------------------------------------------------------------------
 * Q1: 纯陀螺 yaw 的缺点？如何弥补？
 * A: 积分航向长期会漂移（零偏温漂）。本项目用开机零偏 + 温度补偿 + 低通保证短期精度，
 * A: 位置又有视觉绝对校正；若需长期绝对航向可加磁力计。
 * ----------------------------------------------------------------------
 * ====================================================================== */
void Quaternion_Update(float gx, float gy, float gz, float dt)
{
    float wx = gx * (PI / 180.0f);
    float wy = gy * (PI / 180.0f);
    float wz = gz * (PI / 180.0f);

    float q0_old = q0;
    float q1_old = q1;
    float q2_old = q2;
    float q3_old = q3;

    float half_dt = 0.5f * dt;
    q0 += half_dt * (-q1_old * wx - q2_old * wy - q3_old * wz);
    q1 += half_dt * ( q0_old * wx + q2_old * wz - q3_old * wy);
    q2 += half_dt * ( q0_old * wy - q1_old * wz + q3_old * wx);
    q3 += half_dt * ( q0_old * wz + q1_old * wy - q2_old * wx);

    float norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    gyrox = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)); 
    gyroy = asinf(2.0f * (q0 * q2 - q3 * q1));                                    
    
    float raw_yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)); 

    static float prev_raw_yaw = 0.0f;
    static int is_initialized = 0;

    if (!is_initialized) {
        gyroz = raw_yaw;
        prev_raw_yaw = raw_yaw;
        is_initialized = 1;
    } else {
        float delta_yaw = raw_yaw - prev_raw_yaw;
        
        if (delta_yaw < -PI) {
            delta_yaw += 2.0f * PI;  
        } else if (delta_yaw > PI) {
            delta_yaw -= 2.0f * PI;  
        }
        
        gyroz += delta_yaw;
        prev_raw_yaw = raw_yaw;
    }
}




//
int8_t quantize_direction(float angle)
{
    while (angle >= PI)  angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;

    if (fabsf(angle) < ANGLE_TOL)
        return 0;
    if (fabsf(angle - PI_2) < ANGLE_TOL)
        return 1;
    if (fabsf(fabsf(angle) - PI) < ANGLE_TOL)
        return 2;
    if (fabsf(angle + PI_2) < ANGLE_TOL)
        return 3;
    return -1;
}



bool is_same_as_history(uint16_t x, uint16_t y, int8_t dir)
{
    for (uint8_t i = 0; i < hist_cnt; i++)
    {
        int32_t dx = (int32_t)x - (int32_t)history[i].x;
        int32_t dy = (int32_t)y - (int32_t)history[i].y;

        if (abs(dx) <= POSE_DISTANCE_TOLERANCE &&
            abs(dy) <= POSE_DISTANCE_TOLERANCE &&
            history[i].z == dir)
        {
            return true;
        }
    }

    return false;
}

void record_pose(uint16_t x, uint16_t y, int8_t dir)
{
    if (hist_cnt < MAX_RECORD)
    {
        history[hist_cnt].x = x;
        history[hist_cnt].y = y;
        history[hist_cnt].z = dir;
        hist_cnt++;
    }
}


void clear_pose_history(void)
{
    memset(history, 0, sizeof(history));
    hist_cnt = 0;
}

/* ======================================================================
 * >> 识别位姿去重
 * 【面试问答】
 * Q1: allow_recognition / 位姿历史的作用？
 * A: 识别只在四个正朝向上触发（角度容差 0.05rad），且同一位置（距离容差 15）同一
 * A: 朝向只允许一次，靠历史数组去重，防止重复计数和斜向误识别。
 * ----------------------------------------------------------------------
 * ====================================================================== */
bool allow_recognition(uint16_t x,uint16_t y,int8_t *dir_out)
{

    int8_t dir = quantize_direction(gyroz);
    if (dir < 0)
        return false;
    if (is_same_as_history(x, y, dir))
        return false;
    *dir_out = dir;
    return true;
}


void Posture_Transition(Vector3f * vector)
	{
		float cos_p = cosf(install_roll);
    float sin_p = sinf(install_roll);
    float cos_r = cosf(install_pitch);
    float sin_r = sinf(install_pitch);

    float x = vector->x;
    float y = vector->y;
    float z = vector->z;

    vector->x = x * cos_p+ z * sin_p;
    vector->y = x * sin_r * sin_p + y * cos_r - z * sin_r * cos_p;
    vector->z = -x * cos_r * sin_p + y * sin_r + z * cos_r * cos_p;
	}	
	
/* ======================================================================
 * >> 开机校准
 * 【面试问答】
 * Q1: 开机校准做了什么？为什么采 400 次？
 * A: 先静止采 400 次加速度求均值，用重力向量反解安装倾角；再采 400 次陀螺求均值作为
 * A: 三轴零偏。400 次平均把随机噪声压到约 1/20，2 秒采样时间也可接受。
 * ----------------------------------------------------------------------
 * Q1: 安装角校准的原理？
 * A: 静止时加速度计只测重力，由重力在各轴投影反解 IMU 相对车体的倾斜：
 * A: roll=atan2(Ay,Az)，pitch=atan2(-Ax,sqrt(Ay*Ay+Az*Az))；Posture_Transition 再按
 * A: 这两个角把 IMU 数据旋转到车体坐标系，解决安装不水平问题。
 * ----------------------------------------------------------------------
 * ====================================================================== */
void calibrate(void)
{
	for(uint16_t i=0;i<400;i++)
	{
		imu660rb_get_acc();
		Gx+=imu660rb_acc_transition(imu660rb_acc_x);
		Gy+=imu660rb_acc_transition(imu660rb_acc_y);
		Gz+=imu660rb_acc_transition(imu660rb_acc_z);
		system_delay_ms(5);
	}
	Gx/=400;
	Gy/=400;
	Gz/=400;
	install_roll  = atan2(Gy, Gz); 
	install_pitch = atan2(-Gx, sqrt(Gy*Gy + Gz*Gz)); 
	for(uint16_t i=0;i<400;i++)
	{
		imu660rb_get_gyro();
		Processed_gyro.x=imu660rb_gyro_transition(imu660rb_gyro_x);
		Processed_gyro.y=imu660rb_gyro_transition(imu660rb_gyro_y);
		Processed_gyro.z=imu660rb_gyro_transition(imu660rb_gyro_z);
		Posture_Transition(&Processed_gyro);
		gyrox_bias +=Processed_gyro.x;
		gyroy_bias +=Processed_gyro.y;
		gyroz_bias +=Processed_gyro.z;
		system_delay_ms(5);
	}
		gyrox_bias /= 400;
		gyroy_bias /= 400;
		gyroz_bias /= 400;
		Quaternion_Init(0.0f,0.0f,0.0f);

}
