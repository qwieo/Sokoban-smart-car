/* ======================================================================
 * >> PID / 模糊 PD 数据结构（pid.h）
 * 【面试问答】
 * Q: pid_param_t 中各字段的用途？
 * A: kp/ki/kd 为参数；error 与 out_p/out_i/out_d 为当前误差和三个分项；integrator 为
 * A: 积分累加；last_input/dinput 支持对测量值微分；last_error/last_derivative 保存
 * A: 历史误差；out 为最终输出。
 * ----------------------------------------------------------------------
 * Q: fuzzy_pd_t 的关键字段？
 * A: error_domain/speed_domain 是两个输入的模糊论域端点；derivative_tau_s 是微分滤波
 * A: 时间常数；target_jump_cm 是目标跳变阈值；initialized 防止首拍产生巨大微分。
 * ----------------------------------------------------------------------
 * ====================================================================== */
#ifndef __PID_H__
#define __PID_H__
#include "zf_common_headfile.h"
#include "math.h"
typedef struct
{
    float                kp;         //P
    float                ki;         //I
    float                kd;         //D
    float                imax;       //积分限幅
		float     					error;
    float                out_p;  //KP输出
    float                out_i;  //KI输出
    float                out_d;  //KD输出
    float                out;    //pid输出;

		float last_input;
		float dinput;

    float                integrator; //< 积分值
    float                last_error; //< 上次误差
    float                last_derivative;//< 上次误差与上上次误差之差
    unsigned long        last_t;     //< 上次时间
}pid_param_t;
//incremental
float PidIncCtrl(pid_param_t * pid, float Encoder,float Target);
//positional
float PidLocCtrl(pid_param_t * pid, float Encoder,float Target);
void PID_Reset(pid_param_t *pid);

extern pid_param_t w1;
extern pid_param_t w2;
extern pid_param_t w3;
extern pid_param_t w4;
extern pid_param_t gyro;
extern pid_param_t Y;
extern pid_param_t X;

#define FUZZY_SET_NUM    3


/*===========================================================
 * 模糊语言变量
 *
 * 对误差：
 *
 * SMALL  -> 小误差
 * MEDIUM -> 中误差
 * LARGE  -> 大误差
 *
 * 对接近速度：
 *
 * SMALL  -> 慢
 * MEDIUM -> 中
 * LARGE  -> 快
 *==========================================================*/
typedef enum
{
    FUZZY_SMALL = 0,
    FUZZY_MEDIUM,
    FUZZY_LARGE

} fuzzy_level_t;


/*===========================================================
 * 控制轴
 *==========================================================*/
typedef enum
{
    FUZZY_AXIS_X = 0,
    FUZZY_AXIS_Y

} fuzzy_axis_t;


/*===========================================================
 * 三模糊集合论域
 * mid_peak：
 *      MEDIUM 的峰值
 *
 * high_peak：
 *      从这里开始完全属于 LARGE
 *==========================================================*/
typedef struct
{
    float mid_peak;
    float high_peak;

} fuzzy_domain_t;


/*===========================================================
 * 模糊 PD 控制器
 *==========================================================*/
typedef struct
{
    /* 控制轴 */
    fuzzy_axis_t axis;
    /*-------------------------------------------------------
     * 基础参数
     *------------------------------------------------------*/
    /* 控制周期，单位 s */
    float dt_s;
    /* 最大输出速度，单位 dm/s */
    float max_output_dmps;
    /*-------------------------------------------------------
     * 模糊论域
     *------------------------------------------------------*/
    /* |error| 论域，单位 cm */
    fuzzy_domain_t error_domain;
    /* 接近目标速度论域，单位 dm/s */
    fuzzy_domain_t speed_domain;
    /*-------------------------------------------------------
     * D项
     *------------------------------------------------------*/
    /* D项低通滤波时间常数，单位 s */
    float derivative_tau_s;

    /*
     * 目标误差突然跳变超过该值，
     * 清除D项历史，避免微分冲击。
     *
     * 单位 cm
     */
    float target_jump_cm;
    /*-------------------------------------------------------
     * 动态状态
     *------------------------------------------------------*/
    /* 上一次误差，cm */
    float last_error_cm;
    /* 滤波后的误差变化率，cm/s */
    float filtered_error_rate_cm_s;
    /* 上一次输出，dm/s */
    float last_output_dmps;
    /* 是否已经初始化 */
    uint8_t initialized;

} fuzzy_pd_t;
/*===========================================================
 * 初始化
 *==========================================================*/
void FuzzyPosPD_Init(
    fuzzy_pd_t *controller,
    fuzzy_axis_t axis);
/*===========================================================
 * 清除历史状态
 *==========================================================*/
void FuzzyPosPD_Reset(
    fuzzy_pd_t *controller);


/*===========================================================
 * 模糊位置PD
 *
 * 输入：
 *
 * error_cm = target - current
 *
 * 输出：
 *
 * 速度指令，单位 dm/s
 *==========================================================*/
float FuzzyPosPD_Update(
    fuzzy_pd_t *controller,
    float error_cm);


#endif
