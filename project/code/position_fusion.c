/* ======================================================================
 * >> 位置融合策略
 * 【面试问答】
 * Q1: 位置融合的整体策略？
 * A: 编码器里程计做高频短时递推（平滑但累积漂移），摄像头绝对位置做低频校正（无漂移
 * A: 但有抖动/延迟），用带门限的互补加权融合：残差小时 alpha=0.707，检测到打滑
 * A: （残差大于阈值）时 alpha=1 完全采信视觉。
 * ----------------------------------------------------------------------
 * Q1: 里程计是如何计算的？
 * A: 四轮速度平均得纵向、对角组合得横向（麦克纳姆正运动学），乘标定系数 0.9 修正，
 * A: 再按当前航向旋转到世界坐标系累加；符号与场地坐标系对齐。
 * ----------------------------------------------------------------------
 * Q1: 为什么必须有视觉校正？
 * A: 打滑、轮胎变形、地面不平使轮式里程计误差单调累积且无法自恢复；视觉给出无漂移的
 * A: 绝对位置，两者互补。
 * ----------------------------------------------------------------------
 * Q1: 这是卡尔曼滤波吗？为什么不用 EKF？
 * A: 不是，是带野值剔除和打滑门限的互补加权。参数直观、运算量小、现场好验证，精度已
 * A: 满足任务；EKF 需要建立噪声模型并做矩阵运算，本场景收益有限。
 * ----------------------------------------------------------------------
 * ====================================================================== */
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
 
    /* ======================================================================
     * >> 视觉野值剔除与权重
     * 【面试问答】
     * Q1: 视觉野值如何剔除？
     * A: 残差超过 max_camera_jump 判为异常跳变先拒绝；但连续 3 次异常就接受并把位置重置
     * A: 到视觉值——既防单次误识别把位置拉飞，又防真实大位移（被推动、严重打滑后重
     * A: 定位）被永久拒绝。
     * ----------------------------------------------------------------------
     * Q1: alpha 取值的含义？
     * A: 视觉修正权重，越大越信视觉。0.707 是平滑与纠偏的经验折中；打滑后里程计已不可
     * A: 信，直接取 1。
     * ----------------------------------------------------------------------
     * ====================================================================== */
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
 
    /* ======================================================================
     * >> 里程计标定常量
     * 【面试问答】
     * Q1: ENCODER_COUNT_PER_PIXEL 和 ODOM_SCALE 怎么来？
     * A: 120 脉冲对应场地坐标系 1 像素位移，由减速比、编码器线数、轮径和场地比例标定；
     * A: ODOM_SCALE=0.9 是实测修正系数，让里程计位移与视觉位移对齐。
     * ----------------------------------------------------------------------
     * ====================================================================== */
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