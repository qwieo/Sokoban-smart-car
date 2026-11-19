/* ======================================================================
 * >> 麦克纳姆轮运动学与控制结构
 * 【面试问答】
 * Q1: 麦克纳姆轮逆运动学公式？
 * A: 四轮目标速度（车体坐标）：1 轮 y-x+w*Kw，2 轮 y+x-w*Kw，3 轮 y-x-w*Kw，
 * A: 4 轮 y+x+w*Kw。x 前后、y 横移、w 自转；符号由轮组安装方向（O/X 型布置）和
 * A: 接线决定，M2/M3 取负就是修正接线方向。
 * ----------------------------------------------------------------------
 * Q1: Kw 是什么、如何确定？
 * A: 自转时轮速与角速度的比例，理论值 = 轮子到车体中心距离 / 轮半径，先按机械尺寸
 * A: 算，再实测原地自转修正，当前为 1.9。
 * ----------------------------------------------------------------------
 * Q1: 整体控制结构是怎样的？
 * A: 三级串级：最外环位置 PD（世界系，输出期望平移速度），并联网的航向环 PD（输出
 * A: 期望角速度），坐标变换到车体系后，最内环四轮轮速增量 PI 跟踪并输出 PWM。串级
 * A: 抗扰强、各环带宽可分开设计。
 * ----------------------------------------------------------------------
 * ====================================================================== */
#include "motor&motion.h"
#define Kw 1.9f
#define MAX_ANGLE_SPEED  2*PI   // rad/s
#define CONTROL_DT       0.005f  // 100Hz
extern volatile char back_home;
float ACCEL_LIMIT_TRIGGER_DMPS =8.f;
int16 M1,M2,M3,M4;
float max_step = MAX_ANGLE_SPEED * CONTROL_DT;
Odometry_t robot_pos = {0, 0, 0};

uint16_t	target_x;
uint16_t	target_y;

float vel_x_robot;
float vel_y_robot;
float vel_w_cmd;

extern  fuzzy_pd_t fuzzy_x;
extern  fuzzy_pd_t fuzzy_y;

//
float fclip(float x,float low,float up)
	{
    return x > up ? up : x < low ? low : x;
	}

int signf(float x)
{
    return (x > 0.0f) - (x < 0.0f);
}

static float quantize_angle_90(float angle)
{
    float q = roundf(angle / HALF_PI_F) * HALF_PI_F;

    return atan2f(sinf(q), cosf(q));
}


/* ======================================================================
 * >> 平移加速度限制
 * 【面试问答】
 * Q1: 为什么要做加速度限制？
 * A: 速度指令阶跃会使轮子打滑，打滑直接破坏里程计精度，还可能撞箱。做法：只在加速
 * A: 方向限制（减速不限制以保证制动快），把速度向量增量钳到 a_max*dt、方向不变按
 * A: 比例缩放，等效梯形速度规划。
 * ----------------------------------------------------------------------
 * ====================================================================== */
static void limit_translation_acceleration(
    float current_x, float current_y,
    float target_x, float target_y,
    float max_accel_dmps2, float dt,
    float *output_x, float *output_y)
{
    float current_speed;
    float target_speed;
    float speed_change;
    float delta_x;
    float delta_y;
    float delta_speed;
    float max_delta_speed;
    float scale;
    current_speed = sqrtf(current_x * current_x +
                          current_y * current_y);
    target_speed = sqrtf(target_x * target_x +
                         target_y * target_y);
    speed_change = fabsf(target_speed - current_speed);
    if (target_speed <= current_speed ||
        speed_change <= ACCEL_LIMIT_TRIGGER_DMPS)
    {
        *output_x = target_x;
        *output_y = target_y;
        return;
    }
    delta_x = target_x - current_x;
    delta_y = target_y - current_y;

    delta_speed = sqrtf(delta_x * delta_x +
                        delta_y * delta_y);
    max_delta_speed = max_accel_dmps2 * dt;

    if (delta_speed <= max_delta_speed)
    {
        *output_x = target_x;
        *output_y = target_y;
        return;
    }
    scale = max_delta_speed / delta_speed;
    *output_x = current_x + delta_x * scale;
    *output_y = current_y + delta_y * scale;
}


/* ======================================================================
 * >> 航向处理与"先转后走"
 * 【面试问答】
 * Q1: 目标航向为什么量化到 90 度？
 * A: 推箱子只沿网格四向运动，目标角量化到 0/90/180/270，避免追求无意义的中间角度；
 * A: 用 atan2(sin,cos) 把量化结果归一化到 [-PI,PI]。
 * ----------------------------------------------------------------------
 * Q1: 航向角为什么要 unroll 展开？
 * A: atan2 输出在正负 PI 处跳变，直接做差会把 359 度到 1 度误判成转 358 度。代码用
 * A: 相邻周期增量（归一化到正负 PI）累加出连续角度，保证转向走最短路径。
 * ----------------------------------------------------------------------
 * Q1: 为什么航向误差大于 0.08rad 时禁止平移？
 * A: "先转后走"：航向偏差大时世界系到车体系的旋转变换误差被放大，横移方向不可靠；
 * A: 先原地转向到位再平移，避免走斜线、蹭歪箱子。
 * ----------------------------------------------------------------------
 * Q1: smoothed_target_angle 平滑的作用？
 * A: 对目标角做斜坡，每周期步进不超过最大角速度*dt，把阶跃目标变成可跟随的转向轨迹，
 * A: 防止猛打方向。
 * ----------------------------------------------------------------------
 * Q1: 位置环输出为何限幅 8.9dm/s？停止死区有何用？
 * A: 限速保证机械安全且在内环可跟踪范围内；速度指令小于 0.05dm/s 直接清零，防止轮子
 * A: 在零点来回抖动。
 * ----------------------------------------------------------------------
 * ====================================================================== */
void mecanum_cascade_control(
    float raw_target_y,
    float raw_target_x,
    float raw_target_angle,
    float curr_y,
    float curr_x,
    float curr_angle)
{
    static float smoothed_target_angle = 0.0f;
    static float transform_angle = 0.0f;
    static int is_first_run = 1;
    static float filtered_vel_x_robot = 0.0f;
    static float filtered_vel_y_robot = 0.0f;

    static float last_curr_angle = 0.0f;
    static float curr_angle_unwrapped = 0.0f;

    float quantized_target_angle;
    float angle_err;
    float smooth_err;
    float angle_step_limit;
    float step;
    float delta_angle;

    float vel_y_world;
    float vel_x_world;

    float cos_theta;
    float sin_theta;

    float raw_vel_x_robot;
    float raw_vel_y_robot;

    quantized_target_angle =quantize_angle_90(raw_target_angle);

    if (is_first_run)
    {
        last_curr_angle =curr_angle;
        curr_angle_unwrapped =curr_angle;

        smoothed_target_angle =curr_angle_unwrapped;
        transform_angle =quantized_target_angle;

        is_first_run =0;
    }
    else
    {
        delta_angle =atan2f(sinf(curr_angle-last_curr_angle),
                            cosf(curr_angle-last_curr_angle));

        curr_angle_unwrapped +=delta_angle;
        last_curr_angle =curr_angle;
    }

    angle_err =raw_target_angle-curr_angle_unwrapped;

    if (fabsf(angle_err) <0.08f)
    {
        smoothed_target_angle =raw_target_angle;
        transform_angle =quantized_target_angle;
    }
    else
    {
        smooth_err =raw_target_angle-smoothed_target_angle;

        angle_step_limit =MAX_ANGLE_SPEED*CONTROL_DT;

        step =smooth_err*0.2f;

        step =fclip(step,
                   -angle_step_limit,
                   angle_step_limit);

        smoothed_target_angle +=step;
    }

    vel_w_cmd =PidLocCtrl(&gyro,
                          curr_angle_unwrapped,
                          smoothed_target_angle);

    if (fabsf(angle_err) >=0.08f)
    {
        filtered_vel_x_robot =0.0f;
        filtered_vel_y_robot =0.0f;

        vel_x_robot =0.0f;
        vel_y_robot =0.0f;

//        FuzzyPosPD_Reset(&fuzzy_x);
//        FuzzyPosPD_Reset(&fuzzy_y);

        return;
    }

//    vel_y_world =FuzzyPosPD_Update(&fuzzy_y,
//                                   raw_target_y-curr_y);
//
//    vel_x_world =FuzzyPosPD_Update(&fuzzy_x,
//                                   raw_target_x-curr_x);

    vel_y_world =PidLocCtrl(&Y,
                            curr_y,
                            raw_target_y);

    vel_x_world =PidLocCtrl(&X,
                            curr_x,
                            raw_target_x);

        vel_y_world =fclip(vel_y_world,
                           -8.9f,
                           8.9f);

        vel_x_world =fclip(vel_x_world,
                           -8.9f,
                           8.9f);


    cos_theta =cosf(transform_angle);
    sin_theta =sinf(transform_angle);

    raw_vel_x_robot =
        vel_x_world*cos_theta+
        vel_y_world*sin_theta;

    raw_vel_y_robot =
        -vel_x_world*sin_theta+
        vel_y_world*cos_theta;

    if (fabsf(raw_vel_x_robot) <
            TRANSLATION_STOP_EPS_DMPS &&
        fabsf(raw_vel_y_robot) <
            TRANSLATION_STOP_EPS_DMPS)
    {
        filtered_vel_x_robot =0.0f;
        filtered_vel_y_robot =0.0f;
    }
    else
    {
        limit_translation_acceleration(
            filtered_vel_x_robot,
            filtered_vel_y_robot,
            raw_vel_x_robot,
            raw_vel_y_robot,
            MAX_TRANSLATION_ACCEL_DMPS2,
            CONTROL_DT,
            &filtered_vel_x_robot,
            &filtered_vel_y_robot);
    }

    vel_x_robot =
        filtered_vel_x_robot;

    vel_y_robot =
        filtered_vel_y_robot;
}


/* ======================================================================
 * >> 四轮速度分配（mecanum_basic）
 * 【面试问答】
 * Q1: mecanum_basic 做什么？
 * A: 按逆运动学把车体三自由度速度拆成四轮目标转速（乘 ENCODER_PER_dM 换成脉冲/周期），
 * A: 各自跑增量式 PI，再按正负设置方向 GPIO 和 PWM 占空比。
 * ----------------------------------------------------------------------
 * ====================================================================== */
void mecanum_basic(float y,float x,float w)
{

	float i=PidIncCtrl(&w1,M1,ENCODER_PER_dM*(y-x+w*Kw));
	float j=PidIncCtrl(&w2,M2,ENCODER_PER_dM*(y+x-w*Kw));
	float k=PidIncCtrl(&w3,M3,ENCODER_PER_dM*(y-x-w*Kw));
	float l=PidIncCtrl(&w4,M4,ENCODER_PER_dM*(y+x+w*Kw));
	if(i>=0)
	{
		pwm_set_duty(mo1,(int)i);
		gpio_set_level(D0,0);
	}
	else
	{
		pwm_set_duty(mo1,-(int)i);
		gpio_set_level(D0,1);
	}


	if(j>=0)
	{
		pwm_set_duty(mo2,(int)j);
		gpio_set_level(D1,1);
	}
	else
	{
		pwm_set_duty(mo2,-(int)j);
		gpio_set_level(D1,0);
	}


	if(k>=0)
	{
		pwm_set_duty(mo3,(int)k);
		gpio_set_level(C8,1);
	}
	else
	{
		pwm_set_duty(mo3,-(int)k);
		gpio_set_level(C8,0);
	}
	if(l>=0)
	{
		pwm_set_duty(mo4,(int)l);
		gpio_set_level(C6,0);
	}
	else
	{
		pwm_set_duty(mo4,-(int)l);
		gpio_set_level(C6,1);
	}
}


/* ======================================================================
 * >> 电机驱动
 * 【面试问答】
 * Q1: PWM 为什么用 17kHz？
 * A: 高于人耳可听频段，避免电机啸叫，同时处于驱动 MOS 和电机感量的合适工作区间；
 * A: 改变占空比即调节平均电压调速，方向脚 + PWM 组成 H 桥正反转控制。
 * ----------------------------------------------------------------------
 * ====================================================================== */
void Motor_Init(void)
{
		pwm_init(mo1, 17000, 0);
		gpio_init(D0,GPO,GPIO_LOW,GPO_PUSH_PULL);
		pwm_init(mo2, 17000, 0);
    gpio_init(D1,GPO,GPIO_LOW,GPO_PUSH_PULL);
		pwm_init(mo3, 17000, 0);
		gpio_init(C8,GPO,GPIO_LOW,GPO_PUSH_PULL);
		pwm_init(mo4, 17000, 0);
		gpio_init(C6,GPO,GPIO_LOW,GPO_PUSH_PULL);
}

void encoder_init(void)
{
	encoder_dir_init(QTIMER1_ENCODER1,QTIMER1_ENCODER1_CH1_C0,QTIMER1_ENCODER1_CH2_C1);
	encoder_dir_init(QTIMER2_ENCODER1,QTIMER2_ENCODER1_CH1_C3,QTIMER2_ENCODER1_CH2_C25);//M3
	encoder_dir_init(QTIMER1_ENCODER2,QTIMER1_ENCODER2_CH1_C2,QTIMER1_ENCODER2_CH2_C24);//M2
	encoder_dir_init(QTIMER3_ENCODER2,QTIMER3_ENCODER2_CH1_B18,QTIMER3_ENCODER2_CH2_B19);//M1
}

/* ======================================================================
 * >> 编码器读取
 * 【面试问答】
 * Q1: 编码器怎么读？为什么读后清零？
 * A: QTIMER 工作在正交编码器模式，相位差自动判方向。每 5ms 读一次增量计数并清零，
 * A: 得到该周期脉冲数，用于轮速反馈和里程计；符号按接线统一到车体方向。
 * ----------------------------------------------------------------------
 * ====================================================================== */
int16 get_encoder(void)
{
	M1=encoder_get_count(QTIMER2_ENCODER1);
	encoder_clear_count(QTIMER2_ENCODER1);
	M2=-encoder_get_count(QTIMER3_ENCODER2);
	encoder_clear_count(QTIMER3_ENCODER2);
	M3=-encoder_get_count(QTIMER1_ENCODER1);
	encoder_clear_count(QTIMER1_ENCODER1);
	M4=encoder_get_count(QTIMER1_ENCODER2);
	encoder_clear_count(QTIMER1_ENCODER2);
	return (M1+M2+M3+M4)/4;
}
