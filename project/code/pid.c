/* ======================================================================
 * >> 位置式 / 增量式 PID
 * 【面试问答】
 * Q1: 位置式和增量式 PID 的区别？
 * A: 位置式输出与全部历史误差有关的全量；增量式只输出本周期增量 Δu、只依赖最近三次
 * A: 误差，再由执行端累加。
 * ----------------------------------------------------------------------
 * Q1: 轮速环为什么用增量式？
 * A: 增量式输出是对上次 PWM 的微调，切换模式/目标时冲击小，积分靠自然累加、不会出现
 * A: 位置式那样的大幅积分饱和，对电机这种天然记忆上一拍电压的执行机构更平滑。
 * ----------------------------------------------------------------------
 * Q1: 增量式公式？
 * A: Δu = Kp(e(k)-e(k-1)) + Ki*e(k) + Kd(e(k)-2e(k-1)+e(k-2))，再 u += Δu 并限幅。
 * ----------------------------------------------------------------------
 * ====================================================================== */
#include  "zf_common_headfile.h"
#include "pid.h"
#define  Max 	7000
extern volatile char back_home;
/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：pid增量式控制器输出
 *  参数说明：
  * @param    pid     pid参数
  * @param    error   pid输入误差
 *  函数返回：PID输出结果   注意输出结果已经包涵了上次结果
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float PidIncCtrl(pid_param_t * pid, float Encoder,float Target)
{

		pid->error=(float)(Target - Encoder);//当前误差
    pid->out_p = pid->kp * (pid->error - pid->last_error);

    pid->out_i = pid->ki * pid->error;
    pid->out_d = pid->kd * ((pid->error - 2*(pid->last_error)) + pid->last_derivative);

    pid->last_derivative = pid->error - pid->last_error;
    pid->last_error = pid->error;

    pid->out += pid->out_p + pid->out_i + pid->out_d;

    return pid->out=func_limit_ab(pid->out,-Max,Max);
}


/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：pid位置式控制器输出
 *  参数说明：
  * @param    pid     pid参数
  * @param    error   pid输入误差
 *  函数返回：PID输出结果
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
/* ======================================================================
 * >> 位置式 PD 的工程细节
 * 【面试问答】
 * Q1: D 项为什么对输入（测量值）微分而不是误差？
 * A: 这是微分先行。目标值经常阶跃，对误差微分会产生很大的微分尖峰（设定值 kick），
 * A: 对测量值微分可避免该冲击；给定不变时两种写法等价。
 * ----------------------------------------------------------------------
 * Q1: D 项为什么再串一阶滤波（alpha=0.3）？
 * A: 微分会放大编码器量化噪声，用一阶低通平滑微分项、抑制抖动；alpha 越小滤波越强，
 * A: 按噪声与响应折中。
 * ----------------------------------------------------------------------
 * Q1: 积分防饱和有哪些措施？
 * A: 误差小于 0.005 直接清积分器（同时防到位后的极限环振荡）；积分输出分量限幅正负 1；
 * A: 位置环干脆不用 I，消静差交给轮速内环的积分。
 * ----------------------------------------------------------------------
 * ====================================================================== */
float PidLocCtrl(pid_param_t *pid, float Encoder, float Target)
{
    float derivative_raw;
    float alpha = 0.3f;//D项滤波系数，越大越平滑，响应越慢
    pid->error = Target - Encoder;
    if (fabsf(pid->error) < 0.005f)
    {
        pid->error = 0.0f;
        pid->integrator = 0.0f;
    }
    pid->integrator += pid->error;
    pid->out_p = pid->kp * pid->error;
    pid->out_i = pid->ki * pid->integrator;
    pid->dinput = Encoder - pid->last_input;
    derivative_raw = pid->dinput;
    pid->last_derivative =
        alpha * pid->last_derivative +
        (1.0f - alpha) * derivative_raw;
    pid->out_d = -pid->kd * pid->last_derivative;
    pid->last_input = Encoder;
    pid->out = pid->out_p +func_limit_ab(pid->out_i, -1.0f, 1.0f) +pid->out_d;
    return pid->out;
}
void PID_Reset(pid_param_t *pid)
{
    pid->error=0;
    pid->last_error=0;
    pid->last_derivative=0;
    pid->last_t=0;

    pid->out=0;
    pid->out_p=0;
    pid->out_i=0;
    pid->out_d=0;
}

/* ======================================================================
 * >> 参数整定
 * 【面试问答】
 * Q1: PID 参数怎么整定？
 * A: 先内环后外环、先 P 后 D 再 I：轮速环先加 Kp 到响应迅速，加 Ki 消除静差；位置环
 * A: Kp 控制接近速度，Kd 压制超调振荡。全程用 VOFA+ 看阶跃响应，菜单在线改、Flash
 * A: 保存，每次改参做同样的阶跃测试对比。
 * ----------------------------------------------------------------------
 * Q1: Kp/Ki/Kd 作用一句话总结？
 * A: Kp 决定纠偏力度，Ki 消除稳态误差但带来超调，Kd 预测趋势、抑制超调但放大噪声。
 * ----------------------------------------------------------------------
 * ====================================================================== */
pid_param_t w1=
{
	.error=0,
	.kd=0,
	.ki=2.8740,
	.kp=24.1727,
	.last_derivative=0,
	.last_error=0,
	.out=0,
	.out_d=0,
	.out_i=0,
	.out_p=0
};
pid_param_t w2=
{
	.error=0,
	.kd=0,
	.ki=2.8740,
	.kp=24.1727,
	.last_derivative=0,
	.last_error=0,
	.out=0,
	.out_d=0,
	.out_i=0,
	.out_p=0
};
pid_param_t w3=
{
	.error=0,
	.kd=0,
	.ki=2.8740,
	.kp=24.1727,
	.last_derivative=0,
	.last_error=0,
	.out=0,
	.out_d=0,
	.out_i=0,
	.out_p=0
};
pid_param_t w4=
{
	.error=0,
	.kd=0,
	.ki=2.8740,
	.kp=24.1727,
	.last_derivative=0,
	.last_error=0,
	.out=0,
	.out_d=0,
	.out_i=0,
	.out_p=0
};

pid_param_t gyro=
{
	.error=0,
	.kd=40.0,
	.ki=0.00,
	.kp=10.2,
	.last_derivative=0,
	.last_error=0,
	.last_t=0,
	.out=0,
	.out_d=0,
	.out_i=0,
	.out_p=0
};

pid_param_t X=
{
	.error=0,
	.kp = 0.24f,
  .ki = 0.00,
  .kd = 18.0f,
	.last_derivative=0,
	.last_error=0,
	.last_t=0,
	.out=0,
	.out_d=0,
	.out_i=0,
	.out_p=0
};

pid_param_t Y=
{
	.error=0,
	.kp = 0.16,
  .ki = 0.00,
  .kd = 6.80f,
	.last_derivative=0,
	.last_error=0,
	.last_t=0,
	.out=0,
	.out_d=0,
	.out_i=0,
	.out_p=0
};

/*===========================================================
 *                    参数调节区域
 * 实际调车主要修改这里。
 *
 *==========================================================*/

/*===========================================================
 * 最大输出速度
 *==========================================================*/
#define FUZZY_MAX_OUTPUT_DMPS        9.0f

/*===========================================================
 * X轴误差论域
 *
 *
 * SMALL -------- MEDIUM ---------------- LARGE
 *
 *   0              4cm                    20cm
 * |error| = 0：
 *      完全属于小误差
 *
 * |error| = 4：
 *      完全属于中误差
 *
 * |error| >= 20：
 *      完全属于大误差
 *==========================================================*/



/*===========================================================
 * X轴接近速度论域
 *
 *
 * SLOW -------- MEDIUM ---------------- FAST
 *   0            1.5 dm/s               5 dm/s
 *                  ↓                       ↓
 *                15 cm/s                50 cm/s
 *==========================================================*/


/*===========================================================
 * Y轴误差论域
 *==========================================================*/

/*===========================================================
 * Y轴接近速度论域
 *==========================================================*/


/*===========================================================
 * X轴 Kp 规则表
 *
 *                    接近目标速度
 *
 *                慢       中       快
 *             --------------------------
 * 小误差         0.17     0.14     0.10
 * 中误差         0.24     0.20     0.15
 * 大误差         0.36     0.30     0.22
 *
 *
 * 行：
 *      位置误差大小
 *
 * 列：
 *      接近目标速度
 *
 *
 * 规则思想：
 *
 * 1. 误差越大：
 *
 *      Kp 增大
 *
 *    让车辆更积极地向目标移动。
 *
 *
 * 2. 接近目标速度越快：
 *
 *      Kp 减小
 *
 *    防止高速接近目标时 P 项仍然很强。
 *
 *
 * 例如：
 *
 * 大误差 + 慢：
 *      0.36
 *
 * 小误差 + 快：
 *      0.10
 *==========================================================*/
#define FUZZY_X_ERROR_MID_CM         10.0f
#define FUZZY_X_ERROR_HIGH_CM        60.0f
#define FUZZY_X_SPEED_MID_DMPS       2.0f
#define FUZZY_X_SPEED_HIGH_DMPS      6.0f
#define FUZZY_Y_ERROR_MID_CM         10.0f
#define FUZZY_Y_ERROR_HIGH_CM        60.0f
#define FUZZY_Y_SPEED_MID_DMPS       2.0f
#define FUZZY_Y_SPEED_HIGH_DMPS      6.0f
static const float fuzzy_kp_x
    [FUZZY_SET_NUM][FUZZY_SET_NUM] =
{
    /*                  慢       中       快      */
    /* 小误差 */       {0.26f,  0.24f,  0.20f},
    /* 中误差 */       {0.28f,  0.26f,  0.24f},
    /* 大误差 */       {0.35f,  0.30f,  0.28f}
};
static const float fuzzy_kd_x
    [FUZZY_SET_NUM][FUZZY_SET_NUM] =
{
    /*                  慢        中        快       */
    /* 小误差 */       {0.150f,  0.180f,  0.200f},
    /* 中误差 */       {0.130f,  0.150f,  0.180f},
    /* 大误差 */       {0.025f,  0.050f,  0.100f}
};
static const float fuzzy_kp_y
    [FUZZY_SET_NUM][FUZZY_SET_NUM] =
{
    /*                  慢       中        快       */
    /* 小误差 */       {0.18f,  0.16f,   0.15f},
    /* 中误差 */       {0.18f,  0.18f,   0.18f},
    /* 大误差 */       {0.24f,  0.20f,   0.18f}
};
static const float fuzzy_kd_y
    [FUZZY_SET_NUM][FUZZY_SET_NUM] =
{
    /*                  慢        中        快       */
    /* 小误差 */       {0.052f,  0.057f,  0.062f},
    /* 中误差 */       {0.052f,  0.052f,  0.052f},
    /* 大误差 */       {0.010f,  0.020f,  0.040f}
};
/*===========================================================
 * X轴 Kd 规则表
 *
 *                    接近目标速度
 *
 *                慢        中        快
 *             -----------------------------
 * 小误差         0.040     0.050     0.060
 * 中误差         0.030     0.045     0.060
 * 大误差         0.015     0.030     0.050
 *
 *
 * 规则思想：
 * 误差大、速度慢：
 *      D小
 *      避免D项妨碍加速。
 *
 * 接近目标速度越快：
 *      D增大
 *      提高阻尼。
 *
 * 小误差、速度快：
 *      D最大
 *      防止快速冲过目标。
 *==========================================================*/

/*===========================================================
 * Y轴 Kp 规则表
 *                    接近目标速度
 *                慢       中       快
 *             --------------------------
 * 小误差         0.12     0.10     0.075
 * 中误差         0.18     0.15     0.11
 * 大误差         0.28     0.23     0.17
 * Y轴整体 Kp 比 X轴低。
 *==========================================================*/


/*===========================================================
 * Y轴 Kd 规则表
 *                    接近目标速度
 *                慢        中        快
 *             -----------------------------
 * 小误差         0.038     0.048     0.058
 * 中误差         0.028     0.043     0.058
 * 大误差         0.014     0.028     0.048
 *==========================================================*/


/*===========================================================
 * 限幅函数
 *==========================================================*/
static float fuzzy_clamp(
    float value,
    float low,
    float high)
{
    if (value < low)
    {
        return low;
    }

    if (value > high)
    {
        return high;
    }

    return value;
}


/*===========================================================
 * 符号函数
 *
 * > 0  -> +1
 *
 * < 0  -> -1
 *
 * = 0  -> 0
 *==========================================================*/
static float fuzzy_sign(
    float value)
{
    if (value > 0.0f)
    {
        return 1.0f;
    }

    if (value < 0.0f)
    {
        return -1.0f;
    }

    return 0.0f;
}


/*===========================================================
 * 三模糊集合隶属度计算
 *
 *
 *       SMALL         MEDIUM          LARGE
 *
 * μ
 *
 * 1 ─────\            /\           /────
 *         \          /  \         /
 *          \        /    \       /
 *           \      /      \     /
 * 0 ─────────\────/────────\───/────────
 *              mid           high
 *
 *
 * 输出：
 *
 * weight[FUZZY_SMALL]
 * weight[FUZZY_MEDIUM]
 * weight[FUZZY_LARGE]
 *
 *
 * 三个隶属度之和始终为 1。
 *==========================================================*/
/* ======================================================================
 * >> 模糊 PD：隶属度
 * 【面试问答】
 * Q1: 模糊 PD 解决什么问题？
 * A: 固定 PD 参数在"远处要快、近处要稳"之间顾此失彼。模糊 PD 按位置误差（小/中/大）
 * A: 和接近目标速度（慢/中/快）在线改变 Kp/Kd：大误差给大 Kp 快速逼近，接近目标且
 * A: 速度快时收小 Kp、靠 Kd 制动，本质是带连续插值的分段变增益 PD。
 * ----------------------------------------------------------------------
 * Q1: 隶属度函数怎么设计？
 * A: 三角形隶属度，相邻模糊集线性重叠且权重和恒为 1，切换平滑无跳变；超出端点即完全
 * A: 隶属端点集合。
 * ----------------------------------------------------------------------
 * ====================================================================== */
static void fuzzy_get_membership(float value,const fuzzy_domain_t *domain,
																	float weight[FUZZY_SET_NUM])
{
    float mid;
    float high;

    mid = domain->mid_peak;
    high = domain->high_peak;
    /*=======================================================
     * 0及以下
     *
     * 完全属于 SMALL
     *======================================================*/
    if (value <= 0.0f)
    {
        weight[FUZZY_SMALL] =1.0f;
        weight[FUZZY_MEDIUM]=0.0f;
        weight[FUZZY_LARGE] =0.0f;
    }

    /*=======================================================
     * 0 ~ mid
     *
     * SMALL逐渐下降
     *
     * MEDIUM逐渐上升
     *======================================================*/
    else if (value < mid)
    {
        weight[FUZZY_SMALL] =(mid - value) /mid;
        weight[FUZZY_MEDIUM] =value /mid;
        weight[FUZZY_LARGE] =0.0f;
    }
    /*=======================================================
     * mid ~ high
     *
     * MEDIUM逐渐下降
     *
     * LARGE逐渐上升
     *======================================================*/
    else if (value < high)
    {
        weight[FUZZY_SMALL] =0.0f;
        weight[FUZZY_MEDIUM] =(high - value)/(high - mid);
        weight[FUZZY_LARGE] =(value - mid) /(high - mid);
    }
    /*=======================================================
     * >= high
     *
     * 完全属于 LARGE
     *======================================================*/
    else
    {
        weight[FUZZY_SMALL] =0.0f;
        weight[FUZZY_MEDIUM] =0.0f;
        weight[FUZZY_LARGE] =1.0f;
    }
}


/*===========================================================
 * 清除动态状态
 *
 * 不改变规则和论域。
 *==========================================================*/
static void fuzzy_clear_state(fuzzy_pd_t *controller)
{
    controller->last_error_cm =0.0f;
    controller->filtered_error_rate_cm_s =0.0f;
    controller->last_output_dmps =0.0f;
    controller->initialized =0U;
}

/*===========================================================
 * 初始化
 *==========================================================*/
void FuzzyPosPD_Init(fuzzy_pd_t *controller,fuzzy_axis_t axis)
{
    if (controller == NULL)
    {
        return;
    }
    controller->axis =
        axis;
    /*=======================================================
     * 控制周期
     *
     * 100Hz
     *======================================================*/
    controller->dt_s =
        0.01f;
    /*=======================================================
     * 最大速度
     *
     * 8dm/s = 80cm/s
     *======================================================*/
    controller->max_output_dmps =
        FUZZY_MAX_OUTPUT_DMPS;
    /*=======================================================
     * D项滤波
     *
     * tau = 0.03s
     *
     * dt = 0.01s
     *
     * alpha =
     *
     *     0.01
     * -------------
     * 0.03 + 0.01
     *
     * = 0.25
     *======================================================*/
    controller->derivative_tau_s =
        0.03f;
    /*=======================================================
     * 目标跳变保护
     *======================================================*/
    controller->target_jump_cm =
        5.0f;
    /*=======================================================
     * X / Y 独立论域
     *======================================================*/
    if (axis == FUZZY_AXIS_Y)
    {
        controller->error_domain.mid_peak =FUZZY_Y_ERROR_MID_CM;
        controller->error_domain.high_peak =FUZZY_Y_ERROR_HIGH_CM;
        controller->speed_domain.mid_peak = FUZZY_Y_SPEED_MID_DMPS;
        controller->speed_domain.high_peak =FUZZY_Y_SPEED_HIGH_DMPS;
    }
    else
    {
        controller->error_domain.mid_peak = FUZZY_X_ERROR_MID_CM;
        controller->error_domain.high_peak =FUZZY_X_ERROR_HIGH_CM;
        controller->speed_domain.mid_peak =FUZZY_X_SPEED_MID_DMPS;
        controller->speed_domain.high_peak =FUZZY_X_SPEED_HIGH_DMPS;
    }

    fuzzy_clear_state(controller);
}


/*===========================================================
 * Reset
 *==========================================================*/
void FuzzyPosPD_Reset(fuzzy_pd_t *controller)
{
    if (controller == NULL)
    {
        return;
    }
    fuzzy_clear_state(controller);
}


/*===========================================================
 * 模糊规则推理
 *
 * 输入：
 *
 * error_weight：
 *      误差 小/中/大 隶属度
 *
 * speed_weight：
 *      接近速度 慢/中/快 隶属度
 *
 *
 * 输出：
 *
 * kp
 * kd
 *
 *
 * 使用二维加权插值：
 *
 * weight(i,j)
 *
 * =
 *
 * μ_error(i) × μ_speed(j)
 *
 *
 * Kp =
 *
 * Σ weight(i,j) × Kp_rule(i,j)
 *
 *
 * Kd =
 *
 * Σ weight(i,j) × Kd_rule(i,j)
 *==========================================================*/
/* ======================================================================
 * >> 模糊推理与解模糊
 * 【面试问答】
 * Q1: 模糊推理与解模糊怎么做？
 * A: 规则激活权重 = 误差隶属度 x 速度隶属度，对 3x3 规则表参数做加权平均（重心法
 * A: 思想）直接得到 Kp/Kd，省去 max-min 运算，计算量小适合单片机。
 * ----------------------------------------------------------------------
 * ====================================================================== */
static void fuzzy_calculate_gain(const fuzzy_pd_t *controller,
																 const float error_weight[FUZZY_SET_NUM],
																 const float speed_weight[FUZZY_SET_NUM],
																 float *kp,float *kd)
{
    const float (*kp_table)[FUZZY_SET_NUM];
    const float (*kd_table)[FUZZY_SET_NUM];
    int error_level;
    int speed_level;
    /*=======================================================
     * 选择 X / Y 规则表
     *======================================================*/
    if (controller->axis == FUZZY_AXIS_Y)
    {
        kp_table =fuzzy_kp_y;
        kd_table =fuzzy_kd_y;
    }
    else
    {
        kp_table =fuzzy_kp_x;
        kd_table =fuzzy_kd_x;
    }
		*kp =0.0f;
    *kd =0.0f;
    /*=======================================================
     * 3×3 模糊规则推理
     *======================================================*/
    for (error_level = 0;error_level < FUZZY_SET_NUM;++error_level)
    {
        for (speed_level = 0;speed_level < FUZZY_SET_NUM;++speed_level)
        {
            float rule_weight;
            rule_weight =error_weight[error_level] *speed_weight[speed_level];
            *kp +=rule_weight *kp_table[error_level][speed_level];
            *kd +=rule_weight *kd_table[error_level][speed_level];
        }
    }
}


/*===========================================================
 * 模糊 PD 主函数
 *
 *
 *              位置误差 error
 *                    │
 *                    │
 *           ┌────────┴────────┐
 *           │                 │
 *           ▼                 ▼
 *       |error|             de/dt
 *           │                 │
 *           │              低通滤波
 *           │                 │
 *           │                 ▼
 *           │          接近目标速度
 *           │                 │
 *           ▼                 ▼
 *       误差模糊化         速度模糊化
 *           │                 │
 *           └────────┬────────┘
 *                    │
 *                    ▼
 *                3×3规则表
 *                    │
 *              ┌─────┴─────┐
 *              ▼           ▼
 *             Kp           Kd
 *              │           │
 *              └─────┬─────┘
 *                    ▼
 *
 *      output = Kp*error + Kd*de/dt
 *
 *                    │
 *                    ▼
 *                  限幅
 *
 *==========================================================*/
/* ======================================================================
 * >> 模糊 PD 更新细节
 * 【面试问答】
 * Q1: target_jump_cm 跳变检测的作用？
 * A: 目标点切换时误差突变会产生巨大虚假微分，检测到单拍变化超过 5cm 就复位微分历史，
 * A: 避免 D 项冲击。
 * ----------------------------------------------------------------------
 * Q1: 位置环输出速度指令，为什么不加 I？
 * A: 内环轮速 PI 已能消除静差；位置环加 I 容易在到位附近积分过量导致来回推箱。误差
 * A: 死区清零也避免在零点反复调整。
 * ----------------------------------------------------------------------
 * ====================================================================== */
float FuzzyPosPD_Update(
    fuzzy_pd_t *controller,
    float error_cm)
{
    float error_weight[FUZZY_SET_NUM];
    float speed_weight[FUZZY_SET_NUM];
    float error_sign;
    float raw_error_rate_cm_s;
    float alpha;
    float approach_speed_dmps;
    float fuzzy_speed_input_dmps;
    float kp;
    float kd;
    float output_dmps;
    /*=======================================================
     * 参数检查
     *======================================================*/
    if (controller == NULL)
    {
        return 0.0f;
    }

    if (!isfinite(error_cm))
    {
        fuzzy_clear_state(
            controller);

        return 0.0f;
    }

    /*=======================================================
     * 误差方向
     *======================================================*/
    error_sign =fuzzy_sign(error_cm);
    /*=======================================================
     * 1. 计算误差变化率
     *======================================================*/
    if (controller->initialized == 0U)
    {
        /*
         * 第一次进入，没有历史值。
         *
         * D项直接初始化为0，
         * 防止第一次出现巨大微分。
         */
        controller->initialized =1U;
        controller->last_error_cm =error_cm;
        controller->filtered_error_rate_cm_s =0.0f;
    }


    else if (fabs(error_cm-controller->last_error_cm)>
        controller->target_jump_cm)
    {
        /*
         * 如果目标发生明显跳变，
         *
         * 不让这个跳变进入D项。
         */
        controller->last_error_cm =
            error_cm;
        controller->filtered_error_rate_cm_s =
            0.0f;
    }
    else
    {
        /*---------------------------------------------------
         * 原始误差变化率
         *
         *              e(k) - e(k-1)
         * de/dt = -------------------------
         *                    dt
         *
         * 单位：
         *
         * cm/s
         *--------------------------------------------------*/
        raw_error_rate_cm_s =
            (error_cm -
             controller->last_error_cm)
            /
            controller->dt_s;
        /*---------------------------------------------------
         * 一阶低通
         *
         *             dt
         * alpha = -----------
         *           tau + dt
         *--------------------------------------------------*/
        alpha =
            controller->dt_s /
            (controller->derivative_tau_s +
             controller->dt_s);
        controller->filtered_error_rate_cm_s +=
            alpha *
            (raw_error_rate_cm_s -
             controller->filtered_error_rate_cm_s);
        controller->last_error_cm =
            error_cm;
    }
    /*=======================================================
     * 2. 计算接近目标速度
     *
     *
     * approach_speed
     *
     * =
     *
     * -sign(error) × de/dt
     *
     *
     * 最后 /10：
     *
     * cm/s -> dm/s
     *
     *
     * approach_speed > 0：
     *
     *      正在接近目标
     *
     *
     * approach_speed < 0：
     *
     *      正在远离目标
     *======================================================*/
    approach_speed_dmps =
        -error_sign *
        controller->filtered_error_rate_cm_s
        /
        10.0f;
    /*=======================================================
     * 模糊速度论域只定义：
     *
     * 慢 / 中 / 快
     *
     * 所以负的接近速度统一当作0。
     *
     * 即：
     *
     * 如果正在远离目标，
     * 将其视为“慢”。
     *======================================================*/
    if (approach_speed_dmps > 0.0f)
    {
        fuzzy_speed_input_dmps =
            approach_speed_dmps;
    }
    else
    {
        fuzzy_speed_input_dmps =
            0.0f;
    }
		
    /*=======================================================
     * 3. 位置误差模糊化
     *======================================================*/
    fuzzy_get_membership(fabsf(error_cm),&controller->error_domain,
													error_weight);
    /*=======================================================
     * 4. 接近速度模糊化
     *======================================================*/
    fuzzy_get_membership(fuzzy_speed_input_dmps,
				&controller->speed_domain,speed_weight);
    /*=======================================================
     * 5. 模糊推理
     *
     * 根据 X/Y 的规则表得到：
     *
     * Kp
     * Kd
     *======================================================*/
    fuzzy_calculate_gain(controller,error_weight,speed_weight,&kp,&kd);

    /*=======================================================
     * 6. 纯 PD
     *
     *
     *              de
     * u = Kp*e + Kd*--
     *              dt
     *
     *
     * 当正在接近目标时：
     *
     * 如果 error > 0
     *
     * 通常：
     *
     * de/dt < 0
     *
     * 所以：
     *
     * D项自然与P项方向相反。
     *
     * 这就是PD本身的阻尼作用，
     * 没有额外加入任何制动逻辑。
     *======================================================*/
    output_dmps =
        kp *
        error_cm
        +
        kd *
        controller->filtered_error_rate_cm_s;


    /*=======================================================
     * 7. 纯输出限幅
     *
     * 这只是执行器限幅，
     * 不属于制动策略。
     *======================================================*/
		if(!back_home)
			output_dmps = fuzzy_clamp(output_dmps,-controller->max_output_dmps,
															 controller->max_output_dmps);
		else
			output_dmps = fuzzy_clamp(output_dmps,-15.f,15.f);
		
    controller->last_output_dmps = output_dmps;
    return output_dmps;
}