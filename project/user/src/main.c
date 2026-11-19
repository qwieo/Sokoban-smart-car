/* ======================================================================
 * >> 项目整体与启动流程（main.c）
 * 【面试问答】
 * Q1: 请整体介绍一下这个项目。
 * A: 这是一个基于 NXP i.MX RT1064（Cortex-M7，主频 600MHz，逐飞开源库 + Keil MDK）的
 * A: 四轮麦克纳姆轮推箱子机器人，共三个关卡、难度递增。RT1064 经 UART1 与上位机/AI
 * A: 板交互（接收 12x16 地图、目标点和过关等标志，回传动作指令），经 UART4 与识别
 * A: 模块交互获取箱子种类。软件分三层：5ms 定时中断跑实时感知与闭环；主循环跑关卡
 * A: 状态机与通信；菜单 + Flash 负责现场调参与参数掉电保存。
 * ----------------------------------------------------------------------
 * Q1: 上电后的完整启动流程是什么？
 * A: 时钟配到 600MHz -> OLED/按键/蜂鸣器初始化 -> IMU 静止校准（安装倾角 + 陀螺零偏）
 * A: -> Flash 读回上次参数 -> 进入菜单调参，退出时写 Flash -> 初始化编码器、电机、
 * A: 双串口、位置融合与模糊控制器，启动 5ms 周期中断 -> 主循环状态机按关卡运行。
 * ----------------------------------------------------------------------
 * Q1: 为什么不使用 RTOS？
 * A: 任务数量少、实时性要求明确：一个固定 200Hz 的闭环加一个非实时状态机。裸机中断
 * A: + 状态机占用资源小、时序可预测、调试简单；上 RTOS 反而带来调度抖动、栈空间和
 * A: 优先级管理的额外复杂度。
 * ----------------------------------------------------------------------
 * Q1: 主循环和中断如何分工？
 * A: 中断只做严格周期、耗时确定的事：读编码器/陀螺、姿态解算、位置融合、位置/航向/
 * A: 轮速三级控制、更新 PWM；主循环做地图解析、推箱子求解、串口收发应答、关卡切换等
 * A: 非实时逻辑。共享标志用 volatile，主循环不能阻塞中断。
 * ----------------------------------------------------------------------
 * Q1: 系统里有哪些周期任务？
 * A: PIT_CH0 每 10ms 按键扫描消抖；PIT_CH3 每 5ms 跑完整控制环（200Hz）；串口数据
 * A: 由接收中断异步处理；OLED 只在按键事件时局部刷新。
 * ----------------------------------------------------------------------
 * Q1: 怎么保证控制环的实时性？
 * A: 硬件 PIT 中断触发，优先级低于串口接收（避免丢字节）、高于普通任务；中断内禁止
 * A: delay 和串口打印等阻塞操作；浮点用 M7 硬件 FPU；中断末尾 __DSB 保证外设写入完成。
 * ----------------------------------------------------------------------
 * ====================================================================== */
#include "zf_common_headfile.h"
#include "vofa.h"
#include "motor&motion.h"
#include "filter.h"
#include "math.h"
#include "game.h"
#include "sokoban_types1.h"
#include "sokoban_solver1.h"
#include "sokoban_types2.h"
#include "sokoban_solver2.h"
#include "sokoban_types3.h"
#include "sokoban_solver3.h"
#include "sokoban_map1.h"
#include "sokoban_workspace.h"
#include <stdio.h>
#include "stdlib.h"
#include "art.h"
#include "position_fusion.h"
#include "menu.h"
#include "flash.h"

PositionFusion_t position_fusion;
CameraMedianFilter_t camera_median_filter;

uint8 num_box2=6;
uint8 reg_times2=11;
uint8 num_box3=6;
uint8 reg_times3=11;
volatile uint16_t camera_x_raw ;
volatile uint16_t camera_y_raw ;

volatile uint8_t camera_new_frame;

float estimated_x ;
float estimated_y ;
float estimated_gyroz ;
extern float residual_x;
extern float residual_y;

float gyrox,gyroy,gyroz;
float	dgyrox,dgyroy,dgyroz;
float new_gyro_x;
float new_gyro_y;
float new_gyro_z;

char map_strings[12][16];
char vision_stream[20];
uint8_t stream_counter;
////////////////////////////////////
char variety;
char last_variety;
volatile char art0ack;
volatile char regpoint;
volatile char level_passed;
volatile char back_home;
char art1sendvariety;
int8_t dir;
char locked;

fuzzy_pd_t fuzzy_x;
fuzzy_pd_t fuzzy_y;

/*  private function */
void PositionControl_Init(void)
{
    FuzzyPosPD_Init(&fuzzy_x,FUZZY_AXIS_X);
    FuzzyPosPD_Init(&fuzzy_y,FUZZY_AXIS_Y);
}

void update_map2(void)
{
	for (int i = 0; i < expected_cnt2; i++) 
	{
			int tx = expected_targets2[i].x;
			int ty = expected_targets2[i].y;
			map_strings[ty][tx] = vision_stream[i];
	}
}

void update_map3(void)
{
	for (int i = 0; i < expected_cnt3; i++) 
	{
			int tx = expected_targets3[i].x;
			int ty = expected_targets3[i].y;
			map_strings[ty][tx] = vision_stream[i];
	}
}

/* ======================================================================
 * >> 指令发送协议
 * 【面试问答】
 * Q1: Polling_transmit 的帧格式是什么？为什么字节间延时 100us？
 * A: 帧头 0xFF + 命令字符串 + CRC8 + 帧尾 0xFE。字节间加 100us 间隔是配合对端的接收
 * A: 处理能力、降低连续发送丢字节概率；CRC8 保证命令传错不会被执行。
 * ----------------------------------------------------------------------
 * Q1: while(cmd_received) 阻塞发送的风险？
 * A: 这是停等式发送：发一帧等对端应答（cmd_received 被清零）才继续，期间 5ms 控制
 * A: 中断照常运行，小车安全。风险是对端宕机会永久死等，工程上应加超时计数后退出并
 * A: 报警。
 * ----------------------------------------------------------------------
 * ====================================================================== */
void Polling_transmit(char * commands)
{
	uart_write_byte(UART_1,255);
	for(uint8_t i=0;commands[i]!=0;i++)
	{
		uart_write_byte(UART_1,commands[i]);
		system_delay_us(100);
	}
	uart_write_byte(UART_1,cal_crc8(commands,get_length_to_zero(commands)));
	uart_write_byte(UART_1,254);
}

/* ======================================================================
 * >> 位置估计任务
 * 【面试问答】
 * Q1: 位置估计任务做了什么？
 * A: 先用编码器和航向做里程计递推（高频短时预测）；当摄像头产生新一帧绝对位置时做
 * A: 一次融合校正（低频纠偏），输出 estimated_x/y 给位置环。
 * ----------------------------------------------------------------------
 * ====================================================================== */
void PositionEstimator_Task_5ms(void)
{
    /*
     * 编码器里程预测。
     */
    PositionFusion_UpdateOdometry(&position_fusion,M1,M2,M3,M4,gyroz);

    /*
     * 有新摄像头帧时才校正一次。
     */
    if (camera_new_frame)
    {

        camera_new_frame = 0;

        PositionFusion_CorrectCamera(
            &position_fusion,
            camera_x_raw,
            camera_y_raw);
    }
		
    estimated_x = position_fusion.x;
    estimated_y = position_fusion.y;
}

#define GYRO_TEMP_KX    0.02523f
#define GYRO_TEMP_KY    0.03037f
#define GYRO_TEMP_KZ   -0.01549f

float gyro_bias_temperature = 0.0f;
float gyro_temperature = 0.0f;
void Gyro_SetBiasTemperature(float temperature)
{
    gyro_bias_temperature = temperature;
}

/* ======================================================================
 * >> 陀螺仪温度补偿
 * 【面试问答】
 * Q1: 陀螺温度补偿的原理？
 * A: 陀螺零偏随温度变化。先记录基准温度，实验拟合三轴零偏-温度系数 Kx/Ky/Kz；运行
 * A: 时按 bias = K*(T-T0) 计算当前零偏并从角速度中扣除，抑制长时间温漂。
 * ----------------------------------------------------------------------
 * ====================================================================== */
void Gyro_TemperatureCompensate(float temperature,
                                float *gx, float *gy, float *gz)
{
    float dT = temperature - gyro_bias_temperature;

    *gx -= GYRO_TEMP_KX * dT;
    *gy -= GYRO_TEMP_KY * dT;
    *gz -= GYRO_TEMP_KZ * dT;
}

#define BUZZER_PWM   PWM1_MODULE3_CHB_B11

static void buzzer_note(uint32_t freq, uint32_t time_ms)
{
    pwm_init(BUZZER_PWM, freq, PWM_DUTY_MAX / 2);

    system_delay_ms(time_ms * 85 / 100);

    pwm_set_duty(BUZZER_PWM, 0);

    system_delay_ms(time_ms * 15 / 100);
}

/* ======================================================================
 * >> 蜂鸣器音乐
 * 【面试问答】
 * Q1: 蜂鸣器如何播放音乐？
 * A: 无源蜂鸣器用 PWM 驱动：音符音高对应 PWM 频率（频率表按十二平均律标定），音符
 * A: 时值对应发声时长；每个音符 95% 时间发声、5% 静音形成断奏，音符之间清晰可分。
 * ----------------------------------------------------------------------
 * ====================================================================== */
void buzzer_fur_elise(void)
{
    buzzer_note(659, 140);    // E5
    buzzer_note(622, 140);    // D#5
    buzzer_note(659, 140);    // E5
    buzzer_note(622, 140);    // D#5
    buzzer_note(659, 140);    // E5

//    buzzer_note(494, 140);    // B4
//    buzzer_note(587, 140);    // D5
//    buzzer_note(523, 140);    // C5

//    buzzer_note(440, 300);    // A4
}

#include "zf_common_headfile.h"

/*========================================================
 * 蜂鸣器
 *========================================================*/
#define BUZZER_PWM          PWM1_MODULE3_CHB_B11
#define BUZZER_DUTY         (PWM_DUTY_MAX / 2)

/*
 * 整体速度
 *
 * 八分音符 = 230ms
 *
 * 想慢：
 * 230 -> 240 -> 250
 *
 * 想快：
 * 230 -> 220 -> 210
 */
#define EIGHTH_MS           230

#define E                   (EIGHTH_MS)        // 八分音符
#define Q                   (EIGHTH_MS * 2)    // 四分音符
#define DQ                  (EIGHTH_MS * 3)    // 附点四分音符
#define H                   (EIGHTH_MS * 4)    // 二分音符

/*
 * 每个音最后留一点空隙。
 * 主要为了让连续相同音符能够听出节奏。
 */
#define NOTE_GATE_PERCENT   95


/*========================================================
 * 音高
 *
 * 保留上一版降低后的音高。
 *
 * LL = 更低八度
 * L  = 低音
 * N  = 中音
 * H  = 高音
 *========================================================*/

/* 更低音 */
#define LL6     139

/* 低音 */
#define L1      165
#define L2      185
#define L3      208
#define L4      220
#define L5      247
#define L6      277
#define L7      311

/* 中音 */
#define N1      330
#define N2      370
#define N3      415
#define N4      440
#define N5      494
#define N6      554
#define N7      622

/* 高音 */
#define H1      659
#define H2      740
#define H3      831
#define H4      880
#define H5      988
#define H6      1109
#define H7      1245

#define REST    0


typedef struct
{
    uint16_t freq;
    uint16_t time;
} MusicNote_t;


/*========================================================
 * 单音播放
 *========================================================*/
static void Buzzer_Note(
    uint16_t freq,
    uint16_t time)
{
    uint32_t sound_time;
    uint32_t silence_time;

    if(freq == REST)
    {
        pwm_set_duty(
            BUZZER_PWM,
            0);

        system_delay_ms(time);

        return;
    }

    /*
     * 设置对应音高
     */
    pwm_init(
        BUZZER_PWM,
        freq,
        BUZZER_DUTY);

    sound_time =
        (uint32_t)time *
        NOTE_GATE_PERCENT /
        100;

    silence_time =
        (uint32_t)time -
        sound_time;

    /*
     * 发声
     */
    system_delay_ms(
        sound_time);

    /*
     * 短暂断音
     */
    pwm_set_duty(
        BUZZER_PWM,
        0);

    if(silence_time > 0)
    {
        system_delay_ms(
            silence_time);
    }
}


/*========================================================
 * 《采石》前奏
 *
 * 重点：
 *
 * 这次不再所有音都用同一个时值。
 *
 * 6/8拍每小节 = 6个八分音符
 *
 * Q  = 2个八分音符
 * E  = 1个八分音符
 * DQ = 3个八分音符
 * H  = 4个八分音符
 *========================================================*/
static const MusicNote_t caishi_intro[] =
{
    /*====================================================
     * 第一行
     *====================================================*/

    /*
     * | 低6  6  高1  高5 |
     *
     * 2 + 1 + 2 + 1 = 6
     */
    {L6, Q},
    {N6, E},
    {H1, Q},
    {H5, E},

    /*
     * | 高3  低4  6. |
     *
     * 2 + 1 + 3 = 6
     */
    {H3, Q},
    {L4, E},
    {N6, DQ},

    /*
     * | 高3  1 6 高1 高2 |
     *
     * 2 + 1 + 1 + 1 + 1 = 6
     */
    {H3, Q},
    {N1, E},
    {N6, E},
    {H1, E},
    {H2, E},

    /*
     * | 低3  5 低3 6 5 |
     */
    {L3, Q},
    {N5, E},
    {L3, E},
    {N6, E},
    {N5, E},

    /*
     * | 低6  3 高1 6 |
     *
     * 2 + 1 + 2 + 1
     */
    {LL6, Q},
    {N3, E},
    {H1, Q},
    {N6, E},

    /*
     * | 2 5 6 6 6 |
     *
     * 前四个八分音符，
     * 最后一个四分音符
     */
    {N2, E},
    {N5, E},
    {N6, E},
    {N6, E},
    {N6, Q},

    /*
     * | 低6 3 高1 6 |
     */
    {LL6, Q},
    {N3, E},
    {H1, Q},
    {N6, E},


    /*====================================================
     * 第二行
     *====================================================*/

    /*
     * | 2 5 6 6 低6 低1 |
     *
     * 六个八分音符
     */
    {N2, E},
    {N5, E},
    {N6, E},
    {N6, E},
    {L6, E},
    {L1, E},

    /*
     * | 低2 1 4 5 6 |
     */
    {L2, Q},
    {N1, E},
    {N4, E},
    {N5, E},
    {N6, E},

    /*
     * | 低3 5 低3 6 5 |
     */
    {L3, Q},
    {N5, E},
    {L3, E},
    {N6, E},
    {N5, E},

    /*
     * | 低低6 6 高1 高2 6 |
     */
    {LL6, Q},
    {N6, E},
    {H1, E},
    {H2, E},
    {N6, E},

    /*
     * | 高1 6 3 5 6 |
     */
    {H1, Q},
    {N6, E},
    {N3, E},
    {N5, E},
    {N6, E},

    /*
     * | 低2 1 4 5 6 |
     */
    {L2, Q},
    {N1, E},
    {N4, E},
    {N5, E},
    {N6, E},

    /*
     * | 低3 5 低3 6 5 |
     */
    {L3, Q},
    {N5, E},
    {L3, E},
    {N6, E},
    {N5, E},


    /*====================================================
     * 第三行
     *====================================================*/

    /*
     * | 低6 6 高3 6 3 |
     */
    {LL6, Q},
    {N6, E},
    {H3, E},
    {N6, E},
    {N3, E},

    /*
     * | 高2 6 3 高1 6 |
     */
    {H2, Q},
    {N6, E},
    {N3, E},
    {H1, E},
    {N6, E},

    /*
     * | 低2 1 4 5 6 |
     */
    {L2, Q},
    {N1, E},
    {N4, E},
    {N5, E},
    {N6, E},

    /*
     * | 低3 5 低3 6 5 |
     */
    {L3, Q},
    {N5, E},
    {L3, E},
    {N6, E},
    {N5, E},

    /*
     * | 低低6 6 6 7 |
     *
     * 2 + 1 + 1 + 2
     */
    {LL6, Q},
    {N6, E},
    {N6, E},
    {L7, Q},

    /*
     * | 1 5 1 低6 |
     *
     * 2 + 2 + 1 + 1
     */
    {N1, Q},
    {N5, Q},
    {N1, E},
    {L6, E},

    /*
     * | 低2 1 4 5 6 |
     */
    {L2, Q},
    {N1, E},
    {N4, E},
    {N5, E},
    {N6, E},

    /*
     * | 低3 5 - |
     *
     * 低3 = 四分音符
     * 5后面的 "-" 延长一个四分音符
     *
     * 所以：
     * 低3 = 2
     * 5   = 4
     */
    {L3, Q},
    {N5, H}
};


#define CAISHI_INTRO_NOTE_NUM \
    (sizeof(caishi_intro) / sizeof(caishi_intro[0]))


/*========================================================
 * 初始化
 *========================================================*/
void Buzzer_Init(void)
{
    pwm_init(
        BUZZER_PWM,
        1000,
        0);
}


/*========================================================
 * 停止
 *========================================================*/
void Buzzer_Stop(void)
{
    pwm_set_duty(
        BUZZER_PWM,
        0);
}


/*========================================================
 * 播放《采石》前奏
 *
 * 阻塞式
 *========================================================*/
void Buzzer_Play_Caishi(void)
{
    uint16_t i;

    for(i = 0;
        i < CAISHI_INTRO_NOTE_NUM;
        i++)
    {
        Buzzer_Note(
            caishi_intro[i].freq,
            caishi_intro[i].time);
    }

    Buzzer_Stop();
}
/*  private function end */
int main(void)
{
  clock_init(SYSTEM_CLOCK_600M);  // 不可删除
	system_delay_ms(300);
//	wireless_uart_init();
    // 此处编写用户代码 例如外设初始化代码等
	ips200_init(IPS200_TYPE_SPI);	
	Buzzer_Init();
	key_init(10);
	
	interrupt_global_enable(0);
	interrupt_set_priority(PIT_IRQn,5);
	pit_ms_init(PIT_CH0,10);
	
	
	ips200_show_string(0,0,"System Booting...");	
	imu660rb_init();
	calibrate();
//	Buzzer_Play_Caishi();
	buzzer_fur_elise();
	Buzzer_Stop();
	
	flash_init();
	Flash_Read();
	App_MenuInit();
  Menu_Run_Main();
	Write_Flash();
	pit_disable(PIT_CH0);
  ips200_clear();
  ips200_show_string(0,0,"Motion Starting...");
  system_delay_ms(300);
	
	encoder_init();
	Motor_Init();
	ART_init();	
	PositionControl_Init();
	pit_ms_init(PIT_CH3,5);
	
	PositionFusion_Init(&position_fusion);
	Game_level=1;
	frame_ready=0;
	ips200_clear();
	
	

#if 1
  while(1)
  {
//		JF_Data.data[0] =estimated_x;
//		JF_Data.data[1] =estimated_y;
//		JF_Data.data[2] =target_x;
//		JF_Data.data[3] =target_y;
//		JustFloat_Send();
//		if(back_home)
//		{
//			position_fusion.camera_enable = 0;
//		}
//		else
//		{
//			position_fusion.camera_enable = 1;
//		}
		/* ======================================================================
		 * >> 主状态机与识别策略
		 * 【面试问答】
		 * Q1: 主状态机是怎么组织的？
		 * A: 两层 switch：外层 Game_level 是关卡（Game1/2/3），内层 Game_phase 是阶段（Idle
		 * A: 收图与规划、Explore 探索识别、Feedback 应答、Upload 回填地图并规划推箱、Start_push
		 * A: 执行、Completed 清场进下一关）。
		 * ----------------------------------------------------------------------
		 * Q1: 为什么识别次数是 2*箱子数-1？
		 * A: 每个箱子要从两个相对方向各识别一次做交叉验证，防止遮挡和朝向造成误识别；N 个
		 * A: 箱子共 2N 个方向，初始方向无需识别，所以是 2N-1 次。
		 * ----------------------------------------------------------------------
		 * ====================================================================== */
		switch (Game_level)
		{
			case Game1:
				switch(Game_phase)
				{
					case Idle:
						level_passed=0;
						while(!frame_ready);
						update_map_from_uart(map_strings,uart_buffer);
						parse_map_with_vision1(map_strings);
						solve_sokoban1();
						cmd_received=1;
						while(cmd_received)
						{
							Polling_transmit(final_commands1);
						}
						Game_phase=Start_push;
						break;
					case Start_push:
						if(level_passed)
						{
							Game_phase=Completed;
							back_home=0;
						}
						break;			
					case Completed:
						Game_level=Game2;
						frame_ready=0;
						Game_phase=Idle;
						clear_sokoban_memory1();
						break;
					default :
						Game_phase=Idle;
						break;
				}
				break;
				
///////////////////////////////////////////		
			case Game2:
				switch (Game_phase)
				{
					case Idle:

					if (frame_ready == 0)
					{
						break;
					}
					update_map_from_uart(map_strings, uart_buffer);

					frame_ready = 0;

					level_passed = 0;
					stream_counter = 0;
					locked = 0;
					art1sendvariety = 0;
					regpoint = 0;
					/* ======================================================================
					 * >> 第二关 探索-识别 握手
					 * 【面试问答】
					 * Q1: 探索-识别阶段的完整握手流程？
					 * A: 到识别点（对端发 233，regpoint=1）后 MCU 持续发 222 请求识别 -> 识别模块经
					 * A: UART4 返回 1 字节种类 -> MCU 进入 Feedback 发 169 应答，对端发 196 清除握手 ->
					 * A: 重复到规定次数 -> Upload 阶段按记录位姿把种类填回地图，再规划推箱动作。
					 * ----------------------------------------------------------------------
					 * Q1: record_pose / allow_recognition 的作用？
					 * A: 识别时记录当时的位置和朝向，并保证同一位置同一朝向只识别一次（历史去重）；识别
					 * A: 必须在 0/90/180/270 正朝向上，斜向不触发，保证结果能正确对应到地图格子。
					 * ----------------------------------------------------------------------
					 * ====================================================================== */
					plan_explore_phase2(map_strings);

					cmd_received = 1;

					while (cmd_received)
					{
						Polling_transmit(explore_commands2);
					}

					Game_phase = Explore;
					break;
					case Explore:
						if(regpoint&&!art1sendvariety)
						{
							while(!art1sendvariety)
							uart_write_byte(UART_4,222);
							
						}
						else if(art1sendvariety&&regpoint)
						{
							if (!locked)
							{
								if (allow_recognition(estimated_x, estimated_y, &dir))
								{
									locked = 1;
									record_pose(estimated_x, estimated_y, dir);
								}					
							}
							if(locked)
							{	
								vision_stream[stream_counter++]=variety;
								Game_phase=Feedback;//after 5 times ensure,turn to state feedback ,send ack to art 4
								locked=0;
								last_variety=variety;
							}
							art1sendvariety=0;
						}

						break;
					case Feedback:
						art0ack = 1;
						while (art0ack)
						uart_write_byte(UART_1, 169);
						
						regpoint = 0;
						locked = 0;
						art1sendvariety = 0;

						if (stream_counter >= reg_times2)
						{
							Game_phase = Upload;
						}
						else
						{
							Game_phase = Explore;
						}
						break;

					case Upload:
						update_map2();
						plan_push_phase2(map_strings);
						cmd_received=1;
						while(cmd_received)
						{
							Polling_transmit(push_commands2);
						}
						Game_phase=Start_push;
						break;
					case Start_push:
						if(level_passed)
						{Game_phase=Completed;
							back_home=0;
						}
						break;
					case Completed:
						Game_level=Game3;
						frame_ready=0;
						Game_phase=Idle;
						clear_pose_history();
						clear_sokoban_memory2();
						break;						
					default:
						Game_phase=Idle;
						break;
				}
				
				break;		
				
				
///////////////////////////////							
			case Game3:
				switch (Game_phase)
				{
					case Idle:

					if (frame_ready == 0)
					{
						break;
					}
					update_map_from_uart(map_strings, uart_buffer);

					frame_ready = 0;

					level_passed = 0;
					stream_counter = 0;
					locked = 0;
					art1sendvariety = 0;
					regpoint = 0;
					plan_bomb_explore_phase3(map_strings);

					cmd_received = 1;

					while (cmd_received)
					{
						Polling_transmit(bomb_explore_commands3);
						system_delay_ms(1);
					}

					Game_phase = Explore;
					break;
					case Explore:
						if(regpoint&&!art1sendvariety)
						{
							while(!art1sendvariety)
							uart_write_byte(UART_4,222);
							
						}
						else if(art1sendvariety&&regpoint)
						{
							if (!locked)
							{
								if (allow_recognition(estimated_x, estimated_y, &dir))
								{
									locked = 1;
									record_pose(estimated_x, estimated_y, dir);
								}					
							}
							if(locked)
							{	
								vision_stream[stream_counter++]=variety;
								Game_phase=Feedback;//after 5 times ensure,turn to state feedback ,send ack to art 4
								last_variety=variety;								
							}
							art1sendvariety=0;
						}

						break;
					case Feedback:
						art0ack = 1;
						while (art0ack)
						uart_write_byte(UART_1, 169);
						
						regpoint = 0;
						locked = 0;
						art1sendvariety = 0;

						if (stream_counter>=reg_times3)
						{
							Game_phase = Upload;
						}
						else
						{
							Game_phase = Explore;
						}
						break;
					case Upload:
						update_map3();
						plan_push_phase3(map_strings);	// 2
						cmd_received=1;
						while(cmd_received)
						{
							Polling_transmit(level3_commands3);
						}
						Game_phase=Start_push;
						break;
					case Start_push:
						if(level_passed)
						{
							Game_phase=Completed;
							back_home=0;
						}
						break;
					case Completed:
						frame_ready=0;
						clear_pose_history();
						return 0;
						break;						
					default:
						Game_phase=Idle;
						break;
				}
				break;
		}
	system_delay_ms(5);

        // 此处编写需要循环执行的代码
}
	#endif
	return 0;
}
#define STATIC_COUNT_MAX   100      // 5ms * 200 = 1s
#define BIAS_ALPHA         0.005f

static uint16_t gyro_static_cnt = 0;

/* ======================================================================
 * >> 陀螺在线零偏
 * 【面试问答】
 * Q1: 在线零偏更新的原理？为什么最终注释掉？
 * A: 四个轮子速度都小于 2 个脉冲并持续 1 秒即判定静止，再用 alpha=0.005 的一阶低通
 * A: 缓慢修正零偏以跟踪温漂。但静止判定有误判风险（小车被挡住等），运动中误更新会
 * A: 把真实角速度当零偏扣掉；现场验证后选择"开机校准 + 温度补偿"，故注释保留。
 * ----------------------------------------------------------------------
 * ====================================================================== */
void Gyro_Bias_OnlineUpdate(void)
{
    if(fabsf(M1) < 2&&
       fabsf(M2) < 2 &&
       fabsf(M3) < 2 &&
       fabsf(M4) < 2)
    {
        if(gyro_static_cnt < STATIC_COUNT_MAX)
        {
            gyro_static_cnt++;
        }
    }
    else
    {
        gyro_static_cnt = 0;
    }

    if(gyro_static_cnt >= STATIC_COUNT_MAX)
    {
        gyrox_bias += BIAS_ALPHA * (Processed_gyro.x - gyrox_bias);
        gyroy_bias += BIAS_ALPHA * (Processed_gyro.y - gyroy_bias);
        gyroz_bias += BIAS_ALPHA * (Processed_gyro.z - gyroz_bias);
    }
}

/* ======================================================================
 * >> 5ms 实时中断
 * 【面试问答】
 * Q1: 5ms 中断里的执行顺序？
 * A: 读编码器增量 -> 读陀螺 -> 安装角坐标变换 -> 扣零偏 -> 二阶低通 -> 死区置零 ->
 * A: 四元数更新姿态 -> 位置估计（里程计 + 视觉校正）-> 位置/航向级联控制输出车体
 * A: 速度 -> 四轮轮速增量 PI -> PWM 输出。
 * ----------------------------------------------------------------------
 * Q1: 中断执行超过 5ms 会怎样？如何测量耗时？
 * A: 会造成周期抖动、下一拍来不及执行，控制性能下降甚至丢节拍。每步都是 O(1) 运算、
 * A: 不放任何阻塞代码；调试时翻转一个空闲 GPIO，用示波器测高电平宽度即中断实际耗时。
 * ----------------------------------------------------------------------
 * Q1: 角速度死区（0.13/0.14）的作用？
 * A: 静止时陀螺噪声会产生微小非零读数，直接积分会让航向缓慢漂移；小于阈值直接置零，
 * A: 配合零偏扣除保证静止时姿态稳定。
 * ----------------------------------------------------------------------
 * ====================================================================== */
void PIT_IRQHandler(void)
{
		if(pit_flag_get(PIT_CH0))
    {	
			key_scanner();
      pit_flag_clear(PIT_CH0);
    }
		
		
    if(pit_flag_get(PIT_CH3))
    {
      pit_flag_clear(PIT_CH3);
			get_encoder();

			imu660rb_get_gyro();

			Processed_gyro.x = imu660rb_gyro_transition(imu660rb_gyro_x);
			Processed_gyro.y = imu660rb_gyro_transition(imu660rb_gyro_y);
			Processed_gyro.z = imu660rb_gyro_transition(imu660rb_gyro_z);

			Posture_Transition(&Processed_gyro);

	
//			Gyro_Bias_OnlineUpdate();

			new_gyro_x = Processed_gyro.x - gyrox_bias;
			new_gyro_y = Processed_gyro.y - gyroy_bias;
			new_gyro_z = Processed_gyro.z - gyroz_bias;

			dgyrox = GyroLPF2_Update(&gyro_lpf_x, new_gyro_x);
			dgyroy = GyroLPF2_Update(&gyro_lpf_y, new_gyro_y);
			dgyroz = GyroLPF2_Update(&gyro_lpf_z, new_gyro_z);

			if(fabsf(dgyrox) < 0.13f) dgyrox = 0;
			if(fabsf(dgyroy) < 0.14f) dgyroy = 0;
			if(fabsf(dgyroz) < 0.13f) dgyroz = 0;

			Quaternion_Update(dgyrox, dgyroy, dgyroz, 0.005f);

			PositionEstimator_Task_5ms();

			mecanum_cascade_control(-target_y,target_x, target_angle, 
															-estimated_y,estimated_x, gyroz);
			mecanum_basic(vel_y_robot, vel_x_robot, vel_w_cmd);
    }

    __DSB();
}





void LPUART8_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        
				wireless_module_uart_handler();
		}
				
    }
        
    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);    // 不允许删除
}