

#include "zf_common_headfile.h"
#include "vofa.h"
#include "motor&motion.h"
#include "filter.h"
#include "math.h"
#include "game.h"
#include "sokoban.h"
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
		

		switch (Game_level)
		{
			case Game1:
				switch(Game_phase)
				{
					case Idle:
						level_passed=0;
						while(!frame_ready);
						update_map_from_uart(map_strings,uart_buffer);
						sokoban_plan(SOKOBAN_LEVEL_1, SOKOBAN_STAGE_PREPARE, map_strings);
						
						cmd_received=1;
						while(cmd_received)
						{
							Polling_transmit(sokoban_commands(SOKOBAN_LEVEL_1, SOKOBAN_STAGE_PREPARE));
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
						sokoban_reset(SOKOBAN_LEVEL_1);
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
					

					sokoban_plan(SOKOBAN_LEVEL_2, SOKOBAN_STAGE_PREPARE, map_strings);

					cmd_received = 1;

					while (cmd_received)
					{
						Polling_transmit(sokoban_commands(SOKOBAN_LEVEL_2, SOKOBAN_STAGE_PREPARE));
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
						sokoban_apply_vision(SOKOBAN_LEVEL_2, map_strings, vision_stream, stream_counter);
						sokoban_plan(SOKOBAN_LEVEL_2, SOKOBAN_STAGE_PUSH, map_strings);
						cmd_received=1;
						while(cmd_received)
						{
							Polling_transmit(sokoban_commands(SOKOBAN_LEVEL_2, SOKOBAN_STAGE_PUSH));
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
						sokoban_reset(SOKOBAN_LEVEL_2);
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
					sokoban_plan(SOKOBAN_LEVEL_3, SOKOBAN_STAGE_PREPARE, map_strings);

					cmd_received = 1;

					while (cmd_received)
					{
						Polling_transmit(sokoban_commands(SOKOBAN_LEVEL_3, SOKOBAN_STAGE_PREPARE));
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
						sokoban_apply_vision(SOKOBAN_LEVEL_3, map_strings, vision_stream, stream_counter);
						sokoban_plan(SOKOBAN_LEVEL_3, SOKOBAN_STAGE_PUSH, map_strings);	// 2
						cmd_received=1;
						while(cmd_received)
						{
							Polling_transmit(sokoban_commands(SOKOBAN_LEVEL_3, SOKOBAN_STAGE_PUSH));
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
						sokoban_reset(SOKOBAN_LEVEL_3);
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