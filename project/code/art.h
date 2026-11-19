/* ======================================================================
 * >> 协议帧常量与状态（art.h）
 * 【面试问答】
 * Q: 帧头/帧尾常量如何区分？
 * A: 250/251 为地图帧头尾，253/254 为指令帧头尾，245/246 为标志帧头尾，
 * A: 0xF8/0xF9 为识别模块帧头尾；不同帧头让同一个状态机在同一串口上复用解析
 * A: 多种帧。
 * ----------------------------------------------------------------------
 * Q: RxState_t 覆盖了哪些状态？
 * A: 等帧头、收数据、等 CRC、等帧尾，以及指令帧、标志帧各自的接收分支，覆盖三类
 * A: 帧的完整接收路径。
 * ----------------------------------------------------------------------
 * ====================================================================== */
#ifndef __ART__H__
#define __ART__H__
#include "zf_common_headfile.h"
#define FRAME_HEADER     250
#define FRAME_TAIL       251
#define RAW_DATA_LEN     192
#define TOTAL_FRAME_LEN  195

#define FRAME_HEADER_CMD 253
#define FRAME_TAIL_CMD   254
#define CMD_DATA_LEN     9

#define FRAME_HEAD_REC 0xF8
#define FRAME_TAIL_REC 0xF9

#define FRAME_HEADER_FLAG 245
#define FRAME_TAIL_FLAG   246
typedef enum {
    STATE_WAIT_HEADER=0,
    STATE_RECEIVE_DATA,      
    STATE_WAIT_CRC,          
    STATE_WAIT_TAIL,         
    STATE_RECEIVE_CMD_DATA,  
    STATE_WAIT_CMD_TAIL ,
		STATE_RECEIVE_FLAG,
		STATE_WAIT_FLAG_TAIL 
} RxState_t;
extern volatile uint8_t frame_ready;
extern RxState_t rx_state;
extern char	uart_buffer[RAW_DATA_LEN]; 
extern uint8_t cmd_buffer[CMD_DATA_LEN];
extern volatile uint8_t cmd_received; 
extern	uint16_t	target_x;
extern	uint16_t	target_y;
extern	uint16_t	rec_x;
extern	uint16_t	rec_y;
void ART_init(void);
void update_map_from_uart(char map[12][16],char *buffer) ;
uint16_t get_length_to_zero(char *arr);
uint8_t cal_crc8(const char *data, uint16_t length);	
#endif 