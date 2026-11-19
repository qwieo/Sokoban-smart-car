/* ======================================================================
 * >> 串口协议与解析
 * 【面试问答】
 * Q1: 串口协议如何保证可靠传输？
 * A: 帧头 + 定长数据 + CRC8 + 帧尾，用不同帧头区分地图帧(250)、指令帧(253)、标志帧
 * A: (245)；只有定长收齐且 CRC 校验通过才置 frame_ready，错帧丢弃并重新等帧头。
 * ----------------------------------------------------------------------
 * Q1: 为什么用状态机解析？
 * A: 字节逐个异步到达且总线可能有噪声，状态机是流式解析的标准做法：等帧头 -> 收数据
 * A: -> 等 CRC -> 等帧尾，任何一步出错都能自动重新同步、不会错位。
 * ----------------------------------------------------------------------
 * Q1: 中断与主循环共享变量为什么加 volatile？
 * A: 防止编译器把变量优化缓存进寄存器，保证主循环每次都从内存读到中断更新的最新值；
 * A: 帧标志同时是异步握手信号。
 * ----------------------------------------------------------------------
 * Q1: 115200 波特率够吗？
 * A: 最大的地图帧 192 字节约 17ms，且每关只发一次，指令和标志都很短，带宽完全够用；
 * A: 该协议不传原始图像。
 * ----------------------------------------------------------------------
 * ====================================================================== */
#include "art.h"
char uart_buffer[RAW_DATA_LEN];
uint8_t cmd_buffer[CMD_DATA_LEN];
uint8_t crc_received = 0;
uint8_t calc_crc;

RxState_t rx_state = STATE_WAIT_HEADER;
RxState_t rx_state_rec = STATE_WAIT_HEADER;

volatile uint8_t frame_ready = 0;
volatile uint8_t cmd_received; 

extern	uint16_t	target_x;
extern	uint16_t	target_y;
extern float target_angle;

extern	volatile uint16_t camera_x_raw ;
extern	volatile uint16_t camera_y_raw ;
extern 	volatile uint8_t camera_new_frame;

extern char art1sendvariety;
extern char variety;
extern volatile char art0ack;
extern volatile char regpoint;
extern volatile char level_passed;
extern volatile char back_home;

void ART_init(void)
{

	 uart_init (UART_1,115200, UART1_TX_B12, UART1_RX_B13);																					
	 interrupt_set_priority(LPUART1_IRQn, 1);
	 uart_rx_interrupt(UART_1,1);	
	 uart_init (UART_4,115200, UART4_TX_C16, UART4_RX_C17);																						
	 interrupt_set_priority(LPUART4_IRQn, 1);
	 uart_rx_interrupt(UART_4,1);
}

uint8_t cal_crc8(const char *data, uint16_t length)
{
    uint8_t crc = 0x00; 
    const uint8_t POLY = 0x31; 

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ POLY;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint16_t get_length_to_zero(char *arr) 
{
    uint16_t length = 0;
    
    while (length < 1000 && arr[length] != 0) {
        length++;
    }    
    return length;
}

void update_map_from_uart(char map[12][16], char *buffer) {
    int buf_index = 0;

    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 16; j++) { 
            
            map[i][j] = buffer[buf_index];
            buf_index++;
        }
        
    }
}

void LPUART1_IRQHandler(void)
{
		static uint8_t temp = 0;
    static uint16_t data_index = 0;
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
        uart_query_byte(UART_1, &temp);
        switch (rx_state)
        {
             case STATE_WAIT_HEADER:
                if (temp == FRAME_HEADER)
                {
                    data_index = 0;
                    rx_state = STATE_RECEIVE_DATA; 
                }
                else if (temp == FRAME_HEADER_CMD)
                {
                    data_index = 0;
                    rx_state = STATE_RECEIVE_CMD_DATA; 
                }
								else if	(temp == FRAME_HEADER_FLAG)
								{
									data_index = 0;
                    rx_state = STATE_RECEIVE_FLAG; 
								}
                break;

            case STATE_RECEIVE_DATA:
                uart_buffer[data_index++] = temp;
                if (data_index >= RAW_DATA_LEN)
                {
                    rx_state = STATE_WAIT_CRC;     
                }
                break;

            case STATE_WAIT_CRC:
                crc_received = temp;
                rx_state = STATE_WAIT_TAIL;        
                break;

            case STATE_WAIT_TAIL:
                if (temp == FRAME_TAIL)
                {
                     calc_crc = cal_crc8(uart_buffer, RAW_DATA_LEN);
                     if(calc_crc == crc_received) { 
                         frame_ready = 1;
                    }
										 else
                    {
                        frame_ready = 0;
                    }
                    
                }
                rx_state = STATE_WAIT_HEADER;
                break;
            case STATE_RECEIVE_CMD_DATA:
                cmd_buffer[data_index++] = temp;
                if (data_index >= CMD_DATA_LEN) 
                {
									rx_state = STATE_WAIT_CMD_TAIL;
                }
                break;

            case STATE_WAIT_CMD_TAIL:
                if (temp == FRAME_TAIL_CMD) 
                {
									/* ======================================================================
									 * >> 指令帧解析
									 * 【面试问答】
									 * Q1: 为什么按高字节在前拼 16 位数？
									 * A: 协议约定多字节按大端传输（高字节先发），拼回时高字节左移 8 位再或上低字节，
									 * A: 收发双方字节序必须一致。
									 * ----------------------------------------------------------------------
									 * Q1: target_angle 为什么写成 (PI/2)*(字节-128)？
									 * A: 用一个字节偏移编码方向：128 对应 0，每增减 1 对应增减 90 度，一个字节即可表示
									 * A: 离散的四向目标角。
									 * ----------------------------------------------------------------------
									 * Q1: 标志字节 196/233/247/248 的含义？
									 * A: 单字节事件：233 到达识别点，247 过关，248 回家，196 握手应答清除；中断里只改
									 * A: volatile 标志，动作响应放主循环。
									 * ----------------------------------------------------------------------
									 * ====================================================================== */
									target_x= ((uint16_t)cmd_buffer[0] << 8) | cmd_buffer[1];
									target_y=((uint16_t)cmd_buffer[2] << 8) | cmd_buffer[3];
									camera_x_raw=((uint16_t)cmd_buffer[4] << 8) | cmd_buffer[5];
									camera_y_raw=((uint16_t)cmd_buffer[6] << 8) | cmd_buffer[7];
									target_angle=(PI/2)*(cmd_buffer[8]-128);
									
                  if (cmd_received == 1)
                  {
										cmd_received = 0;
                  }
									camera_new_frame=1;
                }
                rx_state = STATE_WAIT_HEADER; 
                break;
						case STATE_RECEIVE_FLAG:
								if(temp==196)
								{	
									art0ack=0;
									regpoint=0;
								}
								else if(temp==233)
								regpoint=1;
								else if(temp==247)
								level_passed=1;
								else if(temp==248)
								back_home=1;
								rx_state = STATE_WAIT_FLAG_TAIL;
							 break;
						case STATE_WAIT_FLAG_TAIL:
							 if (temp == FRAME_TAIL_FLAG) 
								rx_state = STATE_WAIT_HEADER; 
							  
						 default:
                rx_state = STATE_WAIT_HEADER;
                break;
        }
    }
        
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);    
}


/* ======================================================================
 * >> 识别模块帧（UART4）
 * 【面试问答】
 * Q1: UART4 识别帧的协议？
 * A: 0xF8 帧头 + 1 字节箱子种类 + 0xF9 帧尾，收齐后置 art1sendvariety，主循环读取
 * A: variety 并按位姿记录；同样用状态机解析。
 * ----------------------------------------------------------------------
 * Q1: 接收中断里为什么不做重计算？
 * A: 中断要快进快出，只搬字节、切状态；192 字节 CRC 计算量尚可接受，地图解析等重
 * A: 处理全部放到主循环。
 * ----------------------------------------------------------------------
 * ====================================================================== */
void LPUART4_IRQHandler(void)
{
    static uint8_t temp = 0;

    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        uart_query_byte(UART_4, &temp); 

        switch (rx_state_rec)
        {
            case STATE_WAIT_HEADER: 
                if (temp == FRAME_HEAD_REC)
                {
                    rx_state_rec = STATE_RECEIVE_DATA; 
                }
//								else if
//                break;

            case STATE_RECEIVE_DATA: 
                variety = temp;     
                rx_state_rec = STATE_WAIT_TAIL;     
                break;

            case STATE_WAIT_TAIL: 
                if (temp == FRAME_TAIL_REC)
                {
                    art1sendvariety = 1; 
                }
                else
                {
                    art1sendvariety = 0; 
                }
                rx_state_rec = STATE_WAIT_HEADER; 
                break;

            default:
                rx_state_rec = STATE_WAIT_HEADER;
                break;
        }
    }
		
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);  
}