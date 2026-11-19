/* ======================================================================
 * >> 参数掉电保存
 * 【面试问答】
 * Q1: 参数如何掉电保存？
 * A: 固定使用 Flash 最后一页（127 页、扇区 7，远离程序区），用 union 把 uint8/float
 * A: 参数组织成 4 字节缓冲，开机读回、退出菜单时写入。
 * ----------------------------------------------------------------------
 * Q1: 写 Flash 前为什么要擦除？为什么先 check？
 * A: NOR Flash 只能把位从 1 写成 0，必须擦除恢复全 1 才能重写；先 check 内容，相同
 * A: 则跳过擦写——Flash 擦写次数有限（约 10 万次），无谓擦写会缩短寿命。
 * ----------------------------------------------------------------------
 * Q1: union 在这里的作用？
 * A: 同一段内存按 uint8/float 等类型分别解释，方便把异构参数装进统一缓冲、与 Flash
 * A: 按字节交互。
 * ----------------------------------------------------------------------
 * Q1: 改了箱子数量，识别次数如何同步？
 * A: Flash 读写后都重算 reg_times = 2*num_box - 1，保证菜单修改立即生效。
 * ----------------------------------------------------------------------
 * ====================================================================== */
#include "flash.h"
extern uint8  num_box2;
extern uint8 reg_times2;
extern uint8  num_box3;
extern uint8 reg_times3;
extern float ACCEL_LIMIT_TRIGGER_DMPS;
void Flash_Read(void)
{

	flash_buffer_clear();	
	flash_read_page_to_buffer(127, FLASH_PAGE_7);
	num_box2	= flash_union_buffer[0].uint8_type;
	X.kp  	=	flash_union_buffer[1].float_type;
	X.kd		=	flash_union_buffer[2].float_type;
	Y.kp		=	flash_union_buffer[3].float_type;
	Y.kd	  =	flash_union_buffer[4].float_type;
	num_box3	= flash_union_buffer[5].uint8_type;
	ACCEL_LIMIT_TRIGGER_DMPS	= flash_union_buffer[6].float_type;
	reg_times2=2*num_box2-1;
	reg_times3=2*num_box3-1;
	system_delay_ms(100);
}

void Write_Flash(void)
{
	flash_buffer_clear();
	flash_union_buffer[0].uint8_type = num_box2;
	flash_union_buffer[1].float_type = X.kp;
	flash_union_buffer[2].float_type = X.kd;
	flash_union_buffer[3].float_type = Y.kp;
	flash_union_buffer[4].float_type = Y.kd;
	flash_union_buffer[5].uint8_type = num_box3;
	flash_union_buffer[6].float_type = ACCEL_LIMIT_TRIGGER_DMPS;
	reg_times2=2*num_box2-1;
	reg_times3=2*num_box3-1;
	if(flash_check(127, FLASH_PAGE_7)) 
	flash_erase_page(127, FLASH_PAGE_7);
	flash_write_page_from_buffer(127,FLASH_PAGE_7);
	system_delay_ms(100);
}