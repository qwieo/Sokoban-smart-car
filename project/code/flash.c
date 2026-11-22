

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