/* ======================================================================
 * >> VOFA+ 上位机调试
 * 【面试问答】
 * Q1: VOFA+ 是什么？两种协议的区别？
 * A: PC 端波形调试上位机。FireWater 是文本 CSV（逗号分隔、换行结尾），可读性好但
 * A: 效率低；JustFloat 直接发 float 原始字节 + 固定 4 字节帧尾，效率高、适合多通道
 * A: 实时波形。
 * ----------------------------------------------------------------------
 * Q1: JustFloat 帧尾的作用？
 * A: 0x00 0x00 0x80 0x7f 作为同步标志，让上位机在字节流中对齐帧边界，丢字节后能
 * A: 自动重新同步。
 * ----------------------------------------------------------------------
 * Q1: 调试时怎么用？
 * A: 把目标/实际位置、速度、误差、PID 输出填入数组发送，上位机实时观察阶跃响应和
 * A: 跟踪滞后，判断超调、振荡、静差，辅助整定 PID。
 * ----------------------------------------------------------------------
 * ====================================================================== */
#include "zf_common_headfile.h"
#include "vofa.h"


//-----------------------------------------------------------------------------------------------
// 函数简介  FireWater数据引擎
// 参数说明  
// 返回参数  void
// 使用示例  ->
// int8    data_1 = -13;
// uint16  data_2 = 234;
// float   data_3 = 3.4;
// FW_Data[0].type = 'd';
// FW_Data[1].type = 'd';
// FW_Data[2].type = 'f';
// FW_Data[0].int_data   = data_1;
// FW_Data[1].int_data   = data_2;
// FW_Data[2].float_data = data_3;
// FireWater_Send();
// 备注信息  1.其中FW_Data[x].type可以不放在主循环或中断中
// 备注信息  2.
// 不要忘了给数据类型(.type)赋值 ,  .int_data 或 .float_data要与.type匹配!
// 不要忘了给数据类型(.type)赋值 ,  .int_data 或 .float_data要与.type匹配!
// 不要忘了给数据类型(.type)赋值 ,  .int_data 或 .float_data要与.type匹配!
//-----------------------------------------------------------------------------------------------
static char data_str[12];
FireWater_Struct FW_Data[CH_COUNT] = {[0 ... CH_COUNT-1] = {0, 0, 'd'}};//特殊初始化语法，将索引0到CH_COUNT-1的元素初始化为0, 0.0, 'd'}

void FireWater_Send()
{
    for(uint8 i = 0; i < CH_COUNT; i++)
    {
        switch (FW_Data[i].type) 
        {
            case 'd':
                snprintf(data_str, sizeof(data_str), "%d", FW_Data[i].int_data);
                break;
            case 'f':
                snprintf(data_str, sizeof(data_str), "%f", FW_Data[i].float_data);
                break;			
            default:
                wireless_uart_send_string("error: Unknown type!\n");
                continue;
        }
        if(i != CH_COUNT-1)
        {
            wireless_uart_send_string(data_str);
            wireless_uart_send_string(",");
        }
        else
        {
            wireless_uart_send_string(data_str);
            wireless_uart_send_string("\n");
        }
    }
}


//-----------------------------------------------------------------------------------------------
// 函数简介  JustFloat数据引擎
// 参数说明  
// 返回参数  void
// 使用示例  :
//JF_Data.data[0] = Kp_T;
//JF_Data.data[1] = Kd_T;
//JF_Data.data[2] = 0.45;
//JF_Data.data[3] = -3.36;
//JustFloat_Send();
// 备注信息  
//-----------------------------------------------------------------------------------------------
JustFloat_Struct JF_Data;

void JustFloat_Send()
{
  JF_Data.tail[0] = 0x00;
	JF_Data.tail[1] = 0x00;
	JF_Data.tail[2] = 0x80;
	JF_Data.tail[3] = 0x7f;
  wireless_uart_send_buffer((const uint8*)JF_Data.data, sizeof(float) * CH_COUNT);
 	wireless_uart_send_buffer((const uint8*)JF_Data.tail, 4);
}
