/* ======================================================================
 * >> VOFA+ 发送结构（vofa.h）
 * 【面试问答】
 * Q: 两种发送结构的区别？
 * A: FireWater_Struct 每通道带类型字段（d 整数 / f 浮点）；JustFloat_Struct 为固定
 * A: float 数组加 4 字节帧尾；CH_COUNT=8 是通道数。
 * ----------------------------------------------------------------------
 * ====================================================================== */
#ifndef _vofa_h_
#define _vofa_h_

#define CH_COUNT    8              //自定义通道数量


// type： 'd'-->整数 , 'f'-->浮点数  
typedef struct 
{
    int int_data;
    float float_data;  
    char type;              //标记字段，用以标记数据类型
} FireWater_Struct;

typedef struct 
{
    float data[CH_COUNT];	//JustFloat格式只能发送float类型的数据
    unsigned char tail[4];	//帧尾
} JustFloat_Struct;

extern FireWater_Struct FW_Data[CH_COUNT];
extern JustFloat_Struct JF_Data;
void FireWater_Send();
void JustFloat_Send();

#endif