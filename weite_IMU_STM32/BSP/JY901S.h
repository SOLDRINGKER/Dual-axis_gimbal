#ifndef __JY901S_H
#define __JY901S_H

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

// 数据包类型定义
#define FRAME_HEAD      0x55    // 数据包头
#define TYPE_ACC       0x51    // 加速度包
#define TYPE_GYRO      0x52    // 角速度包
#define TYPE_ANGLE     0x53    // 角度包
#define TYPE_MAG       0x54    // 磁场包

// 数据结构体
typedef struct 
{
    float acc[3];     // 加速度，单位：g
    float gyro[3];    // 角速度，单位：°/s
    float angle[3];   // 欧拉角，单位：°
    float temp;       // 温度，单位：℃
} JY901S_DATA;

// 接收缓冲区
#define RX_BUF_SIZE  11  // 一帧数据的大小：1(包头) + 1(功能字) + 8(数据) + 1(校验和)
extern uint8_t rx_buf[RX_BUF_SIZE];
extern uint8_t rx_cnt;
extern JY901S_DATA jy901s_data;

// 函数声明
void JY901S_Init(void);
void JY901S_ParseData(void);

#endif

