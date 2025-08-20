#ifndef __JY901S_H
#define __JY901S_H

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

// ���ݰ����Ͷ���
#define FRAME_HEAD      0x55    // ���ݰ�ͷ
#define TYPE_ACC       0x51    // ���ٶȰ�
#define TYPE_GYRO      0x52    // ���ٶȰ�
#define TYPE_ANGLE     0x53    // �ǶȰ�
#define TYPE_MAG       0x54    // �ų���

// ���ݽṹ��
typedef struct 
{
    float acc[3];     // ���ٶȣ���λ��g
    float gyro[3];    // ���ٶȣ���λ����/s
    float angle[3];   // ŷ���ǣ���λ����
    float temp;       // �¶ȣ���λ����
} JY901S_DATA;

// ���ջ�����
#define RX_BUF_SIZE  11  // һ֡���ݵĴ�С��1(��ͷ) + 1(������) + 8(����) + 1(У���)
extern uint8_t rx_buf[RX_BUF_SIZE];
extern uint8_t rx_cnt;
extern JY901S_DATA jy901s_data;

// ��������
void JY901S_Init(void);
void JY901S_ParseData(void);

#endif

