#include "jy901s.h"

JY901S_DATA jy901s_data;
uint8_t rx_buf[RX_BUF_SIZE];
uint8_t rx_cnt = 0;

// JY901S��ʼ��
void JY901S_Init(void)
{
    // �������ݽṹ��
    memset(&jy901s_data, 0, sizeof(JY901S_DATA));
    rx_cnt = 0;
    
    // �������ڽ����ж�
    HAL_UART_Receive_IT(&huart1, &rx_buf[0], 1);
}

// �������ݰ�
void JY901S_ParseData(void)
{
    uint8_t sum = 0;
    
    // ����У���
    for(uint8_t i = 0; i < RX_BUF_SIZE-1; i++)
    {
        sum += rx_buf[i];
    }
    
    // У��Ͳ���ȷ
    if(sum != rx_buf[RX_BUF_SIZE-1])
    {
        return;
    }
    
    // �����������ͽ�������
    switch(rx_buf[1])
    {
        case TYPE_ACC:  // ���ٶ�����
            jy901s_data.acc[0] = (float)((int16_t)(rx_buf[3]<<8 | rx_buf[2])) / 32768.0f * 16.0f;
            jy901s_data.acc[1] = (float)((int16_t)(rx_buf[5]<<8 | rx_buf[4])) / 32768.0f * 16.0f;
            jy901s_data.acc[2] = (float)((int16_t)(rx_buf[7]<<8 | rx_buf[6])) / 32768.0f * 16.0f;
            jy901s_data.temp = (float)((int16_t)(rx_buf[9]<<8 | rx_buf[8])) / 100.0f;
            break;
            
        case TYPE_GYRO:  // ���ٶ�����
            jy901s_data.gyro[0] = (float)((int16_t)(rx_buf[3]<<8 | rx_buf[2])) / 32768.0f * 2000.0f;
            jy901s_data.gyro[1] = (float)((int16_t)(rx_buf[5]<<8 | rx_buf[4])) / 32768.0f * 2000.0f;
            jy901s_data.gyro[2] = (float)((int16_t)(rx_buf[7]<<8 | rx_buf[6])) / 32768.0f * 2000.0f;
            break;
            
        case TYPE_ANGLE:  // ŷ��������
            jy901s_data.angle[0] = (float)((int16_t)(rx_buf[3]<<8 | rx_buf[2])) / 32768.0f * 180.0f;
            jy901s_data.angle[1] = (float)((int16_t)(rx_buf[5]<<8 | rx_buf[4])) / 32768.0f * 180.0f;
            jy901s_data.angle[2] = (float)((int16_t)(rx_buf[7]<<8 | rx_buf[6])) / 32768.0f * 180.0f;
            break;
    }
}

// ���ڽ����жϻص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        // ���յ�һ���ֽڣ��ж��Ƿ���֡ͷ
        if(rx_cnt == 0)
        {
            if(rx_buf[0] != FRAME_HEAD)
            {
                HAL_UART_Receive_IT(&huart1, &rx_buf[0], 1);
                return;
            }
        }
        
        rx_cnt++;
        
        // ������һ֡����
        if(rx_cnt >= RX_BUF_SIZE)
        {
            JY901S_ParseData();  // ��������
            rx_cnt = 0;  // ���ü�����
        }
        
        // ����������һ���ֽ�
        HAL_UART_Receive_IT(&huart1, &rx_buf[rx_cnt], 1);
    }
}









