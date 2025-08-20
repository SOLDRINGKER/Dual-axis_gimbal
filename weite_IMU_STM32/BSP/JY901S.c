#include "jy901s.h"

JY901S_DATA jy901s_data;
uint8_t rx_buf[RX_BUF_SIZE];
uint8_t rx_cnt = 0;

// JY901S初始化
void JY901S_Init(void)
{
    // 清零数据结构体
    memset(&jy901s_data, 0, sizeof(JY901S_DATA));
    rx_cnt = 0;
    
    // 开启串口接收中断
    HAL_UART_Receive_IT(&huart1, &rx_buf[0], 1);
}

// 解析数据包
void JY901S_ParseData(void)
{
    uint8_t sum = 0;
    
    // 计算校验和
    for(uint8_t i = 0; i < RX_BUF_SIZE-1; i++)
    {
        sum += rx_buf[i];
    }
    
    // 校验和不正确
    if(sum != rx_buf[RX_BUF_SIZE-1])
    {
        return;
    }
    
    // 根据数据类型解析数据
    switch(rx_buf[1])
    {
        case TYPE_ACC:  // 加速度数据
            jy901s_data.acc[0] = (float)((int16_t)(rx_buf[3]<<8 | rx_buf[2])) / 32768.0f * 16.0f;
            jy901s_data.acc[1] = (float)((int16_t)(rx_buf[5]<<8 | rx_buf[4])) / 32768.0f * 16.0f;
            jy901s_data.acc[2] = (float)((int16_t)(rx_buf[7]<<8 | rx_buf[6])) / 32768.0f * 16.0f;
            jy901s_data.temp = (float)((int16_t)(rx_buf[9]<<8 | rx_buf[8])) / 100.0f;
            break;
            
        case TYPE_GYRO:  // 角速度数据
            jy901s_data.gyro[0] = (float)((int16_t)(rx_buf[3]<<8 | rx_buf[2])) / 32768.0f * 2000.0f;
            jy901s_data.gyro[1] = (float)((int16_t)(rx_buf[5]<<8 | rx_buf[4])) / 32768.0f * 2000.0f;
            jy901s_data.gyro[2] = (float)((int16_t)(rx_buf[7]<<8 | rx_buf[6])) / 32768.0f * 2000.0f;
            break;
            
        case TYPE_ANGLE:  // 欧拉角数据
            jy901s_data.angle[0] = (float)((int16_t)(rx_buf[3]<<8 | rx_buf[2])) / 32768.0f * 180.0f;
            jy901s_data.angle[1] = (float)((int16_t)(rx_buf[5]<<8 | rx_buf[4])) / 32768.0f * 180.0f;
            jy901s_data.angle[2] = (float)((int16_t)(rx_buf[7]<<8 | rx_buf[6])) / 32768.0f * 180.0f;
            break;
    }
}

// 串口接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        // 接收第一个字节，判断是否是帧头
        if(rx_cnt == 0)
        {
            if(rx_buf[0] != FRAME_HEAD)
            {
                HAL_UART_Receive_IT(&huart1, &rx_buf[0], 1);
                return;
            }
        }
        
        rx_cnt++;
        
        // 接收完一帧数据
        if(rx_cnt >= RX_BUF_SIZE)
        {
            JY901S_ParseData();  // 解析数据
            rx_cnt = 0;  // 重置计数器
        }
        
        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart1, &rx_buf[rx_cnt], 1);
    }
}









