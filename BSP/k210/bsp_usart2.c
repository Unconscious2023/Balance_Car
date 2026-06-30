#include "bsp_usart2.h"

extern UART_HandleTypeDef huart2;

uint8_t g_k210_rx_byte;

void USART2_init(void)
{
    HAL_UART_Receive_IT(&huart2, &g_k210_rx_byte, 1);
}

void USART2_Send_U8(uint8_t ch)
{
    HAL_UART_Transmit(&huart2, &ch, 1, 0xffff);
}

void USART2_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
    HAL_UART_Transmit(&huart2, BufferPtr, Length, 0xffff);
}

void USART2_RX_deal(uint8_t rx_data)
{
    static uint8_t buf[7];
    static uint8_t idx = 0;

    if (mode == ChaseLine_Mode) {
        if (idx == 0 && rx_data == 0xA5) {
            buf[0] = rx_data;
            idx = 1;
            return;
        }
        if (idx > 0 && idx < 7) {
            buf[idx++] = rx_data;
            if (idx == 7 && buf[6] == 0x5A) {
                idx = 0;
                // 请求帧: A5 FF 00 00 00 chk 5A → 回传遥测
                if (buf[1] == 0xFF && buf[2] == 0 && buf[3] == 0 && buf[4] == 0) {
                    uint8_t telem[7];
                    float hdg = Car_Diff_Heading();
                    int8_t s = (int8_t)((int)hdg);
                    extern float Car_Target_Velocity;
                    uint8_t sp = (uint8_t)((int)Car_Target_Velocity);
                    telem[0]=0xA5; telem[1]=sp; telem[2]=(uint8_t)s;
                    telem[3]=1; telem[4]=(uint8_t)s;
                    telem[5]=sp+(uint8_t)s+1+(uint8_t)s; telem[6]=0x5A;
                    USART2_Send_ArrayU8(telem, 7);
                    return;
                }
                // 不是请求帧 → 转发给 VisionTurn_ParseByte 逐字节
                for (int i = 0; i < 7; i++)
                    Deal_K210_Vision(buf[i]);
                return;
            }
            return;
        }
        idx = 0;
    }
}
