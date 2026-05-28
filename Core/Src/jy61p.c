#include "JY61P.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "usart.h"
#include "oled.h"
// 协议相关定义
#define PACKET_HEADER 0x55
#define PACKET_SIZE 11

// 转换系数
#define ACC_SCALE (16.0f * 9.8f / 32768.0f)    // 加速度量程16g
#define GYRO_SCALE (2000.0f / 32768.0f)       // 角速度量程2000°/s
#define ANGLE_SCALE (180.0f / 32768.0f)       // 角度量程±180°

// 全局变量定义
SensorData_t sensorData = {0};
uint8_t accValid = 0;
uint8_t gyroValid = 0;
uint8_t angleValid = 0;
uint32_t lastPacketTime = 0;
extern char displayBuffer[];
// 数据包解析状态变量
static ParserState_t parserState = STATE_WAIT_HEADER;

// 传感器解锁
void JY61P_Unlock(void)
{
    uint8_t unlockCmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    HAL_UART_Transmit(&huart4, unlockCmd, sizeof(unlockCmd), 100);
    HAL_Delay(200);
}

// 保存设置
void JY61P_Save(void)
{
    uint8_t saveCmd[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    HAL_UART_Transmit(&huart4, saveCmd, sizeof(saveCmd), 100);
}

// 写寄存器
void JY61P_WriteRegister(uint8_t addr, int16_t data)
{
    uint8_t cmd[5] = {0xFF, 0xAA, addr, (uint8_t)(data & 0xFF), (uint8_t)(data >> 8)};
    HAL_UART_Transmit(&huart4, cmd, 5, 100);
}

// 请求特定类型数据
void JY61P_RequestData(uint8_t type)
{
    uint8_t cmd[] = {0xFF, 0xAA, 0x27, type, 0x00};
    HAL_UART_Transmit(&huart4, cmd, sizeof(cmd), 100);
}

// 传感器初始化配置
void JY61P_InitConfig(void)
{
    //printf("Initializing JY61P Sensor...\r\n");
    
    // 1. 解锁
    JY61P_Unlock();
    
    // 2. 设置输出速率 (0x06 = 10Hz, 0x09 = 100Hz)
    JY61P_WriteRegister(0x03, 0x09); // 100Hz输出
    
    // 3. 设置输出内容 (默认0x001E = 加速度+角速度+角度+磁场+端口)
    // 如果需要只输出特定数据，可以修改这个寄存器
    // JY61P_WriteRegister(0x02, 0x0007); // 只输出加速度+角速度+角度
    
    // 4. 保存设置
    JY61P_Save();
    
    HAL_Delay(100);
    //printf("JY61P Initialization Complete\r\n");
}

// 处理接收到的字节
void ProcessReceivedData(uint8_t data)
{
    static uint8_t packet[PACKET_SIZE];
    static uint8_t packetIndex = 0;
    
    switch(parserState)
    {
        case STATE_WAIT_HEADER:
            if(data == PACKET_HEADER)
            {
                packet[0] = data;
                packetIndex = 1;
                parserState = STATE_WAIT_TYPE;
            }
            break;
            
        case STATE_WAIT_TYPE:
            packet[1] = data;
            packetIndex = 2;
            parserState = STATE_WAIT_DATA;
            break;
            
        case STATE_WAIT_DATA:
            packet[packetIndex++] = data;
            if(packetIndex >= PACKET_SIZE)
            {
                // 完整数据包接收完成
                JY61P_ParsePacket(packet);
                parserState = STATE_WAIT_HEADER;
                packetIndex = 0;
                lastPacketTime = HAL_GetTick();
            }
            break;
    }
}

// 解析数据包
void JY61P_ParsePacket(uint8_t *packet)
{
    if(packet[0] != PACKET_HEADER) return;
    
    // 计算校验和
    uint8_t checksum = 0;
    for(int i = 0; i < PACKET_SIZE - 1; i++)
    {
        checksum += packet[i];
    }
    
    if(checksum != packet[PACKET_SIZE - 1])
    {
        //printf("Checksum error!\r\n");
        return;
    }
    
    uint8_t type = packet[1];
    
    switch(type)
    {
        case PACKET_ACCEL:  // 加速度数据
        {
            int16_t ax = BytesToInt16(packet[2], packet[3]);
            int16_t ay = BytesToInt16(packet[4], packet[5]);
            int16_t az = BytesToInt16(packet[6], packet[7]);
            int16_t temp = BytesToInt16(packet[8], packet[9]);
            
            sensorData.ax = ax * ACC_SCALE;
            sensorData.ay = ay * ACC_SCALE;
            sensorData.az = az * ACC_SCALE;
            sensorData.temperature = temp / 100.0f;
            
            accValid = 1;
            //printf("Acc: X=%.2f, Y=%.2f, Z=%.2f m/s2\r\n", sensorData.ax, sensorData.ay, sensorData.az);
            break;
        }
        
        case PACKET_GYRO:  // 角速度数据
        {
            int16_t wx = BytesToInt16(packet[2], packet[3]);
            int16_t wy = BytesToInt16(packet[4], packet[5]);
            int16_t wz = BytesToInt16(packet[6], packet[7]);
            
            sensorData.wx = wx * GYRO_SCALE;
            sensorData.wy = wy * GYRO_SCALE;
            sensorData.wz = wz * GYRO_SCALE;
            
            gyroValid = 1;
            //printf("Gyro: X=%.2f, Y=%.2f, Z=%.2f °/s\r\n", sensorData.wx, sensorData.wy, sensorData.wz);
            break;
        }
        
        case PACKET_ANGLE:  // 角度数据
        {
            int16_t roll = BytesToInt16(packet[2], packet[3]);
            int16_t pitch = BytesToInt16(packet[4], packet[5]);
            int16_t yaw = BytesToInt16(packet[6], packet[7]);
            
            sensorData.roll = roll * ANGLE_SCALE;
            sensorData.pitch = pitch * ANGLE_SCALE;
            sensorData.yaw = yaw * ANGLE_SCALE;
            
            angleValid = 1;
            //printf("Angle: Roll=%.2f, Pitch=%.2f, Yaw=%.2f °\r\n", sensorData.roll, sensorData.pitch, sensorData.yaw);
            break;
        }
        
        default:
            // 其他数据包类型
            break;
    }
}

// 字节转有符号16位整数
int16_t BytesToInt16(uint8_t low, uint8_t high)
{
    int16_t value = (high << 8) | low;
    return value;
}

void UpdateDisplay(void)
{
    int32_t ax_i = (int32_t)sensorData.ax;
    int32_t ay_i = (int32_t)sensorData.ay;
    int32_t az_i = (int32_t)sensorData.az;
    int32_t wx_i = (int32_t)sensorData.wx;
    int32_t wy_i = (int32_t)sensorData.wy;
    int32_t wz_i = (int32_t)sensorData.wz;
    int32_t roll_i = (int32_t)sensorData.roll;
    int32_t pitch_i = (int32_t)sensorData.pitch;

    /* 每行左/右各8列，避免越界覆盖 */
    snprintf(displayBuffer, 20, "AX:%4ld", (long)ax_i);
    OLED_ShowString(1, 1, displayBuffer);
    snprintf(displayBuffer, 20, "WX:%5ld", (long)wx_i);
    OLED_ShowString(1, 9, displayBuffer);

    snprintf(displayBuffer, 20, "AY:%4ld", (long)ay_i);
    OLED_ShowString(2, 1, displayBuffer);
    snprintf(displayBuffer, 20, "WY:%5ld", (long)wy_i);
    OLED_ShowString(2, 9, displayBuffer);

    snprintf(displayBuffer, 20, "AZ:%4ld", (long)az_i);
    OLED_ShowString(3, 1, displayBuffer);
    snprintf(displayBuffer, 20, "WZ:%5ld", (long)wz_i);
    OLED_ShowString(3, 9, displayBuffer);

    snprintf(displayBuffer, 20, "R:%5ld", (long)roll_i);
    OLED_ShowString(4, 1, displayBuffer);
    snprintf(displayBuffer, 20, "P:%5ld", (long)pitch_i);
    OLED_ShowString(4, 9, displayBuffer);
}










