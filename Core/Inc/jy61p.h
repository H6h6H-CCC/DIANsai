#ifndef __JY61P_H
#define __JY61P_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

// 定义传感器数据结构
typedef struct {
    float ax, ay, az;     // 加速度 (m/s2)
    float wx, wy, wz;     // 角速度 (°/s)
    float roll, pitch, yaw; // 角度 (°)
    float temperature;    // 温度 (°C)
} SensorData_t;

// 数据包类型
typedef enum {
    PACKET_TIME = 0x50,
    PACKET_ACCEL = 0x51,
    PACKET_GYRO = 0x52,
    PACKET_ANGLE = 0x53,
    PACKET_MAG = 0x54,
    PACKET_PORT = 0x55,
    PACKET_PRESS = 0x56,
    PACKET_LATLON = 0x57,
    PACKET_GPSVEL = 0x58,
    PACKET_QUAT = 0x59,
    PACKET_GPSACC = 0x5A,
    PACKET_READ = 0x5F
} PacketType_t;

// 数据包解析状态
typedef enum {
    STATE_WAIT_HEADER,
    STATE_WAIT_TYPE,
    STATE_WAIT_DATA
} ParserState_t;

// 外部变量声明
extern SensorData_t sensorData;
extern uint8_t accValid;
extern uint8_t gyroValid;
extern uint8_t angleValid;
extern uint32_t lastPacketTime;

// 函数声明
void JY61P_InitConfig(void);
void JY61P_Unlock(void);
void JY61P_Save(void);
void JY61P_WriteRegister(uint8_t addr, int16_t data);
void JY61P_ReadRegister(uint8_t addr);
void JY61P_RequestData(uint8_t type);
void JY61P_ParsePacket(uint8_t *packet);
void ProcessReceivedData(uint8_t data);
int16_t BytesToInt16(uint8_t low, uint8_t high);
void UpdateDisplay(void);



#ifdef __cplusplus
}
#endif

#endif /* __JY61P_H */


