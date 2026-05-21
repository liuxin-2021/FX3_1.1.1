/*
 * JY901.h
 *
 * FX3 平台 JY901/JY901S 9轴惯性传感器驱动头文件
 */

#ifndef _INCLUDED_JY901_H_
#define _INCLUDED_JY901_H_

#include "cyu3types.h"
#include "cyu3error.h"
#include "cyu3externcstart.h"

/* ------------------ 常量定义 ------------------ */

/* JY901 默认 I2C 地址 */
#define JY901_I2C_ADDR_DEFAULT   (0x50)

/* 寄存器地址定义 */
#define JY901_SAVE              0x00    /* 保存配置 */
#define JY901_CALSW             0x01    /* 校准模式 */
#define JY901_RSW               0x02    /* 返回内容设置 */
#define JY901_RRATE             0x03    /* 返回速率 */
#define JY901_BAUD              0x04    /* 波特率 */
#define JY901_AXOFFSET          0x05    /* X轴加速度偏移 */
#define JY901_AYOFFSET          0x06    /* Y轴加速度偏移 */
#define JY901_AZOFFSET          0x07    /* Z轴加速度偏移 */
#define JY901_GXOFFSET          0x08    /* X轴陀螺仪偏移 */
#define JY901_GYOFFSET          0x09    /* Y轴陀螺仪偏移 */
#define JY901_GZOFFSET          0x0a    /* Z轴陀螺仪偏移 */
#define JY901_HXOFFSET          0x0b    /* X轴磁力计偏移 */
#define JY901_HYOFFSET          0x0c    /* Y轴磁力计偏移 */
#define JY901_HZOFFSET          0x0d    /* Z轴磁力计偏移 */
#define JY901_ORIENT            0x23    /* 安装方向 */
#define JY901_AXIS6             0x24    /* 6轴/9轴算法选择 */

/* 数据寄存器 */
#define JY901_YYMM              0x30    /* 年月 */
#define JY901_DDHH              0x31    /* 日时 */
#define JY901_MMSS              0x32    /* 分秒 */
#define JY901_MS                0x33    /* 毫秒 */
#define JY901_AX                0x34    /* X轴加速度 */
#define JY901_AY                0x35    /* Y轴加速度 */
#define JY901_AZ                0x36    /* Z轴加速度 */
#define JY901_GX                0x37    /* X轴角速度 */
#define JY901_GY                0x38    /* Y轴角速度 */
#define JY901_GZ                0x39    /* Z轴角速度 */
#define JY901_HX                0x3a    /* X轴磁场 */
#define JY901_HY                0x3b    /* Y轴磁场 */
#define JY901_HZ                0x3c    /* Z轴磁场 */
#define JY901_Roll              0x3d    /* 横滚角 */
#define JY901_Pitch             0x3e    /* 俯仰角 */
#define JY901_Yaw               0x3f    /* 航向角 */
#define JY901_TEMP              0x40    /* 温度 */
#define JY901_KEY               0x69    /* 解锁寄存器：写 0xB588 解锁写操作 */

/* 方向/算法选择 */
#define JY901_ORIENT_HERIZONE   0x00
#define JY901_ORIENT_VERTICLE   0x01
#define JY901_ALGRITHM9         0x00
#define JY901_ALGRITHM6         0x01

/* 数据转换比例常量 */
#define JY901_GYRO_SCALE        (2000.0f/32768.0f)   /* 陀螺仪量程：±2000°/s */
#define JY901_ACCEL_SCALE       (16.0f/32768.0f)     /* 加速度量程：±16g */
#define JY901_ANGLE_SCALE       (180.0f/32768.0f)    /* 角度范围：±180° */
#define JY901_MAG_SCALE         (1.0f)               /* 磁力计：原始值 */
#define JY901_TEMP_SCALE        (100.0f)             /* 温度：原始值/100 = °C */

/* 校准模式 */
#define JY901_CALSW_NORMAL      0x00    /* 正常模式 */
#define JY901_CALSW_ACC         0x01    /* 加速度计校准 */
#define JY901_CALSW_MAG_START   0x07    /* 开始磁力计校准 */
#define JY901_CALSW_MAG_STOP    0x00    /* 停止磁力计校准 */
#define JY901_CALSW_HEIGHT_RST  0x08    /* 高度清零 */

/* ------------------ 全局变量 ------------------ */

/* 全局寄存器数组 - 存储原始传感器数据（int16_t） */
extern int16_t JY901_sReg[0x60];

/* ------------------ 函数声明 ------------------ */

CyU3PReturnStatus_t JY901_I2C_Init(void);
void JY901_SetAddress(uint8_t addr);
CyU3PReturnStatus_t JY901_Init(void);

CyU3PReturnStatus_t JY901_ReadAll(void);
CyU3PReturnStatus_t JY901_ReadRawGyro(int16_t *gx, int16_t *gy, int16_t *gz);
CyU3PReturnStatus_t JY901_ReadRawAccel(int16_t *ax, int16_t *ay, int16_t *az);
CyU3PReturnStatus_t JY901_ReadAngles(int16_t *roll, int16_t *pitch, int16_t *yaw);
CyU3PReturnStatus_t JY901_ReadMag(int16_t *mx, int16_t *my, int16_t *mz);
CyU3PReturnStatus_t JY901_ReadTemp(int16_t *temp);
CyU3PReturnStatus_t JY901_ReadAnglesFloat(float *rollDeg, float *pitchDeg, float *yawDeg);
CyU3PReturnStatus_t JY901_ReadGyroFloat(float *gx, float *gy, float *gz);
CyU3PReturnStatus_t JY901_ReadAccelFloat(float *ax, float *ay, float *az);
CyU3PReturnStatus_t JY901_ReadTempFloat(float *tempC);
void JY901_DumpSnapshot(const char *tag);

CyU3PReturnStatus_t JY901_WriteReg(uint8_t reg, uint16_t value);
CyU3PReturnStatus_t JY901_StartAccCali(void);
CyU3PReturnStatus_t JY901_StopAccCali(void);
CyU3PReturnStatus_t JY901_StartMagCali(void);
CyU3PReturnStatus_t JY901_StopMagCali(void);
CyU3PReturnStatus_t JY901_ResetHeight(void);
CyU3PReturnStatus_t JY901_SaveConfig(void);
CyU3PReturnStatus_t Jy901_IIC_Read_Bytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);

#include "cyu3externcend.h"

#endif /* _INCLUDED_JY901_H_ */
