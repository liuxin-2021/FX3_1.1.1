/*
 * JY901.c
 *
 * 基于 FX3 I2C 总线的 JY901 / JY901S 9轴惯性传感器完整驱动
 * 参考 WitMotion SDK 和 STM32 示例实现
 *
 * 使用方法：
 *   1. 在主程序中初始化 I2C：JY901_I2C_Init();
 *   2. 可选：设置设备地址 JY901_SetAddress(0x50);
 *   3. 初始化传感器：JY901_Init();
 *   4. 读取数据：
 *      - JY901_ReadAll();                         // 一次性读取所有数据到全局寄存器
 *      - JY901_ReadAngles(&roll,&pitch,&yaw);     // 读取姿态角
 *      - JY901_ReadRawGyro(&gx,&gy,&gz);          // 读取原始角速度
 *      - JY901_ReadRawAccel(&ax,&ay,&az);         // 读取原始加速度
 *      - JY901_ReadMag(&mx,&my,&mz);              // 读取磁力计
 *   5. 校准：
 *      - JY901_StartAccCali();                    // 开始加速度计校准
 *      - JY901_StartMagCali() / JY901_StopMagCali(); // 磁力计校准
 */

#include "cyu3types.h"
#include "cyu3i2c.h"
#include "cyu3error.h"
#include "cyu3system.h"
#include "cyu3utils.h"
#include "cyu3os.h"
#include "cyu3gpio.h"
#include "JY901.h"

/* ------------------ 全局变量与内部工具 ------------------ */
static uint8_t g_jy901Addr = JY901_I2C_ADDR_DEFAULT;
static CyBool_t g_jy901I2CInited = CyFalse;

/* 全局寄存器数组 - 存储从传感器读取的原始数据 */
int16_t JY901_sReg[0x60];  /* 寄存器 0x00 - 0x5F */

/* JY901 模块初始化检查 - 假设主程序已初始化 I2C 总线
 * 注意：此函数不再调用 CyU3PI2cInit()，避免与主程序的 CyFxI2cInit() 冲突
 * I2C 总线应该在主线程中通过 CyFxI2cInit() 完成初始化
 */
CyU3PReturnStatus_t JY901_I2C_Init(void)
{
	if (g_jy901I2CInited)
		return CY_U3P_SUCCESS;

	/* 标记为已初始化（实际初始化由主程序 CyFxI2cInit 完成） */
	g_jy901I2CInited = CyTrue;

	CyU3PDebugPrint(4, "JY901: I2C (GPIO58=SCL, GPIO59=SDA, 400kHz)\r\n");

	return CY_U3P_SUCCESS;
}


/* 将 I2C 传输封装：写寄存器地址后读取 len 字节（8 位寄存器地址） */
static CyU3PReturnStatus_t JY901_I2C_Read (uint8_t reg, uint8_t *buf, uint16_t len)
{
	if (buf == NULL || len == 0)
		return CY_U3P_ERROR_BAD_ARGUMENT;

	CyU3PI2cPreamble_t preamble;
	CyU3PReturnStatus_t status;

	preamble.length    = 3;                    /* 设备地址 + 寄存器地址 + 设备地址|读 */
	preamble.buffer[0] = g_jy901Addr << 1;     /* 写：设备地址 (需左移1位) */
	preamble.buffer[1] = reg;                  /* 写：寄存器地址（8位） */
	preamble.buffer[2] = (g_jy901Addr << 1) | 0x01; /* 读：设备地址 + ReadBit */
	preamble.ctrlMask  = 0x0002;               /* 在第2字节后产生重复起始，之后切换到读 */

	status = CyU3PI2cReceiveBytes(&preamble, buf, len, 0);

	return status;
}

/* I2C 写：reg 后跟若干数据字节 */
static CyU3PReturnStatus_t JY901_I2C_Write (uint8_t reg, const uint8_t *data, uint16_t len)
{
	if ((data == NULL && len > 0) || len == 0)
		return CY_U3P_ERROR_BAD_ARGUMENT;

	CyU3PI2cPreamble_t preamble;
	CyU3PReturnStatus_t status;

	/* 使用标准 I2C 写格式：前导包含设备地址 + 寄存器地址 */
	preamble.length    = 2;
	preamble.buffer[0] = g_jy901Addr << 1; /* 设备地址（需左移1位） */
	preamble.buffer[1] = reg;              /* 寄存器地址 */
	preamble.ctrlMask  = 0x0000;

	status = CyU3PI2cTransmitBytes(&preamble, (uint8_t*)data, len, 0);

	return status;
}

/* 读取 16 位寄存器（小端） */
static CyU3PReturnStatus_t JY901_ReadReg16(uint8_t reg, uint16_t *val)
{
	uint8_t buf[2];
	CyU3PReturnStatus_t st = JY901_I2C_Read(reg, buf, 2);
	if (st != CY_U3P_SUCCESS)
		return st;
	if (val)
		*val = (uint16_t)((buf[1] << 8) | buf[0]);
	return CY_U3P_SUCCESS;
}


void JY901_SetAddress(uint8_t addr)
{
	g_jy901Addr = addr;
	CyU3PDebugPrint(4, "JY901: Set device addr=%d\r\n", addr);
}

/* ------------------ 公开的读取函数 ------------------ */

/* 初始化 JY901 传感器
 * 前提条件：主程序已通过 CyFxI2cInit() 初始化 I2C 总线
 */
CyU3PReturnStatus_t JY901_Init(void)
{
	/* Mark I2C ready (bus init done by main app) */
	g_jy901I2CInited = CyTrue;

	/* Candidate addresses: default 0x50, plus common alternates (0x69,0x68,0x48). */
	const uint8_t addrs[] = {0x50, 0x69, 0x68, 0x48};
	CyU3PReturnStatus_t st = CY_U3P_ERROR_NOT_CONFIGURED;
	uint8_t testBuf[2];
	uint8_t i;

	for (i = 0; i < sizeof(addrs); ++i)
	{
		g_jy901Addr = addrs[i];
		CyU3PDebugPrint(4, "JY901: Probing addr %d\r\n", g_jy901Addr);

		/* 解锁寄存器：写 0xB588 到 0x69 后再读 */
		st = JY901_WriteReg(JY901_KEY, 0xB588);
		if (st != CY_U3P_SUCCESS)
		{
			CyU3PDebugPrint(4, "JY901: WriteReg failed at %d, status=%d\r\n", g_jy901Addr, st);
			continue;
		}

		st = JY901_I2C_Read(JY901_Roll, testBuf, 2);
		if (st == CY_U3P_SUCCESS)
		{
			CyU3PDebugPrint(4, "JY901: Found at %d\r\n", g_jy901Addr);

			/* 对齐上位机九轴：确保水平安装 + 9轴融合 */
			{
				uint16_t axis6 = 0xFFFF;
				uint16_t orient = 0xFFFF;
				CyU3PReturnStatus_t stAxis = JY901_ReadReg16(JY901_AXIS6, &axis6);
				CyU3PReturnStatus_t stOrient = JY901_ReadReg16(JY901_ORIENT, &orient);
				CyU3PDebugPrint(4, "JY901: AXIS6=%d ORIENT=%d (read %d/%d)\r\n",
					(int)axis6, (int)orient, stAxis, stOrient);
				if (stAxis == CY_U3P_SUCCESS && axis6 != JY901_ALGRITHM9)
				{
					JY901_WriteReg(JY901_KEY, 0xB588);
					CyU3PThreadSleep(2);
					JY901_WriteReg(JY901_AXIS6, JY901_ALGRITHM9);
				}
				if (stOrient == CY_U3P_SUCCESS && orient != JY901_ORIENT_HERIZONE)
				{
					JY901_WriteReg(JY901_KEY, 0xB588);
					CyU3PThreadSleep(2);
					JY901_WriteReg(JY901_ORIENT, JY901_ORIENT_HERIZONE);
				}
			}

			return CY_U3P_SUCCESS;
		}
		else
		{
			CyU3PDebugPrint(4, "JY901: Read failed at %d, status=%d\r\n", g_jy901Addr, st);
		}
	}

	return st;
}

/* 一次性读取所有常用数据到全局寄存器（加速度、陀螺仪、磁力计、角度、温度）*/
CyU3PReturnStatus_t JY901_ReadAll(void)
{
	uint8_t buf[24];
	CyU3PReturnStatus_t st;

	/* 从 0x34 开始连续读取 12 个寄存器
	 * 0x34-0x36: AX, AY, AZ (加速度)
	 * 0x37-0x39: GX, GY, GZ (陀螺仪)
	 * 0x3A-0x3C: HX, HY, HZ (磁力计)
	 * 0x3D-0x3F: Roll, Pitch, Yaw (角度)
	 */
	st = JY901_I2C_Read(JY901_AX, buf, 24);  /* 12个寄存器 × 2字节 = 24字节 */
	if (st != CY_U3P_SUCCESS)
		return st;

	/* 解析数据并存入全局寄存器数组（小端模式） */
	uint8_t i;
	for (i = 0; i < 12; i++)
	{
		JY901_sReg[JY901_AX + i] = (int16_t)((buf[i*2+1] << 8) | buf[i*2]);
	}

	return CY_U3P_SUCCESS;
}

/* 读取原始陀螺仪数据 */
CyU3PReturnStatus_t JY901_ReadRawGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
	uint8_t buf[6];
	CyU3PReturnStatus_t st = JY901_I2C_Read(JY901_GX, buf, 6);
	if (st != CY_U3P_SUCCESS) return st;
	int16_t vgx = (int16_t)((buf[1] << 8) | buf[0]);
	int16_t vgy = (int16_t)((buf[3] << 8) | buf[2]);
	int16_t vgz = (int16_t)((buf[5] << 8) | buf[4]);
	if (gx) *gx = vgx;
	if (gy) *gy = vgy;
	if (gz) *gz = vgz;

	/* 同时更新全局寄存器 */
	JY901_sReg[JY901_GX] = vgx;
	JY901_sReg[JY901_GY] = vgy;
	JY901_sReg[JY901_GZ] = vgz;

	return CY_U3P_SUCCESS;
}

/* 读取原始加速度数据 */
CyU3PReturnStatus_t JY901_ReadRawAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
	uint8_t buf[6];
	CyU3PReturnStatus_t st = JY901_I2C_Read(JY901_AX, buf, 6);
	if (st != CY_U3P_SUCCESS) return st;
	int16_t vax = (int16_t)((buf[1] << 8) | buf[0]);
	int16_t vay = (int16_t)((buf[3] << 8) | buf[2]);
	int16_t vaz = (int16_t)((buf[5] << 8) | buf[4]);
	if (ax) *ax = vax;
	if (ay) *ay = vay;
	if (az) *az = vaz;

	/* 同时更新全局寄存器 */
	JY901_sReg[JY901_AX] = vax;
	JY901_sReg[JY901_AY] = vay;
	JY901_sReg[JY901_AZ] = vaz;

	return CY_U3P_SUCCESS;
}

/* 读取原始角度数据 */
CyU3PReturnStatus_t JY901_ReadAngles(int16_t *roll, int16_t *pitch, int16_t *yaw)
{
	uint8_t buf[6];
	CyU3PReturnStatus_t st = JY901_I2C_Read(JY901_Roll, buf, 6);
	if (st != CY_U3P_SUCCESS) return st;
	int16_t vroll  = (int16_t)((buf[1] << 8) | buf[0]);
	int16_t vpitch = (int16_t)((buf[3] << 8) | buf[2]);
	int16_t vyaw   = (int16_t)((buf[5] << 8) | buf[4]);
	if (roll)  *roll  = vroll;
	if (pitch) *pitch = vpitch;
	if (yaw)   *yaw   = vyaw;

	/* 同时更新全局寄存器 */
	JY901_sReg[JY901_Roll]  = vroll;
	JY901_sReg[JY901_Pitch] = vpitch;
	JY901_sReg[JY901_Yaw]   = vyaw;

	return CY_U3P_SUCCESS;
}

/* 读取原始磁力计数据 */
CyU3PReturnStatus_t JY901_ReadMag(int16_t *mx, int16_t *my, int16_t *mz)
{
	uint8_t buf[6];
	CyU3PReturnStatus_t st = JY901_I2C_Read(JY901_HX, buf, 6);
	if (st != CY_U3P_SUCCESS) return st;
	int16_t vmx = (int16_t)((buf[1] << 8) | buf[0]);
	int16_t vmy = (int16_t)((buf[3] << 8) | buf[2]);
	int16_t vmz = (int16_t)((buf[5] << 8) | buf[4]);
	if (mx) *mx = vmx;
	if (my) *my = vmy;
	if (mz) *mz = vmz;

	/* 同时更新全局寄存器 */
	JY901_sReg[JY901_HX] = vmx;
	JY901_sReg[JY901_HY] = vmy;
	JY901_sReg[JY901_HZ] = vmz;

	return CY_U3P_SUCCESS;
}

/* 读取温度 */
CyU3PReturnStatus_t JY901_ReadTemp(int16_t *temp)
{
	uint8_t buf[2];
	CyU3PReturnStatus_t st = JY901_I2C_Read(JY901_TEMP, buf, 2);
	if (st != CY_U3P_SUCCESS) return st;
	int16_t vtemp = (int16_t)((buf[1] << 8) | buf[0]);
	if (temp) *temp = vtemp;

	/* 同时更新全局寄存器 */
	JY901_sReg[JY901_TEMP] = vtemp;

	return CY_U3P_SUCCESS;
}

/* 读取浮点角度数据（转换为度） */
CyU3PReturnStatus_t JY901_ReadAnglesFloat(float *rollDeg, float *pitchDeg, float *yawDeg)
{
	CyU3PReturnStatus_t st = JY901_ReadAll();
	if (st != CY_U3P_SUCCESS) return st;
	int16_t r = JY901_sReg[JY901_Roll];
	int16_t p = JY901_sReg[JY901_Pitch];
	int16_t y = JY901_sReg[JY901_Yaw];
	if (rollDeg)  *rollDeg  = (float)r * JY901_ANGLE_SCALE;
	if (pitchDeg) *pitchDeg = (float)p * JY901_ANGLE_SCALE;
	if (yawDeg)   *yawDeg   = (float)y * JY901_ANGLE_SCALE;
	return CY_U3P_SUCCESS;
}

/* 读取浮点陀螺仪数据（转换为 °/s） */
CyU3PReturnStatus_t JY901_ReadGyroFloat(float *gx, float *gy, float *gz)
{
	int16_t x=0,y=0,z=0;
	CyU3PReturnStatus_t st = JY901_ReadRawGyro(&x,&y,&z);
	if (st != CY_U3P_SUCCESS) return st;
	if (gx) *gx = (float)x * JY901_GYRO_SCALE;
	if (gy) *gy = (float)y * JY901_GYRO_SCALE;
	if (gz) *gz = (float)z * JY901_GYRO_SCALE;
	return CY_U3P_SUCCESS;
}

/* 读取浮点加速度数据（转换为 g） */
CyU3PReturnStatus_t JY901_ReadAccelFloat(float *ax, float *ay, float *az)
{
	int16_t x=0,y=0,z=0;
	CyU3PReturnStatus_t st = JY901_ReadRawAccel(&x,&y,&z);
	if (st != CY_U3P_SUCCESS) return st;
	if (ax) *ax = (float)x * JY901_ACCEL_SCALE;
	if (ay) *ay = (float)y * JY901_ACCEL_SCALE;
	if (az) *az = (float)z * JY901_ACCEL_SCALE;
	return CY_U3P_SUCCESS;
}

/* 读取浮点温度（转换为 °C） */
CyU3PReturnStatus_t JY901_ReadTempFloat(float *tempC)
{
	int16_t t=0;
	CyU3PReturnStatus_t st = JY901_ReadTemp(&t);
	if (st != CY_U3P_SUCCESS) return st;
	if (tempC) *tempC = (float)t / JY901_TEMP_SCALE;
	return CY_U3P_SUCCESS;
}

/* ------------------ 快照与调试 ------------------ */
void JY901_DumpSnapshot(const char *tag)
{
	uint8_t buf[24];
	CyU3PReturnStatus_t st;

	st = JY901_I2C_Read(JY901_AX, buf, 24);
	if (st != CY_U3P_SUCCESS)
		return;

	uint32_t ts = CyU3PGetTime();

	/* 解析原始值（小端） */
	int16_t ax = (int16_t)((buf[1] << 8) | buf[0]);
	int16_t ay = (int16_t)((buf[3] << 8) | buf[2]);
	int16_t az = (int16_t)((buf[5] << 8) | buf[4]);
	int16_t r  = (int16_t)((buf[19] << 8) | buf[18]);
	int16_t p  = (int16_t)((buf[21] << 8) | buf[20]);
	int16_t y  = (int16_t)((buf[23] << 8) | buf[22]);

	CyU3PDebugPrint(4, "%d %d %d %d %d %d %d %d\r\n",
		(int)ts, ax, ay, az, r, p, y, 0);
}


/* ------------------ 配置与校准函数 ------------------ */

/* 写配置寄存器 */
CyU3PReturnStatus_t JY901_WriteReg(uint8_t reg, uint16_t value)
{
	uint8_t buf[2];
	buf[0] = value & 0xFF;         /* 低字节 */
	buf[1] = (value >> 8) & 0xFF;  /* 高字节 */
	return JY901_I2C_Write(reg, buf, 2);
}

/* 开始加速度计校准 */
CyU3PReturnStatus_t JY901_StartAccCali(void)
{
	CyU3PReturnStatus_t status;
	CyU3PDebugPrint(4, "JY901: Start AccCali (place horizontally)\r\n");

	/* 解锁寄存器 */
	status = JY901_WriteReg(JY901_KEY, 0xB588);
	if (status != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint(4, "JY901: Unlock failed\r\n");
		return status;
	}
	CyU3PThreadSleep(10);

	/* 写入校准命令 */
	status = JY901_WriteReg(JY901_CALSW, JY901_CALSW_ACC);
	if (status != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint(4, "JY901: AccCali command failed\r\n");
	}
	return status;
}

/* 停止加速度计校准（恢复正常模式） */
CyU3PReturnStatus_t JY901_StopAccCali(void)
{
	CyU3PReturnStatus_t status;
	CyU3PDebugPrint(4, "JY901: End AccCali\r\n");

	/* 恢复正常模式 */
	status = JY901_WriteReg(JY901_CALSW, JY901_CALSW_NORMAL);
	if (status != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint(4, "JY901: Stop AccCali failed\r\n");
		return status;
	}
	CyU3PThreadSleep(10);

	/* 保存配置到Flash */
	status = JY901_WriteReg(JY901_SAVE, 0x0000);
	if (status != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint(4, "JY901: Save config failed\r\n");
	}
	else
	{
		CyU3PDebugPrint(4, "JY901: AccCali saved to Flash\r\n");
	}
	return status;
}

/* 开始磁力计校准 */
CyU3PReturnStatus_t JY901_StartMagCali(void)
{
	CyU3PReturnStatus_t status;
	CyU3PDebugPrint(4, "JY901: Start MagCali (rotate sensor in figure-8 pattern for 20s)\r\n");

	/* 解锁寄存器 */
	status = JY901_WriteReg(JY901_KEY, 0xB588);
	if (status != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint(4, "JY901: Unlock failed\r\n");
		return status;
	}
	CyU3PThreadSleep(20);

	/* 写入磁力计校准命令 */
	status = JY901_WriteReg(JY901_CALSW, JY901_CALSW_MAG_START);
	if (status != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint(4, "JY901: MagCali command failed\r\n");
	}
	else
	{
		CyU3PDebugPrint(4, "JY901: MagCali started, rotate sensor now!\r\n");
	}
	return status;
}

/* 停止磁力计校准 */
CyU3PReturnStatus_t JY901_StopMagCali(void)
{
	CyU3PReturnStatus_t status;
	CyU3PDebugPrint(4, "JY901: End MagCali\r\n");

	/* 解锁寄存器 */
	status = JY901_WriteReg(JY901_KEY, 0xB588);
	if (status != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint(4, "JY901: Unlock failed\r\n");
		return status;
	}
	CyU3PThreadSleep(20);

	/* 停止校准，恢复正常模式 */
	status = JY901_WriteReg(JY901_CALSW, JY901_CALSW_NORMAL);
	if (status != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint(4, "JY901: Stop MagCali failed\r\n");
		return status;
	}
	CyU3PThreadSleep(10);

	/* 保存配置到Flash */
	status = JY901_WriteReg(JY901_SAVE, 0x0000);
	if (status != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint(4, "JY901: Save config failed\r\n");
	}
	else
	{
		CyU3PDebugPrint(4, "JY901: MagCali saved to Flash\r\n");
	}
	return status;
}

/* 高度清零 */
CyU3PReturnStatus_t JY901_ResetHeight(void)
{
	CyU3PDebugPrint(4, "JY901: Reset Height\r\n");
	return JY901_WriteReg(JY901_CALSW, JY901_CALSW_HEIGHT_RST);
}

/* 保存当前配置到 FLASH */
CyU3PReturnStatus_t JY901_SaveConfig(void)
{
	CyU3PDebugPrint(4, "JY901: Save to FLASH\r\n");
	return JY901_WriteReg(JY901_SAVE, 0x0000);
}

/* ------------------ 兼容性函数 ------------------ */

/* 陀螺仪传感器JY901读取数据：先写入从机地址，再写入要读写的寄存器地址，然后读取数据 */
CyU3PReturnStatus_t
Jy901_IIC_Read_Bytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
{
    CyU3PI2cPreamble_t preamble;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint8_t retryCount = 0;

    preamble.length    = 3;
    preamble.buffer[0] = dev;        /* 从机地址（写） */
    preamble.buffer[1] = reg;        /* 寄存器地址 */
    preamble.buffer[2] = dev|0x01;   /* 从机地址（读） */
    preamble.ctrlMask  = 0x0002;

    do {
        status = CyU3PI2cReceiveBytes (&preamble, data, length, 0);
        if (status == CY_U3P_SUCCESS)
            break;
        retryCount++;
        CyU3PThreadSleep(1);
    } while (retryCount < 3);

    return status;
}
