#include "ar0234.h"
#include "cyfxslfifosync.h"
#include "cyu3error.h"
#include "cyu3system.h"
#include "standalone_spi/cyfx_gpio_spi_standalone.h"


uint8_t glAr0234Buffer[64]  __attribute__ ((aligned (32)));  // Reduced from 1024 to 64 bytes - only needs ~10 bytes
AR0234ContextConfig_t AR0234ContextConfig;

// 温度监控相关变量（仅保留最近一次的温度值；去除10s定时器与触发标志）
static float glLastTempSensor1 = 55.5f;  // 改成 55.5 用于调试区分：没启动时为55.5，传感器读错时为55.0
static float glLastTempSensor2 = 55.5f;  // 改成 55.5 用于调试区分：没启动时为55.5，传感器读错时为55.0
static CyBool_t glIsTempMonitorActive = CyFalse; // 是否启动了后台循环读取

/* AR0234寄存器访问事务锁，避免多线程下读写序列被交叉打断。 */
static CyU3PMutex glAr0234TxnMutex;
static CyBool_t glAr0234TxnMutexReady = CyFalse;

/* 温度链路优化：使能与校准值缓存，避免每次轮询重复做重操作。 */
static CyBool_t glTempSensorEnabled = CyFalse;
static CyBool_t glTempCalibrationValid = CyFalse;
static uint16_t glTempCalibSensor1 = 0;
static uint16_t glTempCalibSensor2 = 0;

static void AR0234_TxnInit(void)
{
	if (!glAr0234TxnMutexReady)
	{
		if (CyU3PMutexCreate(&glAr0234TxnMutex, CYU3P_NO_INHERIT) == CY_U3P_SUCCESS)
		{
			glAr0234TxnMutexReady = CyTrue;
		}
	}
}

static void AR0234_TxnLock(void)
{
	if (glAr0234TxnMutexReady)
	{
		CyU3PMutexGet(&glAr0234TxnMutex, CYU3P_WAIT_FOREVER);
	}
}

static void AR0234_TxnUnlock(void)
{
	if (glAr0234TxnMutexReady)
	{
		CyU3PMutexPut(&glAr0234TxnMutex);
	}
}

static void AR0234_LoadSensor1_NoLock(uint16_t regAddr, uint16_t regData)
{
	uint8_t registerAddrHigh;
	uint8_t registerAddrLow;
	uint8_t registerDataHigh;
	uint8_t registerDataLow;

	registerAddrHigh = (uint8_t)(regAddr >> 8);
	registerAddrLow = (uint8_t)regAddr;
	registerDataHigh = (uint8_t)(regData >> 8);
	registerDataLow = (uint8_t)regData;

	CyFxSpiProtoWrite8 (0x01, 0x82, registerAddrHigh);
	CyFxSpiProtoWrite8 (0x01, 0x83, registerAddrLow);
	CyFxSpiProtoWrite8 (0x01, 0x84, registerDataHigh);
	CyFxSpiProtoWrite8 (0x01, 0x85, registerDataLow);
}

static void AR0234_LoadSensor2_NoLock(uint16_t regAddr, uint16_t regData)
{
	uint8_t registerAddrHigh;
	uint8_t registerAddrLow;
	uint8_t registerDataHigh;
	uint8_t registerDataLow;

	registerAddrHigh = (uint8_t)(regAddr >> 8);
	registerAddrLow = (uint8_t)regAddr;
	registerDataHigh = (uint8_t)(regData >> 8);
	registerDataLow = (uint8_t)regData;

	CyFxSpiProtoWrite8 (0x01, 0x92, registerAddrHigh);
	CyFxSpiProtoWrite8 (0x01, 0x93, registerAddrLow);
	CyFxSpiProtoWrite8 (0x01, 0x94, registerDataHigh);
	CyFxSpiProtoWrite8 (0x01, 0x95, registerDataLow);
}

static void AR0234_CommitLoaded_NoLock(uint8_t commitMask)
{
	CyFxSpiProtoWrite8 (0x01, 0x81, commitMask);
	CyFxSpiProtoWrite8 (0x01, 0x81, 0x00);
}

void TemperatureMonitor_Start(void)
{
	AR0234_TxnInit();
    glIsTempMonitorActive = CyTrue;
}

// I2C写操作：向AR0234传感器写入寄存器配置
CyU3PReturnStatus_t CyFxUsbI2cTransfer_AR0234_WR (
        uint8_t   devAddr,// RGB传感器：0x20, MONO传感器：0x30
        uint16_t  regAddr,// 16位寄存器地址
        uint16_t  byteCount,// 数据字节数（固定为2字节）
        uint16_t  data1)// 16位配置数据
{
    CyU3PI2cPreamble_t preamble;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint8_t  buffer[IIC_WR_AR0234_BYTES];

	// 构造I2C数据包：[设备地址] [高位寄存器] [低位寄存器] [高位数据] [低位数据]
	preamble.length    = 3;
	preamble.buffer[0] = devAddr;
	preamble.buffer[1] = (uint8_t)(regAddr >> 8);
	preamble.buffer[2] = (uint8_t)(regAddr);
	preamble.ctrlMask  = 0x0000;

	buffer[0] = (uint8_t)(data1 >> 8);
	buffer[1] = (uint8_t)data1;
	status = CyU3PI2cTransmitBytes (&preamble, buffer, byteCount, 0);
	if (status != CY_U3P_SUCCESS)
	{
		return status;
	}

    return CY_U3P_SUCCESS;
}

/* I2C read mcu(ar0234) */
CyU3PReturnStatus_t
CyFxUsbI2cTransfer_AR0234_RD (
        uint8_t   devAddr,
        uint16_t  regAddr,
        uint16_t  byteCount,
        uint8_t   *buffer)
{
    CyU3PI2cPreamble_t preamble;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	/* Update the preamble information. */
	preamble.length    = 4;
	preamble.buffer[0] = devAddr;
	preamble.buffer[1] = (uint8_t)(regAddr >> 8);
	preamble.buffer[2] = (uint8_t)(regAddr);
	preamble.buffer[3] = (devAddr | 0x01);
	preamble.ctrlMask  = 0x0004;

	status = CyU3PI2cReceiveBytes (&preamble, buffer, byteCount, 0);
	if (status != CY_U3P_SUCCESS)
	{
		return status;
	}
    return CY_U3P_SUCCESS;
}

void AR0234_Write_Sensor1(uint16_t  regAddr,uint16_t  regData)
{
	AR0234_TxnLock();
	AR0234_LoadSensor1_NoLock(regAddr, regData);
	AR0234_TxnUnlock();

	return;
}

uint16_t AR0234_read_sensor1(uint16_t  regAddr)
{
	uint8_t high_value;
	uint8_t low_value;

	high_value = CyFxSpiProtoRead8(0x01, 0x82);
	low_value = CyFxSpiProtoRead8(0x01, 0x83);
	
	return (uint16_t)((high_value << 8) | low_value);
}

void AR0234_Write_Sensor2(uint16_t  regAddr,uint16_t  regData)
{
	AR0234_TxnLock();
	AR0234_LoadSensor2_NoLock(regAddr, regData);
	AR0234_TxnUnlock();

	return;
}

void AR0234_Write_Sensor1_Commit(uint16_t regAddr, uint16_t regData)
{
	AR0234_TxnLock();
	AR0234_LoadSensor1_NoLock(regAddr, regData);
	AR0234_CommitLoaded_NoLock(0x01);
	AR0234_TxnUnlock();
}

void AR0234_Write_Sensor2_Commit(uint16_t regAddr, uint16_t regData)
{
	AR0234_TxnLock();
	AR0234_LoadSensor2_NoLock(regAddr, regData);
	AR0234_CommitLoaded_NoLock(0x10);
	AR0234_TxnUnlock();
}

void AR0234_Write_SensorsSame_Commit(uint16_t regAddr, uint16_t regData)
{
	AR0234_TxnLock();
	AR0234_LoadSensor2_NoLock(regAddr, regData);
	AR0234_LoadSensor1_NoLock(regAddr, regData);
	AR0234_CommitLoaded_NoLock(0x11);
	AR0234_TxnUnlock();
}

void AR0234_Write_SensorsIndependent_Commit(uint16_t regAddr, uint16_t sensor1Data, uint16_t sensor2Data)
{
	AR0234_TxnLock();
	AR0234_LoadSensor2_NoLock(regAddr, sensor2Data);
	AR0234_LoadSensor1_NoLock(regAddr, sensor1Data);
	AR0234_CommitLoaded_NoLock(0x11);
	AR0234_TxnUnlock();
}

//读取第一路传感器寄存器参数
uint16_t CyFxGetSensor1param(uint16_t sensoraddr)
{
	uint16_t sensor1Value = 0;
	uint16_t sensor2Value = 0;

	CyFxGetBothSensorParams (sensoraddr, &sensor1Value, &sensor2Value);
	return sensor1Value;
}


//读取第二路传感器寄存器参数
uint16_t CyFxGetSensor2param(uint16_t sensoraddr)
{
	uint16_t sensor1Value = 0;
	uint16_t sensor2Value = 0;

	CyFxGetBothSensorParams (sensoraddr, &sensor1Value, &sensor2Value);
	return sensor2Value;
}

/* 同时读取两路传感器的同一寄存器：先丢弃一次触发结果，再返回第二次实时读。
 * 第一次触发用于刷新 FPGA 状态寄存器，避免读到上一条 AR0234 事务留下的结果。
 */
void CyFxGetBothSensorParams (uint16_t sensoraddr, uint16_t *sensor1Out, uint16_t *sensor2Out)
{
	uint8_t  s1High = 0, s1Low = 0;
	uint8_t  s2High = 0, s2Low = 0;
	uint8_t  addrHigh = (uint8_t)((sensoraddr >> 8) & 0xFFu);
	uint8_t  addrLow  = (uint8_t)(sensoraddr & 0xFFu);
	uint8_t  attempt;

	AR0234_TxnLock();
	for (attempt = 0; attempt < 2u; attempt++)
	{
		CyFxSpiProtoWrite8 (0x01, 0x82, addrHigh);
		CyFxSpiProtoWrite8 (0x01, 0x83, addrLow);
		CyFxSpiProtoWrite8 (0x01, 0x92, addrHigh);
		CyFxSpiProtoWrite8 (0x01, 0x93, addrLow);
		CyFxSpiProtoWrite8 (0x01, 0x81, 0x22);
		CyFxSpiProtoWrite8 (0x01, 0x81, 0x00);
		CyU3PThreadSleep (50);
		(void)CyFxSpiProtoReadSelectedStatus8 (0x01, &s1High);
		(void)CyFxSpiProtoReadSelectedStatus8 (0x02, &s1Low);
		(void)CyFxSpiProtoReadSelectedStatus8 (0x03, &s2High);
		(void)CyFxSpiProtoReadSelectedStatus8 (0x04, &s2Low);
	}
	(void)CyFxSpiProtoWrite8 (0x02, 0xff, 0x00);
	AR0234_TxnUnlock();

	if (sensor1Out != 0) { *sensor1Out = (uint16_t)((s1High << 8) | s1Low); }
	if (sensor2Out != 0) { *sensor2Out = (uint16_t)((s2High << 8) | s2Low); }
}

/**
 * 启用温度传感器
 */
void TemperatureSensor_Enable(void)
{
    // 写入 R0x30B4 [0] = 1 和 R0x30B4 [4] = 1
    // 0x0011 = 0000 0000 0001 0001
	AR0234_Write_SensorsSame_Commit(0x30B4, 0x0011);
	#ifdef AR0234_DIAG_DEBUG
	CyU3PDebugPrint(4, "TEMP sensor-enable\n");
	#endif
}

/**
 * 读取温度校准值 (R0x30C6 [9:0])
 * @param pCalibSensor1: 指向传感器1校准值的指针（55°C时的ADC输出值，10位）
 * @param pCalibSensor2: 指向传感器2校准值的指针（55°C时的ADC输出值，10位）
 */
void TemperatureSensor_ReadCalibration(uint16_t *pCalibSensor1, uint16_t *pCalibSensor2)
{
	uint16_t calibSensor1 = 0;
	uint16_t calibSensor2 = 0;

	if ((pCalibSensor1 == NULL) && (pCalibSensor2 == NULL))
	{
		return;
	}

	CyFxGetBothSensorParams (0x30C6, &calibSensor1, &calibSensor2);

	// 只取低10位 [9:0]
	if (pCalibSensor1 != NULL)
	{
		*pCalibSensor1 = calibSensor1 & 0x03FF;
		#ifdef AR0234_DIAG_DEBUG
		CyU3PDebugPrint(4, "TEMP calib-s1=%d\n", *pCalibSensor1);
		#endif
	}
	if (pCalibSensor2 != NULL)
	{
		*pCalibSensor2 = calibSensor2 & 0x03FF;
		#ifdef AR0234_DIAG_DEBUG
		CyU3PDebugPrint(4, "TEMP calib-s2=%d\n", *pCalibSensor2);
		#endif
	}
}



/**
 * 读取温度传感器原始值 (R0x30B2 [9:0])
 * @param pTempSensor1: 指向传感器1温度原始值的指针（当前温度ADC输出值，10位）
 * @param pTempSensor2: 指向传感器2温度原始值的指针（当前温度ADC输出值，10位）
 */
void TemperatureSensor_ReadRawValue(uint16_t *pTempSensor1, uint16_t *pTempSensor2)
{
	uint16_t rawSensor1 = 0;
	uint16_t rawSensor2 = 0;

	if ((pTempSensor1 == NULL) && (pTempSensor2 == NULL))
	{
		return;
	}

	CyFxGetBothSensorParams (0x30B2, &rawSensor1, &rawSensor2);

	// 只取低10位 [9:0]
	if (pTempSensor1 != NULL)
	{
		*pTempSensor1 = rawSensor1 & 0x03FF;
		#ifdef AR0234_DIAG_DEBUG
		CyU3PDebugPrint(4, "TEMP raw-s1=%d\n", *pTempSensor1);
		#endif
	}
	if (pTempSensor2 != NULL)
	{
		*pTempSensor2 = rawSensor2 & 0x03FF;
		#ifdef AR0234_DIAG_DEBUG
		CyU3PDebugPrint(4, "TEMP raw-s2=%d\n", *pTempSensor2);
		#endif
	}
}

/**
 * 计算温度值（摄氏度）
 * 公式：温度 = 0.7 × (R0x30B2[9:0] - R0x30C6[9:0]) + 55
 * 
 * @param rawValue: 当前温度原始值 (R0x30B2[9:0])
 * @param calibValue: 校准值 (R0x30C6[9:0])
 * @return: 温度值（摄氏度）
 */
float TemperatureSensor_CalculateTemperature(uint16_t rawValue, uint16_t calibValue)
{
    // 温度 = 0.7 × (当前值 - 校准值) + 55
    float temperature = 0.7f * ((float)rawValue - (float)calibValue) + 55.0f;
    return temperature;
}


/**
 * 读取两个传感器的温度值
 * @param pTempSensor1: 指向传感器1温度值的指针（摄氏度）
 * @param pTempSensor2: 指向传感器2温度值的指针（摄氏度）
 */
void TemperatureSensor_ReadTemperature(float *pTempSensor1, float *pTempSensor2)
{
    uint16_t rawValue1, rawValue2;
	#ifdef AR0234_DIAG_DEBUG
	uint32_t tStart = CyU3PGetTime();
	#endif
    
	// 仅首次使能一次温度传感器。
	if (!glTempSensorEnabled)
	{
		TemperatureSensor_Enable();
		CyU3PThreadSleep(5);
		glTempSensorEnabled = CyTrue;
	}

	// 仅首次读取校准值，后续复用缓存。
	// 注意：只有校准值非零时才标记有效，避免I2C失败时缓存错误的0值导致后续温度计算严重偏差。
	if (!glTempCalibrationValid)
	{
		TemperatureSensor_ReadCalibration(&glTempCalibSensor1, &glTempCalibSensor2);
		if (glTempCalibSensor1 != 0 && glTempCalibSensor2 != 0)
		{
			glTempCalibrationValid = CyTrue;
		}
		#ifdef AR0234_DIAG_DEBUG
		else
		{
			CyU3PDebugPrint(4, "TEMP calib read failed (got 0), will retry\n");
		}
		#endif
	}
    
    // 读取当前温度原始值
    TemperatureSensor_ReadRawValue(&rawValue1, &rawValue2);
    
    // 计算温度
	// rawValue为0表示FPGA/I2C读取失败（sensorparam初始值），0x3FF表示总线异常；均跳过计算，输出55.0标记错误。
	if (pTempSensor1 != NULL)
	{
		if (rawValue1 == 0 || rawValue1 == 0x03FF)
		{
			*pTempSensor1 = 55.0f;  // 读取失败标记
			#ifdef AR0234_DIAG_DEBUG
			CyU3PDebugPrint(4, "TEMP raw-s1 invalid=%d, skip\n", rawValue1);
			#endif
		}
		else
		{
			*pTempSensor1 = TemperatureSensor_CalculateTemperature(rawValue1, glTempCalibSensor1);
			#ifdef AR0234_DIAG_DEBUG
			CyU3PDebugPrint(4, "TEMP calc-s1=%dC\n", (int)(*pTempSensor1));
			#endif
		}
	}
	if (pTempSensor2 != NULL)
	{
		if (rawValue2 == 0 || rawValue2 == 0x03FF)
		{
			*pTempSensor2 = 55.0f;  // 读取失败标记
			#ifdef AR0234_DIAG_DEBUG
			CyU3PDebugPrint(4, "TEMP raw-s2 invalid=%d, skip\n", rawValue2);
			#endif
		}
		else
		{
			*pTempSensor2 = TemperatureSensor_CalculateTemperature(rawValue2, glTempCalibSensor2);
			#ifdef AR0234_DIAG_DEBUG
			CyU3PDebugPrint(4, "TEMP calc-s2=%dC\n", (int)(*pTempSensor2));
			#endif
		}
	}

	#ifdef AR0234_DIAG_DEBUG
	CyU3PDebugPrint(4, "TEMP read-ms=%d\n", (int)(CyU3PGetTime() - tStart));
	#endif
}


/**
 * 获取上次读取的温度值（不重新读取传感器）
 * @param pTempSensor1: 指向传感器1温度值的指针
 * @param pTempSensor2: 指向传感器2温度值的指针
 */
void TemperatureMonitor_GetLastTemperature(float *pTempSensor1, float *pTempSensor2)
{
    if (pTempSensor1 != NULL)
    {
        *pTempSensor1 = glLastTempSensor1;
    }
    if (pTempSensor2 != NULL)
    {
        *pTempSensor2 = glLastTempSensor2;
    }
}

/**
 * 温度读取函数
 * 每次调用就读取一次（立即读取，无10秒定时与触发标志）。
 */
void TemperatureMonitor_Process(void)
{
	if (!glIsTempMonitorActive)
	{
		return;
	}

    float temp1, temp2;
    TemperatureSensor_ReadTemperature(&temp1, &temp2);

    // 仅保存合理范围内的温度值（AR0234结温范围 -40~125°C），超范围则保留上次有效值，防止I2C偶发错误污染数据。
    if (temp1 > -40.0f && temp1 < 125.0f)
    {
        glLastTempSensor1 = temp1;
    }
    else
    {
        #ifdef AR0234_DIAG_DEBUG
        CyU3PDebugPrint(4, "TEMP s1 out-of-range=%dC, keep last=%dC\n", (int)temp1, (int)glLastTempSensor1);
        #endif
    }
    if (temp2 > -40.0f && temp2 < 125.0f)
    {
        glLastTempSensor2 = temp2;
    }
    else
    {
        #ifdef AR0234_DIAG_DEBUG
        CyU3PDebugPrint(4, "TEMP s2 out-of-range=%dC, keep last=%dC\n", (int)temp2, (int)glLastTempSensor2);
        #endif
    }

    // 调试输出
    #ifdef AR0234_DIAG_DEBUG
	CyU3PDebugPrint(4, "TEMP result s1=%dC s2=%dC\n", (int)glLastTempSensor1, (int)glLastTempSensor2);
    #endif
}


//读取AR0234核心模式寄存器
void AR0234_DumpCoreModeRegisters_Sequential100ms(void)
{
	uint16_t s1, s2;
	/* 0x30B0 */
	CyFxGetBothSensorParams(0x30B0, &s1, &s2);
	#ifdef AR0234_DIAG_DEBUG
	CyU3PDebugPrint(4, "REG read reg=0x30B0 s1=%d\n", s1);
	CyU3PDebugPrint(4, "REG read reg=0x30B0 s2=%d\n", s2);
	#endif
	CyU3PThreadSleep(100);
	/* 0x30A2 */
	CyFxGetBothSensorParams(0x30A2, &s1, &s2);
	#ifdef AR0234_DIAG_DEBUG
	CyU3PDebugPrint(4, "REG read reg=0x30A2 s1=%d\n", s1);
	CyU3PDebugPrint(4, "REG read reg=0x30A2 s2=%d\n", s2);
	#endif
	CyU3PThreadSleep(100);

	/* 0x30A6 */
	CyFxGetBothSensorParams(0x30A6, &s1, &s2);
	#ifdef AR0234_DIAG_DEBUG
	CyU3PDebugPrint(4, "REG read reg=0x30A6 s1=%d\n", s1);
	CyU3PDebugPrint(4, "REG read reg=0x30A6 s2=%d\n", s2);
	#endif
	CyU3PThreadSleep(100);

	/* 0x3040 */
	CyFxGetBothSensorParams(0x3040, &s1, &s2);
	#ifdef AR0234_DIAG_DEBUG
	CyU3PDebugPrint(4, "REG read reg=0x3040 s1=%d\n", s1);
	CyU3PDebugPrint(4, "REG read reg=0x3040 s2=%d\n", s2);
	#endif
	CyU3PThreadSleep(100);

	/* 0x30AE */
	CyFxGetBothSensorParams(0x30AE, &s1, &s2);
	#ifdef AR0234_DIAG_DEBUG
	CyU3PDebugPrint(4, "REG read reg=0x30AE s1=%d\n", s1);
	CyU3PDebugPrint(4, "REG read reg=0x30AE s2=%d\n", s2);
	#endif
	CyU3PThreadSleep(100);

	/* 0x30A8 */
	CyFxGetBothSensorParams(0x30A8, &s1, &s2);
	#ifdef AR0234_DIAG_DEBUG
	CyU3PDebugPrint(4, "REG read reg=0x30A8 s1=%d\n", s1);
	CyU3PDebugPrint(4, "REG read reg=0x30A8 s2=%d\n", s2);
	#endif
	(void)s1;
	(void)s2;
}

void AR0234_Context_Init ()
{
	AR0234_TxnInit();
	glTempSensorEnabled = CyFalse;
	glTempCalibrationValid = CyFalse;
	glTempCalibSensor1 = 0;
	glTempCalibSensor2 = 0;

	AR0234ContextConfig.laserExposureSensor1 = 0x0093;   //MONO
	AR0234ContextConfig.laserExposureSensor2 = 0x0093;   //RGB
	AR0234ContextConfig.whiteLightExposureSensor1 = 0x0126;
	AR0234ContextConfig.whiteLightExposureSensor2 = 0x0126;
	AR0234ContextConfig.infraredLightExposureSensor1 = 0x0126;
	AR0234ContextConfig.infraredLightExposureSensor2 = 0x0126;

	AR0234ContextConfig.laserGainSensor1 = 0x0080;
	AR0234ContextConfig.laserGainSensor2 = 0x0080;
	AR0234ContextConfig.whiteLightGainSensor1 = 0x0080;
	AR0234ContextConfig.whiteLightGainSensor2 = 0x0080;
	AR0234ContextConfig.infraredLightGainSensor1 = 0x0080;
	AR0234ContextConfig.infraredLightGainSensor2 = 0x0080;

	return;
}


