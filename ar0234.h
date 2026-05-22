
#ifndef _INCLUDED_AR0234_H_
#define _INCLUDED_AR0234_H_

#include "cyu3i2c.h"
#include "cyu3externcstart.h"
#include "cyu3types.h"
#include "cyu3usbconst.h"
#include "cyu3externcend.h"


#ifndef AR0234_DIAG_DEBUG
#define AR0234_DIAG_DEBUG 1
#endif

#define  IIC_AR0234_RGB_ADDRESS                     (0x20)   //AR0234_RGB iic addres
#define  IIC_AR0234_MONO_ADDRESS                    (0x30)   //AR0234_RGB iic addres
#define  IIC_WR_AR0234_BYTES                        (2)
#define  IIC_RD_AR0234_BYTES                        (2)

typedef struct AR0234ContextConfig_t
{
	uint16_t laserExposureSensor1 ;
	uint16_t whiteLightExposureSensor1 ;
	uint16_t infraredLightExposureSensor1 ;
	uint16_t laserExposureSensor2 ;
	uint16_t whiteLightExposureSensor2 ;
	uint16_t infraredLightExposureSensor2 ;
	uint16_t laserGainSensor1 ;
	uint16_t whiteLightGainSensor1 ;
	uint16_t infraredLightGainSensor1 ;
	uint16_t laserGainSensor2 ;
	uint16_t whiteLightGainSensor2 ;
	uint16_t infraredLightGainSensor2 ;
	uint16_t laserOffsetSensor1_x_start;
	uint16_t laserOffsetSensor1_y_start;
	uint16_t laserOffsetSensor1_x_end;
	uint16_t laserOffsetSensor1_y_end;
	uint16_t whiteOffsetSensor1_x_start;
	uint16_t whiteOffsetSensor1_y_start;
	uint16_t whiteOffsetSensor1_x_end;
	uint16_t whiteOffsetSensor1_y_end;

	uint16_t laserOffsetSensor2_x_start;
	uint16_t laserOffsetSensor2_y_start;
	uint16_t laserOffsetSensor2_x_end;
	uint16_t laserOffsetSensor2_y_end;
	uint16_t whiteOffsetSensor2_x_start;
	uint16_t whiteOffsetSensor2_y_start;
	uint16_t whiteOffsetSensor2_x_end;
	uint16_t whiteOffsetSensor2_y_end;
}AR0234ContextConfig_t;

extern AR0234ContextConfig_t AR0234ContextConfig;

extern void TemperatureSensor_ReadCalibration(uint16_t *pCalibSensor1, uint16_t *pCalibSensor2);
extern void TemperatureSensor_ReadRawValue(uint16_t *pTempSensor1, uint16_t *pTempSensor2);
extern float TemperatureSensor_CalculateTemperature(uint16_t rawValue, uint16_t calibValue);
extern void TemperatureSensor_ReadTemperature(float *pTempSensor1, float *pTempSensor2);


extern void TemperatureMonitor_TriggerRead(void);
extern void TemperatureMonitor_Start(void);
extern void TemperatureMonitor_GetLastTemperature(float *pTempSensor1, float *pTempSensor2);
extern void TemperatureMonitor_Process(void);

extern void AR0234_Context_Init ();
extern void AR0234_Write_Sensor1(uint16_t  regAddr,uint16_t  regData);
extern void AR0234_Write_Sensor2(uint16_t  regAddr,uint16_t  regData);
extern uint16_t CyFxGetSensor1param(uint16_t sensoraddr);
extern uint16_t CyFxGetSensor2param(uint16_t sensoraddr);

extern void AR0234_DumpCoreModeRegisters_Sequential100ms(void);

extern CyU3PReturnStatus_t
CyFxUsbI2cTransfer_AR0234_WR (
		uint8_t   devAddr,
		uint16_t  regAddr,
		uint16_t  byteCount,
		uint16_t  data1);

extern CyU3PReturnStatus_t
CyFxUsbI2cTransfer_AR0234_RD (
        uint8_t   devAddr,
        uint16_t  regAddr,
        uint16_t  byteCount,
        uint8_t   *buffer);

extern void AR0234_Config (void);

extern AR0234ContextConfig_t AR0234ContextConfig;

/* FPGA register access helpers (defined in standalone_spi/cyfx_gpio_spi_standalone_template.c) */
extern uint8_t CyFxSpiProtoRead8 (uint8_t chip_id, uint8_t address);
extern CyU3PReturnStatus_t CyFxSpiProtoWrite8 (uint8_t chip_id, uint8_t address, uint8_t data);


#endif /* AR0234_H_ */
