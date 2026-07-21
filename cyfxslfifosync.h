/*
 ## Cypress USB 3.0 Platform header file (cyfxslfifosync.h)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2011,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file contains the constants and definitions used by the Slave FIFO application example */

#ifndef _INCLUDED_CYFXSLFIFOASYNC_H_
#define _INCLUDED_CYFXSLFIFOASYNC_H_

#include "cyu3externcstart.h"
#include "cyu3types.h"
#include "cyu3usbconst.h"
#include "cyu3externcend.h"
#include "cyu3os.h"
/* 16/32 bit GPIF Configuration select */
/* Set CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT = 0 for 16 bit GPIF data bus.
 * Set CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT = 1 for 32 bit GPIF data bus.
 */
#define CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT (1)

/* set up DMA channel for loopback/short packet/ZLP transfers */
//#define LOOPBACK_SHRT_ZLP

/* set up DMA channel for stream IN/OUT transfers */
#define STREAM_IN_OUT

/* configure crystal to 54Mhz(else crystal is 37.125Mhz ) */
#define EVM

/* Resistor value = 20m */
#define R20

/* 控制启动时是否对 FPGA 进行“断电重启”
 * 1 = 先断电，稍后再上电并释放复位（最干净，但上电慢，且可能引入电源抖动）
 * 0 = 不断电，仅保持复位为低，待 FX3 初始化完成后释放复位（启动更快，减少上电冲击）
 */
#ifndef POWER_CYCLE_FPGA_ON_BOOT
#define POWER_CYCLE_FPGA_ON_BOOT 0
#endif



#ifdef LOOPBACK_SHRT_ZLP
#define DMA_BUF_SIZE						  (3)
#define CY_FX_SLFIFO_DMA_BUF_COUNT_P_2_U      (2)       //2             /* Slave FIFO P_2_U channel buffer count */
#define CY_FX_SLFIFO_DMA_BUF_COUNT_U_2_P 	  (2)		//2				/* Slave FIFO U_2_P channel buffer count */
#endif

#ifdef STREAM_IN_OUT
#define DMA_BUF_SIZE					 	  (32)    //16
/* Slave FIFO P_2_U channel buffer count */
#define CY_FX_SLFIFO_DMA_BUF_COUNT_P_2_U      (4)  //2
/* Slave FIFO U_2_P channel buffer count */
//#define CY_FX_SLFIFO_DMA_BUF_COUNT_U_2_P 	  (4)
#endif

#define CY_FX_SLFIFO_DMA_TX_SIZE        (0)	                  
#define CY_FX_SLFIFO_DMA_RX_SIZE        (0)	                  
#define CY_FX_UART_DMA_TX_SIZE          (0)	                 
#define CY_FX_SLFIFO_THREAD_STACK       (0x0400)
#define CY_FX_MagneticSwitch_SIZE       (0x0200)                
#define CY_FX_SLFIFO_THREAD_PRIORITY    (8)                
#define CY_FX_GPIO_THREAD_PRIORITY      (12)
#define CY_FX_JY901_THREAD_PRIORITY     (10)           /* JY901 陀螺仪线程优先级 */


#define CY_FX_EP_PRODUCER               0x01    /* EP 1 OUT */
#define CY_FX_EP_CONSUMER               0x81    /* EP 1 IN */
#define CY_FX_EP_GYRO_IN                0x82    /* EP 2 IN 用于陀螺仪数据 */

#define CY_FX_PRODUCER_USB_SOCKET    CY_U3P_UIB_SOCKET_PROD_1    /* USB Socket 1 is producer */
#define CY_FX_CONSUMER_USB_SOCKET    CY_U3P_UIB_SOCKET_CONS_1    /* USB Socket 1 is consumer */
#define CY_FX_GYRO_USB_SOCKET        CY_U3P_UIB_SOCKET_CONS_2    /* USB Socket 2 用于陀螺仪 */


/* Used with FX3 Silicon. */
#define CY_FX_PRODUCER_PPORT_SOCKET    CY_U3P_PIB_SOCKET_0    /* P-port Socket 0 is producer */
#define CY_FX_CONSUMER_PPORT_SOCKET    CY_U3P_PIB_SOCKET_3    /* P-port Socket 3 is consumer */


#define BURST_LEN 16   //16
//  ------------- SPI BUS --------------
#define FX3_SPI_CLK             (22)/* GPIO Id 22[flagb] will be used for providing SPI Clock */
#define FX3_SPI_MOSI            (25) /* GPIO Id 25[flagc] will be used as MOSI line */
#define FX3_SPI_MISO            (26) /* GPIO Id 26[flagd] will be used as MISO line */
#define FX3_SPI_SS_FPGA         (50) /* GPIO Id 50 will be used as slave select fpga     low active */

/* 定义磁吸开关GPIO */
#define FX3_GPIO_HALL           (51) /* GPIO Id 51 will be used as hall sensor */
//  ------------- GPIO(OUTPUT) --------------
#define FPGA_PWR_EN             (23)
#define FX3_SNAP                (60)
/* U8 (SN74CBTLV3384) 通道 1 (FX3-SPI <-> U5 QSPI Flash) 使能反相输入:
 *   GPIO28 = FX3_A1 -> U7 (SN74LVC2GU04) 反相 -> U8 1OE
 *   输出 0 -> U8 1OE = 1 -> 通道 1 断开 (FPGA 独占 U5, 默认运行态)
 *   输出 1 -> U8 1OE = 0 -> 通道 1 导通 (FX3 SPI 在线读写 U5, 仅烧写 bit 时用)
 * 硬件默认 R2=4.7k 下拉保底. 该脚不控制 FPGA 的 PROGRAM_B, 请勿望文生义. */
#define FX3_ISP_ENABLE          (28)



#define BUTTON                  (27)
#define BUTTON1_ON                    (45) 
#define BUTTON2_ON                    (57) 





#define  CMOS_TEMP_CONTR_FAN_VALUE     4500   // Temperature control program fan control threshold, when scanner head temperature reaches this value
//#define  HEAT_THREAD_READ_TEMP_INTERVAL   3000  // 温度控制程序读取温度值间隔，每3000毫秒读取一次温度值



// Gyroscope register address definition
#define AX   0x34      // Acceleration x
#define AY   0x35      // Acceleration y
#define AZ   0x36      // Acceleration z

#define GX  0x37       // Gyroscope x
#define GY  0x38	   // Gyroscope y
#define GZ  0x39       // Gyroscope z

#define  Roll   0x3D    // Roll angle
#define  Pitch  0x3E    // Pitch angle
#define  Yaw    0x3F    // Yaw angle    

#define  normal_mode    0x00
#define  binning_sum    0x01
#define  binning_average 0x02
//协议指令功能定义
//A类功能
#define  CY_FX_RQT_FPGA_ANALOG                        (0xA0)
#define  CY_FX_RQT_FPGA_TEMPERATURE                   (0xA1) //(0xA1) //temperature inside device(mounted on FPGA)
#define  CY_FX_RQT_FPGA_VCC                           (0xA2)
#define  CY_FX_RQT_COMMAND_MAGNETIC                   (0xA3)  //never issued
#define  CY_FX_RQT_COMMAND_CLOSE_DEVICE               (0xA4)  //never issued
#define  CY_FX_RQT_COMMAND_OPEN_DEVICE                (0xA5)  //never issued
#define  REPORT_EXPOSURE                              (0xA6)  //曝光时间上报
#define  REPORT_GAIN                                  (0xA7)  //增益上报
#define  REPORT_OFFSET                                (0xA8)  //偏置上报
#define  REPORT_BINNING_MODE                          (0xA9)  //Binning模式上报
//B类功能
#define  CY_FX_RQT_ID_CHECK_FX3                       (0xB0) //查询FX3版本号
#define  CY_FX_RQT_ID_CHECK_FPGA                      (0xB1) //FPGA版本号
#define  CY_FX_RQT_STATUS                             (0xB2) //读取设备状态
#define  CY_FX_RQT_JY61                               (0xB3) //陀螺仪传感器指令
#define  CY_FX_RQT_BLKLEVEL                           (0xB6)
#define  CY_FX_RQT_MODE                               (0xB8)  //Mode: D[3:2]->5/25/8/9pics D[1]->trig src D[0]->data src
#define  CY_FX_RQT_STATUS_DEVICE                      (0xBE)  //读取扫描头状态
#define  CY_FX_STATUS_DETECTION                       (0xE0)  //扫描头有效开关
#define  CY_FX_BINNING_STATE						  (0xE1)  //Binning状态
#define  CY_FX_LASER_CYCLE_SETTING                    (0xE2)  //激光周期设置
#define  CY_FX_WHITE_LIGHT_CYCLE_SETTING              (0xE3)  //白光周期设置
#define  CY_FX_BLUE_LIGHT                             (0xE4)  //蓝光设置
#define  CY_FX_GREEN_LIGHT                            (0xE6)  //绿光设置
#define  CY_FX_WHITE_LIGHT                            (0xE7)  //白光设置
#define  CY_FX_RQT_GYRO_CONTROL                      (0xE8)  //陀螺仪数据流控制

/* I2C 初始化完成事件标志位 */
#define CY_FX_I2C_INIT_COMPLETE_EVENT    (1 << 0)


//C类功能
#define  CY_FX_RQT_COMMAND_INIT_RUN                   (0xC0)
#define  CY_FX_RQT_COMMAND_CAPTURE                    (0xC1) //图像采集命令
#define  CY_FX_RQT_COMMAND_INIT                       (0xC2)
#define  CY_FX_RQT_COMMAND_LED_1                      (0xC8)
#define  CY_FX_RQT_COMMAND_LED_2                      (0xC9)
#define  CY_FX_RQT_COMMAND_LED_3                      (0xCA) // uvc led
#define  CY_FX_RQT_COMMAND_UPDATE                     (0xCB)
#define  CY_FX_RQT_COMMAND_RESUME                     (0xCC)
#define  CY_FX_RQT_COMMAND_FAN                        (0xCD)
#define  CY_FX_RQT_COMMAND_HEATING                    (0xCE)
#define  CY_FX_RQT_COMMAND_STOPPROJECT                (0xCF)



//D类功能
#define  CY_FX_RQT_COMMAND_SETCURRENT1_BLUE           (0xd3) //设置激光1电流
#define  CY_FX_RQT_COMMAND_SETCURRENT2_GREEN          (0xd4) //设置激光2电流
#define  CY_FX_RQT_COMMAND_SETGAIN                    (0xd5)
#define  CY_FX_RQT_COMMAND_EXPOSURE                   (0xd6)
#define  CY_FX_RQT_COMMAND_SETCURRENT3_WHITE          (0xd7)
#define  CY_FX_RQT_COMMAND_SETOFFSET				  (0xd8)
#define  CY_FX_RQT_COMMAND_SENSOR1                    (0xdA) // SENSOR1 MONO
#define  CY_FX_RQT_COMMAND_SENSOR2                    (0xDF) // SENSOR2 RGB


//E类功能
#define  CY_FX_RQT_GAIN                               (0xe5)
#define  CY_FX_RQT_OFFSET                   		  (0xe0)
#define  CY_FX_RQT_EXPO                               (0xe9)  //Exposure time:D[15:12]->R D[11:8]->G D[7:4]->B D[3:0]->PATTERN
#define  CY_FX_RQT_CURRENT1_BLUE                      (0xeA)  //Current: DATA[3:0] -> R/G/B current
#define  CY_FX_RQT_SETCURRENT2_GREEN                  (0xeB)  //LED1: DATA[10:8] -> frequency  DATA[7:0] -> PWM
#define  CY_FX_RQT_SETCURRENT3_WHITE                  (0xeC)  //LED2: DATA[10:8] -> frequency  DATA[7:0] -> PWM
#define  CY_FX_RQT_SENSOR_TEMPERATURE                 (0xEF)  //AR0234 sensor temperature query


//F类功能
#define  CY_FX_RQT_Fpga_Read_ID                       (0xF0)
#define  CY_FX_RQT_fx3_Read_Calibration               (0xF1)
#define  CY_FX_RQT_fx3_Read_Cali_LCC_CMC              (0xF2)
#define  CY_FX_RQT_fx3_Reconfig_SPI                   (0xF3)
#define  CY_FX_RQT_Reboot_Fx3                         (0xF4)
#define  CY_FX_RQT_SPI_FLASH_READ                     (0xF5)
#define  CY_FX_RQT_SPI_FLASH_ERASE_Cali               (0xF6)
#define  CY_FX_RQT_SPI_FLASH_WRITE                    (0xF7)
#define  CY_FX_RQT_SPI_FLASH_ERASE_Cali_LCC           (0xF8)



//     Byte count per uart transfer
#define  UART_LENGTH                                  (64)
//     wait for 5s
#define  CY_FX_UART_TIMEOUT                           (5000)
#define  CY_FX_USB_SPI_TIMEOUT                        (5000)         //SPI operation time
#define  CY_FX_USBI2C_I2C_BITRATE                     (100000)


/* Function prototypes */
extern void CyFxSlFifoApplnInit (void);
extern void CyFxSlFifoApplnStart (void);
extern void CyFxSlFifoApplnStop (void);
extern void CyFxSlFifoApplnDebugInit (void);

/* 按键处理函数已移除，现在直接使用GPIO读取 */

#define JY901_ADDRESS                (0x50)

#define  TIPS_TEMPRATURE_HIGH                         (650)   //65x10
#define  TIPS_TEMPRATURE_LOW                          (450)   //45x10
#define  DEVICE_TEMPRATURE_HIGH                       (800)   //80x10
#define  DEVICE_TEMPRATURE_LOW                        (0)     //0x10
/* Extern definitions for the USB Descriptors */
extern const uint8_t CyFxUSB20DeviceDscr[];
extern const uint8_t CyFxUSB30DeviceDscr[];
extern const uint8_t CyFxUSBDeviceQualDscr[];
extern const uint8_t CyFxUSBFSConfigDscr[];
extern const uint8_t CyFxUSBHSConfigDscr[];
extern const uint8_t CyFxUSBBOSDscr[];
extern const uint8_t CyFxUSBSSConfigDscr[];
extern const uint8_t CyFxUSBStringLangIDDscr[];
extern const uint8_t CyFxUSBManufactureDscr[];
extern const uint8_t CyFxUSBProductDscr[];

#define  SECTOR_NUMBER     (8)        //M25P40 contain 8 sectors
#define  SECTOR_SIZE       (64*1024)
#define  PAGE_SIZE         (256)

#define LONG_PRESS_TIME    (70)     //约0.7s（与 ios36 保持一致；10ms/次 * 70 次）

typedef struct currentConfig_t
{
	uint32_t trigger_period;
	uint32_t led1_period;
	uint32_t led2_period;
	uint32_t led3_period;
	uint32_t laserCurrent1Blue;
	uint32_t laserCurrent2Green;
	uint32_t whiteLightCurrent;
	uint32_t infraredLightCurrent;
    uint32_t collectionPeriod; //采集周期
	uint32_t laserPeriod; //激光周期
	uint32_t blueLightPeriod; //蓝光周期
	uint32_t greenLightPeriod; //绿光周期
	uint32_t whiteLightPeriod; //白光周期
	uint32_t blueLightPWM;//蓝光占空比
	uint32_t greenLightPWM;//绿光占空比
	uint32_t whiteLightPWM;//白光占空比
}currentConfig_t;

extern CyU3PReturnStatus_t
CyFxSpiProtoWrite8 (uint8_t chip_id, uint8_t address, uint8_t data);

extern uint8_t
CyFxSpiProtoRead8 (uint8_t chip_id, uint8_t address);

/* 按键相关函数已移除，现在直接使用GPIO */

#endif /* _INCLUDED_CYFXSLFIFOASYNC_H_ */

/*[]*/
