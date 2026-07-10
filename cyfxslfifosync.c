#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3i2c.h"
#include "cyu3uart.h"
#include "cyu3spi.h"
#include "cyfxslfifosync.h"
#include "cyu3gpif.h"
#include "cyu3pib.h"
#include "pib_regs.h"
#include <cyu3gpio.h>
#include "cyu3utils.h"
#include <math.h>
#include "ar0234.h"
#include "version.h"
#include "JY901.h"
#include "cyfxgpif2config.h"
#include "standalone_spi/cyfx_gpio_spi_standalone.h"

#define CY_FX_GPIOAPP_GPIO_HIGH_EVENT    (1 << 0)   /* GPIO high event */
#define CY_FX_GPIOAPP_GPIO_LOW_EVENT     (1 << 1)   /* GPIO low event */
/* Magnetic switch state change event (used to drive the magnetic thread) */
#define CY_FX_GPIOAPP_MAGNETIC_EVENT     (1 << 2)
#define CY_FX_APP_SELF_INIT_EVENT        (1 << 3)   /* Request device self-init in thread context. */
/* Runtime IO matrix GPIO[63:32] bitmap. Keep this in sync with CyFxApplicationDefine. */
#define CY_FX_RUNTIME_GPIO_SIMPLE_EN1    (0x121C2000u)
#define CY_FX_BINNING_SETTLE_DELAY_MS    (10u)
#define CY_FX_BINNING_VERIFY_RETRY_DELAY_MS (20u)
//#define DEF_UART_BAUDRATE                (115200)    /* Baud rate for UART communication. */
CyU3PEvent glFxGpioAppEvent;            /* GPIO input event group. */
CyU3PThread slHeatingAppThread;            /* Temperature monitor application thread structure  */
CyU3PThread slMagneticSwitchAppThread;            /* Slave Heating application thread structure  */
CyU3PThread slFifoAppThread;	        /* Slave FIFO application thread structure */
CyU3PDmaChannel glChHandleSlFifoUtoP;   /* DMA Channel handle for U2P transfer. */
CyU3PDmaChannel glChHandleSlFifoPtoU;   /* DMA Channel handle for P2U transfer. */
CyU3PDmaChannel glChHandleGyro;         /* DMA Channel handle for Gyro (CPU->USB). */
CyU3PDmaChannel glUARTRxHandle;         /* UART Rx channel handle */
CyU3PDmaChannel glSpiTxHandle;          /* SPI Tx channel handle */
CyU3PDmaChannel glSpiRxHandle;          /* SPI Rx channel handle */

uint32_t glDMARxCount = 0;               /* Counter to track the number of buffers received from USB. */
uint32_t glDMATxCount = 0;               /* Counter to track the number of buffers sent to USB. */
CyBool_t glIsApplnActive = CyFalse;      /* Whether the loopback application is active or not. */
CyBool_t gJY901Enabled   = CyFalse;      /* 陀螺仪数据流使能标志 */

CyU3PEvent glFxI2cEvent;                 /* I2C 初始化完成事件 */
CyU3PThread slJY901AppThread;            /* JY901 陀螺仪线程 */

CyBool_t glIsSnapActive = CyFalse;       /* The signal of snap is active or not. */
CyBool_t glIsPingpangActive = CyFalse;   /* The signal of pingpang is active or not. */

CyBool_t glIsCapMode     = CyFalse;      /*工作模式：工作模式：C:扫描开*/
CyBool_t glIsDeviceRun   = CyFalse;       /* Whether the device is working or not. */
CyBool_t glIsInitialized = CyFalse;       /* 业务 ready 状态：设备端自初始化完成，可正常处理 B2 */
CyBool_t glFpgaPoweredReady = CyFalse;    /* 硬件状态：FPGA 供电且已释放复位，可进行交互 */
uint8_t burstLength =0;
uint16_t glwValue   =0;
uint16_t glwIndex   =0;
uint16_t glSpiPageSize = 0x100;          /* SPI Page size to be used for transfers. */
CyBool_t glIsReconfigure   = CyFalse;       /* Whether the device is working or not. */
uint8_t  glMode   =  0;        /* Mode: DATA[3:2]->5/25/8/9pics DATA[1]->trig src DATA[0]->data src*/

uint16_t glLED_1   = 0;        /* LED1: DATA[10:8] -> frequency  DATA[7:0] -> duty */
uint16_t glLED_2   = 0;        /* LED2: DATA[10:8] -> frequency  DATA[7:0] -> duty */
CyBool_t glInResume     = CyFalse;       /* Whether the start/stop operation is in progress. */
uint8_t  glTemperature[3];
uint8_t  glStatus_Device[1];
uint8_t  glStatus_Extra;                 /* bit[7]: long press  bit[6]: short press */
uint8_t  glStatus_FPGA;                  /* alarm_all,ot_out,user_temp_alarm_out,res res,vccint,vccaux,vccbram */
CyBool_t glPress_Detect  = CyTrue;      //按键检测
CyBool_t glMagneticSwitch  = CyFalse;   //磁吸开关
currentConfig_t currentData;
uint8_t  glsaomswitchState = 1;          //扫描头检测失效标志

/* Standalone GPIO-SPI worker is initialized once and kept alive across USB reconnects. */
static CyBool_t gSpiStandaloneInited = CyFalse;
static uint32_t glLastStopTime = 0u;
static uint8_t gOffsetTxnExpectedIndex = 1u;
static uint8_t gOffsetTxnReceivedMask = 0u;

typedef enum CyFxDeviceReadyState_e {
	CY_FX_DEVICE_READY_NOT_READY = 0,
	CY_FX_DEVICE_READY_INIT_PENDING,
	CY_FX_DEVICE_READY_INIT_IN_PROGRESS,
	CY_FX_DEVICE_READY_READY
} CyFxDeviceReadyState_t;

#define CY_FX_SELF_INIT_WVALUE           (0x000Cu)
#define CY_FX_SELF_INIT_WINDEX           (0u)

static volatile CyFxDeviceReadyState_t glDeviceReadyState = CY_FX_DEVICE_READY_NOT_READY;

static void CyFxAr0234WritePairAndCommit (uint16_t regAddr, uint16_t regData);
static void CyFxAr0234WriteIndependentAndCommit (uint16_t regAddr, uint16_t sensor1Data, uint16_t sensor2Data);
static CyBool_t CyFxWriteBinningModeRegisters (uint8_t mode, uint16_t s1AXEnd, uint16_t s2AXEnd, uint16_t s1BXEnd, uint16_t s2BXEnd);
static CyBool_t CyFxApplyBinningMode (uint8_t mode);
static CyBool_t CyFxVerifyBinningModeReadback (uint8_t mode, uint16_t s1AXEnd, uint16_t s2AXEnd, uint16_t s1BXEnd, uint16_t s2BXEnd);
static CyBool_t CyFxVerifySensorRegPair (uint16_t regAddr, uint16_t expectedSensor1Data, uint16_t expectedSensor2Data);
static CyBool_t CyFxVerifyOffsetReadback (void);
static CyBool_t CyFxVerifyGainReadback (void);
static CyBool_t CyFxVerifyExposureReadback (void);
static void CyFxApplyOffsetRegisters (void);
static void CyFxUpdateDeviceReadyState (CyFxDeviceReadyState_t newState);
static void CyFxScheduleSelfInit (void);
static void CyFxRunSelfInit (void);
//static void WaitFPGAStable (void);
static void CyFxWaitForAr0234InitReady (void);
CyU3PReturnStatus_t CyFxGetFPGAVersion (uint8_t *data, uint16_t length);
void CyFxDeviceInit (uint16_t wValue, uint16_t wIndex, CyBool_t powerCycleFpga);

static uint16_t
CyFxAppendVersionDecimal (uint8_t *data, uint16_t length, uint16_t offset, uint8_t value)
{
	if (value < 10u)
	{
		if (offset + 1u < length)
		{
			data[offset++] = (uint8_t)('0' + value);
		}
	}
	else if (value < 100u)
	{
		if (offset + 1u < length)
		{
			data[offset++] = (uint8_t)('0' + (value / 10u));
		}
		if (offset + 1u < length)
		{
			data[offset++] = (uint8_t)('0' + (value % 10u));
		}
	}
	else
	{
		if (offset + 1u < length)
		{
			data[offset++] = (uint8_t)('0' + (value / 100u));
		}
		if (offset + 1u < length)
		{
			data[offset++] = (uint8_t)('0' + ((value % 100u) / 10u));
		}
		if (offset + 1u < length)
		{
			data[offset++] = (uint8_t)('0' + (value % 10u));
		}
	}

	return offset;
}

/* Simplified state management - direct access for better performance and less memory */
static inline void SetDeviceState(CyBool_t isDeviceRun, CyBool_t isInitialized)
{
    glIsDeviceRun = isDeviceRun;
    glIsInitialized = isInitialized;
}

static void
CyFxUpdateDeviceReadyState (CyFxDeviceReadyState_t newState)
{
	glDeviceReadyState = newState;
	glIsInitialized = (newState == CY_FX_DEVICE_READY_READY) ? CyTrue : CyFalse;
}

static void
CyFxScheduleSelfInit (void)
{
	if (!glIsApplnActive)
	{
		CyFxUpdateDeviceReadyState (CY_FX_DEVICE_READY_NOT_READY);
		return;
	}

	if (glDeviceReadyState == CY_FX_DEVICE_READY_NOT_READY)
	{
		CyFxUpdateDeviceReadyState (CY_FX_DEVICE_READY_INIT_PENDING);
		CyU3PEventSet (&glFxGpioAppEvent, CY_FX_APP_SELF_INIT_EVENT, CYU3P_EVENT_OR);
	}
}

static inline CyBool_t
CyFxDeviceIsReady (void)
{
	return (glDeviceReadyState == CY_FX_DEVICE_READY_READY) ? CyTrue : CyFalse;
}

static void
CyFxRunSelfInit (void)
{
	if ((!glIsApplnActive) || (glDeviceReadyState != CY_FX_DEVICE_READY_INIT_PENDING))
	{
		return;
	}

	CyFxUpdateDeviceReadyState (CY_FX_DEVICE_READY_INIT_IN_PROGRESS);
	glMode = 0x0c;
	CyFxDeviceInit (CY_FX_SELF_INIT_WVALUE, CY_FX_SELF_INIT_WINDEX, CyFalse);
}

static inline void SetFpgaPoweredReady(CyBool_t isReady)
{
	glFpgaPoweredReady = isReady;
}

static inline void SetSnapState(CyBool_t isSnapActive, CyBool_t isPingpangActive)
{
    glIsSnapActive = isSnapActive;
    glIsPingpangActive = isPingpangActive;
}

static inline void CyFxResetOffsetTxnState (void)
{
	gOffsetTxnExpectedIndex = 1u;
	gOffsetTxnReceivedMask = 0u;
}

CyBool_t SclGpioValue = CyTrue;

//uint8_t Jy901ReceiveBuf[200];

uint16_t  TempAddTimes;
//按键按下次数
uint8_t KeyAddTimes = 0;


//按键不正常的次数；

uint8_t key1TimesTrue,key1TimesFalse;
uint8_t key2TimesTrue,key2TimesFalse;



uint8_t glFpgaVersion[32] __attribute__((aligned(32)));
uint8_t glEp0Buffer[4096]  __attribute__ ((aligned (32)));
uint8_t glEp0Calibration[4096]  __attribute__ ((aligned (32)));
uint8_t glEp0FpgaID[256]  __attribute__ ((aligned (32)));
uint8_t glEp0CaliLccCMC[512]  __attribute__ ((aligned (32)));

CyBool_t  button1[10];
CyBool_t  button2[10];

#define CY_FX_STATUS_DEVICE_SAMPLE_WAIT_US        (300)
#define CY_FX_STATUS_DEVICE_SAMPLE_COUNT          (10)
#define CY_FX_STOP_START_SETTLE_MS                (1000u)

static void
CyFxReadTouchSwitchFiltered (CyBool_t *touch1, CyBool_t *touch2)
{
	uint8_t idx;
	uint8_t key1High = 0;
	uint8_t key1Low  = 0;
	uint8_t key2High = 0;
	uint8_t key2Low  = 0;
	CyBool_t key1Sample = CyTrue;
	CyBool_t key2Sample = CyTrue;

	for (idx = 0; idx < CY_FX_STATUS_DEVICE_SAMPLE_COUNT; idx++)
	{
		CyU3PGpioGetValue (BUTTON1_ON, &key1Sample);
		CyU3PGpioGetValue (BUTTON2_ON, &key2Sample);

		if (key1Sample == CyFalse)
			key1Low++;
		else
			key1High++;

		if (key2Sample == CyFalse)
			key2Low++;
		else
			key2High++;

		if ((idx + 1) < CY_FX_STATUS_DEVICE_SAMPLE_COUNT)
		{
			CyU3PBusyWait (CY_FX_STATUS_DEVICE_SAMPLE_WAIT_US);
		}
	}

	*touch1 = (key1High > key1Low) ? CyTrue : CyFalse;
	*touch2 = (key2High > key2Low) ? CyTrue : CyFalse;
}
#if 0
static void
WaitFPGAStable (void)
{
	CyU3PReturnStatus_t status = CY_U3P_ERROR_TIMEOUT;
	uint8_t stableMark = 0u;
	for (;;)
	{
		status = CyFxSpiProtoReadSelectedStatus8 (0x00u, &stableMark);
		CyU3PDebugPrint (4, "WaitFPGAStable: stableMark=%d\n",(int)stableMark);
		CyU3PThreadSleep (20);
	}
}
#endif

/* AR0234 回读校验总超时（ms）。到期后强制返回就绪，避免因 SPI 读不通死循环。 */
#define CY_FX_AR0234_INIT_READY_TIMEOUT_MS  (20000u)

static void
CyFxWaitForAr0234InitReady (void)
{
	const uint16_t yStart = 10u;
	const uint16_t xStart = 20u;
	const uint16_t yEnd = 999u;
	const uint16_t xEnd = 1299u;
	uint16_t s1yStart = 0u;
	uint16_t s2yStart = 0u;
	uint16_t s1xStart = 0u;
	uint16_t s2xStart = 0u;
	uint16_t s1yEnd = 0u;
	uint16_t s2yEnd = 0u;
	uint16_t s1xEnd = 0u;
	uint16_t s2xEnd = 0u;
	uint32_t startTime = CyU3PGetTime ();

	for (;;)
	{
		CyFxAr0234WritePairAndCommit (0x3002, yStart);
		CyFxAr0234WritePairAndCommit (0x3004, xStart);
		CyFxAr0234WritePairAndCommit (0x3006, yEnd);
		CyFxAr0234WritePairAndCommit (0x3008, xEnd);

		CyFxGetBothSensorParams (0x3002, &s1yStart, &s2yStart);
		CyFxGetBothSensorParams (0x3004, &s1xStart, &s2xStart);
		CyFxGetBothSensorParams (0x3006, &s1yEnd, &s2yEnd);
		CyFxGetBothSensorParams (0x3008, &s1xEnd, &s2xEnd);

		if ((s1yStart == yStart) && (s2yStart == yStart) &&
			(s1xStart == xStart) && (s2xStart == xStart) &&
			(s1yEnd == yEnd) && (s2yEnd == yEnd) &&
			(s1xEnd == xEnd) && (s2xEnd == xEnd))
		{
			break;
		}

		CyU3PDebugPrint (4, "CyFxWaitForAr0234InitReady: s1yStart=%d, s2yStart=%d, s1xStart=%d, s2xStart=%d, s1yEnd=%d, s2yEnd=%d, s1xEnd=%d, s2xEnd=%d\n",
				s1yStart, s2yStart, s1xStart, s2xStart,
				s1yEnd, s2yEnd, s1xEnd, s2xEnd);

		if ((CyU3PGetTime () - startTime) >= CY_FX_AR0234_INIT_READY_TIMEOUT_MS)
		{
			CyU3PDebugPrint (4, "CyFxWaitForAr0234InitReady: timeout after %d ms, force ready.\n",
					(int)CY_FX_AR0234_INIT_READY_TIMEOUT_MS);
			break;
		}
	}
}


void CyFxAppErrorHandler (CyU3PReturnStatus_t apiRetStatus)/* API return status */
{
    for (;;)
    {
        /* Thread sleep : 100 ms */
        CyU3PThreadSleep (100);
    }
}


void CyFxSlFifoApplnDebugInit (void)
{
    CyU3PUartConfig_t uartConfig;
    CyU3PDmaChannelConfig_t dmaConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    /* Initialize the UART for printing debug messages */
    apiRetStatus = CyU3PUartInit();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyFxAppErrorHandler(apiRetStatus);
    }
    //
    /* Set UART configuration */
    CyU3PMemSet ((uint8_t *)&uartConfig, 0, sizeof (uartConfig));
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyTrue;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma    = CyTrue;

    apiRetStatus = CyU3PUartSetConfig (&uartConfig, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* STEP3:Now create the DMA channels required for read and write. */
	  CyU3PMemSet ((uint8_t *)&dmaConfig, 0, sizeof(dmaConfig));
	  dmaConfig.size           = UART_LENGTH;
	  /* No buffers need to be allocated as this will be used
	   * only in override mode. */
	  dmaConfig.count          = 0;
	  dmaConfig.prodAvailCount = 0;
	  dmaConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
	  dmaConfig.prodHeader     = 0;
	  dmaConfig.prodFooter     = 0;
	  dmaConfig.consHeader     = 0;
	  dmaConfig.notification   = 0;
	  dmaConfig.cb             = NULL;

	  /* Create a channel to read from the UART. */
	  dmaConfig.prodSckId = CY_U3P_LPP_SOCKET_UART_PROD;
	  dmaConfig.consSckId = CY_U3P_CPU_SOCKET_CONS;
	  apiRetStatus = CyU3PDmaChannelCreate (&glUARTRxHandle, CY_U3P_DMA_TYPE_MANUAL_IN, &dmaConfig);
	  if (apiRetStatus != CY_U3P_SUCCESS)
	  {
		  CyFxAppErrorHandler(apiRetStatus);
	  }

    /* Set the UART transfer to a really large value. */
    apiRetStatus = CyU3PUartTxSetBlockXfer (0xFFFFFFFF);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
    apiRetStatus = CyU3PUartRxSetBlockXfer (0xFFFFFFFF);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyFxAppErrorHandler(apiRetStatus);
    }

    apiRetStatus = CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 8);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
}

/* Transfer data from UART DMA operation. */
CyU3PReturnStatus_t CyFxUARTDMATransfer (uint8_t  *buffer)
{
    CyU3PDmaBuffer_t buf_p;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Update the buffer address. */
     buf_p.buffer = buffer;
     buf_p.status = 0;
     buf_p.size   = UART_LENGTH;
	 buf_p.count  = UART_LENGTH;

    status = CyU3PDmaChannelSetupRecvBuffer (&glUARTRxHandle, &buf_p);
     if (status != CY_U3P_SUCCESS)
     {
         return status;
     }
     status = CyU3PDmaChannelWaitForCompletion(&glUARTRxHandle, CY_FX_UART_TIMEOUT);
     if (status != CY_U3P_SUCCESS)
     {
         return status;
     }

    return CY_U3P_SUCCESS;
}

/* I2c initialization for DLPC3470 programming. */
CyU3PReturnStatus_t CyFxI2cInit (void)
{
    CyU3PI2cConfig_t i2cConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize and configure the I2C master module. */
    status = CyU3PI2cInit ();
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Start the I2C master block. The bit rate is set at 100KHz.
     * The data transfer is done via BUFFER. */
    CyU3PMemSet ((uint8_t *)&i2cConfig, 0, sizeof(i2cConfig));
    i2cConfig.bitRate    = CY_FX_USBI2C_I2C_BITRATE;
    i2cConfig.busTimeout = 0xFFFFFFFF;
    i2cConfig.dmaTimeout = 0xFFFF;
    i2cConfig.isDma      = CyFalse;

    status = CyU3PI2cSetConfig (&i2cConfig, NULL);
    return status;
}

//数据传输启动流程
void CyFxSlFifoApplnStart (void)
{
    uint16_t size = 0;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();

	 // 1. 根据USB速度配置参数
    switch (usbSpeed)
    {
        case CY_U3P_FULL_SPEED:
            size = 64;
            break;

        case CY_U3P_HIGH_SPEED:
            size = 512;
            burstLength=1;
            break;

        case  CY_U3P_SUPER_SPEED:
            size = 1024;
            burstLength=16; //突发指的是不需要接收端的单独ACK信令的一系列BULK数据包；
            break;

        default:
            #ifdef DEBUG
            CyU3PDebugPrint (4, "Error! Invalid USB speed.\n");
            #endif
            CyFxAppErrorHandler (CY_U3P_ERROR_FAILURE);
            break;
    }

    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
	
    #ifdef STREAM_IN_OUT
       epCfg.burstLen = burstLength;
    #else
       epCfg.burstLen = BURST_LEN;
    #endif
	
    epCfg.streams = 0;
    epCfg.pcktSize = size;


	 // 3. 只配置Consumer端点(EP1 IN)
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

    /* 如果不使用MANUAL模式，则使用AUTO模式 */
    dmaCfg.size  = (DMA_BUF_SIZE*size); // 缓冲区大小：32 * 1024 = 32KB
    dmaCfg.count = CY_FX_SLFIFO_DMA_BUF_COUNT_P_2_U; // 缓冲区数量：4个
    dmaCfg.prodSckId = CY_FX_PRODUCER_PPORT_SOCKET;// 并行端口socket 0
    dmaCfg.consSckId = CY_FX_CONSUMER_USB_SOCKET; // USB socket 1
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;// 字节模式传输

    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;

	apiRetStatus = CyU3PDmaChannelCreate (&glChHandleSlFifoPtoU,CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

	apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_RX_SIZE);//DMA传输启动
    
	//这个函数无意义，需要改                                                                                              
	if (apiRetStatus != CY_U3P_SUCCESS)
	 {
		 CyFxAppErrorHandler(apiRetStatus);
	 }

    /* 配置 Gyro EP (EP2 IN) 并创�?DMA 通道 (CPU->USB) */
    epCfg.burstLen = 1;
    epCfg.pcktSize = (usbSpeed == CY_U3P_SUPER_SPEED) ? 1024 :
                     (usbSpeed == CY_U3P_HIGH_SPEED)  ? 512  : 64;
	apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_GYRO_IN, &epCfg);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

    CyU3PMemSet((uint8_t *)&dmaCfg, 0, sizeof(dmaCfg));
    dmaCfg.size      = size;
    dmaCfg.count     = 2;
    dmaCfg.prodSckId = CY_U3P_CPU_SOCKET_PROD;
    dmaCfg.consSckId = CY_FX_GYRO_USB_SOCKET;
    dmaCfg.dmaMode   = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = 0;
    dmaCfg.cb        = NULL;
	apiRetStatus = CyU3PDmaChannelCreate(&glChHandleGyro, CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaCfg);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}
    CyU3PUsbFlushEp(CY_FX_EP_GYRO_IN);
	apiRetStatus = CyU3PDmaChannelSetXfer(&glChHandleGyro, 0);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* Keep FPGA in reset until USB endpoints and DMA paths are ready. */
	CyU3PGpioSetValue (FX3_DEVICE_RESET, CyTrue);
	CyU3PThreadSleep (50);
	SetFpgaPoweredReady(CyTrue);

    /* Update the status flag. */
    glIsApplnActive = CyTrue;//应用激活标志

}


 /*添加完整的资源清理 */
void CyFxSlFifoApplnStop(void)
{
    CyU3PEpConfig_t epCfg;
    glIsApplnActive = CyFalse;
    CyU3PReturnStatus_t apiRetStatus;

	/* Quiesce producer side first to avoid stop-window overrun on PIB thread0. */
	CyU3PGpifSMControl(CyTrue);
	CyU3PGpioSetValue (FX3_SNAP, CyTrue);
	CyFxSpiProtoWrite8 (0x01, 0x11, 0x01);
	CyU3PThreadSleep (2);

	CyU3PDmaChannelReset (&glChHandleSlFifoPtoU);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
	CyU3PUsbResetEp (CY_FX_EP_CONSUMER);

    CyU3PDmaChannelDestroy (&glChHandleSlFifoPtoU);

    /* 清理陀螺仪 DMA 通道 */
    CyU3PUsbFlushEp(CY_FX_EP_GYRO_IN);
    CyU3PDmaChannelDestroy(&glChHandleGyro);

    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
	epCfg.enable = CyFalse;
	apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
		#ifdef DEBUG
        CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
		#endif
        CyFxAppErrorHandler (apiRetStatus);
    }
	apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_GYRO_IN, &epCfg);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		#ifdef DEBUG
		CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
		#endif
		CyFxAppErrorHandler (apiRetStatus);
	}
}


/* FPGA SPI proto helpers are implemented in standalone_spi/cyfx_gpio_spi_standalone_template.c. */


void Current_Init ()
{

	currentData.trigger_period = 0x0017D784;
	currentData.led1_period = 0x0017D784;
	currentData.led2_period = 0x0017D784;
	currentData.led3_period = 0x0017D784;
	currentData.laserCurrent1Blue = 0x0002625A;
	currentData.laserCurrent2Green = 0x0002625A;
	currentData.whiteLightCurrent = 0x0002625A;
	currentData.infraredLightCurrent = 0x0002625A;

	return;
}

void Current_Device_Init ()
{
	CyFxSpiProtoWrite8 (0x01, 0x96, (uint8_t)(currentData.trigger_period >> 24));
	CyFxSpiProtoWrite8 (0x01, 0x97, (uint8_t)(currentData.trigger_period >> 16));
	CyFxSpiProtoWrite8 (0x01, 0x98, (uint8_t)(currentData.trigger_period >> 8));
	CyFxSpiProtoWrite8 (0x01, 0x99, (uint8_t)(currentData.trigger_period));

	CyFxSpiProtoWrite8 (0x01, 0x9a, (uint8_t)(currentData.led1_period >> 16));
	CyFxSpiProtoWrite8 (0x01, 0x9b, (uint8_t)(currentData.led1_period >> 8));
	CyFxSpiProtoWrite8 (0x01, 0x9c, (uint8_t)(currentData.led1_period ));

	CyFxSpiProtoWrite8 (0x01, 0x9d, (uint8_t)(currentData.led2_period >> 16));
	CyFxSpiProtoWrite8 (0x01, 0x9e, (uint8_t)(currentData.led2_period >> 8));
	CyFxSpiProtoWrite8 (0x01, 0x9f, (uint8_t)(currentData.led2_period ));

	CyFxSpiProtoWrite8 (0x01, 0xa1, (uint8_t)(currentData.led3_period >> 16));
	CyFxSpiProtoWrite8 (0x01, 0xa2, (uint8_t)(currentData.led3_period >> 8));
	CyFxSpiProtoWrite8 (0x01, 0xa3, (uint8_t)(currentData.led3_period ));

	CyFxSpiProtoWrite8 (0x01, 0xa4, (uint8_t)(currentData.laserCurrent1Blue >> 16));
	CyFxSpiProtoWrite8 (0x01, 0xa5, (uint8_t)(currentData.laserCurrent1Blue >> 8));
	CyFxSpiProtoWrite8 (0x01, 0xa6, (uint8_t)(currentData.laserCurrent1Blue ));

	CyFxSpiProtoWrite8 (0x01, 0xa7, (uint8_t)(currentData.laserCurrent2Green >> 16));
	CyFxSpiProtoWrite8 (0x01, 0xa8, (uint8_t)(currentData.laserCurrent2Green >> 8));
	CyFxSpiProtoWrite8 (0x01, 0xa9, (uint8_t)(currentData.laserCurrent2Green ));

	CyFxSpiProtoWrite8 (0x01, 0xaa, (uint8_t)(currentData.whiteLightCurrent >> 16));
	CyFxSpiProtoWrite8 (0x01, 0xab, (uint8_t)(currentData.whiteLightCurrent >> 8));
	CyFxSpiProtoWrite8 (0x01, 0xac, (uint8_t)(currentData.whiteLightCurrent ));

	return;
}

/*
  Stop flow:
    1. stop dlp;
    2. standby sensor;
    3. stop bitslip;
    4. reset fpga;
 */
void CyFxButtonPressed_Stop(void)
{
	glInResume = CyTrue;


	glIsDeviceRun = CyFalse;
	CyU3PGpifSMControl(CyTrue);    // PAUSE GPIF State machine
	//  停止内部图案并拉低投影使能，确保投影实际熄灭
	//CyFxSpiProtoWrite8 (0x01, 0x03, 0x00);               // PROJ_OFF
	//CyU3PThreadSleep (10);
	CyU3PGpioSetValue (FX3_SNAP, CyTrue);          //RESET FIFO&STATE MACHINE
	CyFxSpiProtoWrite8 (0x01, 0x11, 0x01);        // RESET pingpang working
	CyU3PThreadSleep (10);
	CyU3PGpioSetValue (FX3_DEVICE_RESET, CyFalse); //reset FPGA
	CyU3PThreadSleep (50);
	SetFpgaPoweredReady(CyFalse);

	//STEP5: RESET DMA BUFFER
	if(glIsApplnActive)
    {
		CyU3PDmaChannelReset (&glChHandleSlFifoPtoU);
		CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
		CyU3PUsbResetEp(CY_FX_EP_CONSUMER);
		CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_RX_SIZE);
	}
	CyFxSpiProtoWrite8 (0x01, 0x0B, 0x01);   // open led2 send command，蓝色灯亮；//
	CyU3PThreadSleep (100);

	glLastStopTime = CyU3PGetTime();
	glInResume    = CyFalse;
	return;
}



void CyFxButtonPressed_Start(void)
{
	uint32_t elapsed;

	glInResume = CyTrue;
	if (glLastStopTime != 0u)
	{
		elapsed = CyU3PGetTime() - glLastStopTime;
		if (elapsed < CY_FX_STOP_START_SETTLE_MS)
		{
			CyU3PThreadSleep(CY_FX_STOP_START_SETTLE_MS - elapsed);
		}
	}
	CyU3PGpioSetValue (FX3_SNAP, CyTrue);          //RESET FIFO&STATE MACHINE
	CyFxSpiProtoWrite8 (0x01, 0x11, 0x01);        // RESET pingpang working
	CyU3PThreadSleep (10);
	CyU3PGpioSetValue (FX3_DEVICE_RESET, CyTrue);  //FPGA starts working
	CyU3PThreadSleep (50);
	SetFpgaPoweredReady(CyTrue);

	if(glIsApplnActive)
	{
		CyU3PDmaChannelReset (&glChHandleSlFifoPtoU);
		CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
		CyU3PUsbResetEp(CY_FX_EP_CONSUMER);
		CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_RX_SIZE);
	}
	CyU3PThreadSleep (10);

	/* Start the state machine. */
	CyU3PGpifSMControl(CyFalse);        // RESUME GPIF State machine
	CyU3PThreadSleep (10);
	CyFxSpiProtoWrite8 (0x01, 0x0B, 0x00);   // open led2 send command，蓝色灯灭；//
	CyU3PThreadSleep (100);

	glIsDeviceRun = CyTrue;
	glInResume    = CyFalse;
	glIsSnapActive = CyTrue;

	return;
}



void CyFxDeviceInit (uint16_t wValue, uint16_t wIndex, CyBool_t powerCycleFpga)
{
	glInResume = CyTrue;
	
	glwValue = wValue;  //store arguments for CyFxButtonPressed_Start();
	glwIndex = wIndex;  //store arguments for CyFxButtonPressed_Start();
    CyFxUpdateDeviceReadyState (CY_FX_DEVICE_READY_INIT_IN_PROGRESS);
	if (powerCycleFpga)
	{
		CyU3PGpioSetValue (FPGA_PWR_EN, CyFalse);   //FPGA power_off   comment@0126
		SetFpgaPoweredReady(CyFalse);
		//CyU3PGpioSetValue (DLP_PWR_EN,  CyFalse);   //DLP  power_off
		CyU3PThreadSleep (100);                      //delay 100ms;

		CyU3PGpioSetValue (FPGA_PWR_EN, CyTrue);    //FPGA power on
		CyU3PThreadSleep (1000); 
	}

	glIsPingpangActive = CyTrue;           //open pingpang（恒开，对齐 ios 行为）

	CyU3PGpifSMControl(CyTrue);     // STEP2: 暂停GPIF状态机

	CyU3PGpioSetValue (FX3_SNAP, CyTrue);           // 复位FIFO状态机
	
	CyFxSpiProtoWrite8 (0x01, 0x11, 0x01);     // 复位PingPong缓冲
	CyU3PThreadSleep (10);
  
	if (powerCycleFpga)
	{
		CyU3PGpioSetValue (FX3_DEVICE_RESET, CyFalse); //reset FPGA(SPI module still works)
		CyU3PThreadSleep (100);
	}
	CyU3PGpioSetValue (FX3_DEVICE_RESET, CyTrue);  // release reset, FPGA starts working
	SetFpgaPoweredReady(CyTrue);
	if (powerCycleFpga)
	{
		CyU3PThreadSleep (100);
	}
	else
	{
		CyU3PThreadSleep (10);
	}
	if(glIsApplnActive)
	{
		CyU3PDmaChannelReset (&glChHandleSlFifoPtoU);
		CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
		CyU3PUsbResetEp(CY_FX_EP_CONSUMER);
		CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_RX_SIZE);
	}

	CyU3PGpifSMControl(CyFalse);        // STEP6: 恢复GPIF状态机
	CyU3PThreadSleep (10);
	if (powerCycleFpga)
	{
		CyU3PThreadSleep (1000);

		// 加延时，控制灯。
		CyU3PThreadSleep (4000);   // 等待FPGA重启稳定，后续命令才生效。
	}
	else
	{
		// 普通C2初始化路径不做固定长等待，避免额外阻塞。
		CyU3PThreadSleep (20);
	}
	CyFxSpiProtoWrite8 (0x01, 0x0B, 0x01);   // open led2 send command，蓝灯亮；0
	glIsDeviceRun = CyFalse;
	glIsCapMode   = CyTrue;    //标定模式or扫描模式与这个全局变量相关；
	CyFxWaitForAr0234InitReady ();

	/* FPGA/AR0234 初始化已就绪：将 FPGA_PROG_CTRL (GPIO28) 释放为高阻输入，
	 * 之后该脚不再由 FX3 主动驱动，电平由 PCB 外部电路决定。 */
	
	CyU3PGpioSimpleConfig_t progReleaseCfg;
	CyU3PMemSet ((uint8_t *)&progReleaseCfg, 0, sizeof(progReleaseCfg));
	progReleaseCfg.outValue    = CyFalse;
	progReleaseCfg.inputEn     = CyTrue;
	progReleaseCfg.driveLowEn  = CyFalse;
	progReleaseCfg.driveHighEn = CyFalse;
	progReleaseCfg.intrMode    = CY_U3P_GPIO_NO_INTR;
	(void)CyU3PGpioSetSimpleConfig (FPGA_PROG_CTRL, &progReleaseCfg);
	
	CyFxUpdateDeviceReadyState (CY_FX_DEVICE_READY_READY);
	glInResume = CyFalse;
}


CyU3PReturnStatus_t CyFxDeviceReConfigure ()
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    CyU3PMemSet ((uint8_t *)&io_cfg, 0, sizeof(io_cfg));
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.s0Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.s1Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.useUart   = CyFalse;
    io_cfg.useI2C    = CyFalse;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyTrue;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_SPI_ONLY;
//    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    io_cfg.gpioSimpleEn[0]  = 0;
    io_cfg.gpioSimpleEn[1]  = 0;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    return status;
}


/* SPI initialization for application. */
CyU3PReturnStatus_t CyFxSpiInit (uint16_t pageLen)
{
    CyU3PSpiConfig_t spiConfig;
    CyU3PDmaChannelConfig_t dmaConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Start the SPI module and configure the master. */
    status = CyU3PSpiInit();
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }
    CyU3PMemSet ((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
    spiConfig.isLsbFirst = CyFalse;
    spiConfig.cpol       = CyTrue;
    spiConfig.ssnPol     = CyFalse;
    spiConfig.cpha       = CyTrue;
    spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
    spiConfig.clock      = 8000000;
    spiConfig.wordLen    = 8;

    status = CyU3PSpiSetConfig (&spiConfig, NULL);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Create the DMA channels for SPI write and read. */
    CyU3PMemSet ((uint8_t *)&dmaConfig, 0, sizeof(dmaConfig));
    dmaConfig.size           = pageLen;
    /* No buffers need to be allocated as this channel
     * will be used only in override mode. */
    dmaConfig.count          = 0;
    dmaConfig.prodAvailCount = 0;
    dmaConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    dmaConfig.prodHeader     = 0;
    dmaConfig.prodFooter     = 0;
    dmaConfig.consHeader     = 0;
    dmaConfig.notification   = 0;
    dmaConfig.cb             = NULL;

    /* Channel to write to SPI flash. */
    dmaConfig.prodSckId = CY_U3P_CPU_SOCKET_PROD;
    dmaConfig.consSckId = CY_U3P_LPP_SOCKET_SPI_CONS;
    status = CyU3PDmaChannelCreate (&glSpiTxHandle,
            CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaConfig);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Channel to read from SPI flash. */
    dmaConfig.prodSckId = CY_U3P_LPP_SOCKET_SPI_PROD;
    dmaConfig.consSckId = CY_U3P_CPU_SOCKET_CONS;
    status = CyU3PDmaChannelCreate (&glSpiRxHandle,
            CY_U3P_DMA_TYPE_MANUAL_IN, &dmaConfig);

    if (status == CY_U3P_SUCCESS)
    {
        glSpiPageSize = pageLen;
    }

    return status;
}

/* SPI initialization for calibration. */
CyU3PReturnStatus_t
CyFxSpiInitCali (uint16_t pageLen)
{
    CyU3PSpiConfig_t spiConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Start the SPI module and configure the master. */
    status = CyU3PSpiInit();
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Start the SPI master block. Run the SPI clock at 8MHz
     * and configure the word length to 8 bits. Also configure
     * the slave select using FW. */
    CyU3PMemSet ((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
    spiConfig.isLsbFirst = CyFalse;
    spiConfig.cpol       = CyTrue;
    spiConfig.ssnPol     = CyFalse;
    spiConfig.cpha       = CyTrue;
    spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
    spiConfig.clock      = 8000000;
    spiConfig.wordLen    = 8;

    status = CyU3PSpiSetConfig (&spiConfig, NULL);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    return status;
}

CyU3PReturnStatus_t CyFxDeviceReConfigureAll ()
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	CyU3PGpifDisable(CyTrue);
	CyU3PThreadSleep (1);   // delay 1ms

	CyU3PGpioSetValue (FPGA_PWR_EN, CyFalse);
	SetFpgaPoweredReady(CyFalse);
	status = CyFxDeviceReConfigure();
	if (status != CY_U3P_SUCCESS)
	{
		return status;
	}
	 status = CyFxSpiInit (0x100);
	 glIsReconfigure = CyTrue;
	 return status;
}

/* Wait for the status response from the SPI flash. */
CyU3PReturnStatus_t CyFxSpiWaitForStatus (void)
{
    uint8_t buf[2], rd_buf[2];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Wait for status response from SPI flash device. */
    do
    {
        buf[0] = 0x06;  /* Write enable command. */

       CyU3PSpiSetSsnLine(CyFalse);
        status = CyU3PSpiTransmitWords (buf, 1);
       CyU3PSpiSetSsnLine(CyTrue);
        if (status != CY_U3P_SUCCESS)
        {
            return status;
        }

        buf[0] = 0x05;  /* Read status command */

       CyU3PSpiSetSsnLine(CyFalse);
       status = CyU3PSpiTransmitWords (buf, 1);
        if (status != CY_U3P_SUCCESS)
        {

            CyU3PSpiSetSsnLine(CyTrue);
            return status;
        }

        status = CyU3PSpiReceiveWords (rd_buf, 2);
        CyU3PSpiSetSsnLine(CyTrue);
        if(status != CY_U3P_SUCCESS)
        {
            return status;
        }

    } while ((rd_buf[0] & 1)|| (!(rd_buf[0] & 0x2)));

    return CY_U3P_SUCCESS;
}


CyU3PReturnStatus_t
CyFxSpiTransferSector (
        uint16_t  src,
        uint16_t  des,
        uint8_t  *buffer
        )
{
    CyU3PDmaBuffer_t buf_p;
    uint8_t location_rd[4],location_wr[4];
    int i;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    buf_p.buffer = buffer;
    buf_p.status = 0;
	buf_p.size  = glSpiPageSize;
	buf_p.count = glSpiPageSize;

     for(i=0;i<256;i++)       //sector = 64KBytes = 256*page = 256*256Bytes
     {
    	location_rd[0] = 0x03; /* Read command. */
		location_rd[1] = (src >> 8) & 0xFF;       /* MS byte */
		location_rd[2] = (src ) & 0xFF;
		location_rd[3] = 0;                       /* LS byte */

		 status = CyFxSpiWaitForStatus ();
		 if (status != CY_U3P_SUCCESS)
			 return status;

		CyU3PSpiSetSsnLine(CyFalse);
		 status = CyU3PSpiTransmitWords (location_rd, 4);
		 if (status != CY_U3P_SUCCESS)
		 {
			CyU3PSpiSetSsnLine(CyTrue);
			 return status;
		 }

		 CyU3PSpiSetBlockXfer (0, glSpiPageSize);

		 status = CyU3PDmaChannelSetupRecvBuffer (&glSpiRxHandle,
				 &buf_p);
		 if (status != CY_U3P_SUCCESS)
		 {
			CyU3PSpiSetSsnLine(CyTrue);
			 return status;
		 }
		 status = CyU3PDmaChannelWaitForCompletion (&glSpiRxHandle,
				 CY_FX_USB_SPI_TIMEOUT);
		 if (status != CY_U3P_SUCCESS)
		 {
			CyU3PSpiSetSsnLine(CyTrue);
			 return status;
		 }

		CyU3PSpiSetSsnLine(CyTrue);
		CyU3PSpiDisableBlockXfer (CyFalse, CyTrue);


		location_wr[0] = 0x02; /* Write command */
		location_wr[1] = (des >> 8) & 0xFF;       /* MS byte */
		location_wr[2] = (des ) & 0xFF;
		location_wr[3] = 0;                       /* LS byte */

		status = CyFxSpiWaitForStatus ();
		if (status != CY_U3P_SUCCESS)
		return status;

		CyU3PSpiSetSsnLine(CyFalse);
		status = CyU3PSpiTransmitWords (location_wr, 4);
		if (status != CY_U3P_SUCCESS)
		{
		    CyU3PSpiSetSsnLine(CyTrue);
		    return status;
		}

		CyU3PSpiSetBlockXfer (glSpiPageSize, 0);

		status = CyU3PDmaChannelSetupSendBuffer (&glSpiTxHandle,
			&buf_p);
		if (status != CY_U3P_SUCCESS)
		{
		    CyU3PSpiSetSsnLine(CyTrue);
		    return status;
		}
		status = CyU3PDmaChannelWaitForCompletion(&glSpiTxHandle,
			CY_FX_USB_SPI_TIMEOUT);
		if (status != CY_U3P_SUCCESS)
		{
		    CyU3PSpiSetSsnLine(CyTrue);
		    return status;
		}

		CyU3PSpiSetSsnLine(CyTrue);
		CyU3PSpiDisableBlockXfer (CyTrue, CyFalse);


    	src++;
    	des++;
		CyU3PThreadSleep (6);   // max page program time is 5ms,  add 1ms for safety

     }
     return CY_U3P_SUCCESS;
}

/* Function to erase SPI flash sectors. */
static CyU3PReturnStatus_t
CyFxSpiEraseSector (
	uint8_t   sector
	)
{
    uint32_t temp = 0;
    uint8_t  location[4];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    location[0] = 0x06;  /* Write enable. */
    CyU3PSpiSetSsnLine(CyFalse);
    status = CyU3PSpiTransmitWords (location, 1);
    CyU3PSpiSetSsnLine(CyTrue);
    if (status != CY_U3P_SUCCESS)
        return status;

	location[0] = 0xD8; /* Sector erase. */
	temp        = sector * 0x10000;
	location[1] = (temp >> 16) & 0xFF;
	location[2] = (temp >> 8) & 0xFF;
	location[3] = temp & 0xFF;

    CyU3PSpiSetSsnLine(CyFalse);
	status = CyU3PSpiTransmitWords (location, 4);
    CyU3PSpiSetSsnLine(CyTrue);

    return status;
}


void CyFxUpdate ()
{

  uint8_t buffer[256];  // store page data
  uint16_t add;
  add = (uint16_t)((SECTOR_NUMBER-1)*SECTOR_SIZE/PAGE_SIZE);
  CyFxSpiEraseSector(SECTOR_NUMBER-1);
  CyU3PThreadSleep (800);    // wait for 0.6s at least
  CyFxSpiTransferSector(0,add,buffer);
  CyFxSpiEraseSector(0);
  CyU3PThreadSleep (800);    // wait for 0.6s at least
  CyU3PDeviceReset(CyFalse);
}


CyU3PReturnStatus_t
CyFxSpiFx3Transfer (
        uint16_t  pageAddress,
        uint32_t  byteCount,
        uint8_t  *buffer,
        CyBool_t  isRead)
{
    CyU3PDmaBuffer_t buf_p;
    uint8_t location[4];
    uint32_t byteAddress = 0;
    uint16_t pageCount = (byteCount / glSpiPageSize);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (byteCount == 0)
    {
        return CY_U3P_SUCCESS;
    }
    if ((byteCount % glSpiPageSize) != 0)
    {
        pageCount ++;
    }

    buf_p.buffer = buffer;
    buf_p.status = 0;

    byteAddress  = pageAddress * glSpiPageSize;


        location[1] = (byteAddress >> 16) & 0xFF;       /* MS byte */
        location[2] = (byteAddress >> 8) & 0xFF;
        location[3] = byteAddress & 0xFF;               /* LS byte */

        if (isRead)
        {
            location[0] = 0x03; /* Read command. */

            buf_p.size  = glSpiPageSize;
            buf_p.count = glSpiPageSize;

            status = CyFxSpiWaitForStatus ();
            if (status != CY_U3P_SUCCESS)
                return status;

            //CyFxChipSelection(chip, CyFalse);
            CyU3PSpiSetSsnLine(CyFalse);
            status = CyU3PSpiTransmitWords (location, 4);
            if (status != CY_U3P_SUCCESS)
            {

                CyU3PSpiSetSsnLine(CyTrue);
                return status;
            }

            status = CyU3PSpiReceiveWords (buffer, byteCount);
            if (status != CY_U3P_SUCCESS)
            {
            	CyU3PSpiSetSsnLine(CyTrue);
                return status;
            }
            status = CyFxSpiWaitForStatus ();
            if (status != CY_U3P_SUCCESS)
            {
            	//CyFxChipSelection(chip, CyTrue);
            	CyU3PSpiSetSsnLine(CyTrue);
                return status;
            }

            //CyFxChipSelection(chip, CyTrue);
            CyU3PSpiSetSsnLine(CyTrue);
        }
        else /* Write */
        {
            while (pageCount != 0)
        	{
                location[1] = (byteAddress >> 16) & 0xFF;       /* MS byte */
                location[2] = (byteAddress >> 8) & 0xFF;
                location[3] = byteAddress & 0xFF;               /* LS byte */

				location[0] = 0x02; /* Write command */

				buf_p.size  = glSpiPageSize;
				buf_p.count = glSpiPageSize;

				status = CyFxSpiWaitForStatus ();
				if (status != CY_U3P_SUCCESS)
					return status;

				CyU3PSpiSetSsnLine(CyFalse);
				status = CyU3PSpiTransmitWords (location, 4);
				if (status != CY_U3P_SUCCESS)
				{
					CyU3PSpiSetSsnLine(CyTrue);
					return status;
				}

	            CyU3PSpiSetBlockXfer (glSpiPageSize, 0);

	            status = CyU3PDmaChannelSetupSendBuffer (&glSpiTxHandle,
	                    &buf_p);
	            if (status != CY_U3P_SUCCESS)
	            {
	                //CyFxChipSelection(chip, CyTrue);
	                CyU3PSpiSetSsnLine(CyTrue);
	                return status;
	            }
	            status = CyU3PDmaChannelWaitForCompletion(&glSpiTxHandle,
	                    CY_FX_USB_SPI_TIMEOUT);
	            if (status != CY_U3P_SUCCESS)
	            {
	                //CyFxChipSelection(chip, CyTrue);
	                CyU3PSpiSetSsnLine(CyTrue);
	                return status;
	            }

				CyU3PSpiSetSsnLine(CyTrue);
				CyU3PSpiDisableBlockXfer (CyTrue, CyFalse);

				CyU3PThreadSleep (6);   // max page program time is 5ms,  add 1ms for safety

				/* Update the parameters */
		        byteAddress  += glSpiPageSize;
		        buf_p.buffer += glSpiPageSize;
		        pageCount --;
        	}
        }

        /* Update the parameters */
        byteAddress  += glSpiPageSize;
        buf_p.buffer += glSpiPageSize;
        pageCount --;


    return CY_U3P_SUCCESS;
}


CyU3PReturnStatus_t  CyFxGetFPGAVersion(uint8_t *data, uint16_t length)
{
	static const char versionPrefix[] = "FPGA_V";
	static const char errorString[] = "FPGA_VERR";
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	CyU3PReturnStatus_t restoreStatus;
	uint8_t highValue = 0u;
	uint8_t midValue = 0u;
	uint8_t lowValue = 0u;
	uint16_t offset = 0u;
	uint32_t i;

	if ((data == 0) || (length == 0u))
	{
		return CY_U3P_ERROR_BAD_ARGUMENT;
	}

	CyU3PMemSet (data, 0, length);

	status = CyFxSpiProtoReadSelectedStatus8 (0x05, &highValue);
	if (status != CY_U3P_SUCCESS)
	{
		goto cleanup;
	}

	status = CyFxSpiProtoReadSelectedStatus8 (0x06, &midValue);
	if (status != CY_U3P_SUCCESS)
	{
		goto cleanup;
	}

	status = CyFxSpiProtoReadSelectedStatus8 (0x07, &lowValue);
	if (status != CY_U3P_SUCCESS)
	{
		goto cleanup;
	}

	CyU3PDebugPrint (4, "FPGA raw ver dec: %d %d %d\n",
	    (int)highValue, (int)midValue, (int)lowValue);

	for (i = 0u; (i < (sizeof(versionPrefix) - 1u)) && (offset + 1u < length); ++i)
	{
		data[offset++] = (uint8_t)versionPrefix[i];
	}

	if (offset + 1u < length)
	{
		offset = CyFxAppendVersionDecimal (data, length, offset, highValue);
	}
	if (offset + 1u < length)
	{
		data[offset++] = (uint8_t)'.';
	}
	if (offset + 1u < length)
	{
		offset = CyFxAppendVersionDecimal (data, length, offset, midValue);
	}
	if (offset + 1u < length)
	{
		data[offset++] = (uint8_t)'.';
	}
	offset = CyFxAppendVersionDecimal (data, length, offset, lowValue);

cleanup:
	restoreStatus = CyFxSpiProtoWrite8 (0x02, 0xff, 0x00);
	if ((status == CY_U3P_SUCCESS) && (restoreStatus != CY_U3P_SUCCESS))
	{
		status = restoreStatus;
	}

	if (status != CY_U3P_SUCCESS)
	{
		CyU3PMemSet (data, 0, length);
		for (i = 0u; (i < (sizeof(errorString) - 1u)) && (i + 1u < length); ++i)
		{
			data[i] = (uint8_t)errorString[i];
		}
	}

	return status;
}


uint8_t CyFxGetFPGA_ANALOG(uint8_t* data)
{
	uint8_t data_h = 0;
	uint8_t data_l = 0;

	(void)CyFxSpiProtoReadSelectedStatus8 (0x04, &data_h);     //set address to read analog(highest byte)(including alarm)
	(void)CyFxSpiProtoReadSelectedStatus8 (0x05, &data_l);     //set address to read analog(low byte)

	// set bit[7]:alarm_out
	if((data_h&0x80) == 0x80)   glStatus_FPGA |= 0x80;
	else                        glStatus_FPGA &= 0x7f;
    // calculate voltage
	*data = (uint8_t)(100 * 1.0f * (((data_h & 0x0f) * 256 + data_l)/4096)) ; //Result = 100 * V

	CyFxSpiProtoWrite8 (0x02, 0xff, 0x00);             //set address to device status

	return 1;
}



uint8_t CyFxGetFPGA_TEMPERATURE(uint8_t* data)
{
    uint8_t data_h = 0;
    uint8_t data_l = 0;
    float f;
    uint16_t t;
	(void)CyFxSpiProtoReadSelectedStatus8 (0x06, &data_h);     //set address to read temperature(highest byte)(including ot&alarm)
	(void)CyFxSpiProtoReadSelectedStatus8 (0x07, &data_l);     //set address to read temperature(low byte)

	//set bit[6]:ot_out;
	if((data_h&0x80) == 0x80)   glStatus_FPGA |= 0x40;
	else                        glStatus_FPGA &= 0xbf;
	//set bit[5]:user_temperature_out;
	if((data_h&0x40) == 0x40)   glStatus_FPGA |= 0x20;
	else                        glStatus_FPGA &= 0xdf;

	 f = 503.975f * (((float) ((data_h & 0x0f) * 256 + data_l)) /4096) -  273.15f;
	 if(f > 0)  *data++ = 0;  //sign = 0 means "+"
	 else       *data++ = 1;  //sign = 1 means "-"
	 t = (uint16_t)(fabsf(10*f));
	 *data++ =  t >> 8;
	 *data   =  t ;

	CyFxSpiProtoWrite8 (0x02, 0xff, 0x00);             //set address to device status
	// number of bytes = 3
	return 3;
}

uint8_t CyFxGetFPGA_VCC(uint8_t* data)
{
    uint8_t data_0_h = 0;
    uint8_t data_0_l = 0;
    uint8_t data_1_h = 0;
    uint8_t data_1_l = 0;
    uint8_t data_2_h = 0;
    uint8_t data_2_l = 0;
    uint16_t data_0,data_1,data_2;
	(void)CyFxSpiProtoReadSelectedStatus8 (0x08, &data_0_h);     //set address to read vccint(highest byte)(including alarm)
	(void)CyFxSpiProtoReadSelectedStatus8 (0x09, &data_0_l);     //set address to read vccint(low byte)

	(void)CyFxSpiProtoReadSelectedStatus8 (0x0a, &data_1_h);     //set address to read vccaux(highest byte)(including alarm)
	(void)CyFxSpiProtoReadSelectedStatus8 (0x0b, &data_1_l);     //set address to read vccaux(low byte)

	(void)CyFxSpiProtoReadSelectedStatus8 (0x0c, &data_2_h);     //set address to read vccbram(highest byte)(including alarm)
	(void)CyFxSpiProtoReadSelectedStatus8 (0x0d, &data_2_l);     //set address to read vccbram(low byte)

	//set bit[2]:vccint_alarm;
	if((data_0_h&0x80) == 0x80)   glStatus_FPGA |= 0x04;
	else                          glStatus_FPGA &= 0xfb;
	//set bit[1]:vccaux_alarm;
	if((data_1_h&0x80) == 0x80)   glStatus_FPGA |= 0x02;
	else                          glStatus_FPGA &= 0xfd;
	//set bit[0]:vccbram_alarm;
	if((data_2_h&0x80) == 0x80)   glStatus_FPGA |= 0x01;
	else                          glStatus_FPGA &= 0xfe;

	 data_0 = (100 * 3.0f * (((data_0_h & 0x0f) * 256 + data_0_l)/4096)) ; //Result = 100 * V
	 data_1 = (100 * 3.0f * (((data_1_h & 0x0f) * 256 + data_1_l)/4096)) ; //Result = 100 * V
	 data_2 = (100 * 3.0f * (((data_2_h & 0x0f) * 256 + data_2_l)/4096)) ; //Result = 100 * V

    *data++ = data_0 >> 8;
    *data++ = data_0;
    *data++ = data_1 >> 8;
	*data++ = data_1;
	*data++ = data_2 >> 8;
	*data   = data_2;

	CyFxSpiProtoWrite8 (0x02, 0xff, 0x00);             //set address to device status
	// number of bytes = 6
	return 6;
}



/* 用于处理 USB 设置请求的回调函数。 */
CyBool_t CyFxSlFifoApplnUSBSetupCB (uint32_t setupdat0, uint32_t setupdat1)
{
    uint8_t      i;
	uint8_t  workmode;
    uint8_t  bRequest, bReqType;
    uint8_t  bType, bTarget;
    uint16_t wValue, wIndex, wLength;
    CyBool_t isHandled = CyFalse;
    uint8_t  wGain_t;
    uint8_t  wValue_l;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyBool_t TouchSWitch1,TouchSwitch2;

    /* Decode the fields from the setup request. */
    bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
    bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
    bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
    wLength   = ((setupdat1 & CY_U3P_USB_LENGTH_MASK)   >> CY_U3P_USB_LENGTH_POS);

    if (bType == CY_U3P_USB_STANDARD_RQT)
    {
        /* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
         * requests here. It should be allowed to pass if the device is in configured
         * state and failed otherwise. */
        if ((bTarget == CY_U3P_USB_TARGET_INTF) && ((bRequest == CY_U3P_USB_SC_SET_FEATURE)
                    || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
        {
            if (glIsApplnActive)
                CyU3PUsbAckSetup ();
            else
                CyU3PUsbStall (0, CyTrue, CyFalse);

            isHandled = CyTrue;
        }

        if ((bTarget == CY_U3P_USB_TARGET_ENDPT) && (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
                && (wValue == CY_U3P_USBX_FS_EP_HALT))
        {
            if (glIsApplnActive)
            {
                if (wIndex == CY_FX_EP_PRODUCER)
                {
					//CyU3PDmaChannelReset (&glChHandleSlFifoUtoP);
                    //CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
                    //CyU3PUsbResetEp (CY_FX_EP_PRODUCER);
                    //CyU3PDmaChannelSetXfer (&glChHandleSlFifoUtoP, CY_FX_SLFIFO_DMA_TX_SIZE);
    				//CyU3PDmaChannelReset (&glUARTRxHandle);
                }

                if (wIndex == CY_FX_EP_CONSUMER)
                {
                	//P指的是GPIF-II，U指的是USB电脑端；PtoU从GPIF-II（来自FPGA）传输到USB（电脑）端；
                    CyU3PDmaChannelReset (&glChHandleSlFifoPtoU);
                    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
                    CyU3PUsbResetEp (CY_FX_EP_CONSUMER);
                    CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_RX_SIZE);
                    //CyU3PDmaChannelReset (&glUARTRxHandle);
                }

                if (wIndex == CY_FX_EP_GYRO_IN)
                {
                    CyU3PDmaChannelReset(&glChHandleGyro);
                    CyU3PUsbFlushEp(CY_FX_EP_GYRO_IN);
                    CyU3PUsbResetEp(CY_FX_EP_GYRO_IN);
                    CyU3PDmaChannelSetXfer(&glChHandleGyro, 0);
                }

                CyU3PUsbStall (wIndex, CyFalse, CyTrue);
                isHandled = CyTrue;
            }
        }
    }

    //上位机控制指令处理流程
    if (bType == CY_U3P_USB_VENDOR_RQT)
    {
            /* 初始化完成前，仅响应 FX3 版本号查询，其余 Vendor 指令一律拒绝（返回 STALL）。 */
            if (!CyFxDeviceIsReady () && (bRequest != CY_FX_RQT_ID_CHECK_FX3))
            {
                return CyFalse;
            }

            isHandled = CyTrue;

            switch (bRequest)
            {
                case CY_FX_RQT_ID_CHECK_FX3:
					CyU3PUsbSendEP0Data (16, (uint8_t *)glFX3ID);
					break;

                case CY_FX_RQT_ID_CHECK_FPGA:
                	CyU3PMemSet (glFpgaVersion, 0, sizeof (glFpgaVersion));
	                status = CyFxGetFPGAVersion (glFpgaVersion, sizeof (glFpgaVersion));
	                if (status != CY_U3P_SUCCESS)
	                {
	                	CyU3PDebugPrint (4, "FPGA version read failed: %d\n", (int)status);
	                }
                	CyU3PUsbSendEP0Data (16, glFpgaVersion);
					break;


                case CY_FX_RQT_STATUS:
                	//KeyAddTimes++;
                	CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                	key1TimesTrue=0;
                	key1TimesFalse=0;
                	key2TimesTrue=0;
                	key2TimesFalse=0;
                	for(i=0;i<9;i++)
                	{
                		CyU3PGpioGetValue(BUTTON1_ON,&button1[i]);
                		CyU3PGpioGetValue(BUTTON2_ON,&button2[i]);

                		if(button1[i]==CyFalse)
                		{
                			key1TimesFalse++;
                		}
                		else
                		{
                			key1TimesTrue++;
                		}

                 		if(button2[i]==CyFalse)
                 		{
                 			key2TimesFalse++;
                 		}
                 		else
                 		{
                 			key2TimesTrue++;
                 		}

                	}

                	if(key1TimesTrue>key1TimesFalse)
                	{
                		TouchSWitch1=CyTrue;
                	}
                	else
                	{
                		TouchSWitch1=CyFalse;
                	}

                	if(key2TimesTrue>key2TimesFalse)
                	{
                		TouchSwitch2=CyTrue;
                	}
                	else
                	{
                		TouchSwitch2=CyFalse;
                	}

                	//获取磁吸开关状态；
                    #ifdef IIC_ORDINARY_GPIO
                	CyU3PGpioGetValue(IIC_SCL_GPIO,&SclGpioValue);
                    #endif
	                /* 0x40 表示设备端自初始化完成，允许按 B2 正常返回状态。 */
	                glEp0Buffer[0] |= 0x40;
	                if(glIsDeviceRun)
	                {
	                	glEp0Buffer[0] |= 0x80;
	                }
	                else
	                {
	                	glEp0Buffer[0] &= 0x7f;
	                }
	                glEp0Buffer[1] = glStatus_Extra;//把短按还是长按标识赋值
	                glStatus_Extra &= 0x3f;  //clear long & short press flag
					 //上传触碰开关状态，具体含义见通信协议文档，触碰开关正常没有按下时电平是高，扫描头安上后电平是低
					//E0=0时屏蔽扫描头检测，B2固定上报有扫描头，避免无扫描头分支停扫。
					if(glsaomswitchState == 0)
					{
						glEp0Buffer[0] |= 0x04;

					}
					//触碰开关1是低电平，扫描头1（大扫描头）安装；////////////
					else if((TouchSWitch1 == CyFalse) && (TouchSwitch2 == CyTrue))
                	{
                		glEp0Buffer[0] |= 0x04;

                	}
                	 //触碰开关2是低电平，扫描头2（小扫描头）安装；
                	else if((TouchSWitch1 == CyTrue) && (TouchSwitch2 == CyFalse))
                	 {
                		glEp0Buffer[0] |= 0x04;

                	}
                	//触碰开关1和触碰开关2是高电平，没有扫描头安装，2025.04.16结构有问题，暂时取消这个功能；
                	else if((TouchSWitch1 == CyTrue) && (TouchSwitch2 == CyTrue))
                	 {
						glEp0Buffer[0] &= 0x7b;     //向上位机传输停止扫描状态，即传输第一个字节的最高位bit7是0，bit2是0（无扫描头）；
						if ((glIsDeviceRun == CyTrue) && (glInResume == CyFalse))
						{
	                		CyFxButtonPressed_Stop();
						}
                	}
					
                    //根据磁吸开关状态向上位机传输磁吸开关的状态，并且控制口扫设备的工作状态；
                	//磁吸开关是高电平，磁吸未被触发，设备处于工作状态，向上位机传输1（第1个字节的bit1位）；
                	if(SclGpioValue == CyTrue)
                	{
                		glEp0Buffer[0] |= 0x02;
                	}
                	//磁吸开关是低电平，磁吸被触发，设备停止工作，向上位机传输0（第1个字节的bit1位）；
                	else if(SclGpioValue == CyFalse)
                	{
                		glEp0Buffer[0] &= 0x7d;
						if ((glIsDeviceRun == CyTrue) && (glInResume == CyFalse))
						{
	                			CyFxButtonPressed_Stop();
						}
                	}

					//CyU3PDebugPrint(4, "glEp0Buffer[0] = %d\n", (int)glEp0Buffer[0]);
                	CyU3PUsbSendEP0Data (2, glEp0Buffer);
                	break;

                case CY_FX_RQT_JY61:

				    CyU3PDebugPrint (4, "Step1:Gyro data read test1" );
				    CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
				    Jy901_IIC_Read_Bytes(JY901_ADDRESS<< 1, AX, &glEp0Buffer[0], 2);
				    Jy901_IIC_Read_Bytes(JY901_ADDRESS<< 1, AY, &glEp0Buffer[2], 2);
				    Jy901_IIC_Read_Bytes(JY901_ADDRESS<< 1, AZ, &glEp0Buffer[4], 2);
				    Jy901_IIC_Read_Bytes(JY901_ADDRESS<< 1, GX, &glEp0Buffer[6], 2);
				    Jy901_IIC_Read_Bytes(JY901_ADDRESS<< 1, GY, &glEp0Buffer[8], 2);
				    Jy901_IIC_Read_Bytes(JY901_ADDRESS<< 1, GZ, &glEp0Buffer[10], 2);
				    Jy901_IIC_Read_Bytes(JY901_ADDRESS<< 1, Roll, &glEp0Buffer[12], 2);
				    Jy901_IIC_Read_Bytes(JY901_ADDRESS<< 1, Pitch, &glEp0Buffer[14], 2);
				    Jy901_IIC_Read_Bytes(JY901_ADDRESS<< 1, Yaw, &glEp0Buffer[16], 2);
				    CyU3PDebugPrint (4, "Step2:Gyro data read test2" );
				    CyU3PUsbSendEP0Data (18, glEp0Buffer);
				    CyU3PDebugPrint (4, "Step3:Gyro data read test3" );

				   break;


				case CY_FX_RQT_GAIN:
					CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));



					//CyFxUsbI2cTransfer_AR0234_RD(IIC_AR0234_RGB_ADDRESS,0x305E,IIC_RD_AR0234_BYTES,glEp0Buffer);
					//CyFxUsbI2cTransfer_AR0234_RD(IIC_AR0234_RGB_ADDRESS,0x305E,IIC_RD_AR0234_BYTES,glEp0Buffer + 2);
					glEp0Buffer[0] = (uint8_t)(AR0234ContextConfig.laserGainSensor1);
					glEp0Buffer[1] = (uint8_t)(AR0234ContextConfig.laserGainSensor2);
					glEp0Buffer[2] = (uint8_t)(AR0234ContextConfig.whiteLightGainSensor1);
					glEp0Buffer[3] = (uint8_t)(AR0234ContextConfig.whiteLightGainSensor2);

					CyU3PUsbSendEP0Data (4, glEp0Buffer);
				   break;
				   
                case 0xB5:
					CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
					CyU3PUsbSendEP0Data (2, glEp0Buffer);
				   break;

                case CY_FX_RQT_BLKLEVEL:
					//CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
					CyU3PUsbSendEP0Data (1, glEp0Buffer);
				   break;

              
                case CY_FX_RQT_MODE:
					CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
					glEp0Buffer[0] = glMode;
					CyU3PUsbSendEP0Data (1, glEp0Buffer);
				   break;

				case CY_FX_RQT_EXPO:
					CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
					glEp0Buffer[0] = (uint8_t)(AR0234ContextConfig.laserExposureSensor1);
					glEp0Buffer[1] = (uint8_t)(AR0234ContextConfig.laserExposureSensor1>>8);
					glEp0Buffer[2] = (uint8_t)(AR0234ContextConfig.whiteLightExposureSensor1);
					glEp0Buffer[3] = (uint8_t)(AR0234ContextConfig.whiteLightExposureSensor1>>8);
					CyU3PUsbSendEP0Data (4, glEp0Buffer);
				   break;
                case 0xB9:
					CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
					CyU3PUsbSendEP0Data (2, glEp0Buffer);
				   break;

                case CY_FX_RQT_CURRENT1_BLUE:
					CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
					//glEp0Buffer[0] = glCurrent;
					glEp0Buffer[0] = (uint8_t)(currentData.laserCurrent1Blue >> 16);
					glEp0Buffer[1] = (uint8_t)(currentData.laserCurrent1Blue >> 8);
					glEp0Buffer[2] = (uint8_t)(currentData.laserCurrent1Blue );
					CyU3PUsbSendEP0Data (3, glEp0Buffer);
				   break;
                case 0xBA:
					CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
					CyU3PUsbSendEP0Data (1, glEp0Buffer);
				   break;

                case CY_FX_RQT_SETCURRENT2_GREEN:
					CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
					glEp0Buffer[0] = (uint8_t)(currentData.laserCurrent2Green >> 16);
					glEp0Buffer[1] = (uint8_t)(currentData.laserCurrent2Green >> 8);
					glEp0Buffer[2] = (uint8_t)(currentData.laserCurrent2Green );
					CyU3PUsbSendEP0Data (3, glEp0Buffer);
				   break;

                case CY_FX_RQT_SETCURRENT3_WHITE:
					CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
					glEp0Buffer[0] = (uint8_t)(currentData.whiteLightCurrent >> 16);
					glEp0Buffer[1] = (uint8_t)(currentData.whiteLightCurrent >> 8);
					glEp0Buffer[2] = (uint8_t)(currentData.whiteLightCurrent );
					CyU3PUsbSendEP0Data (3, glEp0Buffer);
				   break;

                case CY_FX_RQT_SENSOR_TEMPERATURE:
                {
                    float tempSensor1 = 0.0f;
                    float tempSensor2 = 0.0f;

                	CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                    // 极速读取缓存的温度值（未激活前返回55度，激活后返回最新值）
                    TemperatureMonitor_GetLastTemperature(&tempSensor1, &tempSensor2);
					CyU3PMemCopy(&glEp0Buffer[0], (uint8_t *)&tempSensor1, sizeof(float));
					CyU3PMemCopy(&glEp0Buffer[4], (uint8_t *)&tempSensor2, sizeof(float));
					CyU3PUsbSendEP0Data (8, glEp0Buffer);
					break;
				}

                case CY_FX_RQT_STATUS_DEVICE:
                {
	                uint8_t statusDeviceRaw = 0;
	                CyU3PMemSet (glEp0Buffer, 0, sizeof(glEp0Buffer));

	                CyFxReadTouchSwitchFiltered(&TouchSWitch1, &TouchSwitch2);
	                //上传触碰开关状态，具体含义见通信协议文档表二，触碰开关正常没有按下时电平是高，扫描头安上后电平是低
	                if(glsaomswitchState == 0)
	                {
	                	statusDeviceRaw = 0x10; //大扫描头安装
	                }
	                else
	                {
	                	//触碰开关1是低电平，大扫描头安装
	                	if((TouchSWitch1 == CyFalse) && (TouchSwitch2 == CyTrue))
	                	{
	                		statusDeviceRaw = 0x10;
	                	}
	                	//触碰开关2是低电平，小扫描头安装
	                	else if((TouchSWitch1 == CyTrue) && (TouchSwitch2 == CyFalse))
	                	{
	                		statusDeviceRaw = 0x11;
	                	}
	                	//触碰开关1和触碰开关2是低电平，标定机构安装
	                	else if((TouchSWitch1 == CyFalse) && (TouchSwitch2 == CyFalse))
	                	{
	                		statusDeviceRaw = 0x20;
	                	}
						else //触碰开关1和触碰开关2是高电平，没有扫描头安装
	                	{
	                		statusDeviceRaw = 0x00;
	                	}	

	                }

	                glEp0Buffer[0] = statusDeviceRaw;
	                CyU3PUsbSendEP0Data (1, glEp0Buffer);
	                break;
                }
                
				case CY_FX_STATUS_DETECTION:
				    CyU3PDebugPrint (4, "E0 value=%d\n",wValue);
				    glsaomswitchState = (uint8_t)(wValue);
					CyU3PDebugPrint (4, "glsaomswitchState value=%d\n",glsaomswitchState);
					CyU3PUsbAckSetup ();
                    break;
                
				case CY_FX_BINNING_STATE:
					workmode = (uint8_t)(wValue & 0xff);
					if (CyFxApplyBinningMode (workmode) == CyTrue)
					{
						CyU3PUsbAckSetup ();
						CyU3PDebugPrint (4, "BINNING request successful, mode=%d\n", workmode);
					}
					else
					{
						CyU3PDebugPrint (4, "BINNING request failed, mode=%d\n", workmode);
						CyU3PUsbStall (0, CyTrue, CyFalse);
					}
					break;
			    
				case CY_FX_LASER_CYCLE_SETTING:

				  	currentData.laserPeriod = ((uint32_t)(wValue*50000));//value单位是毫秒
					currentData.blueLightPeriod = currentData.laserPeriod;
					currentData.greenLightPeriod = currentData.laserPeriod;
                	CyU3PUsbAckSetup ();
					break;

			    case CY_FX_WHITE_LIGHT_CYCLE_SETTING:

				    currentData.whiteLightPeriod = ((uint32_t)(wValue*50000));//value单位是毫秒
					//打印value值，调试用；
					CyU3PDebugPrint (4, "white light period value=%d\n",currentData.whiteLightPeriod);
					
					currentData.collectionPeriod = currentData.laserPeriod + currentData.whiteLightPeriod;
					//打印value值，调试用；
					CyU3PDebugPrint (4, "collection period value=%d\n",currentData.collectionPeriod);
				
					CyFxSpiProtoWrite8 (0x01, 0x96, (uint8_t)(currentData.collectionPeriod >> 24));
					CyFxSpiProtoWrite8 (0x01, 0x97, (uint8_t)(currentData.collectionPeriod >> 16));
					CyFxSpiProtoWrite8 (0x01, 0x98, (uint8_t)(currentData.collectionPeriod >> 8));
					CyFxSpiProtoWrite8 (0x01, 0x99, (uint8_t)(currentData.collectionPeriod)); //总周期

					CyFxSpiProtoWrite8 (0x02, 0x96, (uint8_t)(currentData.laserPeriod >> 24));
					CyFxSpiProtoWrite8 (0x02, 0x97, (uint8_t)(currentData.laserPeriod >> 16));
					CyFxSpiProtoWrite8 (0x02, 0x98, (uint8_t)(currentData.laserPeriod >> 8));
					CyFxSpiProtoWrite8 (0x02, 0x99, (uint8_t)(currentData.laserPeriod));//激光周期
				
					CyFxSpiProtoWrite8 (0x01, 0x9a, (uint8_t)(currentData.blueLightPeriod >> 16));
					CyFxSpiProtoWrite8 (0x01, 0x9b, (uint8_t)(currentData.blueLightPeriod >> 8));
					CyFxSpiProtoWrite8 (0x01, 0x9c, (uint8_t)(currentData.blueLightPeriod));//蓝光周期

					CyFxSpiProtoWrite8 (0x01, 0x9d, (uint8_t)(currentData.greenLightPeriod >> 16));				
					CyFxSpiProtoWrite8 (0x01, 0x9e, (uint8_t)(currentData.greenLightPeriod >> 8));
					CyFxSpiProtoWrite8 (0x01, 0x9f, (uint8_t)(currentData.greenLightPeriod));//绿光周期

					CyFxSpiProtoWrite8 (0x01, 0xa1, (uint8_t)(currentData.whiteLightPeriod >> 16));
					CyFxSpiProtoWrite8 (0x01, 0xa2, (uint8_t)(currentData.whiteLightPeriod >> 8));
					CyFxSpiProtoWrite8 (0x01, 0xa3, (uint8_t)(currentData.whiteLightPeriod));//白光周期

                	CyU3PUsbAckSetup ();
					break;

				case CY_FX_BLUE_LIGHT:
				    currentData.blueLightPWM = ((uint32_t)(wValue & 0x00FF)) << 16;
					currentData.blueLightPWM += (uint32_t)(wIndex);
					//打印value值，调试用；
					CyU3PDebugPrint (4, "blue light PWM value=%d\n",currentData.blueLightPWM);
					CyFxSpiProtoWrite8 (0x01, 0xa4, (uint8_t)(currentData.blueLightPWM >> 16));
					CyFxSpiProtoWrite8 (0x01, 0xa5, (uint8_t)(currentData.blueLightPWM >> 8));
					CyFxSpiProtoWrite8 (0x01, 0xa6, (uint8_t)(currentData.blueLightPWM));
					CyU3PUsbAckSetup ();
					break;

				case CY_FX_GREEN_LIGHT:
				    currentData.greenLightPWM = ((uint32_t)(wValue & 0x00FF)) << 16;
					currentData.greenLightPWM += (uint32_t)(wIndex);
					//打印value值，调试用；
					CyU3PDebugPrint (4, "green light PWM value=%d\n",currentData.greenLightPWM);
					CyFxSpiProtoWrite8 (0x01, 0xa7, (uint8_t)(currentData.greenLightPWM >> 16));
					CyFxSpiProtoWrite8 (0x01, 0xa8, (uint8_t)(currentData.greenLightPWM >> 8));
					CyFxSpiProtoWrite8 (0x01, 0xa9, (uint8_t)(currentData.greenLightPWM));
					CyU3PUsbAckSetup ();
					break;

				case CY_FX_WHITE_LIGHT:
				{
				    currentData.whiteLightPWM = ((uint32_t)(wValue & 0x00FF)) << 16;
					currentData.whiteLightPWM += (uint32_t)(wIndex);
					//打印value值，调试用；
					CyU3PDebugPrint (4, "white light PWM value=%d\n",currentData.whiteLightPWM);	
					CyFxSpiProtoWrite8 (0x01, 0xaa, (uint8_t)(currentData.whiteLightPWM >> 16));
					CyFxSpiProtoWrite8 (0x01, 0xab, (uint8_t)(currentData.whiteLightPWM >> 8));
					CyFxSpiProtoWrite8 (0x01, 0xac, (uint8_t)(currentData.whiteLightPWM));
					CyU3PUsbAckSetup ();
					TemperatureMonitor_Start();
					break;
				}
				case CY_FX_RQT_GYRO_CONTROL:
				{
					/* 只处理 OUT 方向（主机写） */
					CyBool_t isIn = ((bReqType & 0x80) != 0);
					if (isIn == CyFalse)
					{
						CyBool_t enable = (wValue & 0x01) ? CyTrue : CyFalse;
						if (enable != gJY901Enabled)
						{
							gJY901Enabled = enable;
							if (gJY901Enabled)
							{
								CyU3PDmaChannelReset(&glChHandleGyro);
								CyU3PUsbFlushEp(CY_FX_EP_GYRO_IN);
								CyU3PDmaChannelSetXfer(&glChHandleGyro, 0);
								CyU3PDebugPrint(4, "JY901: Enabled (host cmd)\r\n");
							}
							else
							{
								CyU3PUsbFlushEp(CY_FX_EP_GYRO_IN);
								CyU3PDmaChannelReset(&glChHandleGyro);
								CyU3PDebugPrint(4, "JY901: Disabled (host cmd)\r\n");
							}
						}
						CyU3PUsbAckSetup();
					}
					else
					{
						CyU3PUsbStall(0, CyTrue, CyFalse);
					}
					break;
				}

                case CY_FX_RQT_FPGA_ANALOG:
                	CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                	wGain_t = CyFxGetFPGA_ANALOG(glEp0Buffer);
                	CyU3PUsbSendEP0Data (wGain_t, glEp0Buffer);
                	break;

                case CY_FX_RQT_FPGA_TEMPERATURE:
                	CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                	wGain_t = CyFxGetFPGA_TEMPERATURE(glEp0Buffer);
                	CyU3PUsbSendEP0Data (wGain_t, glEp0Buffer);
                	break;

                case CY_FX_RQT_FPGA_VCC:
                	CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                	wGain_t = CyFxGetFPGA_VCC(glEp0Buffer);
                	CyU3PUsbSendEP0Data (wGain_t, glEp0Buffer);
                	break;
                case CY_FX_RQT_COMMAND_MAGNETIC:
                	/* Legacy compatibility: 磁吸开关现已由硬件 GPIO 中断驱动，此命令仅保留握手 */
                	CyU3PUsbAckSetup ();
                	break;
                case CY_FX_RQT_COMMAND_CLOSE_DEVICE:
    				CyFxButtonPressed_Stop();
    				CyU3PGpioSetValue (FPGA_PWR_EN, CyFalse);   //FPGA power_off
				    SetFpgaPoweredReady(CyFalse);
				    CyFxUpdateDeviceReadyState (CY_FX_DEVICE_READY_NOT_READY);
                	CyU3PUsbAckSetup ();
                	break;
                case CY_FX_RQT_COMMAND_OPEN_DEVICE:
    				CyFxDeviceInit (glwValue, glwIndex, CyTrue);
    				CyU3PThreadSleep (1000);                     
	                	CyFxAr0234WriteIndependentAndCommit (0x3012,
	                			AR0234ContextConfig.laserExposureSensor1,
	                			AR0234ContextConfig.laserExposureSensor2);

	                	CyFxAr0234WriteIndependentAndCommit (0x3016,
	                			AR0234ContextConfig.whiteLightExposureSensor1,
	                			AR0234ContextConfig.whiteLightExposureSensor2);
					CyU3PDebugPrint (4, "shanweiji, laser=%d white=%d\n",
	                			AR0234ContextConfig.laserExposureSensor1,
	                		    AR0234ContextConfig.whiteLightExposureSensor1);
    				CyFxButtonPressed_Start();
                	CyU3PUsbAckSetup ();
                	break;

                case CY_FX_RQT_Fpga_Read_ID:
                     CyU3PUsbSendEP0Data (20, glEp0FpgaID);
                     break;

                case CY_FX_RQT_fx3_Read_Calibration:
                     CyU3PUsbSendEP0Data (4096, glEp0Calibration);
                     break;
                case CY_FX_RQT_fx3_Read_Cali_LCC_CMC:
                     CyU3PUsbSendEP0Data (512, glEp0CaliLccCMC);
                     break;
                case CY_FX_RQT_fx3_Reconfig_SPI:
                	CyFxDeviceReConfigureAll();
                    CyU3PUsbSendEP0Data (20, glEp0Buffer);
                    break;
                case CY_FX_RQT_Reboot_Fx3:
                    //4. reboot fx3
                      CyU3PDeviceReset(CyFalse);
                     CyU3PUsbSendEP0Data (20, glEp0FpgaID);
                     break;
                case CY_FX_RQT_SPI_FLASH_READ:
                    CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                    //chip = wValue & 0x03;                                   //  store chip selection
                	if(wLength > 4096)
                	{
                		wLength = 4096;
                	}
                    CyFxSpiFx3Transfer (wIndex, wLength,glEp0Buffer, CyTrue);
                    CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
                    break;
                case CY_FX_RQT_SPI_FLASH_ERASE_Cali:
                    CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                    CyFxSpiEraseSector(5);
                    CyU3PUsbSendEP0Data (0, glEp0Buffer);
                    break;
                case CY_FX_RQT_SPI_FLASH_WRITE:
                	CyU3PMemSet (glEp0Calibration, 0, sizeof (glEp0Calibration));

                	if(wIndex < 0x500)
                	{
                		//CyU3PUsbSendEP0Data (0, glEp0Buffer);
                		break;
                	}
                	if(wLength > 4096)
                	{
                		wLength = 4096;
                	}

                    status = CyU3PUsbGetEP0Data (wLength, glEp0Calibration, NULL);
                        status = CyFxSpiFx3Transfer (wIndex, wLength,
                        		glEp0Calibration, CyFalse);
                    break;
                case CY_FX_RQT_SPI_FLASH_ERASE_Cali_LCC:
                    CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                    CyFxSpiEraseSector(6);
                    CyU3PUsbSendEP0Data (0, glEp0Buffer);
                    break;

                case CY_FX_RQT_COMMAND_INIT_RUN:
                	 glMode    =  wValue & 0x0f;
					 CyU3PUsbAckSetup ();
					break;
               
                case CY_FX_RQT_COMMAND_CAPTURE:
	                // STEP1: 仅在首帧（glIsSnapActive=true）时发SPI复位PingPong，后续帧用GPIO SNAP驱动
	                if(glIsSnapActive)
	                {
                		CyFxSpiProtoWrite8 (0x01, 0x11, 0x01);         //STEP1: RESET PingPong
                		CyU3PThreadSleep (10);
	                }
	                CyU3PGpioSetValue (FX3_SNAP, CyTrue);            //RESET FIFO&STATE MACHINE

					CyU3PThreadSleep (1);                            //WAIT FOR 1ms
					// STEP2: 重置DMA缓冲区
					if(glIsApplnActive)                              //STEP2: RESET DMA BUFFER
					{
						CyU3PDmaChannelReset (&glChHandleSlFifoPtoU);
						CyU3PThreadSleep(10);  // 延迟确保DMA重置完成
						CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
						CyU3PUsbResetEp (CY_FX_EP_CONSUMER);
						CyU3PThreadSleep(10);  // 再次延迟确保稳定
						CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_RX_SIZE);
					}
					CyU3PThreadSleep (1);
					// STEP3: 仅在首帧释放PingPong，后续帧由GPIO SNAP下降沿启动
					if(glIsSnapActive)
					{
						CyFxSpiProtoWrite8 (0x01, 0x11, 0x00);         // 启动PingPong缓冲工作
						CyU3PThreadSleep (10);
					}
					if(glIsPingpangActive)
					{
						glIsSnapActive = CyFalse;
						CyU3PGpioSetValue (FX3_SNAP, CyFalse);           // 开始采集信号
					}
					CyU3PUsbAckSetup ();
					break;
                
                //设置增益d5
				case CY_FX_RQT_COMMAND_SETGAIN:
				{
	                uint16_t combinedGainValue;

	                AR0234ContextConfig.laserGainSensor2 = (uint16_t)(wValue);
	                //增益白光修改为高8位, by zlq 20250620
	                AR0234ContextConfig.whiteLightGainSensor2 = (uint16_t)(wIndex);
	                AR0234ContextConfig.laserGainSensor1 = (uint16_t)(wValue);
	                //增益白光修改为高8位, by zlq 20250620
	                AR0234ContextConfig.whiteLightGainSensor1 = (uint16_t)(wIndex);

	                CyU3PDebugPrint (4, "laser_gain_value = %d\n",wValue);
	                CyU3PDebugPrint (4, "laser_white_value = %d\n",wIndex);


	                //修改地址305E为3060，原来激光的值存储在3060地址的低8位, 白光存储在3060地址的高8位, by zlq 20250620
	                combinedGainValue = (uint16_t)((wValue & 0x00FFu) | ((wIndex & 0x00FFu) << 8));
	                CyFxAr0234WritePairAndCommit (0x3060, combinedGainValue);
	                if (CyFxVerifyGainReadback () == CyFalse)
	                {
                	    CyU3PDebugPrint (4, "GAIN readback fail, retrying write...\n");
                	    CyFxAr0234WritePairAndCommit (0x3060, combinedGainValue);
                	    if (CyFxVerifyGainReadback () == CyFalse)
                	    {
                		   CyU3PDebugPrint (4, "GAIN request failed (readback mismatch).\n");
                		   CyU3PUsbStall (0, CyTrue, CyFalse);
                		   break;
                        }
	                }

                    CyU3PDebugPrint (4, "GAIN request successful, laser=%d white=%d\n",
                                    (unsigned int)AR0234ContextConfig.laserGainSensor1,
                                    (unsigned int)AR0234ContextConfig.whiteLightGainSensor1);
					CyU3PUsbAckSetup ();
					break;
				}
                //读取曝光参数
				case REPORT_EXPOSURE:
				{
						uint16_t lasersensor1Data, lasersensor2Data;
						uint16_t whitesensor1Data, whitesensor2Data;
						CyFxGetBothSensorParams (0x3012, &lasersensor1Data, &lasersensor2Data);
						CyFxGetBothSensorParams (0x3016, &whitesensor1Data, &whitesensor2Data);
						CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
						glEp0Buffer[0] = (uint8_t)(lasersensor1Data & 0x00FFu);
						glEp0Buffer[1] = (uint8_t)((lasersensor1Data >> 8) & 0x00FFu);
						glEp0Buffer[2] = (uint8_t)(whitesensor1Data & 0x00FFu);
						glEp0Buffer[3] = (uint8_t)((whitesensor1Data >> 8) & 0x00FFu);
						glEp0Buffer[4] = (uint8_t)(lasersensor2Data & 0x00FFu);
                        glEp0Buffer[5] = (uint8_t)((lasersensor2Data >> 8) & 0x00FFu);
						glEp0Buffer[6] = (uint8_t)(whitesensor2Data & 0x00FFu);
						glEp0Buffer[7] = (uint8_t)((whitesensor2Data >> 8) & 0x00FFu);

						CyU3PDebugPrint (4, "Readback EXPOSURE s1_laser=%d s1_white=%d s2_laser=%d s2_white=%d\n",
								(unsigned int)lasersensor1Data, (unsigned int)whitesensor1Data,
								(unsigned int)lasersensor2Data, (unsigned int)whitesensor2Data);
						CyU3PUsbSendEP0Data (8, glEp0Buffer);
						break;
				}
				//读取增益参数
				case REPORT_GAIN:
				{
						uint16_t sensor1Data, sensor2Data;
						CyFxGetBothSensorParams (0x3060, &sensor1Data, &sensor2Data);
						CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
						glEp0Buffer[0] = (uint8_t)(sensor1Data & 0x00FFu);
						glEp0Buffer[1] = (uint8_t)((sensor1Data >> 8) & 0x00FFu);
						glEp0Buffer[2] = (uint8_t)(sensor2Data & 0x00FFu);
						glEp0Buffer[3] = (uint8_t)((sensor2Data >> 8) & 0x00FFu);
						CyU3PDebugPrint (4, "Readback GAIN sensor1=%d sensor2=%d\n",
								(unsigned int)sensor1Data, (unsigned int)sensor2Data);
						CyU3PUsbSendEP0Data (4, glEp0Buffer);
						break;
				}
				//读取偏移量参数
				case REPORT_OFFSET:
				{
						uint16_t sensor1_laser_y_start, sensor2_laser_y_start;
						uint16_t sensor1_laser_x_start, sensor2_laser_x_start;
						uint16_t sensor1_laser_y_end,   sensor2_laser_y_end;
						uint16_t sensor1_laser_x_end,   sensor2_laser_x_end;
						uint16_t sensor1_white_y_start, sensor2_white_y_start;
						uint16_t sensor1_white_x_start, sensor2_white_x_start;
						uint16_t sensor1_white_y_end,   sensor2_white_y_end;
						uint16_t sensor1_white_x_end,   sensor2_white_x_end;					
						CyFxGetBothSensorParams (0x3002, &sensor1_laser_y_start, &sensor2_laser_y_start);
						CyFxGetBothSensorParams (0x3004, &sensor1_laser_x_start, &sensor2_laser_x_start);
						CyFxGetBothSensorParams (0x3006, &sensor1_laser_y_end,   &sensor2_laser_y_end);
						CyFxGetBothSensorParams (0x3008, &sensor1_laser_x_end,   &sensor2_laser_x_end);
						CyFxGetBothSensorParams(0x308C,&sensor1_white_y_start, &sensor2_white_y_start);
						CyFxGetBothSensorParams(0x308A,&sensor1_white_x_start, &sensor2_white_x_start);
						CyFxGetBothSensorParams(0x3090,&sensor1_white_y_end,	&sensor2_white_y_end);
						CyFxGetBothSensorParams(0x308E,&sensor1_white_x_end,	&sensor2_white_x_end);
						CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
						/* --- laser sensor1 [0..7] --- */
						glEp0Buffer[0]  = (uint8_t)(sensor1_laser_y_start & 0x00FFu);
						glEp0Buffer[1]  = (uint8_t)((sensor1_laser_y_start >> 8) & 0x00FFu);
						glEp0Buffer[2]  = (uint8_t)(sensor1_laser_x_start & 0x00FFu);
						glEp0Buffer[3]  = (uint8_t)((sensor1_laser_x_start >> 8) & 0x00FFu);
						glEp0Buffer[4]  = (uint8_t)(sensor1_laser_y_end & 0x00FFu);
						glEp0Buffer[5]  = (uint8_t)((sensor1_laser_y_end >> 8) & 0x00FFu);
						glEp0Buffer[6]  = (uint8_t)(sensor1_laser_x_end & 0x00FFu);
						glEp0Buffer[7]  = (uint8_t)((sensor1_laser_x_end >> 8) & 0x00FFu);
						/* --- laser sensor2 [8..15] --- */
						glEp0Buffer[8]  = (uint8_t)(sensor2_laser_y_start & 0x00FFu);
						glEp0Buffer[9]  = (uint8_t)((sensor2_laser_y_start >> 8) & 0x00FFu);
						glEp0Buffer[10] = (uint8_t)(sensor2_laser_x_start & 0x00FFu);
						glEp0Buffer[11] = (uint8_t)((sensor2_laser_x_start >> 8) & 0x00FFu);
						glEp0Buffer[12] = (uint8_t)(sensor2_laser_y_end & 0x00FFu);
						glEp0Buffer[13] = (uint8_t)((sensor2_laser_y_end >> 8) & 0x00FFu);
						glEp0Buffer[14] = (uint8_t)(sensor2_laser_x_end & 0x00FFu);
						glEp0Buffer[15] = (uint8_t)((sensor2_laser_x_end >> 8) & 0x00FFu);
						/* --- white sensor1 [16..23] --- */
						glEp0Buffer[16] = (uint8_t)(sensor1_white_y_start & 0x00FFu);
						glEp0Buffer[17] = (uint8_t)((sensor1_white_y_start >> 8) & 0x00FFu);
						glEp0Buffer[18] = (uint8_t)(sensor1_white_x_start & 0x00FFu);
						glEp0Buffer[19] = (uint8_t)((sensor1_white_x_start >> 8) & 0x00FFu);
						glEp0Buffer[20] = (uint8_t)(sensor1_white_y_end & 0x00FFu);
						glEp0Buffer[21] = (uint8_t)((sensor1_white_y_end >> 8) & 0x00FFu);
						glEp0Buffer[22] = (uint8_t)(sensor1_white_x_end & 0x00FFu);
						glEp0Buffer[23] = (uint8_t)((sensor1_white_x_end >> 8) & 0x00FFu);
						/* --- white sensor2 [24..31] --- */
						glEp0Buffer[24] = (uint8_t)(sensor2_white_y_start & 0x00FFu);
						glEp0Buffer[25] = (uint8_t)((sensor2_white_y_start >> 8) & 0x00FFu);
						glEp0Buffer[26] = (uint8_t)(sensor2_white_x_start & 0x00FFu);
						glEp0Buffer[27] = (uint8_t)((sensor2_white_x_start >> 8) & 0x00FFu);
						glEp0Buffer[28] = (uint8_t)(sensor2_white_y_end & 0x00FFu);
						glEp0Buffer[29] = (uint8_t)((sensor2_white_y_end >> 8) & 0x00FFu);
						glEp0Buffer[30] = (uint8_t)(sensor2_white_x_end & 0x00FFu);
						glEp0Buffer[31] = (uint8_t)((sensor2_white_x_end >> 8) & 0x00FFu);
						CyU3PDebugPrint (4, "REPORT_OFFSET laser s1: ys=%d xs=%d ye=%d xe=%d\n",
								sensor1_laser_y_start, sensor1_laser_x_start,
								sensor1_laser_y_end,  sensor1_laser_x_end);
						CyU3PDebugPrint (4, "REPORT_OFFSET laser s2: ys=%d xs=%d ye=%d xe=%d\n",
								sensor2_laser_y_start, sensor2_laser_x_start,
								sensor2_laser_y_end,   sensor2_laser_x_end);
						CyU3PDebugPrint (4, "REPORT_OFFSET white s1: ys=%d xs=%d ye=%d xe=%d\n",
								sensor1_white_y_start, sensor1_white_x_start,
								sensor1_white_y_end,  sensor1_white_x_end);
						CyU3PDebugPrint (4, "REPORT_OFFSET white s2: ys=%d xs=%d ye=%d xe=%d\n",
								sensor2_white_y_start, sensor2_white_x_start,
								sensor2_white_y_end,  sensor2_white_x_end);
						CyU3PUsbSendEP0Data (32, glEp0Buffer);
                        
						break;
				}

				case  REPORT_BINNING_MODE:
				{
					uint16_t bS1, bS2;
					CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));

					/* 0x30B0 Digital_Binning */
					CyFxGetBothSensorParams (0x30B0, &bS1, &bS2);
					glEp0Buffer[0]  = (uint8_t)(bS1 & 0x00FFu);
					glEp0Buffer[1]  = (uint8_t)((bS1 >> 8) & 0x00FFu);
					glEp0Buffer[2]  = (uint8_t)(bS2 & 0x00FFu);
					glEp0Buffer[3]  = (uint8_t)((bS2 >> 8) & 0x00FFu);

					/* 0x3008 X_ADDR_END sensor A */
					CyFxGetBothSensorParams (0x3008, &bS1, &bS2);
					glEp0Buffer[4]  = (uint8_t)(bS1 & 0x00FFu);
					glEp0Buffer[5]  = (uint8_t)((bS1 >> 8) & 0x00FFu);
					glEp0Buffer[6]  = (uint8_t)(bS2 & 0x00FFu);
					glEp0Buffer[7]  = (uint8_t)((bS2 >> 8) & 0x00FFu);

					/* 0x308E X_ADDR_END sensor B */
					CyFxGetBothSensorParams (0x308E, &bS1, &bS2);
					glEp0Buffer[8]  = (uint8_t)(bS1 & 0x00FFu);
					glEp0Buffer[9]  = (uint8_t)((bS1 >> 8) & 0x00FFu);
					glEp0Buffer[10] = (uint8_t)(bS2 & 0x00FFu);
					glEp0Buffer[11] = (uint8_t)((bS2 >> 8) & 0x00FFu);

					/* 0x30A2 X_ODD_INC */
					CyFxGetBothSensorParams (0x30A2, &bS1, &bS2);
					glEp0Buffer[12] = (uint8_t)(bS1 & 0x00FFu);
					glEp0Buffer[13] = (uint8_t)((bS1 >> 8) & 0x00FFu);
					glEp0Buffer[14] = (uint8_t)(bS2 & 0x00FFu);
					glEp0Buffer[15] = (uint8_t)((bS2 >> 8) & 0x00FFu);

					/* 0x30A6 Y_ODD_INC */
					CyFxGetBothSensorParams (0x30A6, &bS1, &bS2);
					glEp0Buffer[16] = (uint8_t)(bS1 & 0x00FFu);
					glEp0Buffer[17] = (uint8_t)((bS1 >> 8) & 0x00FFu);
					glEp0Buffer[18] = (uint8_t)(bS2 & 0x00FFu);
					glEp0Buffer[19] = (uint8_t)((bS2 >> 8) & 0x00FFu);

					/* 0x3040 Read_Mode */
					CyFxGetBothSensorParams (0x3040, &bS1, &bS2);
					glEp0Buffer[20] = (uint8_t)(bS1 & 0x00FFu);
					glEp0Buffer[21] = (uint8_t)((bS1 >> 8) & 0x00FFu);
					glEp0Buffer[22] = (uint8_t)(bS2 & 0x00FFu);
					glEp0Buffer[23] = (uint8_t)((bS2 >> 8) & 0x00FFu);

					/* 0x30AE X_ODD_INC_CB */
					CyFxGetBothSensorParams (0x30AE, &bS1, &bS2);
					glEp0Buffer[24] = (uint8_t)(bS1 & 0x00FFu);
					glEp0Buffer[25] = (uint8_t)((bS1 >> 8) & 0x00FFu);
					glEp0Buffer[26] = (uint8_t)(bS2 & 0x00FFu);
					glEp0Buffer[27] = (uint8_t)((bS2 >> 8) & 0x00FFu);

					/* 0x30A8 Y_ODD_INC_CB */
					CyFxGetBothSensorParams (0x30A8, &bS1, &bS2);
					glEp0Buffer[28] = (uint8_t)(bS1 & 0x00FFu);
					glEp0Buffer[29] = (uint8_t)((bS1 >> 8) & 0x00FFu);
					glEp0Buffer[30] = (uint8_t)(bS2 & 0x00FFu);
					glEp0Buffer[31] = (uint8_t)((bS2 >> 8) & 0x00FFu);
					CyU3PUsbSendEP0Data (32, glEp0Buffer);

					CyU3PDebugPrint (4, "REPORT_BINNING: 30B0 s1=%d s2=%d | 3008 s1=%d s2=%d | 308E s1=%d s2=%d | 30A2 s1=%d s2=%d\n",
							((uint16_t)(glEp0Buffer[1]<<8)|glEp0Buffer[0]),
							((uint16_t)(glEp0Buffer[3]<<8)|glEp0Buffer[2]),
							((uint16_t)(glEp0Buffer[5]<<8)|glEp0Buffer[4]),
							((uint16_t)(glEp0Buffer[7]<<8)|glEp0Buffer[6]),
							((uint16_t)(glEp0Buffer[9]<<8)|glEp0Buffer[8]),
							((uint16_t)(glEp0Buffer[11]<<8)|glEp0Buffer[10]),
							((uint16_t)(glEp0Buffer[13]<<8)|glEp0Buffer[12]),
							((uint16_t)(glEp0Buffer[15]<<8)|glEp0Buffer[14]));
					CyU3PDebugPrint (4, "REPORT_BINNING: 30A6 s1=%d s2=%d | 3040 s1=%d s2=%d | 30AE s1=%d s2=%d | 30A8 s1=%d s2=%d\n",
							((uint16_t)(glEp0Buffer[17]<<8)|glEp0Buffer[16]),
							((uint16_t)(glEp0Buffer[19]<<8)|glEp0Buffer[18]),
							((uint16_t)(glEp0Buffer[21]<<8)|glEp0Buffer[20]),
							((uint16_t)(glEp0Buffer[23]<<8)|glEp0Buffer[22]),
							((uint16_t)(glEp0Buffer[25]<<8)|glEp0Buffer[24]),
							((uint16_t)(glEp0Buffer[27]<<8)|glEp0Buffer[26]),
							((uint16_t)(glEp0Buffer[29]<<8)|glEp0Buffer[28]),
							((uint16_t)(glEp0Buffer[31]<<8)|glEp0Buffer[30]));
					
					break;
				}

				//设置曝光d6
				case CY_FX_RQT_COMMAND_EXPOSURE:
				{
	                AR0234ContextConfig.laserExposureSensor2 = (uint16_t)(wValue);
	                AR0234ContextConfig.whiteLightExposureSensor2 = (uint16_t)(wIndex);
	                AR0234ContextConfig.laserExposureSensor1 = (uint16_t)(wValue);
	                AR0234ContextConfig.whiteLightExposureSensor1 = (uint16_t)(wIndex);
					CyFxAr0234WritePairAndCommit (0x3012, AR0234ContextConfig.laserExposureSensor1);
	                CyFxAr0234WritePairAndCommit (0x3016, AR0234ContextConfig.whiteLightExposureSensor1);
	                if (CyFxVerifyExposureReadback () == CyFalse)
	                {
                	   CyU3PDebugPrint (4, "EXPOSURE readback fail, retrying write...\n");
                	   CyFxAr0234WritePairAndCommit (0x3012, AR0234ContextConfig.laserExposureSensor1);
                	   CyFxAr0234WritePairAndCommit (0x3016, AR0234ContextConfig.whiteLightExposureSensor1);
                	   if (CyFxVerifyExposureReadback () == CyFalse)
                	   {
                		   CyU3PDebugPrint (4, "EXPOSURE request failed (readback mismatch).\n");
                		   CyU3PUsbStall (0, CyTrue, CyFalse);
                		   break;
                	   }
                    }

                    CyU3PDebugPrint (4, "EXPOSURE request successful, laser=%d white=%d\n",
                		  AR0234ContextConfig.laserExposureSensor1,
                		  AR0234ContextConfig.whiteLightExposureSensor1);
					CyU3PUsbAckSetup ();
					break;
				}
                
                case 0xc7:

                	CyU3PUsbAckSetup ();
					break;
                #if 0
					//补光电流d7
                case CY_FX_RQT_COMMAND_SETCURRENT3_WHITE:

                	currentData.whiteLightCurrent = ((uint32_t)(wValue & 0x00FF))<<16;
                	currentData.whiteLightCurrent += (uint32_t)(wIndex);
                	CyFxSpiProtoWrite8 (0x01, 0xaa, (uint8_t)(currentData.whiteLightCurrent >> 16));
                	CyFxSpiProtoWrite8 (0x01, 0xab, (uint8_t)(currentData.whiteLightCurrent >> 8));
                	CyFxSpiProtoWrite8 (0x01, 0xac, (uint8_t)(currentData.whiteLightCurrent ));
				    CyU3PUsbAckSetup ();
					break;
                #endif
			
			    //设置偏移量D8：必须按 1..8 顺序下发，收齐后统一提交
                case CY_FX_RQT_COMMAND_SETOFFSET:
				{
					if ((wIndex < 0x01u) || (wIndex > 0x08u))
					{
						CyU3PDebugPrint (4, "OFFSET invalid index: idx=%d, val=%d\n", (int)wIndex, (int)wValue);
						CyFxResetOffsetTxnState ();
						CyU3PUsbStall (0, CyTrue, CyFalse);
						break;
					}

					if (wIndex != gOffsetTxnExpectedIndex)
					{
						CyU3PDebugPrint (4, "OFFSET out-of-order: exp=%d got=%d, val=%d\n",
								(int)gOffsetTxnExpectedIndex, (int)wIndex, (int)wValue);
						CyFxResetOffsetTxnState ();
						CyU3PUsbStall (0, CyTrue, CyFalse);
						break;
					}

					gOffsetTxnReceivedMask |= (uint8_t)(1u << (wIndex - 1u));
					CyU3PDebugPrint (4, "HOST OFFSET cmd: idx=%d val=%d mask=%d\n",
							(int)wIndex, (int)wValue, (int)gOffsetTxnReceivedMask);

					switch (wIndex)
					{
						case 0x01:
							AR0234ContextConfig.laserOffsetSensor1_y_start = (uint16_t)(wValue);
							AR0234ContextConfig.whiteOffsetSensor1_y_start = (uint16_t)(wValue);
							break;
						case 0x02:
							AR0234ContextConfig.laserOffsetSensor1_x_start = (uint16_t)(wValue);
							AR0234ContextConfig.whiteOffsetSensor1_x_start = (uint16_t)(wValue);
							break;
						case 0x03:
							AR0234ContextConfig.laserOffsetSensor1_y_end = (uint16_t)(wValue);
							AR0234ContextConfig.whiteOffsetSensor1_y_end = (uint16_t)(wValue);
							break;
						case 0x04:
							AR0234ContextConfig.laserOffsetSensor1_x_end = (uint16_t)(wValue);
							AR0234ContextConfig.whiteOffsetSensor1_x_end = (uint16_t)(wValue);
							break;
						case 0x05:
							AR0234ContextConfig.laserOffsetSensor2_y_start = (uint16_t)(wValue);
							AR0234ContextConfig.whiteOffsetSensor2_y_start = (uint16_t)(wValue);
							break;
						case 0x06:
							AR0234ContextConfig.laserOffsetSensor2_x_start = (uint16_t)(wValue);
							AR0234ContextConfig.whiteOffsetSensor2_x_start = (uint16_t)(wValue);
							break;
						case 0x07:
							AR0234ContextConfig.laserOffsetSensor2_y_end = (uint16_t)(wValue);
							AR0234ContextConfig.whiteOffsetSensor2_y_end = (uint16_t)(wValue);
							break;
						case 0x08:
							AR0234ContextConfig.laserOffsetSensor2_x_end = (uint16_t)(wValue);
							AR0234ContextConfig.whiteOffsetSensor2_x_end = (uint16_t)(wValue);
							break;
						default:
							CyFxResetOffsetTxnState ();
							CyU3PUsbStall (0, CyTrue, CyFalse);
							break;
					}

					if (wIndex != 0x08u)
					{
						gOffsetTxnExpectedIndex = (uint8_t)(wIndex + 1u);
						CyU3PThreadSleep (10);
						CyU3PUsbAckSetup ();
						break;
					}

					if (gOffsetTxnReceivedMask != 0xFFu)
					{
						CyU3PDebugPrint (4, "OFFSET incomplete sequence: mask=%d\n", (int)gOffsetTxnReceivedMask);
						CyFxResetOffsetTxnState ();
						CyU3PUsbStall (0, CyTrue, CyFalse);
						break;
					}

					CyFxApplyOffsetRegisters ();
					if (CyFxVerifyOffsetReadback () == CyFalse)
					{
						CyU3PDebugPrint (4, "OFFSET readback fail, retrying write...\n");
						CyFxApplyOffsetRegisters ();
						if (CyFxVerifyOffsetReadback () == CyFalse)
						{
							CyU3PDebugPrint (4, "OFFSET request failed (readback mismatch).\n");
							CyFxResetOffsetTxnState ();
							CyU3PUsbStall (0, CyTrue, CyFalse);
							break;
						}
					}

					CyU3PDebugPrint (4, "OFFSET request successful, x_start=%d, y_start=%d, x_end=%d, y_end=%d\n",
							AR0234ContextConfig.laserOffsetSensor2_x_start, AR0234ContextConfig.laserOffsetSensor2_y_start,
							AR0234ContextConfig.laserOffsetSensor2_x_end, AR0234ContextConfig.laserOffsetSensor2_y_end);
					CyFxResetOffsetTxnState ();
					CyU3PThreadSleep (10);
					CyU3PUsbAckSetup ();
					break;
				}

                case CY_FX_RQT_COMMAND_LED_1:
                	CyFxSpiProtoWrite8 (0x01, 0x0B, 0);           //turn off LED_2
                	CyU3PThreadSleep (10);

					wValue_l = wValue & 0x01;
					CyFxSpiProtoWrite8 (0x01, 0x09, wValue_l);   // send command
					CyU3PThreadSleep (10);

					CyU3PUsbAckSetup ();
					break;

                case CY_FX_RQT_COMMAND_LED_2:             // LED: 8 frequencies + 256 duty
                	CyFxSpiProtoWrite8 (0x01, 0x09, 0);           //turn off LED_1
                	CyU3PThreadSleep (10);

					wValue_l = wValue & 0x01;
					CyFxSpiProtoWrite8 (0x01, 0x0B, wValue_l);   // send command
					CyU3PThreadSleep (10);

					CyU3PUsbAckSetup ();
					break;

                case CY_FX_RQT_COMMAND_LED_3:             //uvc led: ON or OFF
					CyU3PUsbAckSetup ();
					break;

                case CY_FX_RQT_COMMAND_SENSOR1:             //SENSOR1 MONO

                	//AR0234_Write_Sensor2(0x3012,AR0234ContextConfig.laserExposureSensor2);
	                AR0234_Write_Sensor1_Commit((uint16_t)(wValue),(uint16_t)(wIndex));
                	CyU3PThreadSleep (20);
					CyU3PUsbAckSetup ();
					break;

                case CY_FX_RQT_COMMAND_STOPPROJECT:

					CyU3PThreadSleep (200);
                	CyU3PUsbAckSetup ();
					break;

                case CY_FX_RQT_COMMAND_SENSOR2:   // SENSOR2 RGB
	                AR0234_Write_Sensor2_Commit((uint16_t)(wValue),(uint16_t)(wIndex));
					CyU3PThreadSleep (20);
                	CyU3PUsbAckSetup ();
					break;

                case CY_FX_RQT_COMMAND_RESUME:
			       // start/stop the device when device is NOT in start/stop operation
					if(glInResume == CyFalse)      // //(glIsCapMode == CyTrue) &&
					{
						if((wValue & 0x01) == 0x01)
						{
							CyFxButtonPressed_Start();
							#ifdef DEBUG
							CyU3PDebugPrint (4, "CyFxButtonPressed_Start() is called\n");
							#endif
						}
						else if((wValue & 0x01) == 0x00)
						{
							CyFxButtonPressed_Stop();
							#ifdef DEBUG
							CyU3PDebugPrint (4, "CyFxButtonPressed_Stop() is called\n");
							#endif
						}
					}
					CyU3PUsbAckSetup ();
					break;

                case CY_FX_RQT_COMMAND_FAN:
					wValue_l = wValue & 0x01;
					CyFxSpiProtoWrite8 (0x01, 0x04, wValue_l);   //START/STOP FAN：1（风扇开）；0（风扇关）；
					CyU3PThreadSleep (10);
					CyU3PUsbAckSetup ();
					break;

                case CY_FX_RQT_COMMAND_HEATING:
					wValue_l = wValue & 0x01;
					CyFxSpiProtoWrite8 (0x01, 0x05, wValue_l);   //START/STOP HEATING
					CyU3PThreadSleep (10);
					CyU3PUsbAckSetup ();
					break;

                case CY_FX_RQT_COMMAND_UPDATE:

					CyU3PUsbAckSetup ();
					if(glIsReconfigure == CyFalse)		CyFxDeviceReConfigureAll();
					CyFxUpdate();
					break;

                default:
                    /* This is unknown request. */
                    isHandled = CyFalse;
                    break;
            }
        }
    return isHandled;
}

//test for cal-II；
/* This is the callback function to handle the USB events. */
void CyFxSlFifoApplnUSBEventCB (CyU3PUsbEventType_t evtype,uint16_t evdata)
{
    switch (evtype)
    {
        case CY_U3P_USB_EVENT_SETCONF:// USB配置设置事件
            /* Stop the application before re-starting. */
            if (glIsApplnActive)
            {
                CyFxSlFifoApplnStop ();// 停止当前应用
            }
            CyU3PUsbLPMDisable(); // 禁用低功耗模式
            /* Start the loop back function. */
            CyFxSlFifoApplnStart (); // 启动数据传输应用
            CyU3PDmaChannelReset (&glUARTRxHandle);
			CyFxScheduleSelfInit ();
            break;

        case CY_U3P_USB_EVENT_RESET:
        case CY_U3P_USB_EVENT_DISCONNECT:
			CyFxUpdateDeviceReadyState (CY_FX_DEVICE_READY_NOT_READY);
            /* Stop the loop back function. */
            if (glIsApplnActive)
            {
                CyFxSlFifoApplnStop ();// USB断开时停止应用
            }
            CyU3PDmaChannelReset (&glUARTRxHandle);
            CyU3PGpifSMControl(CyTrue); // 暂停GPIF状态机
            break;

        default:
            break;
    }
}


CyBool_t CyFxApplnLPMRqtCB (CyU3PUsbLinkPowerMode link_mode)
{
    return CyTrue;
}

void gpif_error_cb(CyU3PPibIntrType cbType, uint16_t cbArg)
{

	if(cbType==CYU3P_PIB_INTR_ERROR)
	{
		switch (CYU3P_GET_PIB_ERROR_TYPE(cbArg))
		{
			case CYU3P_PIB_ERR_THR0_WR_OVERRUN:

			CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR0_WR_OVERRUN");

			break;
			case CYU3P_PIB_ERR_THR1_WR_OVERRUN:

			CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR1_WR_OVERRUN");

			break;
			case CYU3P_PIB_ERR_THR2_WR_OVERRUN:

			CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR2_WR_OVERRUN");

			break;
			case CYU3P_PIB_ERR_THR3_WR_OVERRUN:

			CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR3_WR_OVERRUN");

			break;

			case CYU3P_PIB_ERR_THR0_RD_UNDERRUN:

			CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR0_RD_UNDERRUN");

			break;
			case CYU3P_PIB_ERR_THR1_RD_UNDERRUN:

			CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR1_RD_UNDERRUN");

			break;
			case CYU3P_PIB_ERR_THR2_RD_UNDERRUN:

			CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR2_RD_UNDERRUN");

			break;
			case CYU3P_PIB_ERR_THR3_RD_UNDERRUN:

			CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR3_RD_UNDERRUN");

			break;

			default:

			//CyU3PDebugPrint (4, "No Error :%d\n ",CYU3P_GET_PIB_ERROR_TYPE(cbArg));

				break;
		}
	}

}

void CyFxGpioIntrCb (uint8_t gpioId)
{
    CyBool_t gpioValue = CyFalse;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* 霍尔传感器双沿中断：直接通知磁吸线程去读实际电平 */
    if (gpioId == FX3_GPIO_HALL)
    {
        CyU3PEventSet(&glFxGpioAppEvent, CY_FX_GPIOAPP_MAGNETIC_EVENT, CYU3P_EVENT_OR);
        return;
    }

	// 放宽触发条件：只在恢复流程期间屏蔽，其他时刻都允许按键触发
	if (glInResume == CyFalse)
    {
    	/* Get the status of the pin */
		apiRetStatus = CyU3PGpioGetValue (gpioId, &gpioValue);
		if (apiRetStatus == CY_U3P_SUCCESS)
		{
			/* Check status of the pin ，低电平有效*/
			if (gpioValue == CyFalse)
			{
				/* Set GPIO low event */
				CyU3PEventSet(&glFxGpioAppEvent, CY_FX_GPIOAPP_GPIO_LOW_EVENT,  CYU3P_EVENT_OR);
			}
		}
	}
}

/* This function initializes the GPIF interface and initializes
 * the USB interface. */
void CyFxSlFifoApplnInit (void)
{
    CyU3PPibClock_t pibClock;
    CyU3PGpioClock_t gpioClock;
    CyU3PGpioSimpleConfig_t gpioConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
     // 1. 传感器和电流配置初始化
    AR0234_Context_Init ();
    Current_Init ();

    // 2. P-Port时钟配置
    pibClock.clkDiv = 2;
    pibClock.clkSrc = CY_U3P_SYS_CLK;
    pibClock.isHalfDiv = CyFalse;
    /* Disable DLL for sync GPIF */
    pibClock.isDllEnable = CyFalse;
	// 初始化P-Port接口
    CyU3PPibInit(CyTrue, &pibClock);
 
    // 3. 加载GPIF配置
    CyU3PGpifLoad (&CyFxGpifConfig);

     // 4. 配置GPIF Socket 3用于控制通道
     apiRetStatus = CyU3PGpifSocketConfigure(3, CY_U3P_PIB_SOCKET_3, 6, CyFalse, 1);
     if (apiRetStatus != CY_U3P_SUCCESS)
     {
         CyU3PDebugPrint(4, "GPIF Socket configure failed, Error Code = %d\n", apiRetStatus);
         CyFxAppErrorHandler(apiRetStatus);
     }

	 // 5. GPIO初始化
	 gpioClock.fastClkDiv = 2;
	 gpioClock.slowClkDiv = 0;
	 gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
	 gpioClock.clkSrc = CY_U3P_SYS_CLK;
	 gpioClock.halfDiv = 0;
    
	 apiRetStatus = CyU3PGpioInit(&gpioClock, CyFxGpioIntrCb);  // 注册中断回调，引脚发生变化时调用

	/* FPGA 程序加载控制脚 (GPIO28)：必须先于 FPGA_PWR_EN 完成配置并输出低电平，
	 * 以保证 FPGA 上电瞬间该脚就已经被拉低，直到 AR0234/FPGA 初始化完成后再释放。 */
	apiRetStatus = CyU3PDeviceGpioOverride (FPGA_PROG_CTRL, CyTrue);
	gpioConfig.inputEn     = CyFalse;
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
	gpioConfig.outValue    = CyFalse;
	apiRetStatus = CyU3PGpioSetSimpleConfig (FPGA_PROG_CTRL, &gpioConfig);

	apiRetStatus = CyU3PDeviceGpioOverride (FPGA_PWR_EN, CyTrue);// FPGA电源控制

	/* Configure GPIO as output with deterministic initial levels. */
    gpioConfig.inputEn = CyFalse;        // 禁用输入
    gpioConfig.driveLowEn = CyTrue;      // 使能低电平驱动
    gpioConfig.driveHighEn = CyTrue;     // 使能高电平驱动
    gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR; // 无中断
	/* 电源与状态脚默认高；reset脚默认低，避免输出使能瞬间出现释放复位毛刺。 */
	gpioConfig.outValue = CyTrue;
	apiRetStatus = CyU3PGpioSetSimpleConfig(FPGA_PWR_EN, &gpioConfig);
	gpioConfig.outValue = CyFalse;
	apiRetStatus = CyU3PGpioSetSimpleConfig (FX3_DEVICE_RESET, &gpioConfig);
	gpioConfig.outValue = CyTrue;
	apiRetStatus = CyU3PGpioSetSimpleConfig (FX3_SNAP, &gpioConfig);

	  // 配置输入GPIO（按钮）
	gpioConfig.outValue    = CyFalse;
	gpioConfig.inputEn     = CyTrue;
	gpioConfig.driveLowEn  = CyFalse;
	gpioConfig.driveHighEn = CyFalse;
	gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (BUTTON1_ON, &gpioConfig);
	apiRetStatus           = CyU3PGpioSetSimpleConfig (BUTTON2_ON, &gpioConfig);

	/* 配置霍尔传感器 GPIO：双沿中断 + 弱上拉 */
	gpioConfig.intrMode    = CY_U3P_GPIO_INTR_BOTH_EDGE;
	apiRetStatus           = CyU3PGpioSetSimpleConfig(FX3_GPIO_HALL, &gpioConfig);
	if (apiRetStatus == CY_U3P_SUCCESS)
		(void)CyU3PGpioSetIoMode(FX3_GPIO_HALL, CY_U3P_GPIO_IO_MODE_WPU);
	gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
	/* Configure BUTTON as input with pull-up and negative edge interrupt */
	gpioConfig.intrMode    = CY_U3P_GPIO_INTR_NEG_EDGE;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (BUTTON, &gpioConfig);
	/* 如果硬件为低电平按下，这里加弱上拉，避免未按下时悬空导致抖动 */
	if (apiRetStatus == CY_U3P_SUCCESS)
	{
		(void)CyU3PGpioSetIoMode(BUTTON, CY_U3P_GPIO_IO_MODE_WPU); // weak pull-up
	}

	CyU3PGpioSetValue (FX3_DEVICE_RESET, CyFalse);   // FPGA复位保持(低)
	CyU3PGpioSetValue (FX3_SNAP, CyTrue);            // 复位FIFO状态机

	if (gSpiStandaloneInited == CyFalse)
	{
		apiRetStatus = CyFxSpiStandaloneStart ();
		if (apiRetStatus != CY_U3P_SUCCESS)
		{
			#ifdef DEBUG
			CyU3PDebugPrint (4, "CyFxSpiStandaloneStart failed, Error Code = %d\n", apiRetStatus);
			#endif
			CyFxAppErrorHandler (apiRetStatus);
		}

		gSpiStandaloneInited = CyTrue;
	}

	/* 在 USB 对外可见之前完成 I2C 初始化，避免主机控制请求早于外设就绪。 */
	apiRetStatus = CyFxI2cInit ();
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		#ifdef DEBUG
		CyU3PDebugPrint (4, "I2C init failed, Error code = %d\n", apiRetStatus);
		#endif
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* 在启动 GPIF 状态机前注册错误回调，避免早期错误被吞掉。 */
	CyU3PPibRegisterCallback (gpif_error_cb, 0xffff);

	/* GPIO 已经进入安全态后，再启动 GPIF 状态机。 */
	apiRetStatus = CyU3PGpifSMStart (RESET, ALPHA_RESET);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		#ifdef DEBUG
		CyU3PDebugPrint (4, "GPIF state machine start failed, Error code = %d\n", apiRetStatus);
		#endif
		CyFxAppErrorHandler (apiRetStatus);
	}

    /* Start the USB functionality. */
    apiRetStatus = CyU3PUsbStart();//启动USB

    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

   
    //注册USB回调函数
    /* The fast enumeration is the easiest way to setup a USB connection,
     * where all enumeration phase is handled by the library. Only the
     * class / vendor requests need to be handled by the application. */
    // 触发时机: 收到USB Setup包时
	// 用途: 处理来自上位机的控制指令和设备配置请求
    CyU3PUsbRegisterSetupCallback(CyFxSlFifoApplnUSBSetupCB, CyTrue);

    /* Setup the callback to handle the USB events. */
	// 触发时机: USB连接状态变化时（连接、断开、挂起等）
	// 用途: 处理USB连接状态变化事件
    CyU3PUsbRegisterEventCallback(CyFxSlFifoApplnUSBEventCB);

    /* Register a callback to handle LPM requests from the USB 3.0 host. */
	// 触发时机: 收到USB 3.0的低功耗模式（LPM）请求时
	// 用途: 处理USB 3.0低功耗管理请求
    CyU3PUsbRegisterLPMRequestCallback(CyFxApplnLPMRqtCB);    

        /* Set the USB Enumeration descriptors */

	// 10. 设置USB描述符
    /* Super speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB30DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
		#ifdef DEBUG
        CyU3PDebugPrint (4, "USB set device descriptor failed, Error code = %d\n", apiRetStatus);
		#endif
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* High speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
		#ifdef DEBUG
        CyU3PDebugPrint (4, "USB set device descriptor failed, Error code = %d\n", apiRetStatus);
		#endif
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* BOS descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)CyFxUSBBOSDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
		#ifdef DEBUG
        CyU3PDebugPrint (4, "USB set configuration descriptor failed, Error code = %d\n", apiRetStatus);
		#endif
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Device qualifier descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
		#ifdef DEBUG
        CyU3PDebugPrint (4, "USB set device qualifier descriptor failed, Error code = %d\n", apiRetStatus);
		#endif
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Super speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
		#ifdef DEBUG
        CyU3PDebugPrint (4, "USB set configuration descriptor failed, Error code = %d\n", apiRetStatus);
		#endif
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* High speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
		#ifdef DEBUG
        CyU3PDebugPrint (4, "USB Set Other Speed Descriptor failed, Error Code = %d\n", apiRetStatus);
		#endif
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Full speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
		#ifdef DEBUG
        CyU3PDebugPrint (4, "USB Set Configuration Descriptor failed, Error Code = %d\n", apiRetStatus);
		#endif
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 0 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
		#ifdef DEBUG
        CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
		#endif
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 1 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
		#ifdef DEBUG
        CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
		#endif
        CyFxAppErrorHandler(apiRetStatus);
    }

	/* String descriptor 2 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
		#ifdef DEBUG
        CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
		#endif
        CyFxAppErrorHandler(apiRetStatus);
    }
	/* Delay FPGA reset release until SETCONF has built DMA/EP data paths. */
	CyU3PGpioSetValue (FX3_DEVICE_RESET, CyFalse);
	SetFpgaPoweredReady(CyFalse);

     // 11. 连接USB
    apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);   //CyTrue
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
		#ifdef DEBUG
        CyU3PDebugPrint (4, "USB Connect failed, Error code = %d\n", apiRetStatus);
		#endif

        CyFxAppErrorHandler(apiRetStatus);
    }
}

/* Entry function for the slFifoAppThread. */
void SlFifoAppThread_Entry (uint32_t input)
{
	uint32_t eventFlag;
	CyU3PReturnStatus_t txApiRetStatus = CY_U3P_SUCCESS;
    uint8_t press_cnt;
    CyBool_t gpioValue;

    /* 初始化调试输出*/
    CyFxSlFifoApplnDebugInit();
    /* 核心初始化 */
    CyFxSlFifoApplnInit();//上位机处理流程从这里进
	/* I2C 已在核心初始化中完成，这里只通知 JY901 线程。 */
    CyU3PEventSet(&glFxI2cEvent, CY_FX_I2C_INIT_COMPLETE_EVENT, CYU3P_EVENT_OR);

    //AR0234_Config ();
   
    glIsReconfigure = CyFalse;     //NOT re_config
    glInResume      = CyFalse;     //NOT in start/stop operation
    glIsDeviceRun   = CyFalse;     //NOT running
	CyFxUpdateDeviceReadyState (CY_FX_DEVICE_READY_NOT_READY);
	/* 启动阶段已释放 FPGA 复位，因此硬件 ready 状态独立于 glIsInitialized。 */
    glStatus_Device[0] = 0x00;        //default device status
    glStatus_Extra  = 0x00;        //default extra  status
    glStatus_FPGA   = 0x00;        //default fpga   status
   
    for (;;)
    {

        press_cnt = 0;
        gpioValue = CyFalse;

		txApiRetStatus = CyU3PEventGet(&glFxGpioAppEvent,
			CY_FX_GPIOAPP_GPIO_LOW_EVENT | CY_FX_APP_SELF_INIT_EVENT,
			CYU3P_EVENT_OR_CLEAR, &eventFlag, CYU3P_WAIT_FOREVER);//等待按键或自初始化事件
		if (txApiRetStatus != CY_U3P_SUCCESS)
		{
			continue;
		}

		if (eventFlag & CY_FX_APP_SELF_INIT_EVENT)
		{
			CyFxRunSelfInit ();
		}

		if (((eventFlag & CY_FX_GPIOAPP_GPIO_LOW_EVENT) == 0) || (glPress_Detect != CyTrue))
		{
			continue;
		}

            #ifdef DEBUG
	        CyU3PDebugPrint (4, "Step1:Enter thread,glPress_Detect is CyTrue" );
            #endif
            #ifdef DEBUG
			CyU3PDebugPrint (4, "Step2:Button is pressed");
            #endif
            #ifdef DEBUG
			CyU3PDebugPrint (4, "Step3:Enter Button deal");
            #endif

			//按键时间计算
			do{
					CyU3PGpioGetValue(BUTTON, &gpioValue);
					CyU3PThreadSleep(10);                   //恢复原来的10ms延时，保持稳定的时序
					press_cnt++;
					if(press_cnt > LONG_PRESS_TIME)
					{
						glPress_Detect = CyFalse;
						eventFlag &= (~(0x01<<1));
						break;
					}

				}while(gpioValue == CyFalse);

				txApiRetStatus = 1;
				glPress_Detect = CyFalse;
				eventFlag &= (~(0x01<<1));//eventFlag的bit1被强制清零？？？

				if(press_cnt < LONG_PRESS_TIME)
			    {
					if(press_cnt > 0)
					{
						glStatus_Extra &= 0x7f;      // clear long press flag
						glStatus_Extra |= 0x40;      // 短按标志
					}
					if(press_cnt == 0)
					{
						glStatus_Extra &= 0x3f;
					}

				}
				else
				{
					// 长按必关投影：无条件调用停止流程，避免因状态位不同步导致无效
					CyFxButtonPressed_Stop();
					glStatus_Extra &= 0xbf;      // clear short press flag
					glStatus_Extra |= 0x80;      // set long press flag
					CyU3PThreadSleep(3000);                   //delay 3000ms
				}
				glPress_Detect = CyTrue;
					
    } //for
}



/* 磁吸开关线程入口函数（0107 方案：基于硬件 GPIO 双沿中断） */
void slMagneticSwitchAppThread_Entry (uint32_t input)
{
	uint32_t evFlag = 0;

	for (;;)
	{
		/* 等待霍尔中断事件；5ms 超时防止线程饥饿 */
		(void)CyU3PEventGet(&glFxGpioAppEvent,
							CY_FX_GPIOAPP_MAGNETIC_EVENT,
							CYU3P_EVENT_OR_CLEAR,
							&evFlag,
							5);

		if (evFlag & CY_FX_GPIOAPP_MAGNETIC_EVENT)
		{
			CyBool_t hallLevel = CyFalse;

			if ((!glIsInitialized) || (!glFpgaPoweredReady))
				continue;

			if (CyU3PGpioGetValue(FX3_GPIO_HALL, &hallLevel) != CY_U3P_SUCCESS)
				continue;

			/* 低电平 = 磁铁靠近（触发停止） */
			glMagneticSwitch = (hallLevel == CyFalse) ? CyTrue : CyFalse;
			if ((glMagneticSwitch == CyTrue) && (glIsDeviceRun == CyTrue) && (glInResume == CyFalse))
				CyFxButtonPressed_Stop();
			/* 磁铁离开：不自动重启，等待外部恢复命令。 */
		}
	}
}



static void
CyFxAr0234WritePairAndCommit (uint16_t regAddr, uint16_t regData)
{
	AR0234_Write_SensorsSame_Commit (regAddr, regData);
}

static void
CyFxAr0234WriteIndependentAndCommit (uint16_t regAddr, uint16_t sensor1Data, uint16_t sensor2Data)
{
	AR0234_Write_SensorsIndependent_Commit (regAddr, sensor1Data, sensor2Data);
}

static CyBool_t
CyFxVerifyBinningRegPair (uint16_t regAddr, uint16_t expectedData)
{
	uint16_t sensor1Data = 0;
	uint16_t sensor2Data = 0;

	CyFxGetBothSensorParams (regAddr, &sensor1Data, &sensor2Data);

	if ((sensor1Data != expectedData) || (sensor2Data != expectedData))
	{
		CyU3PThreadSleep (CY_FX_BINNING_VERIFY_RETRY_DELAY_MS);
		CyFxGetBothSensorParams (regAddr, &sensor1Data, &sensor2Data);
	}

	if ((sensor1Data == expectedData) && (sensor2Data == expectedData))
	{
		return CyTrue;
	}

	CyU3PDebugPrint (4,
			"BINNING verify fail: reg=%d exp=%d s1=%d s2=%d\n",
			(unsigned int)regAddr,
			(unsigned int)expectedData,
			(unsigned int)sensor1Data,
			(unsigned int)sensor2Data);

	return CyFalse;
}



static CyBool_t
CyFxVerifyBinningModeReadback (uint8_t mode, uint16_t s1AXEnd, uint16_t s2AXEnd, uint16_t s1BXEnd, uint16_t s2BXEnd)
{
	CyBool_t allPassed = CyTrue;

	if (mode == normal_mode)
	{
		if (CyFxVerifyBinningRegPair (0x30B0, 0x0028) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifySensorRegPair (0x3008, s1AXEnd, s2AXEnd) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifySensorRegPair (0x308E, s1BXEnd, s2BXEnd) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifyBinningRegPair (0x30A2, 0x0001) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifyBinningRegPair (0x30A6, 0x0001) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifyBinningRegPair (0x3040, 0x0000) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifyBinningRegPair (0x30AE, 0x0001) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifyBinningRegPair (0x30A8, 0x0001) == CyFalse) { allPassed = CyFalse; }
	}
	else if (mode == binning_sum)
	{
		if (CyFxVerifyBinningRegPair (0x30B0, 0x00A8) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifySensorRegPair (0x3008, s1AXEnd, s2AXEnd) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifySensorRegPair (0x308E, s1BXEnd, s2BXEnd) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifyBinningRegPair (0x30A2, 0x0003) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifyBinningRegPair (0x30A6, 0x0003) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifyBinningRegPair (0x3040, 0x3C20) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifyBinningRegPair (0x30AE, 0x0003) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifyBinningRegPair (0x30A8, 0x0003) == CyFalse) { allPassed = CyFalse; }
	}
	else if (mode == binning_average)
	{
		if (CyFxVerifyBinningRegPair (0x30B0, 0x0028) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifySensorRegPair (0x3008, s1AXEnd, s2AXEnd) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifySensorRegPair (0x308E, s1BXEnd, s2BXEnd) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifyBinningRegPair (0x30A2, 0x0003) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifyBinningRegPair (0x30A6, 0x0003) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifyBinningRegPair (0x3040, 0x3C00) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifyBinningRegPair (0x30AE, 0x0003) == CyFalse) { allPassed = CyFalse; }
		if (CyFxVerifyBinningRegPair (0x30A8, 0x0003) == CyFalse) { allPassed = CyFalse; }
	}
	else
	{
		CyU3PDebugPrint (4, "BINNING verify skip: unknown mode=%d\n", mode);
		return CyFalse;
	}

	return allPassed;
}


static CyBool_t
CyFxVerifySensorRegPair (uint16_t regAddr, uint16_t expectedSensor1Data, uint16_t expectedSensor2Data)
{
	uint16_t sensor1Data = 0;
	uint16_t sensor2Data = 0;

	CyFxGetBothSensorParams (regAddr, &sensor1Data, &sensor2Data);

	if ((sensor1Data != expectedSensor1Data) || (sensor2Data != expectedSensor2Data))
	{
		CyU3PThreadSleep (CY_FX_BINNING_VERIFY_RETRY_DELAY_MS);
		CyFxGetBothSensorParams (regAddr, &sensor1Data, &sensor2Data);
	}

	if ((sensor1Data == expectedSensor1Data) && (sensor2Data == expectedSensor2Data))
	{
		return CyTrue;
	}

	CyU3PDebugPrint (4,
			"READBACK verify fail: reg=%d exp1=%d exp2=%d s1=%d s2=%d\n",
			(unsigned int)regAddr,
			(unsigned int)expectedSensor1Data,
			(unsigned int)expectedSensor2Data,
			(unsigned int)sensor1Data,
			(unsigned int)sensor2Data);

	return CyFalse;
}

static void
CyFxApplyOffsetRegisters (void)
{
	//y_start//
	CyFxAr0234WriteIndependentAndCommit (0x3002,
			AR0234ContextConfig.laserOffsetSensor1_y_start,
			AR0234ContextConfig.laserOffsetSensor2_y_start);

	CyFxAr0234WriteIndependentAndCommit (0x308C,
			AR0234ContextConfig.whiteOffsetSensor1_y_start,
			AR0234ContextConfig.whiteOffsetSensor2_y_start);

	//x_start//
	CyFxAr0234WriteIndependentAndCommit (0x3004,
			AR0234ContextConfig.laserOffsetSensor1_x_start,
			AR0234ContextConfig.laserOffsetSensor2_x_start);

	CyFxAr0234WriteIndependentAndCommit (0x308A,
			AR0234ContextConfig.whiteOffsetSensor1_x_start,
			AR0234ContextConfig.whiteOffsetSensor2_x_start);

	//y_end//
	CyFxAr0234WriteIndependentAndCommit (0x3006,
			AR0234ContextConfig.laserOffsetSensor1_y_end,
			AR0234ContextConfig.laserOffsetSensor2_y_end);

	CyFxAr0234WriteIndependentAndCommit (0x3090,
			AR0234ContextConfig.whiteOffsetSensor1_y_end,
			AR0234ContextConfig.whiteOffsetSensor2_y_end);

	//x_end//
	CyFxAr0234WriteIndependentAndCommit (0x3008,
			AR0234ContextConfig.laserOffsetSensor1_x_end,
			AR0234ContextConfig.laserOffsetSensor2_x_end);

	CyFxAr0234WriteIndependentAndCommit (0x308E,
			AR0234ContextConfig.whiteOffsetSensor1_x_end,
			AR0234ContextConfig.whiteOffsetSensor2_x_end);
}

static CyBool_t
CyFxVerifyOffsetReadback (void)
{
	CyBool_t allPassed = CyTrue;
	uint16_t sensor1Data;
	uint16_t sensor2Data;

#define CY_FX_VERIFY_OFFSET_REG(addr, expected1, expected2) \
	do { \
		CyFxGetBothSensorParams ((addr), &sensor1Data, &sensor2Data); \
		if ((sensor1Data != (expected1)) || (sensor2Data != (expected2))) { \
			CyU3PThreadSleep (CY_FX_BINNING_VERIFY_RETRY_DELAY_MS); \
			CyFxGetBothSensorParams ((addr), &sensor1Data, &sensor2Data); \
		} \
		if ((sensor1Data != (expected1)) || (sensor2Data != (expected2))) { \
			CyU3PDebugPrint (4, "OFFSET readback fail: reg=%d exp1=%d exp2=%d s1=%d s2=%d\n", \
					(int)(addr), (int)(expected1), (int)(expected2), (int)sensor1Data, (int)sensor2Data); \
			allPassed = CyFalse; \
		} \
	} while (0)

	CY_FX_VERIFY_OFFSET_REG(0x3002, AR0234ContextConfig.laserOffsetSensor1_y_start, AR0234ContextConfig.laserOffsetSensor2_y_start);
	CY_FX_VERIFY_OFFSET_REG(0x308C, AR0234ContextConfig.whiteOffsetSensor1_y_start, AR0234ContextConfig.whiteOffsetSensor2_y_start);
	CY_FX_VERIFY_OFFSET_REG(0x3004, AR0234ContextConfig.laserOffsetSensor1_x_start, AR0234ContextConfig.laserOffsetSensor2_x_start);
	CY_FX_VERIFY_OFFSET_REG(0x308A, AR0234ContextConfig.whiteOffsetSensor1_x_start, AR0234ContextConfig.whiteOffsetSensor2_x_start);
	CY_FX_VERIFY_OFFSET_REG(0x3006, AR0234ContextConfig.laserOffsetSensor1_y_end, AR0234ContextConfig.laserOffsetSensor2_y_end);
	CY_FX_VERIFY_OFFSET_REG(0x3090, AR0234ContextConfig.whiteOffsetSensor1_y_end, AR0234ContextConfig.whiteOffsetSensor2_y_end);
	CY_FX_VERIFY_OFFSET_REG(0x3008, AR0234ContextConfig.laserOffsetSensor1_x_end, AR0234ContextConfig.laserOffsetSensor2_x_end);
	CY_FX_VERIFY_OFFSET_REG(0x308E, AR0234ContextConfig.whiteOffsetSensor1_x_end, AR0234ContextConfig.whiteOffsetSensor2_x_end);

#undef CY_FX_VERIFY_OFFSET_REG

	return allPassed;
}

static CyBool_t
CyFxVerifyGainReadback (void)
{
	uint16_t expectedSensor1Gain = (uint16_t)((AR0234ContextConfig.laserGainSensor1 & 0x00FFu) |
			((AR0234ContextConfig.whiteLightGainSensor1 & 0x00FFu) << 8));
	uint16_t expectedSensor2Gain = (uint16_t)((AR0234ContextConfig.laserGainSensor2 & 0x00FFu) |
			((AR0234ContextConfig.whiteLightGainSensor2 & 0x00FFu) << 8));

	return CyFxVerifySensorRegPair (0x3060, expectedSensor1Gain, expectedSensor2Gain);
}

static CyBool_t
CyFxVerifyExposureReadback (void)
{
	CyBool_t allPassed = CyTrue;

	if (CyFxVerifySensorRegPair (0x3012,
			AR0234ContextConfig.laserExposureSensor1,
			AR0234ContextConfig.laserExposureSensor2) == CyFalse) { allPassed = CyFalse; }
	if (CyFxVerifySensorRegPair (0x3016,
			AR0234ContextConfig.whiteLightExposureSensor1,
			AR0234ContextConfig.whiteLightExposureSensor2) == CyFalse) { allPassed = CyFalse; }

	return allPassed;
}

static CyBool_t
CyFxWriteBinningModeRegisters (uint8_t mode, uint16_t s1AXEnd, uint16_t s2AXEnd, uint16_t s1BXEnd, uint16_t s2BXEnd)
{
	if (mode == normal_mode)
	{
		CyFxAr0234WritePairAndCommit (0x30B0, 0x0028);
		CyFxAr0234WritePairAndCommit (0x30A2, 0x0001);
		CyFxAr0234WritePairAndCommit (0x30A6, 0x0001);
		CyFxAr0234WritePairAndCommit (0x3040, 0x0000);
		CyFxAr0234WritePairAndCommit (0x30AE, 0x0001);
		CyFxAr0234WritePairAndCommit (0x30A8, 0x0001);
		CyFxAr0234WriteIndependentAndCommit (0x3008, s1AXEnd, s2AXEnd);
		CyFxAr0234WriteIndependentAndCommit (0x308E, s1BXEnd, s2BXEnd);
		CyFxSpiProtoWrite8 (0x01, 0x86, 0x00);
		return CyTrue;
	}
	else if (mode == binning_sum)
	{
		CyFxAr0234WritePairAndCommit (0x30B0, 0x00A8);
		CyFxAr0234WritePairAndCommit (0x30A2, 0x0003);
		CyFxAr0234WritePairAndCommit (0x30A6, 0x0003);
		CyFxAr0234WritePairAndCommit (0x3040, 0x3C20);
		CyFxAr0234WritePairAndCommit (0x30AE, 0x0003);
		CyFxAr0234WritePairAndCommit (0x30A8, 0x0003);
		CyFxAr0234WriteIndependentAndCommit (0x3008, s1AXEnd, s2AXEnd);
		CyFxAr0234WriteIndependentAndCommit (0x308E, s1BXEnd, s2BXEnd);
		CyFxSpiProtoWrite8 (0x01, 0x86, 0x01);
		return CyTrue;
	}
	else if (mode == binning_average)
	{
		CyFxAr0234WritePairAndCommit (0x30B0, 0x0028);
		CyFxAr0234WritePairAndCommit (0x30A2, 0x0003);
		CyFxAr0234WritePairAndCommit (0x30A6, 0x0003);
		CyFxAr0234WritePairAndCommit (0x3040, 0x3C00);
		CyFxAr0234WritePairAndCommit (0x30AE, 0x0003);
		CyFxAr0234WritePairAndCommit (0x30A8, 0x0003);
		CyFxAr0234WriteIndependentAndCommit (0x3008, s1AXEnd, s2AXEnd);
		CyFxAr0234WriteIndependentAndCommit (0x308E, s1BXEnd, s2BXEnd);
		CyFxSpiProtoWrite8 (0x01, 0x86, 0x01);
		return CyTrue;
	}
	return CyFalse;
}

static CyBool_t
CyFxApplyBinningMode (uint8_t mode)
{
	uint16_t s1AXEnd = AR0234ContextConfig.laserOffsetSensor1_x_end;
	uint16_t s2AXEnd = AR0234ContextConfig.laserOffsetSensor2_x_end;
	uint16_t s1BXEnd = AR0234ContextConfig.whiteOffsetSensor1_x_end;
	uint16_t s2BXEnd = AR0234ContextConfig.whiteOffsetSensor2_x_end;
	if (mode != normal_mode)
	{
		s1AXEnd = (uint16_t)(s1AXEnd + 4u);
		s2AXEnd = (uint16_t)(s2AXEnd + 4u);
		s1BXEnd = (uint16_t)(s1BXEnd + 4u);
		s2BXEnd = (uint16_t)(s2BXEnd + 4u);
	}

	if (CyFxWriteBinningModeRegisters (mode, s1AXEnd, s2AXEnd, s1BXEnd, s2BXEnd) == CyFalse)
	{
		CyU3PDebugPrint (4, "BINNING mode unsupported: %d\n", mode);
		return CyFalse;
	}

	CyU3PThreadSleep (CY_FX_BINNING_SETTLE_DELAY_MS);
	if (CyFxVerifyBinningModeReadback (mode, s1AXEnd, s2AXEnd, s1BXEnd, s2BXEnd) == CyFalse)
	{
		CyU3PDebugPrint (4, "BINNING readback fail, retrying write, mode=%d\n", mode);
		if (CyFxWriteBinningModeRegisters (mode, s1AXEnd, s2AXEnd, s1BXEnd, s2BXEnd) == CyFalse)
		{
			CyU3PDebugPrint (4, "BINNING mode unsupported: %d\n", mode);
			return CyFalse;
		}
		CyU3PThreadSleep (CY_FX_BINNING_SETTLE_DELAY_MS);
		if (CyFxVerifyBinningModeReadback (mode, s1AXEnd, s2AXEnd, s1BXEnd, s2BXEnd) == CyFalse)
		{
			CyU3PDebugPrint (4, "BINNING request failed (readback mismatch), mode=%d\n", mode);
			return CyFalse;
		}
	}

	if (mode == normal_mode)
	{
		CyU3PDebugPrint (4, "Normal mode configuration done (readback ok).\n");
	}
	else if (mode == binning_average)
	{
		CyU3PDebugPrint (4, "Binning average mode configuration done (readback ok).\n");
	}
	else
	{
		CyU3PDebugPrint (4, "Binning sum mode configuration done (readback ok).\n");
	}

	return CyTrue;
}
/* JY901 陀螺仪数据流线程入口 */
void slJY901AppThread_Entry(uint32_t input)
{
    CyU3PReturnStatus_t status;
    CyBool_t gyroDmaStarted = CyFalse;
    uint32_t readFailCount = 0;
    uint32_t backoffMs = 20;
	uint32_t initRetryCount = 0;
	uint32_t recoveryInitRetryCount = 0;

    CyU3PDebugPrint(4, "JY901 Thread Started, waiting for I2C init...\r\n");

    /* 等待 I2C 初始化完成事件（带超时重试） */
    for (;;) {
        uint32_t eventFlag = 0;
        status = CyU3PEventGet(&glFxI2cEvent,
                               CY_FX_I2C_INIT_COMPLETE_EVENT,
                               CYU3P_EVENT_OR,
                               &eventFlag,
                               2000);
        if (status == CY_U3P_SUCCESS) break;
        CyU3PDebugPrint(4, "JY901: wait I2C init timeout, retry...\r\n");
        CyU3PThreadSleep(500);
    } 
    CyU3PDebugPrint(4, "JY901: I2C init confirmed, proceeding...\r\n");

	for (;;) {
		status = JY901_Init();
		if (status == CY_U3P_SUCCESS) break;

		initRetryCount++;
		if (initRetryCount >= 3) {
			CyU3PDebugPrint(4, "JY901_Init failed: %d, give up after %d tries\r\n", status, initRetryCount);
			return;
		}

		CyU3PDebugPrint(4, "JY901_Init failed: %d, retry %d/3\r\n", status, initRetryCount);
		CyU3PThreadSleep(1000);
	}

    for (;;) {
        if (gJY901Enabled == CyFalse) {
			readFailCount = 0; backoffMs = 20;
			recoveryInitRetryCount = 0;
            CyU3PThreadSleep(200);
            continue;
        }
        if (!glIsApplnActive) {
            gyroDmaStarted = CyFalse;
            CyU3PThreadSleep(50);
            continue;
        }
        if (gyroDmaStarted == CyFalse) {
            CyU3PDmaChannelReset(&glChHandleGyro);
            CyU3PUsbFlushEp(CY_FX_EP_GYRO_IN);
            if (CyU3PDmaChannelSetXfer(&glChHandleGyro, 0) != CY_U3P_SUCCESS) {
                CyU3PThreadSleep(50); continue;
            }
            gyroDmaStarted = CyTrue;
        }
        status = JY901_ReadAll();
        if (status == CY_U3P_SUCCESS) {
            /* 包布局(36字节, little-endian): 前12字节兼容旧版
             * [0..2]  Roll/Pitch/Yaw (度,  ×ANGLE_SCALE)
             * [3..5]  AX/AY/AZ       (g,   ×ACCEL_SCALE)
             * [6..8]  GX/GY/GZ       (°/s, ×GYRO_SCALE)
             */
            float outFloats[9];
            outFloats[0] = (float)JY901_sReg[JY901_Roll]  * JY901_ANGLE_SCALE;
            outFloats[1] = (float)JY901_sReg[JY901_Pitch] * JY901_ANGLE_SCALE;
            outFloats[2] = (float)JY901_sReg[JY901_Yaw]   * JY901_ANGLE_SCALE;
            outFloats[3] = (float)JY901_sReg[JY901_AX]    * JY901_ACCEL_SCALE;
            outFloats[4] = (float)JY901_sReg[JY901_AY]    * JY901_ACCEL_SCALE;
            outFloats[5] = (float)JY901_sReg[JY901_AZ]    * JY901_ACCEL_SCALE;
            outFloats[6] = (float)JY901_sReg[JY901_GX]    * JY901_GYRO_SCALE;
            outFloats[7] = (float)JY901_sReg[JY901_GY]    * JY901_GYRO_SCALE;
            outFloats[8] = (float)JY901_sReg[JY901_GZ]    * JY901_GYRO_SCALE;
			readFailCount = 0; backoffMs = 20;
			recoveryInitRetryCount = 0;
            CyU3PDmaBuffer_t buf_p;
            if (CyU3PDmaChannelGetBuffer(&glChHandleGyro, &buf_p, 10) == CY_U3P_SUCCESS) {
                CyU3PMemCopy(buf_p.buffer, (uint8_t*)outFloats, 36);
                CyU3PDmaChannelCommitBuffer(&glChHandleGyro, 36, 0);
            } else {
                gyroDmaStarted = CyFalse;
            }
        } else {
            readFailCount++;
            if (readFailCount >= 50) {
                gyroDmaStarted = CyFalse;
				status = JY901_Init();
                readFailCount = 0;
				if (status == CY_U3P_SUCCESS) {
					recoveryInitRetryCount = 0;
					backoffMs = 20;
				} else {
					recoveryInitRetryCount++;
					backoffMs = 500;

					if (recoveryInitRetryCount >= 3) {
						gJY901Enabled = CyFalse;
						CyU3PUsbFlushEp(CY_FX_EP_GYRO_IN);
						CyU3PDmaChannelReset(&glChHandleGyro);
						CyU3PDebugPrint(4, "JY901 re-init failed: %d, disable stream after %d tries\r\n", status, recoveryInitRetryCount);
					} else {
						CyU3PDebugPrint(4, "JY901 re-init failed: %d, retry %d/3\r\n", status, recoveryInitRetryCount);
					}
				}
            } else if (readFailCount >= 5) {
                backoffMs = 100;
            }
        }
        CyU3PThreadSleep(backoffMs);
    }
}

CyU3PThread slTempMonitorThread;
void slTempMonitorThread_Entry(uint32_t input)
{
	for (;;)
	{
		CyU3PThreadSleep(2000); // 彻底隔绝！自己单独睡2秒读一次
		TemperatureMonitor_Process();
	}
}

/* Application define function which creates the threads. */
void CyFxApplicationDefine (void)
{
    void *ptr = NULL;
    uint32_t ret = CY_U3P_SUCCESS;

    /* 创建 I2C 就绪事件 */
    ret = CyU3PEventCreate(&glFxI2cEvent);
    if (ret != 0)
    {
        goto InitFail;
    }

	/* Create GPIO application event group (提前创建，避免线程先运行而事件组尚未就绪) */
	ret = CyU3PEventCreate(&glFxGpioAppEvent);
	if (ret != 0)
	{
		goto InitFail;
	}

	/* Create main application thread */
    ptr = CyU3PMemAlloc (CY_FX_SLFIFO_THREAD_STACK);
    if (ptr != NULL)
    {
        ret = CyU3PThreadCreate (&slFifoAppThread,           /* 线程结构体 */
                              "21:Slave_FIFO_sync",          /* 线程ID和名称 */
                              SlFifoAppThread_Entry,         /* 线程入口函数 */
                              0,                             /* 线程输入参数 */
                              ptr,                           /* 指向分配的线程栈的指针 */
                              CY_FX_SLFIFO_THREAD_STACK,     /* 线程栈大小 */
                              CY_FX_SLFIFO_THREAD_PRIORITY,  /* 线程优先级 */
                              CY_FX_SLFIFO_THREAD_PRIORITY,  /* 线程抢占阈值 */
                              CYU3P_NO_TIME_SLICE,           /* 无时间片 */
                              CYU3P_AUTO_START               /* 立即启动线程 */
                              );
    }
    else
    {
        ret = CY_U3P_ERROR_MEMORY_ERROR;
    }

    /* Check the return code */
    if (ret != 0)
    {
        goto InitFail;
    }

	/* JY901 陀螺仪线程 */
	ptr = CyU3PMemAlloc(CY_FX_MagneticSwitch_SIZE);
	if (ptr != NULL)
	{
		ret = CyU3PThreadCreate(&slJY901AppThread,
					"25:slJY901AppThread_sync",
					slJY901AppThread_Entry,
					0,
					ptr,
					CY_FX_MagneticSwitch_SIZE,
					CY_FX_JY901_THREAD_PRIORITY,
					CY_FX_JY901_THREAD_PRIORITY,
					CYU3P_NO_TIME_SLICE,
					CYU3P_AUTO_START);
	}
	else
	{
		ret = CY_U3P_ERROR_MEMORY_ERROR;
	}

	/* Check the return code */
	if (ret != 0)
	{
		goto InitFail;
	}

	/* 独立温度监控线程 */
	ptr = CyU3PMemAlloc (CY_FX_MagneticSwitch_SIZE);
	if (ptr != NULL)
	{
		ret = CyU3PThreadCreate (&slTempMonitorThread,                  /* Thread structure */
							   "26:slTempMonitorThread",               /* Thread ID and name */
							   slTempMonitorThread_Entry,              /* Thread entry function */
							   0,                                      /* Thread input parameter */
							   ptr,                                   /* Pointer to the allocated thread stack */
							   CY_FX_MagneticSwitch_SIZE,              /* Thread stack size */
							   CY_FX_GPIO_THREAD_PRIORITY,             /* Thread priority */
							   CY_FX_GPIO_THREAD_PRIORITY,             /* Thread preemption threshold */
							   CYU3P_NO_TIME_SLICE,                    /* No time slice */
							   CYU3P_AUTO_START                        /* Start the thread immediately */
							   );
	}
	else
	{
		 ret = CY_U3P_ERROR_MEMORY_ERROR;
	}

	if (ret != 0)
	{
		goto InitFail;
	}

     return;

    InitFail:
    /* As the initialization failed, there is nothing much we can do. Just reset the device
     * so that we go back to the boot-loader. */
    CyU3PDeviceReset (CyFalse);
}




/*
 * Main function
 * 改进方案：将SPI Flash读取操作移到专门的初始化函数中，
 * 在主函数中只配置一次IO矩阵
 */
int main (void)
{
    CyU3PIoMatrixConfig_t io_cfg, io_cfg1;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PSysClockConfig_t clkCfg;

    /* 系统时钟配置 - setSysClk400 clock configurations */
    clkCfg.setSysClk400  = CyTrue;         /* 主时钟 > 400 MHz */
    clkCfg.cpuClkDiv     = 2;              /* CPU clock divider */
    clkCfg.dmaClkDiv     = 2;              /* DMA clock divider */
    clkCfg.mmioClkDiv    = 2;              /* MMIO clock divider */
    clkCfg.useStandbyClk = CyFalse;        /* device has no 32KHz clock supplied */
    clkCfg.clkSrc        = CY_U3P_SYS_CLK; /* Clock source for a peripheral block  */

    /* Initialize the device */
    status = CyU3PDeviceInit(&clkCfg);     /* 初始化设备 */
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* 配置IO矩阵为SPI专用模式 - 用于初始化阶段读取Flash */
    CyU3PMemSet((uint8_t *)&io_cfg1, 0, sizeof(io_cfg1));
    io_cfg1.isDQ32Bit        = CyFalse;
    io_cfg1.useUart          = CyFalse;
    io_cfg1.useI2C           = CyFalse;
    io_cfg1.useI2S           = CyFalse;
    io_cfg1.useSpi           = CyTrue;
    io_cfg1.lppMode          = CY_U3P_IO_MATRIX_LPP_SPI_ONLY;  /* 设为SPI_ONLY模式 */
    io_cfg1.gpioSimpleEn[0]  = 0;                              /* 初始化阶段禁用所有Simple GPIO，仅保留SPI */
    io_cfg1.gpioSimpleEn[1]  = 0;
    io_cfg1.gpioComplexEn[0] = 0;
    io_cfg1.gpioComplexEn[1] = 0;
    
    status = CyU3PDeviceConfigureIOMatrix(&io_cfg1);           /* 配置IO矩阵，禁用所有IO，除了SPI */
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* 清空全局缓冲区 */
    CyU3PMemSet(glEp0Calibration, 0, sizeof(glEp0Calibration));
    CyU3PMemSet(glEp0CaliLccCMC, 0, sizeof(glEp0CaliLccCMC));
    CyU3PMemSet(glEp0FpgaID, 0, sizeof(glEp0FpgaID));

    /* 初始化SPI用于Flash读取 */
    status = CyFxSpiInitCali(0x100);
    if (status != CY_U3P_SUCCESS)
    {
		CyU3PDebugPrint(4, "SPI init failed: %d\r\n", (int)status);
        goto handle_fatal_error;
    }

    /* SPI Flash读取操作 - 添加错误检查 */
    status = CyFxSpiFx3Transfer(0x500, 0x1000, glEp0Calibration, CyTrue);
    if (status != CY_U3P_SUCCESS) 
    {
		CyU3PDebugPrint(4, "Calibration read failed: %d\r\n", (int)status);
    }
    
    status = CyFxSpiFx3Transfer(0x600, 0x200, glEp0CaliLccCMC, CyTrue);
    if (status != CY_U3P_SUCCESS) 
    {
		CyU3PDebugPrint(4, "LCC/CMC read failed: %d\r\n", (int)status);
    }
    
    status = CyFxSpiFx3Transfer(0x4ff, 0x100, glEp0FpgaID, CyTrue);
    if (status != CY_U3P_SUCCESS) 
    {
		CyU3PDebugPrint(4, "FPGA ID read failed: %d\r\n", (int)status);
    }

    /* 反初始化SPI，释放LPP总线资源 */
    status = CyU3PSpiDeInit();
    if (status != CY_U3P_SUCCESS) 
    {
		CyU3PDebugPrint(4, "SPI deinit failed: %d\r\n", (int)status);
    }

    /* 使能指令缓存，禁用数据缓存 */
    status = CyU3PDeviceCacheControl(CyTrue, CyFalse, CyFalse);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

	CyU3PMemSet((uint8_t *)&io_cfg, 0, sizeof(io_cfg));

    /* 重新配置IO矩阵为UART模式 - 用于正常运行阶段 */
    io_cfg.useUart   = CyTrue;
    io_cfg.useI2C    = CyTrue;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyFalse;
    
#if (CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT == 0)
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_UART_ONLY;
#else
    io_cfg.isDQ32Bit = CyTrue;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
#endif

    /* 启用GPIO引脚 */
    io_cfg.gpioSimpleEn[0]  = 0x18800000;   /* GPIO 28/27/23 */
	io_cfg.gpioSimpleEn[1]  = CY_FX_RUNTIME_GPIO_SIMPLE_EN1;   /* GPIO 60/57/52/51/50/45 */
    /*
     * GPIO60: FX3_SNAP          - 快照控制
     * GPIO57: HALL 输入/输出     - 霍尔传感器/按键2
     * GPIO52: 设备复位          - FPGA复位信号
     * GPIO51: FX3_GPIO_HALL     - 磁吸霍尔传感器输入（双沿中断）
     * GPIO50: FX3_SPI_SS_FPGA  - FPGA SPI片选
     * GPIO45: 按钮输入          - 按键1输入
     * GPIO28: FPGA_PROG_CTRL   - FPGA程序加载控制（上电前拉低，初始化就绪后释放）
     * GPIO27: BUTTON           - 主按键（下降沿中断+弱上拉）
     * GPIO23: FPGA_PWR_EN      - FPGA电源使能
     */
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    
    status = CyU3PDeviceConfigureIOMatrix(&io_cfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* 启动FX3内核 - 这是不可返回的调用 */
    CyU3PKernelEntry();
    
    return 0;

handle_fatal_error:
    /* 发生致命错误时的无限循环 */
    while (1);
}



