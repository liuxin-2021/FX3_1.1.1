#include "cyu3system.h"
#include "cyu3error.h"
#include "cyu3utils.h"
#include "cyu3vic.h"
#include "gpio_regs.h"
#include "cyfx_gpio_spi_standalone.h"

#define CYFX_SPI_MIN_BITCOUNT   (1u)
#define CYFX_SPI_MAX_BITCOUNT   (32u)

#define CYFX_SPI_WORKER_STACK_SIZE    (0x0800u)
#define CYFX_SPI_WORKER_PRIORITY      (5u)

#define CYFX_SPI_GPIO_CLK             (22u)
#define CYFX_SPI_GPIO_MOSI            (25u)
#define CYFX_SPI_GPIO_MISO            (26u)
#define CYFX_SPI_GPIO_CS_FPGA         (50u)

#define CYFX_SPI_SIMPLE_GPIO_MAX_ID   (60u)

#define CYFX_SPI_SIMPLE_WRITE(gpioId, level)                        \
    do                                                              \
    {                                                               \
        uvint32_t gpioValue = CY_U3P_LPP_GPIO_SIMPLE ((gpioId));    \
        if (level)                                                  \
        {                                                           \
            gpioValue |= CY_U3P_LPP_GPIO_OUT_VALUE;                 \
        }                                                           \
        else                                                        \
        {                                                           \
            gpioValue &= ~CY_U3P_LPP_GPIO_OUT_VALUE;                \
        }                                                           \
        CY_U3P_LPP_GPIO_SIMPLE ((gpioId)) = gpioValue;              \
    } while (0)

#define CYFX_SPI_SIMPLE_READ(gpioId)                                \
    (((CY_U3P_LPP_GPIO_SIMPLE ((gpioId)) & CY_U3P_LPP_GPIO_IN_VALUE) != 0u) ? CyTrue : CyFalse)

#define CYFX_SPI_DELAY_CYCLES(iterations)                           \
    do                                                              \
    {                                                               \
        uint32_t delayCycles = (iterations);                        \
        while (delayCycles-- != 0u)                                 \
        {                                                           \
            __asm__ volatile ("nop");                              \
        }                                                           \
    } while (0)

#define CYFX_SPI_800KHZ_BITRATE_HZ      (800000u)
#define CYFX_SPI_800KHZ_DELAY_CYCLES    (10u)
#define CYFX_SPI_800KHZ_HIGH_DELAY_CYCLES  (6u)
#define CYFX_SPI_800KHZ_LOW_DELAY_CYCLES   (4u)

static CyFxGpioSpiWorker_t gSpiWorker;
static uint8_t gSpiWorkerStack[CYFX_SPI_WORKER_STACK_SIZE] __attribute__ ((aligned (32)));
static CyFxSpiSpeedProfile_t gSpiSpeedProfile = CYFX_SPI_SPEED_PROFILE_800KHZ;
static CyBool_t gSpiUseShortCriticalSection = CyFalse;
static CyU3PMutex gSpiProtoMutex;
static CyBool_t gSpiProtoMutexReady = CyFalse;
static CyFxSpiDiagStats_t gSpiDiagStats;

static void
CyFxSpiDiagRecordStatus (CyU3PReturnStatus_t status)
{
    if (status != CY_U3P_SUCCESS)
    {
        gSpiDiagStats.lastError = (uint32_t)status;

        if (status == CY_U3P_ERROR_TIMEOUT)
        {
            gSpiDiagStats.timeoutCount++;
        }
    }
}

static CyU3PReturnStatus_t
CyFxSpiDiagTrackedRead8 (uint8_t chip_id, uint8_t address, uint8_t *value)
{
    CyU3PReturnStatus_t status;

    if (value == 0)
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    gSpiDiagStats.transferCount++;
    gSpiDiagStats.readReqCount++;

    if (gSpiProtoMutexReady)
    {
        status = CyU3PMutexGet (&gSpiProtoMutex, CYU3P_WAIT_FOREVER);
        if (status != CY_U3P_SUCCESS)
        {
            gSpiDiagStats.mutexGetFailCount++;
            gSpiDiagStats.readFailCount++;
            CyFxSpiDiagRecordStatus (status);
            return status;
        }
    }

    status = CyFxSpiStandaloneFpgaRead8 (chip_id, address, value);

    if (status == CY_U3P_SUCCESS)
    {
        gSpiDiagStats.readOkCount++;
        gSpiDiagStats.lastReadValue = *value;
    }
    else
    {
        gSpiDiagStats.readFailCount++;
    }
    CyFxSpiDiagRecordStatus (status);

    if (gSpiProtoMutexReady)
    {
        CyU3PMutexPut (&gSpiProtoMutex);
    }

    return status;
}

CyU3PReturnStatus_t
CyFxSpiProtoReadSelectedStatus8 (uint8_t selectValue, uint8_t *value)
{
    CyU3PReturnStatus_t status;

    if (value == 0)
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    gSpiDiagStats.transferCount += 2u;
    gSpiDiagStats.writeReqCount++;
    gSpiDiagStats.readReqCount++;

    if (gSpiProtoMutexReady)
    {
        status = CyU3PMutexGet (&gSpiProtoMutex, CYU3P_WAIT_FOREVER);
        if (status != CY_U3P_SUCCESS)
        {
            gSpiDiagStats.mutexGetFailCount++;
            gSpiDiagStats.writeFailCount++;
            gSpiDiagStats.readFailCount++;
            CyFxSpiDiagRecordStatus (status);
            return status;
        }
    }

    status = CyFxSpiStandaloneFpgaWrite24 (0x02u, 0xffu, selectValue);
    if (status == CY_U3P_SUCCESS)
    {
        gSpiDiagStats.writeOkCount++;
    }
    else
    {
        gSpiDiagStats.writeFailCount++;
        CyFxSpiDiagRecordStatus (status);
        if (gSpiProtoMutexReady)
        {
            CyU3PMutexPut (&gSpiProtoMutex);
        }
        return status;
    }

    status = CyFxSpiStandaloneFpgaRead8 (0x00u, 0x00u, value);
    if (status == CY_U3P_SUCCESS)
    {
        gSpiDiagStats.readOkCount++;
        gSpiDiagStats.lastReadValue = *value;
    }
    else
    {
        gSpiDiagStats.readFailCount++;
    }
    CyFxSpiDiagRecordStatus (status);

    if (gSpiProtoMutexReady)
    {
        CyU3PMutexPut (&gSpiProtoMutex);
    }

    return status;
}

static uint32_t
CyFxGpioSpiGetDelayCyclesForLevel (const CyFxGpioSpiContext_t *ctx,
                                   CyBool_t level)
{
    if (level == CyTrue)
    {
        if (ctx->timing.highDelayCycles != 0u)
        {
            return ctx->timing.highDelayCycles;
        }
    }
    else
    {
        if (ctx->timing.lowDelayCycles != 0u)
        {
            return ctx->timing.lowDelayCycles;
        }
    }

    return ctx->timing.edgeDelayCycles;
}

static void
CyFxGpioSpiDelayClockLevel (const CyFxGpioSpiContext_t *ctx,
                            CyBool_t level)
{
    if (ctx->timing.halfPeriodUs != 0u)
    {
        CyU3PBusyWait (ctx->timing.halfPeriodUs);
        return;
    }

    CYFX_SPI_DELAY_CYCLES (CyFxGpioSpiGetDelayCyclesForLevel (ctx, level));
}

static CyU3PReturnStatus_t
CyFxGpioSpiSetClock (const CyFxGpioSpiContext_t *ctx, CyBool_t level)
{
    if (ctx == 0)
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    if (ctx->pinMap.clk > CYFX_SPI_SIMPLE_GPIO_MAX_ID)
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    CYFX_SPI_SIMPLE_WRITE (ctx->pinMap.clk, level);
    return CY_U3P_SUCCESS;
}

static CyU3PReturnStatus_t
CyFxGpioSpiSetMosi (const CyFxGpioSpiContext_t *ctx, CyBool_t level)
{
    if (ctx == 0)
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    if (ctx->pinMap.mosi > CYFX_SPI_SIMPLE_GPIO_MAX_ID)
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    CYFX_SPI_SIMPLE_WRITE (ctx->pinMap.mosi, level);
    return CY_U3P_SUCCESS;
}

static CyU3PReturnStatus_t
CyFxGpioSpiSetCs (const CyFxGpioSpiContext_t *ctx, CyBool_t level)
{
    if (ctx == 0)
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    if (ctx->pinMap.cs > CYFX_SPI_SIMPLE_GPIO_MAX_ID)
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    CYFX_SPI_SIMPLE_WRITE (ctx->pinMap.cs, level);
    return CY_U3P_SUCCESS;
}

static uint8_t
CyFxGpioSpiBitIndex (const CyFxGpioSpiTiming_t *timing, uint8_t bitCount, uint8_t i)
{
    if (timing->bitOrder == CYFX_SPI_BIT_ORDER_MSB_FIRST)
    {
        return (uint8_t)((bitCount - 1u) - i);
    }

    return i;
}

static CyBool_t
CyFxGpioSpiCanUseFast24LsbFirstPath (const CyFxGpioSpiContext_t *ctx,
                                     const CyFxGpioSpiFrame_t *frame)
{
    if ((ctx == 0) || (frame == 0))
    {
        return CyFalse;
    }

    if ((frame->bitCount != 24u) ||
            (ctx->timing.cpolIdleHigh != CyFalse) ||
            (ctx->timing.cphaSecondEdge != CyFalse) ||
            (ctx->timing.bitOrder != CYFX_SPI_BIT_ORDER_LSB_FIRST) ||
            (ctx->timing.halfPeriodUs != 0u))
    {
        return CyFalse;
    }

    if ((ctx->pinMap.clk > CYFX_SPI_SIMPLE_GPIO_MAX_ID) ||
            (ctx->pinMap.mosi > CYFX_SPI_SIMPLE_GPIO_MAX_ID) ||
            (ctx->pinMap.miso > CYFX_SPI_SIMPLE_GPIO_MAX_ID))
    {
        return CyFalse;
    }

    return CyTrue;
}

static CyU3PReturnStatus_t
CyFxGpioSpiTransferFast24LsbFirst (const CyFxGpioSpiContext_t *ctx,
                                   const CyFxGpioSpiFrame_t *frame,
                                   uint32_t *rxData)
{
    uint32_t txData = (frame->txData & 0x00FFFFFFu);
    uint32_t rxValue = 0u;
    uint32_t rxMask = 1u;
    uint32_t highDelayCycles = CyFxGpioSpiGetDelayCyclesForLevel (ctx, CyTrue);
    uint32_t lowDelayCycles = CyFxGpioSpiGetDelayCyclesForLevel (ctx, CyFalse);
    uint32_t setupDelayCycles;
    uint32_t lowGapDelayCycles;
    uvint32_t clkLow;
    uvint32_t clkHigh;
    uvint32_t mosiLow;
    uvint32_t mosiHigh;
    uint8_t i;

    clkLow = CY_U3P_LPP_GPIO_SIMPLE (ctx->pinMap.clk) & ~CY_U3P_LPP_GPIO_OUT_VALUE;
    clkHigh = clkLow | CY_U3P_LPP_GPIO_OUT_VALUE;
    mosiLow = CY_U3P_LPP_GPIO_SIMPLE (ctx->pinMap.mosi) & ~CY_U3P_LPP_GPIO_OUT_VALUE;
    mosiHigh = mosiLow | CY_U3P_LPP_GPIO_OUT_VALUE;
    setupDelayCycles = (lowDelayCycles > 2u) ? 2u : lowDelayCycles;
    lowGapDelayCycles = lowDelayCycles - setupDelayCycles;

    if (frame->rxData == 0)
    {
        for (i = 0u; i < 24u; ++i)
        {
            CYFX_SPI_DELAY_CYCLES (lowGapDelayCycles);
            CY_U3P_LPP_GPIO_SIMPLE (ctx->pinMap.mosi) = ((txData & 0x1u) != 0u) ? mosiHigh : mosiLow;
            CYFX_SPI_DELAY_CYCLES (setupDelayCycles);
            CY_U3P_LPP_GPIO_SIMPLE (ctx->pinMap.clk) = clkHigh;
            CYFX_SPI_DELAY_CYCLES (highDelayCycles);
            CY_U3P_LPP_GPIO_SIMPLE (ctx->pinMap.clk) = clkLow;
            txData >>= 1;
        }

        return CY_U3P_SUCCESS;
    }

    for (i = 0u; i < 24u; ++i)
    {
        CYFX_SPI_DELAY_CYCLES (lowGapDelayCycles);
        CY_U3P_LPP_GPIO_SIMPLE (ctx->pinMap.mosi) = ((txData & 0x1u) != 0u) ? mosiHigh : mosiLow;
        CYFX_SPI_DELAY_CYCLES (setupDelayCycles);
        CY_U3P_LPP_GPIO_SIMPLE (ctx->pinMap.clk) = clkHigh;
        CYFX_SPI_DELAY_CYCLES (highDelayCycles);
        CY_U3P_LPP_GPIO_SIMPLE (ctx->pinMap.clk) = clkLow;

        if (CYFX_SPI_SIMPLE_READ (ctx->pinMap.miso) == CyTrue)
        {
            rxValue |= rxMask;
        }

        txData >>= 1;
        rxMask <<= 1;
    }

    *rxData = rxValue;
    return CY_U3P_SUCCESS;
}

static CyU3PReturnStatus_t
CyFxGpioSpiOverridePins (const CyFxGpioSpiPinMap_t *pinMap)
{
    CyU3PReturnStatus_t status;

    status = CyU3PDeviceGpioOverride (pinMap->clk, CyTrue);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    status = CyU3PDeviceGpioOverride (pinMap->miso, CyTrue);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    status = CyU3PDeviceGpioOverride (pinMap->mosi, CyTrue);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    status = CyU3PDeviceGpioOverride (pinMap->cs, CyTrue);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    return CY_U3P_SUCCESS;
}

CyU3PReturnStatus_t
CyFxGpioSpiConfigurePins (const CyFxGpioSpiPinMap_t *pinMap)
{
    CyU3PGpioSimpleConfig_t cfg;
    CyU3PReturnStatus_t status;

    if (pinMap == 0)
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    CyU3PMemSet ((uint8_t *)&cfg, 0, sizeof(cfg));

    cfg.outValue = CyFalse;
    cfg.driveLowEn = CyTrue;
    cfg.driveHighEn = CyTrue;
    cfg.inputEn = CyFalse;
    cfg.intrMode = CY_U3P_GPIO_NO_INTR;
    status = CyU3PGpioSetSimpleConfig (pinMap->clk, &cfg);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    status = CyU3PGpioSetSimpleConfig (pinMap->mosi, &cfg);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    cfg.outValue = CyTrue;
    status = CyU3PGpioSetSimpleConfig (pinMap->cs, &cfg);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    cfg.outValue = CyFalse;
    cfg.driveLowEn = CyFalse;
    cfg.driveHighEn = CyFalse;
    cfg.inputEn = CyTrue;
    status = CyU3PGpioSetSimpleConfig (pinMap->miso, &cfg);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    status = CyU3PGpioSetIoMode (pinMap->miso, CY_U3P_GPIO_IO_MODE_WPD);

    return status;
}

CyU3PReturnStatus_t
CyFxGpioSpiInit (CyFxGpioSpiContext_t *ctx,
                 const CyFxGpioSpiPinMap_t *pinMap,
                 const CyFxGpioSpiTiming_t *timing)
{
    CyU3PReturnStatus_t status;

    if ((ctx == 0) || (pinMap == 0) || (timing == 0))
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    CyU3PMemSet ((uint8_t *)ctx, 0, sizeof(*ctx));
    ctx->pinMap = *pinMap;
    ctx->timing = *timing;

    status = CyFxGpioSpiOverridePins (pinMap);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    status = CyU3PMutexCreate (&ctx->lock, CyFalse);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    status = CyFxGpioSpiConfigurePins (pinMap);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PMutexDestroy (&ctx->lock);
        return status;
    }

    status = CyFxGpioSpiSetClock (ctx, timing->cpolIdleHigh);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PMutexDestroy (&ctx->lock);
        return status;
    }

    status = CyFxGpioSpiSetCs (ctx, CyTrue);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PMutexDestroy (&ctx->lock);
        return status;
    }

    ctx->initialized = CyTrue;
    return CY_U3P_SUCCESS;
}

void
CyFxGpioSpiDeInit (CyFxGpioSpiContext_t *ctx)
{
    if ((ctx == 0) || (ctx->initialized == CyFalse))
    {
        return;
    }

    CyU3PGpioSetValue (ctx->pinMap.cs, CyTrue);
    CyU3PGpioSetValue (ctx->pinMap.clk, ctx->timing.cpolIdleHigh);
    CyU3PMutexDestroy (&ctx->lock);
    ctx->initialized = CyFalse;
}

CyU3PReturnStatus_t
CyFxGpioSpiSetBitRateHint (CyFxGpioSpiContext_t *ctx,
                           uint32_t bitRateHz,
                           uint32_t subUsDelayCycles)
{
    if ((ctx == 0) || (ctx->initialized == CyFalse) || (bitRateHz == 0u))
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    if (bitRateHz <= 500000u)
    {
        ctx->timing.halfPeriodUs = (500000u + bitRateHz - 1u) / bitRateHz;
        ctx->timing.edgeDelayCycles = 0u;
        ctx->timing.highDelayCycles = 0u;
        ctx->timing.lowDelayCycles = 0u;
    }
    else
    {
        ctx->timing.halfPeriodUs = 0u;
        ctx->timing.edgeDelayCycles = subUsDelayCycles;
        ctx->timing.highDelayCycles = subUsDelayCycles;
        ctx->timing.lowDelayCycles = subUsDelayCycles;
    }

    return CY_U3P_SUCCESS;
}

CyU3PReturnStatus_t
CyFxGpioSpiTransferFrame (CyFxGpioSpiContext_t *ctx,
                          const CyFxGpioSpiFrame_t *frame,
                          uint32_t waitOption)
{
    CyU3PReturnStatus_t status;
    CyBool_t csAsserted = CyFalse;
    CyBool_t useShortCriticalSection;
    CyBool_t vicMasked = CyFalse;
    uint32_t vicMask = 0;
    uint32_t rx = 0u;
    uint8_t i;
    CyBool_t miso = CyFalse;
    CyBool_t idleLevel;
    CyBool_t activeLevel;

    if ((ctx == 0) || (frame == 0) || (ctx->initialized == CyFalse))
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    if ((frame->bitCount < CYFX_SPI_MIN_BITCOUNT) || (frame->bitCount > CYFX_SPI_MAX_BITCOUNT))
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    status = CyU3PMutexGet (&ctx->lock, waitOption);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    useShortCriticalSection = (ctx->timing.allowShortCriticalSection && frame->useShortCriticalSection);

    idleLevel = ctx->timing.cpolIdleHigh;
    activeLevel = (idleLevel == CyTrue) ? CyFalse : CyTrue;

    status = CyFxGpioSpiSetCs (ctx, CyFalse);
    if (status != CY_U3P_SUCCESS)
    {
        goto spi_transfer_cleanup;
    }
    csAsserted = CyTrue;

    status = CyFxGpioSpiSetClock (ctx, idleLevel);
    if (status != CY_U3P_SUCCESS)
    {
        goto spi_transfer_cleanup;
    }

    if (useShortCriticalSection)
    {
        vicMask = CyU3PVicDisableAllInterrupts ();
        vicMasked = CyTrue;
    }

    if (CyFxGpioSpiCanUseFast24LsbFirstPath (ctx, frame))
    {
        status = CyFxGpioSpiTransferFast24LsbFirst (ctx, frame, &rx);
        goto spi_transfer_cleanup;
    }

    for (i = 0u; i < frame->bitCount; ++i)
    {
        uint8_t bitIndex = CyFxGpioSpiBitIndex (&ctx->timing, frame->bitCount, i);
        CyBool_t mosi = ((frame->txData >> bitIndex) & 0x1u) ? CyTrue : CyFalse;

        if (ctx->timing.cphaSecondEdge == CyFalse)
        {
            status = CyFxGpioSpiSetMosi (ctx, mosi);
            if (status != CY_U3P_SUCCESS)
            {
                break;
            }

            CyFxGpioSpiDelayClockLevel (ctx, idleLevel);

            status = CyFxGpioSpiSetClock (ctx, activeLevel);
            if (status != CY_U3P_SUCCESS)
            {
                break;
            }

            CyFxGpioSpiDelayClockLevel (ctx, activeLevel);

            status = CyFxGpioSpiSetClock (ctx, idleLevel);
            if (status != CY_U3P_SUCCESS)
            {
                break;
            }
            if (ctx->pinMap.miso > CYFX_SPI_SIMPLE_GPIO_MAX_ID)
            {
                status = CY_U3P_ERROR_BAD_ARGUMENT;
                break;
            }

            miso = CYFX_SPI_SIMPLE_READ (ctx->pinMap.miso);

            if (miso == CyTrue)
            {
                rx |= (1u << bitIndex);
            }
        }
        else
        {
            status = CyFxGpioSpiSetClock (ctx, activeLevel);
            if (status != CY_U3P_SUCCESS)
            {
                break;
            }

            status = CyFxGpioSpiSetMosi (ctx, mosi);
            if (status != CY_U3P_SUCCESS)
            {
                break;
            }

            CyFxGpioSpiDelayClockLevel (ctx, activeLevel);

            status = CyFxGpioSpiSetClock (ctx, idleLevel);
            if (status != CY_U3P_SUCCESS)
            {
                break;
            }

            CyFxGpioSpiDelayClockLevel (ctx, idleLevel);

            if (ctx->pinMap.miso > CYFX_SPI_SIMPLE_GPIO_MAX_ID)
            {
                status = CY_U3P_ERROR_BAD_ARGUMENT;
                break;
            }

            miso = CYFX_SPI_SIMPLE_READ (ctx->pinMap.miso);

            if (miso == CyTrue)
            {
                rx |= (1u << bitIndex);
            }
        }
    }

spi_transfer_cleanup:
    if ((frame->keepChipSelect == CyFalse) && csAsserted)
    {
        CyFxGpioSpiSetCs (ctx, CyTrue);
        if (status == CY_U3P_SUCCESS)
        {
            status = CyFxGpioSpiSetMosi (ctx, CyFalse);
        }
        else
        {
            CyFxGpioSpiSetMosi (ctx, CyFalse);
        }
    }

    if (vicMasked)
    {
        CyU3PVicEnableInterrupts (vicMask);
    }

    if (frame->rxData != 0)
    {
        *frame->rxData = rx;
    }

    CyU3PMutexPut (&ctx->lock);
    return status;
}

static void
CyFxGpioSpiWorkerEntry (uint32_t input)
{
    CyFxGpioSpiWorker_t *worker = (CyFxGpioSpiWorker_t *)input;
    uint32_t eventFlags;
    CyU3PReturnStatus_t status;

    while (worker->running == CyTrue)
    {
        status = CyU3PEventGet (&worker->requestEvent,
                                CYFX_SPI_WORKER_REQ_EVENT,
                                CYU3P_EVENT_OR_CLEAR,
                                &eventFlags,
                                CYU3P_WAIT_FOREVER);
        if (status != CY_U3P_SUCCESS)
        {
            continue;
        }

        if (worker->running == CyFalse)
        {
            break;
        }

        CyU3PMutexGet (&worker->requestLock, CYU3P_WAIT_FOREVER);
        worker->requestStatus = CyFxGpioSpiTransferFrame (&worker->spi,
                                                          &worker->requestFrame,
                                                          CYU3P_WAIT_FOREVER);
        CyU3PMutexPut (&worker->requestLock);

        CyU3PEventSet (&worker->doneEvent, CYFX_SPI_WORKER_DONE_EVENT, CYU3P_EVENT_OR);
    }
}

CyU3PReturnStatus_t
CyFxGpioSpiWorkerCreate (CyFxGpioSpiWorker_t *worker,
                         const CyFxGpioSpiPinMap_t *pinMap,
                         const CyFxGpioSpiTiming_t *timing,
                         void *threadStack,
                         uint32_t threadStackSize,
                         uint32_t threadPriority,
                         uint32_t startImmediately)
{
    CyU3PReturnStatus_t status;

    if ((worker == 0) || (pinMap == 0) || (timing == 0) || (threadStack == 0))
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    CyU3PMemSet ((uint8_t *)worker, 0, sizeof(*worker));

    status = CyFxGpioSpiInit (&worker->spi, pinMap, timing);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    status = CyU3PEventCreate (&worker->requestEvent);
    if (status != CY_U3P_SUCCESS)
    {
        CyFxGpioSpiDeInit (&worker->spi);
        return status;
    }

    status = CyU3PEventCreate (&worker->doneEvent);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PEventDestroy (&worker->requestEvent);
        CyFxGpioSpiDeInit (&worker->spi);
        return status;
    }

    status = CyU3PMutexCreate (&worker->requestLock, CyFalse);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PEventDestroy (&worker->requestEvent);
        CyU3PEventDestroy (&worker->doneEvent);
        CyFxGpioSpiDeInit (&worker->spi);
        return status;
    }

    worker->running = CyTrue;
    status = CyU3PThreadCreate (&worker->thread,
                                "SPI Worker",
                                CyFxGpioSpiWorkerEntry,
                                (uint32_t)worker,
                                threadStack,
                                threadStackSize,
                                threadPriority,
                                threadPriority,
                                CYU3P_NO_TIME_SLICE,
                                startImmediately);
    if (status != CY_U3P_SUCCESS)
    {
        worker->running = CyFalse;
        CyU3PMutexDestroy (&worker->requestLock);
        CyU3PEventDestroy (&worker->requestEvent);
        CyU3PEventDestroy (&worker->doneEvent);
        CyFxGpioSpiDeInit (&worker->spi);
        return status;
    }

    return CY_U3P_SUCCESS;
}

void
CyFxGpioSpiWorkerDestroy (CyFxGpioSpiWorker_t *worker)
{
    if (worker == 0)
    {
        return;
    }

    worker->running = CyFalse;
    CyU3PEventSet (&worker->requestEvent, CYFX_SPI_WORKER_REQ_EVENT, CYU3P_EVENT_OR);
    CyU3PThreadSleep (2);

    CyU3PMutexDestroy (&worker->requestLock);
    CyU3PEventDestroy (&worker->requestEvent);
    CyU3PEventDestroy (&worker->doneEvent);
    CyFxGpioSpiDeInit (&worker->spi);
}

CyU3PReturnStatus_t
CyFxGpioSpiWorkerTransferFrame (CyFxGpioSpiWorker_t *worker,
                                const CyFxGpioSpiFrame_t *frame,
                                uint32_t waitOption)
{
    CyU3PReturnStatus_t status;
    uint32_t eventFlags;

    if ((worker == 0) || (frame == 0) || (worker->running == CyFalse))
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    status = CyU3PMutexGet (&worker->requestLock, waitOption);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    worker->requestFrame = *frame;
    worker->requestStatus = CY_U3P_ERROR_TIMEOUT;

    CyU3PEventSet (&worker->requestEvent, CYFX_SPI_WORKER_REQ_EVENT, CYU3P_EVENT_OR);
    CyU3PMutexPut (&worker->requestLock);

    status = CyU3PEventGet (&worker->doneEvent,
                            CYFX_SPI_WORKER_DONE_EVENT,
                            CYU3P_EVENT_OR_CLEAR,
                            &eventFlags,
                            waitOption);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    return worker->requestStatus;
}

CyU3PReturnStatus_t
CyFxGpioSpiWorkerTransfer24 (CyFxGpioSpiWorker_t *worker,
                             uint32_t tx24,
                             uint32_t *rx24,
                             CyBool_t irqGuard,
                             uint32_t waitOption)
{
    CyFxGpioSpiFrame_t frame;

    frame.txData = (tx24 & 0x00FFFFFFu);
    frame.rxData = rx24;
    frame.bitCount = 24u;
    frame.keepChipSelect = CyFalse;
    frame.useShortCriticalSection = irqGuard;

    return CyFxGpioSpiWorkerTransferFrame (worker, &frame, waitOption);
}

CyU3PReturnStatus_t
CyFxSpiStandaloneSetSpeedProfile (CyFxSpiSpeedProfile_t profile)
{
    CyU3PReturnStatus_t status;
    uint32_t bitRateHz;
    uint32_t edgeDelayCycles;
    uint32_t highDelayCycles;
    uint32_t lowDelayCycles;

    switch (profile)
    {
        case CYFX_SPI_SPEED_PROFILE_800KHZ:
            bitRateHz = CYFX_SPI_800KHZ_BITRATE_HZ;
            edgeDelayCycles = CYFX_SPI_800KHZ_DELAY_CYCLES;
            highDelayCycles = CYFX_SPI_800KHZ_HIGH_DELAY_CYCLES;
            lowDelayCycles = CYFX_SPI_800KHZ_LOW_DELAY_CYCLES;
            break;

        default:
            return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    status = CyFxGpioSpiSetBitRateHint (&gSpiWorker.spi, bitRateHz, edgeDelayCycles);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    gSpiWorker.spi.timing.highDelayCycles = highDelayCycles;
    gSpiWorker.spi.timing.lowDelayCycles = lowDelayCycles;

    gSpiSpeedProfile = profile;
    return CY_U3P_SUCCESS;
}

CyFxSpiSpeedProfile_t
CyFxSpiStandaloneGetSpeedProfile (void)
{
    return gSpiSpeedProfile;
}

void
CyFxSpiStandaloneSetShortCriticalSection (CyBool_t enable)
{
    gSpiUseShortCriticalSection = enable;
}

CyBool_t
CyFxSpiStandaloneIsShortCriticalSectionEnabled (void)
{
    return gSpiUseShortCriticalSection;
}

void
CyFxSpiDiagResetStats (void)
{
    CyU3PMemSet ((uint8_t *)&gSpiDiagStats, 0, sizeof(gSpiDiagStats));
}

CyU3PReturnStatus_t
CyFxSpiDiagGetStats (CyFxSpiDiagStats_t *stats)
{
    CyU3PReturnStatus_t status;

    if (stats == 0)
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    if (gSpiProtoMutexReady)
    {
        status = CyU3PMutexGet (&gSpiProtoMutex, CYU3P_WAIT_FOREVER);
        if (status != CY_U3P_SUCCESS)
        {
            return status;
        }
    }

    *stats = gSpiDiagStats;

    if (gSpiProtoMutexReady)
    {
        CyU3PMutexPut (&gSpiProtoMutex);
    }

    return CY_U3P_SUCCESS;
}

CyU3PReturnStatus_t
CyFxSpiDiagRunSmokeReadCheck (uint8_t chip_id,
                              uint8_t address,
                              uint8_t expectedValue,
                              uint32_t rounds,
                              uint32_t *mismatchCount)
{
    CyU3PReturnStatus_t status;
    uint32_t mismatch = 0u;
    uint8_t value = 0u;
    uint32_t i;

    if ((rounds == 0u) || (mismatchCount == 0))
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    gSpiDiagStats.expectedValue = expectedValue;

    for (i = 0u; i < rounds; ++i)
    {
        status = CyFxSpiDiagTrackedRead8 (chip_id, address, &value);
        if (status != CY_U3P_SUCCESS)
        {
            *mismatchCount = mismatch;
            return status;
        }

        if (value != expectedValue)
        {
            mismatch++;
            gSpiDiagStats.protoCheckFailCount++;
            gSpiDiagStats.lastError = (uint32_t)CY_U3P_ERROR_FAILURE;
            if (gSpiDiagStats.firstMismatchCaptured == 0u)
            {
                gSpiDiagStats.firstMismatchValue = value;
                gSpiDiagStats.firstMismatchCaptured = 1u;
            }
        }
    }

    *mismatchCount = mismatch;

    if (mismatch != 0u)
    {
        return CY_U3P_ERROR_FAILURE;
    }

    return CY_U3P_SUCCESS;
}

CyU3PReturnStatus_t
CyFxSpiDiagRunStabilityReadCheck800kHz (uint8_t chip_id,
                                        uint8_t address,
                                        uint8_t expectedValue,
                                        uint32_t rounds,
                                        uint32_t *mismatchCount)
{
    CyU3PReturnStatus_t status;

    if ((rounds == 0u) || (mismatchCount == 0))
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    status = CyFxSpiStandaloneSetSpeedProfile (CYFX_SPI_SPEED_PROFILE_800KHZ);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    return CyFxSpiDiagRunSmokeReadCheck (chip_id,
                                         address,
                                         expectedValue,
                                         rounds,
                                         mismatchCount);
}

CyU3PReturnStatus_t
CyFxSpiDiagRunStabilityReadCheck800kHz100k (uint8_t chip_id,
                                            uint8_t address,
                                            uint8_t expectedValue,
                                            uint32_t *mismatchCount)
{
    return CyFxSpiDiagRunStabilityReadCheck800kHz (chip_id,
                                                   address,
                                                   expectedValue,
                                                   CYFX_SPI_DIAG_STABILITY_DEFAULT_ROUNDS,
                                                   mismatchCount);
}

CyU3PReturnStatus_t
CyFxSpiProtoWrite8 (uint8_t chip_id, uint8_t address, uint8_t data)
{
    CyU3PReturnStatus_t status;

    gSpiDiagStats.transferCount++;
    gSpiDiagStats.writeReqCount++;

    if (gSpiProtoMutexReady)
    {
        status = CyU3PMutexGet (&gSpiProtoMutex, CYU3P_WAIT_FOREVER);
        if (status != CY_U3P_SUCCESS)
        {
            gSpiDiagStats.mutexGetFailCount++;
            gSpiDiagStats.writeFailCount++;
            CyFxSpiDiagRecordStatus (status);
            return status;
        }
    }

    status = CyFxSpiStandaloneFpgaWrite24 (chip_id, address, data);

    if (status == CY_U3P_SUCCESS)
    {
        gSpiDiagStats.writeOkCount++;
    }
    else
    {
        gSpiDiagStats.writeFailCount++;
    }
    CyFxSpiDiagRecordStatus (status);

    if (gSpiProtoMutexReady)
    {
        CyU3PMutexPut (&gSpiProtoMutex);
    }

    return status;
}

uint8_t
CyFxSpiProtoRead8 (uint8_t chip_id, uint8_t address)
{
    CyU3PReturnStatus_t status;
    uint8_t value = 0;

    status = CyFxSpiDiagTrackedRead8 (chip_id, address, &value);

    if (status != CY_U3P_SUCCESS)
    {
        return (uint8_t)status;
    }

    return value;
}

CyU3PReturnStatus_t
CyFxSpiStandaloneStart (void)
{
    CyFxGpioSpiPinMap_t pinMap;
    CyFxGpioSpiTiming_t timing;
    CyU3PReturnStatus_t status;

    pinMap.clk = CYFX_SPI_GPIO_CLK;
    pinMap.mosi = CYFX_SPI_GPIO_MOSI;
    pinMap.miso = CYFX_SPI_GPIO_MISO;
    pinMap.cs = CYFX_SPI_GPIO_CS_FPGA;

    timing.cpolIdleHigh = CyFalse;
    timing.cphaSecondEdge = CyFalse;
    timing.bitOrder = CYFX_SPI_BIT_ORDER_LSB_FIRST;
    timing.halfPeriodUs = 0u;
    timing.edgeDelayCycles = 20u;
    timing.highDelayCycles = CYFX_SPI_800KHZ_HIGH_DELAY_CYCLES;
    timing.lowDelayCycles = CYFX_SPI_800KHZ_LOW_DELAY_CYCLES;
    timing.allowShortCriticalSection = CyTrue;

    status = CyFxGpioSpiWorkerCreate (&gSpiWorker,
                                      &pinMap,
                                      &timing,
                                      gSpiWorkerStack,
                                      sizeof(gSpiWorkerStack),
                                      CYFX_SPI_WORKER_PRIORITY,
                                      CYU3P_AUTO_START);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    status = CyFxSpiStandaloneSetSpeedProfile (CYFX_SPI_SPEED_PROFILE_800KHZ);
    if (status != CY_U3P_SUCCESS)
    {
        CyFxSpiStandaloneStop ();
        return status;
    }

    status = CyU3PMutexCreate (&gSpiProtoMutex, CYU3P_NO_INHERIT);
    if (status != CY_U3P_SUCCESS)
    {
        CyFxSpiStandaloneStop ();
        return status;
    }
    gSpiProtoMutexReady = CyTrue;
    CyFxSpiDiagResetStats ();

    return CY_U3P_SUCCESS;
}

void
CyFxSpiStandaloneStop (void)
{
    if (gSpiProtoMutexReady)
    {
        CyU3PMutexDestroy (&gSpiProtoMutex);
        gSpiProtoMutexReady = CyFalse;
    }

    gSpiSpeedProfile = CYFX_SPI_SPEED_PROFILE_800KHZ;
    gSpiUseShortCriticalSection = CyFalse;
    CyFxGpioSpiWorkerDestroy (&gSpiWorker);
}

CyU3PReturnStatus_t
CyFxSpiStandaloneFpgaWrite24 (uint8_t chipId, uint8_t address, uint8_t value)
{
    uint32_t tx24;

    /* FPGA decodes sdi_r[23:16]=data, sdi_r[15:8]=addr, sdi_r[7:0]=chipId. */
    tx24 = (((uint32_t)value) << 16) |
           (((uint32_t)address) << 8) |
           ((uint32_t)chipId);

    return CyFxGpioSpiWorkerTransfer24 (&gSpiWorker,
                                        tx24,
                                        0,
                                        gSpiUseShortCriticalSection,
                                        CYU3P_WAIT_FOREVER);
}

CyU3PReturnStatus_t
CyFxSpiStandaloneFpgaRead8 (uint8_t chipId, uint8_t address, uint8_t *value)
{
    CyU3PReturnStatus_t status;
    uint32_t tx24;
    uint32_t rx24 = 0u;

    if (value == 0)
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    /* Read phase only needs SPI clocks on this FPGA, keep addr/chipId for compatibility. */
    tx24 = (((uint32_t)0u) << 16) |
           (((uint32_t)address) << 8) |
           ((uint32_t)chipId);

    status = CyFxGpioSpiWorkerTransfer24 (&gSpiWorker,
                                          tx24,
                                          &rx24,
                                          gSpiUseShortCriticalSection,
                                          CYU3P_WAIT_FOREVER);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    *value = (uint8_t)(rx24 & 0xFFu);
    return CY_U3P_SUCCESS;
}
