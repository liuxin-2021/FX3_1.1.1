#include "cyu3error.h"
#include "cyu3utils.h"
#include "cyfxslfifosync.h"
#include "cyfx_gpio_spi_standalone.h"

#define CYFX_SPI_WORKER_STACK_SIZE    (0x0800u)
#define CYFX_SPI_WORKER_PRIORITY      (5u)

#define CYFX_SPI_GPIO_CLK             ((uint8_t)FX3_SPI_CLK)
#define CYFX_SPI_GPIO_MOSI            ((uint8_t)FX3_SPI_MOSI)
#define CYFX_SPI_GPIO_MISO            ((uint8_t)FX3_SPI_MISO)
#define CYFX_SPI_GPIO_CS_FPGA         ((uint8_t)FX3_SPI_SS_FPGA)

#define CYFX_SPI_TEMPLATE_BITRATE_HZ      (1000000u)
#define CYFX_SPI_TEMPLATE_DELAY_CYCLES    (20u)

static CyFxGpioSpiWorker_t gSpiWorker;
static uint8_t gSpiWorkerStack[CYFX_SPI_WORKER_STACK_SIZE] __attribute__ ((aligned (32)));
static CyFxSpiSpeedProfile_t gSpiSpeedProfile = CYFX_SPI_SPEED_PROFILE_800KHZ;
static CyBool_t gSpiUseShortCriticalSection = CyFalse;
static CyU3PMutex gSpiProtoMutex;
static CyBool_t gSpiProtoMutexReady = CyFalse;

CyU3PReturnStatus_t
CyFxSpiStandaloneSetSpeedProfile (CyFxSpiSpeedProfile_t profile)
{
    CyU3PReturnStatus_t status;
    uint32_t bitRateHz;
    uint32_t edgeDelayCycles;

    switch (profile)
    {
        case CYFX_SPI_SPEED_PROFILE_800KHZ:
            bitRateHz = CYFX_SPI_TEMPLATE_BITRATE_HZ;
            edgeDelayCycles = CYFX_SPI_TEMPLATE_DELAY_CYCLES;
            break;

        default:
            return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    status = CyFxGpioSpiSetBitRateHint (&gSpiWorker.spi, bitRateHz, edgeDelayCycles);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

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

CyU3PReturnStatus_t
CyFxSpiProtoWrite8 (uint8_t chip_id, uint8_t address, uint8_t data)
{
    CyU3PReturnStatus_t status;

    if (gSpiProtoMutexReady)
    {
        status = CyU3PMutexGet (&gSpiProtoMutex, CYU3P_WAIT_FOREVER);
        if (status != CY_U3P_SUCCESS)
        {
            return status;
        }
    }

    status = CyFxSpiStandaloneFpgaWrite24 (chip_id, address, data);

    if (gSpiProtoMutexReady)
    {
        CyU3PMutexPut (&gSpiProtoMutex);
    }

    return status;
}

uint8_t
CyFxSpiProtoRead8 (uint8_t chip_id, uint8_t address)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint8_t value = 0;

    if (gSpiProtoMutexReady)
    {
        status = CyU3PMutexGet (&gSpiProtoMutex, CYU3P_WAIT_FOREVER);
        if (status != CY_U3P_SUCCESS)
        {
            return (uint8_t)status;
        }
    }

    status = CyFxSpiStandaloneFpgaRead8 (chip_id, address, &value);

    if (gSpiProtoMutexReady)
    {
        CyU3PMutexPut (&gSpiProtoMutex);
    }

    if (status != CY_U3P_SUCCESS)
    {
        return (uint8_t)status;
    }

                                      sizeof(gSpiWorkerStack),
                                      CYFX_SPI_WORKER_PRIORITY,
                                      CYU3P_AUTO_START);
        return CyFxSpiStandaloneFpgaWrite24 (chip_id, address, data);
    {
        return status;
    }

    status = CyFxSpiStandaloneSetSpeedProfile (CYFX_SPI_SPEED_PROFILE_800KHZ);
        return CyFxSpiProtoWrite8 (chip_id, address, data);
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
