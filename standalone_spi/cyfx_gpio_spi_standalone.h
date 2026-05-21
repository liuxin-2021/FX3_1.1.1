#ifndef CYFX_GPIO_SPI_STANDALONE_H
#define CYFX_GPIO_SPI_STANDALONE_H

#include "cyu3types.h"
#include "cyu3os.h"
#include "cyu3gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum CyFxSpiBitOrder_e
{
    CYFX_SPI_BIT_ORDER_MSB_FIRST = 0,
    CYFX_SPI_BIT_ORDER_LSB_FIRST = 1
} CyFxSpiBitOrder_t;

typedef enum CyFxSpiSpeedProfile_e
{
    CYFX_SPI_SPEED_PROFILE_800KHZ = 0
} CyFxSpiSpeedProfile_t;

typedef struct CyFxGpioSpiPinMap_s
{
    uint8_t clk;
    uint8_t mosi;
    uint8_t miso;
    uint8_t cs;
} CyFxGpioSpiPinMap_t;

typedef struct CyFxGpioSpiTiming_s
{
    CyBool_t cpolIdleHigh;
    CyBool_t cphaSecondEdge;
    CyFxSpiBitOrder_t bitOrder;
    uint32_t halfPeriodUs;
    uint32_t edgeDelayCycles;
    uint32_t highDelayCycles;
    uint32_t lowDelayCycles;
    CyBool_t allowShortCriticalSection;
} CyFxGpioSpiTiming_t;

typedef struct CyFxGpioSpiContext_s
{
    CyFxGpioSpiPinMap_t pinMap;
    CyFxGpioSpiTiming_t timing;
    CyU3PMutex lock;
    CyBool_t initialized;
} CyFxGpioSpiContext_t;

typedef struct CyFxGpioSpiFrame_s
{
    uint32_t txData;
    uint32_t *rxData;
    uint8_t bitCount;
    CyBool_t keepChipSelect;
    CyBool_t useShortCriticalSection;
} CyFxGpioSpiFrame_t;

typedef struct CyFxGpioSpiWorker_s
{
    CyU3PThread thread;
    CyU3PEvent requestEvent;
    CyU3PEvent doneEvent;
    CyU3PMutex requestLock;
    CyFxGpioSpiContext_t spi;
    CyFxGpioSpiFrame_t requestFrame;
    CyU3PReturnStatus_t requestStatus;
    CyBool_t running;
} CyFxGpioSpiWorker_t;

typedef struct CyFxSpiDiagStats_s
{
    uint32_t transferCount;
    uint32_t writeReqCount;
    uint32_t writeOkCount;
    uint32_t writeFailCount;
    uint32_t readReqCount;
    uint32_t readOkCount;
    uint32_t readFailCount;
    uint32_t timeoutCount;
    uint32_t mutexGetFailCount;
    uint32_t protoCheckFailCount;
    uint32_t lastError;
    uint8_t lastReadValue;
    uint8_t firstMismatchValue;
    uint8_t expectedValue;
    uint8_t firstMismatchCaptured;
} CyFxSpiDiagStats_t;

#define CYFX_SPI_WORKER_REQ_EVENT   (1u << 0)
#define CYFX_SPI_WORKER_DONE_EVENT  (1u << 0)
#define CYFX_SPI_DIAG_STABILITY_DEFAULT_ROUNDS  (100000u)

CyU3PReturnStatus_t
CyFxGpioSpiConfigurePins (const CyFxGpioSpiPinMap_t *pinMap);

CyU3PReturnStatus_t
CyFxGpioSpiInit (CyFxGpioSpiContext_t *ctx,
                 const CyFxGpioSpiPinMap_t *pinMap,
                 const CyFxGpioSpiTiming_t *timing);

void
CyFxGpioSpiDeInit (CyFxGpioSpiContext_t *ctx);

CyU3PReturnStatus_t
CyFxGpioSpiSetBitRateHint (CyFxGpioSpiContext_t *ctx,
                           uint32_t bitRateHz,
                           uint32_t subUsDelayCycles);

CyU3PReturnStatus_t
CyFxGpioSpiTransferFrame (CyFxGpioSpiContext_t *ctx,
                          const CyFxGpioSpiFrame_t *frame,
                          uint32_t waitOption);

CyU3PReturnStatus_t
CyFxGpioSpiWorkerCreate (CyFxGpioSpiWorker_t *worker,
                         const CyFxGpioSpiPinMap_t *pinMap,
                         const CyFxGpioSpiTiming_t *timing,
                         void *threadStack,
                         uint32_t threadStackSize,
                         uint32_t threadPriority,
                         uint32_t startImmediately);

void
CyFxGpioSpiWorkerDestroy (CyFxGpioSpiWorker_t *worker);

CyU3PReturnStatus_t
CyFxGpioSpiWorkerTransferFrame (CyFxGpioSpiWorker_t *worker,
                                const CyFxGpioSpiFrame_t *frame,
                                uint32_t waitOption);

CyU3PReturnStatus_t
CyFxGpioSpiWorkerTransfer24 (CyFxGpioSpiWorker_t *worker,
                             uint32_t tx24,
                             uint32_t *rx24,
                             CyBool_t irqGuard,
                             uint32_t waitOption);

CyU3PReturnStatus_t
CyFxSpiStandaloneStart (void);

void
CyFxSpiStandaloneStop (void);

CyU3PReturnStatus_t
CyFxSpiStandaloneFpgaWrite24 (uint8_t chipId, uint8_t address, uint8_t value);

CyU3PReturnStatus_t
CyFxSpiStandaloneFpgaRead8 (uint8_t chipId, uint8_t address, uint8_t *value);

CyU3PReturnStatus_t
CyFxSpiProtoReadSelectedStatus8 (uint8_t selectValue, uint8_t *value);

CyU3PReturnStatus_t
CyFxSpiStandaloneSetSpeedProfile (CyFxSpiSpeedProfile_t profile);

CyFxSpiSpeedProfile_t
CyFxSpiStandaloneGetSpeedProfile (void);

void
CyFxSpiStandaloneSetShortCriticalSection (CyBool_t enable);

CyBool_t
CyFxSpiStandaloneIsShortCriticalSectionEnabled (void);

void
CyFxSpiDiagResetStats (void);

CyU3PReturnStatus_t
CyFxSpiDiagGetStats (CyFxSpiDiagStats_t *stats);

CyU3PReturnStatus_t
CyFxSpiDiagRunSmokeReadCheck (uint8_t chip_id,
                              uint8_t address,
                              uint8_t expectedValue,
                              uint32_t rounds,
                              uint32_t *mismatchCount);

CyU3PReturnStatus_t
CyFxSpiDiagRunStabilityReadCheck800kHz (uint8_t chip_id,
                                      uint8_t address,
                                      uint8_t expectedValue,
                                      uint32_t rounds,
                                      uint32_t *mismatchCount);

CyU3PReturnStatus_t
CyFxSpiDiagRunStabilityReadCheck800kHz100k (uint8_t chip_id,
                                          uint8_t address,
                                          uint8_t expectedValue,
                                          uint32_t *mismatchCount);

CyU3PReturnStatus_t
CyFxSpiProtoWrite8 (uint8_t chip_id, uint8_t address, uint8_t data);

uint8_t
CyFxSpiProtoRead8 (uint8_t chip_id, uint8_t address);

#ifdef __cplusplus
}
#endif

#endif
