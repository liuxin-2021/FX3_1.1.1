# FX3 GPIO-SPI Standalone Execution Plan

## 1. Objective
Build a clean-slate FX3 GPIO bit-bang SPI module that matches FPGA timing requirements, with:
- dedicated SPI worker thread isolation
- mutex-protected transaction path
- optional short critical section per frame
- configurable bit order and CPOL/CPHA behavior

This module is intentionally decoupled from existing project interfaces.

## 2. Current Deliverables (Phase 1 Done)
- `standalone_spi/cyfx_gpio_spi_standalone.h`
  - public data model for pin map, timing, frame transaction, worker context
  - API for direct transfer and worker-thread transfer
- `standalone_spi/cyfx_gpio_spi_standalone.c`
  - GPIO pin configuration
  - SPI transfer core with bit-order + edge control
  - mutex protection for bus transactions
  - optional short critical section during one frame
  - dedicated worker thread with request/done event handshake
- `standalone_spi/cyfx_gpio_spi_standalone_template.c`
  - minimal start/stop template
  - example 24-bit FPGA write function

## 3. Concurrency Model (Confirmed)
- Thread isolation:
  - all production SPI transactions should go through `CyFxGpioSpiWorkerTransferFrame` or `CyFxGpioSpiWorkerTransfer24`
  - only the worker thread drives CLK/MOSI/CS in normal operation
- Mutex protection:
  - `ctx->lock` protects bus-level GPIO toggling in `CyFxGpioSpiTransferFrame`
  - `worker->requestLock` protects request mailbox state
- Optional jitter control:
  - per-frame short critical section can be enabled by setting:
    - `timing.allowShortCriticalSection = CyTrue`
    - `frame.useShortCriticalSection = CyTrue`

## 4. Bit Rate Configuration Strategy
Use `CyFxGpioSpiSetBitRateHint`:
- `<= 500 kHz`:
  - use `halfPeriodUs` based on `CyU3PBusyWait`
  - stable and simple
- `> 500 kHz`:
  - use `halfPeriodUs = 0` and tune `edgeDelayCycles`
  - required for 1 MHz and 2 MHz targets

### Initial tuning suggestions
- 1 MHz: start with `edgeDelayCycles = 20`, then tune with scope
- 2 MHz: start with `edgeDelayCycles = 4..10`, then tune with scope

## 5. FPGA Matching Checklist
Lock these values from FPGA side before integration:
- bit order: MSB-first or LSB-first
- sampling edge: first or second edge
- clock idle level: low or high
- CS active polarity and setup/hold constraints
- frame width (24-bit currently assumed)

If FPGA is mode-like CPOL=0/CPHA=0 and MSB-first:
- `timing.cpolIdleHigh = CyFalse`
- `timing.cphaSecondEdge = CyFalse`
- `timing.bitOrder = CYFX_SPI_BIT_ORDER_MSB_FIRST`

## 6. Integration Plan (Next Phase)
1. Add standalone sources to build lists in `Debug/subdir.mk` and `Release/subdir.mk`.
2. Initialize GPIO matrix and `CyU3PGpioInit` before calling `CyFxSpiStandaloneStart`.
3. Replace existing FPGA write entry with a thin adapter calling `CyFxSpiStandaloneFpgaWrite24`.
4. Keep old path behind compile switch for rollback only.
5. Run waveform validation on CLK/MOSI/MISO/CS and compare to FPGA expected timing.

## 7. Validation Plan
- Functional:
  - send deterministic 24-bit patterns and verify FPGA register side effects
- Timing:
  - capture SPI waveforms and verify bit order and sampling edge
- Stress:
  - 10k continuous frames at target frequency
  - verify no timeout or malformed frame
- Concurrency:
  - trigger transfers from multiple caller contexts
  - verify no bus contention and no interleaving glitches

## 8. Known Boundaries
- Pure API GPIO toggling can work at 1 MHz after tuning, but depends on firmware load and interrupt activity.
- For robust 2 MHz in heavy-load systems, consider hybrid optimization (API init + tighter inner loop).
- If jitter is unacceptable, keep per-frame critical section enabled only around shortest transaction window.

## 9. Handoff Prompt Template
Use this in a new conversation to continue execution immediately:

"Continue from `standalone_spi/EXECUTION_PLAN.md` Phase 2. Integrate `standalone_spi/cyfx_gpio_spi_standalone.*` into this FX3 firmware build, wire `CyFxSpiStandaloneStart/Stop`, replace FPGA 24-bit write path with worker transfer API, and validate 1 MHz timing first. Keep mutex + thread isolation + optional per-frame critical section."