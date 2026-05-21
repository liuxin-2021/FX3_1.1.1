# Phase 0 FPGA/Register Benchmark - 30 Cold Boots

## Goal
Collect 30 cold-boot samples from firmware logs and verify:
- Per-run log line: `PHASE0-RUN[...]`
- Auto summary after 30 samples: `PHASE0-SUMMARY ... P50/P90/P99`

## Pre-check
1. Build firmware in `ios` with `.\\build`.
2. Program latest `ios.img` to device (RAM download is OK for fast iteration).
3. Open UART log terminal at 115200 and save logs to a file.
4. Keep host app ready to send INIT command (`0xC2`).

## One-run procedure (repeat 30 times)
1. Power off device completely.
2. Wait 3-5 seconds.
3. Power on device.
4. Wait until USB enumerates.
5. Send one INIT command from host.
6. Wait for one line matching `PHASE0-RUN[...]`.
7. Confirm the run index increased by 1.

## Acceptance criteria
- At least 30 valid `PHASE0-RUN[...]` lines.
- `PHASE0-SUMMARY` lines appear after run 30.
- `timeouts` should be 0 or very low.
- `P99` should be notably below fixed wait budget (used later to replace hard sleep).

## Optional data you can track per run
- power->read
- read->stable
- init
- reg100
- reg1000
- fail100/fail1000
- retries/timeouts

## Notes
- Cold boot is required. Repeating INIT without power cycle is not valid for this phase.
- If one run fails to print `PHASE0-RUN`, mark it invalid and redo that run.
