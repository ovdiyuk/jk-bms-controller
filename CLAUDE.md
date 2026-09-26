# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for a battery-powered handheld remote (ESP32-C3) that switches the **discharge MOSFET of a
JK BMS** on and off and mirrors battery state on a three-colour indicator. The remote reaches the BMS
over a **single-fibre WDM optical link**, not copper: the BMS sits at an EW emitter, so the control
path must be immune to the interference that installation generates, and galvanically isolated from
the pack. Distance is only 10–30 m; the fibre is there for immunity, not reach.

Single translation unit, no external libraries, no RTOS tasks — everything runs in `loop()`.

## Commands

PlatformIO lives in the **project-local** `.venv` (not the usual `~/.platformio/penv`):

```powershell
.\.venv\Scripts\Activate.ps1
```

```bash
pio run                      # build (env: esp32c3)
pio run -t upload            # flash over the board's own USB port
pio device monitor           # serial log, 115200
pio device list              # find the COM port
```

There are **no tests** — `test/`, `lib/` and `include/` hold only PlatformIO's placeholder stubs.
Verification is: build, flash, then read the serial log, which prints a decoded `BMS Status` line
for every telemetry frame.

Changes cannot be verified without the physical remote, the fibre link and a live BMS. Say so rather
than claiming a behavioural change works.

## Architecture

All logic is in [src/main.cpp](src/main.cpp).

**Protocol — legacy JK "NW" TTL, 115200 8N1, on the BMS's GPS port.** Two frame builders:
`jkWriteRegister(reg, value)` (`cmd = 0x02`, 22 bytes) writes the discharge MOSFET register `0xAC`;
`jkRequestTelemetry()` (`cmd = 0x06`, 21 bytes) asks for all registers. The checksum is a plain
16-bit sum of every preceding byte, right-aligned in a 4-byte field. A reply carries `src = 0x00`;
our own frames carry `src = 0x03` — that is how an echo is told from an answer.

**Receive path** — `processBmsResponses()` assembles frames by header and length field,
`parseTelemetryData()` walks the register TLVs and fills `bmsCurrentA`, `bmsTempC`,
`bmsSocPercent`, `bmsMinCellV`, `bmsMaxCellV`, `bmsCellDeltaV` and `bmsActualDischargeState`
(register `0xAC`, the real MOSFET state).

**Control is closed-loop.** The switch sets `targetDischargeState`; if the BMS reports something
different, the write is re-sent every 300 ms until the two agree. Telemetry is polled 5×/s.

**Indicator — a state machine in `updateLedIndicators()`.** Exactly one colour is lit at a time:

| Condition | Indicator |
|---|---|
| no faults, discharge off | yellow solid |
| no faults, discharge on, SOC > 20 % | green solid |
| no faults, discharge on, SOC ≤ 20 % | green blink |
| faults, discharge on | red solid |
| faults, discharge off | red blink |
| no BMS reply for 5 s | all three blink at 300 ms — overrides everything |

It reads `bmsActualDischargeState`, not the switch position: the indicator shows what *is*, not what
was commanded. `determineActiveFaultCode()` returns 1…6; the code itself is no longer displayed, only
logged on change. Thresholds live in named constants at the top of the file.

All timing uses unsigned `millis()` subtraction, which is rollover-safe — keep it that way.

`Serial` is routed to the chip's **native USB CDC**, so the board's own port carries both the flash
upload and the log. This depends on the `build_flags` in `platformio.ini`.

## Hardware

Board is an **ESP32-C3 Super Mini** — see [HARDWARE.md](HARDWARE.md) for the full pinout, board
traps and the optical link; read it before touching pin assignments. `platformio.ini` declares
`board = esp32-c3-devkitm-1` deliberately (no Super Mini profile exists; same chip and flash).

| GPIO | Role |
|---|---|
| 1 | latching switch, `INPUT_PULLDOWN`, **closes to 3.3 V** — active HIGH |
| 3 | `Serial1` RX ← optical module `TX1` |
| 4 | `Serial1` TX → optical module `RX1` |
| 5 / 6 / 7 | red / yellow / green, active-high, common cathode, resistors built into the indicator |
| 8 | onboard LED, heartbeat, **active-low** |
| 2, 9 | strapping pins — leave unconnected |

The pin constants were remapped from upstream's defaults to match the unit as actually built. If the
firmware is ever taken to a differently wired remote, that block at the top of `main.cpp` is the only
thing that needs changing.

## Field notes worth keeping

- The BMS is a **JK-B1A8S10P**, 8S LiFePO₄. On this model family the GPS connector is soldered on the
  **top** side, so its pinout reads mirrored — easy to swap GND and VBAT.
- The GPS port's fourth pin is **`VBAT`: raw pack voltage**, not a regulated rail. At full charge the
  pack reaches ≈29.2 V against the optical module's 30 V ceiling — about 3 % margin.
- A matched WDM pair must be **different wavelengths** (1310 nm TX one end, 1550 nm the other). Two
  identical ends never link; that cost a long debugging session once already.
- Baseline link quality in quiet conditions: 100 % of telemetry frames parsed, <1 % line noise.
  Re-measure with the emitter running to see the real degradation.

## Known technical debt

[techDebt.md](techDebt.md) tracks what is still open and records what has been closed. Read it before
reporting a defect — several obvious-looking ones are already known or deliberately accepted.
