<a name="top"></a>

<a href="https://www.electroanalog.com">
  <img src="https://electroanalog.github.io/img/electroanalog_logo.png" alt="Electroanalog" width="270px" />
</a>

# PSX In-Game Reset

[![License](https://img.shields.io/github/license/Electroanalog/PSX-IGR)](LICENSE)
[![Release](https://img.shields.io/github/v/release/Electroanalog/PSX-IGR)](../../releases)
[![Firmware Size](https://img.shields.io/badge/Firmware%20Size-4.45KB-blue)](../../releases)
[![Tested on Hardware](https://img.shields.io/badge/Tested-PlayStation-success)]()

PSX-IGR is a combo-triggered reset mod for the PlayStation 1, enhancing reset control with controller-safe logic and LED feedback.  
Originally based on the [**PlayStation 1 Reset Mod**](https://github.com/pyroesp/PlayStation-1-Reset-Mod) (2019), this version introduces improved input handling as timed combo-triggered reset, and responsive LED support, while being fully optimized for the PIC16F18325/26.

---

## Table of Contents
- [How It Works](#how-it-works)
- [Configurable Timing Parameters](#configurable-timing-parameters)
- [Building & Flashing](#building--flashing)
- [PIC16F18325/26 Pinout](#pic16f1832526-pinout)
- [LED Wiring Options](#led-wiring-options)  

---

## Features

- ✅ Combo hold logic to prevent unintentional resets
- ✅ Combo feedback LED driven by asynchronous timer interrupt
- ✅ Reset feedback LED blink handled via the same non-blocking mechanism
- ✅ Support for PSU-mounted common-anode dual LED (RA5 control only)
- ✅ Optional support for 2-pin bicolor LED between RA4 and RA5 (dual-color feedback)
- ✅ Safe reset duration with post-reset lockout period
- ✅ Controller and GunCon combo support (based on ID)
- ✅ Behavior and timings fully configurable via `#define` macros

---

## How It Works

- Monitors SPI communication from the controller port.
- When a valid controller ID is detected, combo inputs are checked:
  - **Select + Start + L2 + R2** (standard controller)
  - **A + Trigger + B** (GunCon)
  - **Select + X + L2 + R2** (XStation combo)
- A combo must be held for `COMBO_HOLD` ms to trigger a reset.
- Reset is held for `SHORT_DELAY` or `LONG_DELAY` depending on combo.
- During reset, the LED provides visual feedback with `RESET_BLINK_COUNT` blinks.
- After reset, a lockout period (`REBOOT_DELAY`) ensures safe recovery before new inputs are accepted.

---

## Configurable Timing Parameters

You can modify these constants in `main.c` to adjust behavior:

```c
#define COMBO_HOLD           1250  // ms - Time combo must be held to trigger reset
#define RESET_BLINK_COUNT       3  // Number of blink cycles after reset
#define RESET_BLINK_TOTAL_MS  500  // ms - Total blink duration
#define RESET_BLINK_DELAY_MS  200  // ms - Delay after blink before returning to idle
```
---

## Building & Flashing

### Source Code (Optional Compilation)
To build from source:
- Use [MPLAB X IDE](https://www.microchip.com/en-us/tools-resources/develop/mplab-x-ide) and [XC8 Compiler](https://www.microchip.com/en-us/tools-resources/develop/mplab-xc-compilers)
- Target microcontroller: **PIC16F18325** or **PIC16F18326**
- Set `__CONFIG` fuses appropriately for:
  - Internal 32 MHz oscillator
  - MCLR enabled
- Flash using ICSP with PICKit 3 or newer

### ⚡ Precompiled `.hex`
For convenience, a precompiled **`.hex` file** is included in the [Releases](../../releases) section.  
This allows quick flashing using:

- **MPLAB IPE 6.20 or newer**
- **PICKit 3** or compatible programmer

No source compilation is required if using the `.hex`.

---

## PIC16F18325/26 Pinout

| Pin | Name | Function |
|-----|------|----------|
| 1   | VCC  | +3.3V |
| 2   | RA5  | COMBO/RESET LED → Output to **[-]** cathode sinking control (for PSU 3-pin bicolor LED) |
| 3   | RA4  | COMBO/RESET LED → Output to **[+]** anode sourcing control (for 2-pin bicolor LED) |
| 4   | RA3  | ICSP MCLR/VPP |
| 5   | RC5  | RESET |
| 6   | RC4  | *(not used)* |
| 7   | RC3  | DEBUG |
| 8   | RC2  | SPI CMD |
| 9   | RC1  | SPI DATA |
| 10  | RC0  | SPI CLK |
| 11  | RA2  | SPI SS |
| 12  | RA1  | ICSP CLK |
| 13  | RA0  | ICSP DAT |
| 14  | VSS  | GND |

---

## LED Wiring Options

> [!IMPORTANT]
> **Always use one 100–120 Ω resistor in series with the RA5 output pin.**  
> This applies to all LED configurations, whether using a single LED or a dual-color setup.

### 🔴 RA5 alone (cathode sinking)
Use RA5 for a single LED (any color), or for the red side of a dual-color setup:
- Single LED: `K` to RA5, `A` to PSU +3.5V (standalone)
- Red terminal of a 3-pin common-anode LED
- Red side of a 2-pin bicolor LED (when used with RA4)

> [!TIP]
> For 3-pin common-anode LEDs, if the green cathode is tied directly to PSU, a 1–2 kΩ resistor can be added in series with the green `K` connection to reduce brightness and match the red level.

### 🟢🔴 RA4 with RA5 (2-pin bicolor only)  
> [!NOTE]
> A single 100–120 Ω resistor is sufficient and should be placed in series with either side of the bicolor LED.  
> Only one resistor is needed (on RA4 or RA5). Adding a second is optional and typically only used for brightness balancing.

Use RA4 **only** in conjunction with RA5 when using a 2-pin bicolor LED:
- RA4 = 0, RA5 = 1 → Green (idle)
- RA4 = 1, RA5 = 0 → Red (combo/reset)
- During reset blink: RA5 toggles, RA4 remains OFF (no green blink)  

> [!WARNING] 
> RA4 should **not** be used for single LEDs. It does not indicate reset blink and provides incorrect behavior alone.  

---

## License

This is a derivative work licensed under the [Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA 4.0)](https://creativecommons.org/licenses/by-sa/4.0/).

---

## Credits

Modified and extended by **Electroanalog® VICE (2025)**  
Based on [*PlayStation 1 Reset Mod*](https://github.com/pyroesp/PlayStation-1-Reset-Mod) by **pyroesp (2019)**  

*PlayStation is a registered trademark of Sony Interactive Entertainment LLC (SIE), formerly Sony Computer Entertainment Inc. (SCE). All rights reserved.*

---

## Topics / Tags

`ps1` `psx` `playstation` `igr` `reset-mod` `pic16f18325` `pic16f18326` `modchip` `led-feedback` `guncon` `xstation` `diy-console-mod` `retro-gaming`

