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
> Always use a **100 Ω resistor** in series with the **RA5 (pin 2)** output.  
> This applies to all LED configurations: standalone red, 3-pin common-anode, or 2-pin bicolor.

### LED Configurations Overview

#### 🔴 RA5 (pin 2) - Red Output (Mandatory)
- Drives red LED for **combo feedback** and **reset blink**.
- Must always be used, either alone or with RA4.
- Compatible with:
  - **3-pin common-anode LED**: Red cathode to RA5 via 100 Ω resistor.
  - **Single-color LED**: Anode to VCC, cathode to RA5 via 100 Ω resistor.

> [!TIP]
> For 3-pin common-anode LEDs:  
> Replace the **original PSU resistor** (typically 120-130 Ω) in series with the **green cathode** with a **470 Ω resistor**.  
> This reduces the green brightness to better match the red LED (driven by RA5), providing more balanced LED feedback.

#### 🟢 RA4 (pin 3) - Green Output (Optional)
- Used **only** when connecting a **2-pin bicolor LED (antiparallel)** between **RA4** and **RA5**.
- Provides idle green indication; RA4 does not blink during reset.
- Do **not** connect RA4 to the green terminal (cathode) of a 3-pin common-anode LED:  
  - RA4 is not meant to drive LEDs directly to GND.  
  - It only functions as part of a polarity-switching pair (RA4–RA5) for 2-pin bicolor LEDs.  
  - Using RA4 for a discrete green LED will result in incorrect or no indication.

> [!WARNING]
> RA4 is **not suitable** for standalone LED use.  
> It only works correctly in conjunction with RA5 using a 2-pin bicolor LED.

---

### LED Assignment Table

| System State          | RA4 (pin 3) 🟢 | RA5 (pin 2) 🔴 | 2-Pin Bicolor LED (antiparallel) | Single Red LED or 3-Pin Red Terminal |
|-----------------------|----------------|-----------------|----------------------------------|--------------------------------------|
| Idle (system on)      | LO             | HI              | 🟢 Green                         | ⚪ OFF                              |
| Combo detected        | HI             | LO              | 🔴 Red                           | 🔴 ON                               |
| Reset (triggered)     | HI             | LO/HI           | 🔴 Blinking                      | 🔴 Blinking                         |
| Post-reset delay      | LO             | LO              | ⚪ OFF                           | ⚪ OFF                              |
| Idle (after reset)    | LO             | HI              | 🟢 Green                         | ⚪ OFF                              |
| *RA4 only active*     | HI             | HI              | ⚪ OFF                           | Do not use                           |

---

### Summary

> [!NOTE]
> - 🔴 **RA5** handles all meaningful LED signaling (combo, reset blink).  
> - 🟢 **RA4** is used **only** for idle indication when using a 2-pin bicolor LED.  
> - When using a **3-pin LED**, never connect RA4 to the green terminal.  
> - For **single red LEDs**, connect anode to VCC and cathode to RA5 via 100 Ω resistor.

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

