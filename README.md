# RPi-3-VVVF
Making a VVVF inverter with a Raspberry Pi 3.
The firmware includes two-level profiles for JRE E231 Hitachi, JRE 209 Mitsubishi, N1000 Siemens GTO and Osaka Metro 10A Hitachi IGBT.
The JRE 209 profile adapts the original three-level PWM to the board's two-level output. See [src/vvvf-sounds.c](src/vvvf-sounds.c).

# terms of use
## disclaimer
YOU ARE RESPONSIBLE FOR ANYTHING THAT HAPPENS IF YOU USE THE INFORMATION FROM THIS PROJECT.<br>
IT MAY DAMAGE ELECTRONICS.<br>
THIS VVVF CODE IS NOT MADE BY AN ENGINEER OR ANY PROFESSIONAL.<br>

## credit
The main author of this program is VvvfGeeks except for the code with assembler.<br>
Please reference this github url if you upload on youtube or other websites.

# references
[Peripheral Specifics](https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf)<br>
[Multicore code reference](https://github.com/LdB-ECM/Raspberry-Pi-Multicore/tree/master) (I borrowed some code from here.)

# requirements
You need to install cross compiler.<br>
```
sudo apt install gcc-arm-none-eabi
```

# build
Normally, just run <br>
`make Pi3`

# install to RPi 3
When you have built the code, you will find `kernel7.img` inside of build folder.<br>
What you need to do is

## Download the necessary files
From https://github.com/raspberrypi/firmware/tree/master/boot , you have to get <br>
 - bootcode.bin
 - start.elf
 - fixup.dat

## SD card
Now, you need to have a SD card which has more than 2GB but less than 32GB.<br>
Also the SD card needs to be formated with FAT32.<br>
<br>

## Install
Now , it's time to install.<br>
Copy `bootcode.bin` , `start.elf`, ' fixup.dat' and `kernel7.img` that you have built.<br>


# GPIO pin map

The Raspberry Pi 3 firmware configures 17 GPIOs on the 40-pin J8 header.
Use the **physical pin** column for wiring and the **BCM GPIO** column for code.
Inputs and outputs use 3.3 V logic. Pin definitions are in
[vvvf-hardware.h](src/vvvf-hardware.h). Header positions follow the
[Raspberry Pi 3 B+ J8 schematic](https://pip-assets.raspberrypi.com/categories/532-raspberry-pi-3-model-b/documents/RP-008339-DS-1-raspberry-pi-3-b-plus-reduced-schematics.pdf).

| Physical pin | BCM GPIO | Firmware name | Pi direction | Function |
| ---: | ---: | --- | --- | --- |
| 11 | 17 | `PIN_MASCON_BIT1` | Input | Mascon bit 1, weight 2 |
| 13 | 27 | `PIN_MASCON_BIT2` | Input | Mascon bit 2, weight 4 |
| 15 | 22 | `PIN_MASCON_BIT3` | Input | Mascon bit 3, weight 8 |
| 16 | 23 | `PIN_DEBUG_2` | Output | Debug 2, stays LOW |
| 18 | 24 | `PIN_DEBUG_1` | Output | Debug 1, stays LOW |
| 19 | 10 | `PIN_W_LOW_1` | Output | Phase W low-side gate command |
| 21 | 9 | `PIN_V_LOW_1` | Output | Phase V low-side gate command |
| 22 | 25 | `PIN_BTN_L` | Input | Left button, no action assigned |
| 23 | 11 | `PIN_U_LOW_1` | Output | Phase U low-side gate command |
| 24 | 8 | `PIN_BTN_SEL` | Input | Select button, no action assigned |
| 26 | 7 | `PIN_BTN_R` | Input | Right button, LOW selects the next sound profile at zero output frequency |
| 29 | 5 | `PIN_W_HIGH_1` | Output | Phase W high-side gate command |
| 31 | 6 | `PIN_V_HIGH_1` | Output | Phase V high-side gate command |
| 33 | 13 | `PIN_U_HIGH_1` | Output | Phase U high-side gate command |
| 37 | 26 | `PIN_MASCON_BIT0` | Input | Mascon bit 0, weight 1 |
| 38 | 20 | `PIN_EMO` | Input | Normally closed stop loop, LOW latches EB until reboot |
| 40 | 21 | `PIN_REVS` | Input | HIGH requests reverse, LOW requests forward |

| Function | Physical pins |
| --- | --- |
| 3.3 V | 1, 17 |
| 5 V | 2, 4 |
| GND | 6, 9, 14, 20, 25, 30, 34, 39 |
| GPIOs unassigned by this firmware | 3, 5, 7, 8, 10, 12, 32, 35, 36 |
| Reserved for HAT ID EEPROM | 27, 28 |

The dashboard uses HDMI. The boot indicator uses the onboard ACT LED.

## Three-phase gate outputs

Each phase drives one half bridge. Initialization sets all six gate commands LOW.
The firmware uses the following high-side and low-side command pairs.

| State | High-side GPIO | Low-side GPIO | Command |
| --- | --- | --- | --- |
| PHASE_LOW | 0 | 1 | leg tied low |
| PHASE_MIDDLE | 0 | 0 | dead time, both off |
| PHASE_HIGH | 1 | 0 | leg tied high |

`taskCalculationPhases` inserts PHASE_MIDDLE when a leg changes directly
between LOW and HIGH.

## Mascon (Speed controller)

The four mascon inputs rely on the SoC's power-up pull-downs. A closed contact
reads HIGH and contributes its bit weight from the pin map. Add the weights
to get the notch value. With all four contacts open, the value is 0 (EB).

| value | notch | frequency change |
| --- | --- | --- |
| 0 | EB | full emergency rate |
| 1 ~ 8 | B8 ~ B1 | service brake, B8 strongest |
| 9 | N | coasts on the sound profile's jerk setting |
| 10 | P0 | powered, holds speed, not neutral |
| 11 ~ 15 | P1 ~ P5 | power |

The rates come from three constants in `src/main.c`. Power keeps its ceiling
split evenly over P1~P5, braking splits its own ceiling over B1~B8, and EB is
a single step above that:

```
MASCON_MAX_ACCEL                    P5, and P<n> = MAX_ACCEL * n / 5
MASCON_MAX_BRAKE                    B8, and B<n> = MAX_BRAKE * n / 8
MASCON_EB_BRAKE                     EB
```

The cart is light, so braking is set well above power. Retune the three
constants rather than the table.

The notch readout on the dashboard is coloured by stage, so the stage reads at a
glance without parsing the letters:

| stage | colour |
| --- | --- |
| EB | `#FF0000` |
| B1 ~ B8 | `#FF8000` |
| N | `#00FF00` |
| P0 ~ P5 | `#0000FF` |

The framebuffer is RGB565, so each colour is the value above with the low 3/2/3
bits dropped. The colour lives in the notch table next to the label, one entry
per notch.

## EMO (Emergency stop)

The normally closed loop holds the input HIGH during normal operation. When
the loop opens, the default pull-down brings the input LOW. On reading LOW,
`taskMascon` forces the software notch to EB until reboot. The dashboard EMO
lamp blinks while the stop is latched.

## REVS (Reverse)

The firmware samples REVS only when both `sin_angle_freq` and `wave_stat`
are zero and the physical mascon reads EB. At all other times it keeps the
last direction. In reverse it swaps phases V and W and lights the REVS lamp.
