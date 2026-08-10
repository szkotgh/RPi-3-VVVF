# RPi-3-VVVF
Making a VVVF inverter with a Raspberry Pi 3.
It does not contain sample for generating vvvf sound.
You have a way to generate the code using vvvf-simulator.

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


# VVVF pin out
This number is the BCM GPIO number.

One half bridge per phase, so two gate signals each. The earlier four signal
layout was for a three level NPC leg, but every sound profile here is two
level, so the inner switch pair never carried anything the outer pair did not.

- PIN_U_HIGH_1 13
- PIN_U_LOW_1 11

- PIN_V_HIGH_1 6
- PIN_V_LOW_1 9

- PIN_W_HIGH_1 5
- PIN_W_LOW_1 10

- debug_PIN_2 23
- debug_PIN 24

Each leg only ever takes one of three states, and the two switches are never
on together:

| state | H_1 | L_1 | |
| --- | --- | --- | --- |
| PHASE_LOW | 0 | 1 | leg tied low |
| PHASE_MIDDLE | 0 | 0 | dead time, both off |
| PHASE_HIGH | 1 | 0 | leg tied high |

`taskCalculationPhases` inserts PHASE_MIDDLE whenever a leg goes straight from
LOW to HIGH or back, so the leg cannot shoot through.

# Function pin out
This number is the BCM GPIO number

## Mascon (Speed controller)
### pin out
 - mascon_1 26
 - mascon_2 17
 - mascon_3 27
 - mascon_4 22

All four sit in GPIO 9..27, which the SoC powers up with a pull down, so an
open notch switch reads 0. GPIO 0..8 power up with a pull up and would read 1
with nothing attached, and GPIO 2/3 additionally carry fixed 1.8k pull up
resistors on the board. A pin is HIGH when the notch contact is closed.

Inside the program, this will generate a integer by using mascon_1 ~ mason_4.<br>
This is the how the integer will be.
`mascon_status_value = input(mascon_1) | input(mascon_2)<<1 | input(mascon_3)<<2 | input(mascon_4)<<3`<br>

The value selects a notch. All four pins open means 0, which is EB, so the
machine sits in emergency brake until the mascon is actually wired up.

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
### pin out
 - EMO 20

Wired normally closed: the line must be held **HIGH** for the machine to run.
LOW latches an emergency stop, the notch is forced to EB from that moment on
whatever the throttle says, and the latch only clears on reboot. The `EMO` lamp
on the dashboard blinks once a second for as long as it is latched, so a tripped
stop cannot be mistaken for a steady legend.

This is fail safe. GPIO 20 powers up pulled down, so a cut wire, a pulled
connector, or an EMO circuit that was never wired all read LOW and trip, which
is exactly what a normally closed loop is supposed to do. The machine will not
move until the EMO line is actually present and HIGH.

The pin is first read by `taskMascon`, which only starts several seconds after
the GPIO is configured, so the line has long settled before it can latch.

## REVS (Reverse)
### pin out
 - REVS 21

HIGH means reverse, LOW means forward. The V and W phases are swapped, which
flips the direction of the rotating field and turns the motor the other way.

The pin is only sampled while **the machine is fully stopped and the throttle
is physically at EB**. At any other moment the input is ignored and the last
latched direction stays in force, so the field can never reverse under load.
The `REVS` lamp on the dashboard is lit while reverse is active.

## Control button
### pin out
 - button_R 7
 - button_SEL 8
 - button_L 25
