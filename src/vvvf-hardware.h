#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

//
// Here's definition for each pin.
//

//
// One half bridge per phase: a high side and a low side switch. The former
// four-switch-per-phase layout was for a three level NPC leg, but every sound
// profile here is two level, so the inner pair never carried anything the
// outer pair did not.
//
#define PIN_U_HIGH_1 13
#define PIN_U_LOW_1 11

#define PIN_V_HIGH_1 6
#define PIN_V_LOW_1 9

#define PIN_W_HIGH_1 5
#define PIN_W_LOW_1 10

//
// Mascon pins must sit in GPIO 9..27, which the SoC powers up with a pull
// down, so an open notch switch reads 0. GPIO 0..8 power up with a pull up
// and would read 1 forever with nothing attached, and GPIO 2/3 also carry
// fixed 1.8k pull up resistors on the board that software cannot remove.
// BIT0 used to be GPIO 4 and was stuck high for exactly that reason.
//
#define PIN_MASCON_BIT0 26
#define PIN_MASCON_BIT1 17
#define PIN_MASCON_BIT2 27
#define PIN_MASCON_BIT3 22

//
// EMO  : emergency stop, wired normally closed. The line is held HIGH while
//        everything is well, and LOW latches the notch at EB until the next
//        reboot. GPIO 20 powers up pulled down, so an unwired or broken EMO
//        line reads LOW and trips, which is the point of running it this way.
// REVS : reverse. HIGH means reverse, and the state is only sampled while the
//        machine is stopped with the throttle at EB.
//
#define PIN_EMO 20
#define PIN_REVS 21

#define PIN_BTN_R 7
#define PIN_BTN_SEL 8
#define PIN_BTN_L 25

#define PIN_DEBUG_2 23
#define PIN_DEBUG_1 24

#define PIN_DEBUG_2_ENABLE
#define PIN_DEBUG_1_ENABLE

#endif

#ifndef VVVF_RASPBERRYPI_H
#define VVVF_RASPBERRYPI_H

typedef char PhaseStatus;
#define PHASE_HIGH 2
#define PHASE_MIDDLE 1
#define PHASE_LOW 0

typedef struct {
    char H_1;
    char L_1;
} PhasePinStatus;

/**
 * @brief 
 * Blink Default LED
 */
void Flash_LED(void);

/**
 * @brief 
 * This function must implement initialize all of pins and when it is set output mode, it will set to 0;
 */
void initializeVvvfHardware(void);

/**
 * @brief 
 * Blink debug 1 pin
 */
void Flash_Debug1(void);

/**
 * @brief 
 * Blink debug 2 pin
 */
void Flash_Debug2(void);

/**
 * @brief 
 * Reads mascon value and convert to char
 * 
 * @return char 
 */
char readMasconValue(void);

/**
 * @brief 
 * Read Btn status R
 */
char readButtonR();

/**
 * @brief 
 * Read Btn status SEL
 */
char readButtonSel();

/**
 * @brief 
 * Read Btn status L
 */
char readButtonL();

/**
 * @brief Create the PhasePinStatus object
 *                              H_1 L_1
 * stat = PHASE_LOW         :   0   1
 * stat = PHASE_MIDDLE      :   0   0
 * stat = PHASE_HIGH        :   1   0
 *
 * PHASE_MIDDLE is the dead time state that taskCalculationPhases inserts
 * between a LOW and a HIGH: both switches off, so the leg never shoots through.
 *
 * @param stat
 * @return PhasePinStatus
 */
PhasePinStatus createPhasePinStatus(PhaseStatus);

/**
 * @brief Sets pin status for each Phase
 *                              H_1 L_1
 * stat = PHASE_LOW         :   0   1
 * stat = PHASE_MIDDLE      :   0   0
 * stat = PHASE_HIGH        :   1   0
 *
 * @param stat
 * @return void
 */
void setPhasePinStatus(PhaseStatus, PhaseStatus, PhaseStatus);

/**
 * @brief
 * Reads the raw emergency stop line. HIGH is healthy, LOW is tripped.
 */
char readEmoValue(void);

/**
 * @brief
 * Reads the reverse input
 */
char readRevsValue(void);

#endif