#ifndef VVVF_MAIN_H
#define VVVF_MAIN_H

#include "vvvf-struct.h"
#include "vvvf-core.h"

#define ENABLE_DISPLAY

/* Returns the profile's PWM level so the GPIO driver can check its topology. */
int calculatePhases(
    PhaseStatus *outU,              // OUTPUT U
    PhaseStatus *outV,              // OUTPUT V
    PhaseStatus *outW,              // OUTPUT W
    VvvfValues *status,             // SINE AND SAW ANGLE FREQ AND TIME
    VvvfSoundFunction soundFunction // CALCULATE SOUND FUNCTION
);

void calculatePwmPhases(PhaseStatus *outU, PhaseStatus *outV, PhaseStatus *outW,
                        VvvfValues *status, PwmCalculateValues *pwm);
VvvfValues getDisplayVvvfGpio(VvvfSoundFunction *sound);

// VVVF GPIO
void requestStatusVvvfGpio();
bool canGetStatusVvvfGpio();
VvvfValues getStatusVvvfGpio(void);
void setStatusVvvfGpio(VvvfValues *want);
VvvfSoundFunction getSoundVvvfGpio();
void setSoundVvvfGpio(VvvfSoundFunction _sound);

// Reverse. Swaps the V and W phases so the motor turns the other way.
bool getReverseVvvfGpio(void);
void setReverseVvvfGpio(bool _reverse);

extern uint64_t calcTimeElapse;
void taskCalculationPhases(void *param);
#endif
