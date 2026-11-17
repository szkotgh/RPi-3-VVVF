#include <stdbool.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>

#include "vvvf-struct.h"
#include "vvvf-sounds.h"
#include "vvvf-hardware.h"
#include "vvvf-core.h"
#include "vvvf-main.h"
#include "pi-values.h"

#include "task.h"

// Reverse state. Only taskMascon writes it, and only while the machine is
// stopped, so it never changes underneath a running calculation.
static atomic_bool reverseGpio;

bool getReverseVvvfGpio(){
    return atomic_load_explicit(&reverseGpio, memory_order_relaxed);
}

void setReverseVvvfGpio(bool _reverse){
    atomic_store_explicit(&reverseGpio, _reverse, memory_order_relaxed);
}

int calculatePhases(
    PhaseStatus *outU,              // OUTPUT U
    PhaseStatus *outV,              // OUTPUT V
    PhaseStatus *outW,              // OUTPUT W
    VvvfValues *status,             // All values
    VvvfSoundFunction soundFunction // CALCULATE SOUND FUNCTION
)
{
    PwmCalculateValues pwm;
    soundFunction(status, &pwm);
    calculatePwmPhases(outU, outV, outW, status, &pwm);
    return pwm.level;
}

void calculatePwmPhases(PhaseStatus *outU, PhaseStatus *outV, PhaseStatus *outW,
                        VvvfValues *status, PwmCalculateValues *pwm)
{
    PhaseStatus (*phaseCalculate)(CalculateParam *) = pwm->level == 2 ? vvvfTwoLevel : vvvfThreeLevel;

    CalculateParam param;
    param.pwm = pwm;
    param.status = status;

    // Reversing swaps two of the three phases, which flips the rotating field.
    bool reverse = getReverseVvvfGpio();
    double phaseV = reverse ? M_4PI_3 : M_2PI_3;
    double phaseW = reverse ? M_2PI_3 : M_4PI_3;

    // Phase U
    param.initialPhase = M_PI_6;
    if(outU != 0) *outU = phaseCalculate(&param);

    // Phase V
    param.initialPhase = phaseV + M_PI_6;
    if(outV != 0) *outV = phaseCalculate(&param);

    // Phase W
    param.initialPhase = phaseW + M_PI_6;
    if(outW != 0) *outW = phaseCalculate(&param);
}

/* Each reader requests its own copy. The GPIO core never waits for either
 * reader, and the mascon's outgoing command has a separate buffer. */
enum { SNAPSHOT_IDLE, SNAPSHOT_REQUESTED, SNAPSHOT_READY };
typedef struct {
    atomic_uint state;
    VvvfValues values;
    VvvfSoundFunction sound;
} StatusSnapshot;
static StatusSnapshot masconSnapshot, displaySnapshot;
static VvvfValues pendingStatus;
static atomic_bool updateStatusGpio;
static VvvfValues statusGpio = {
    false, false, 0, false, 0,
    true, true,
    0, 0, 0, 0, 0, 0, 0,
};
static _Atomic(VvvfSoundFunction) requestedSound, activeSound;

static void publishSnapshot(StatusSnapshot *snapshot, VvvfSoundFunction sound)
{
    if (atomic_load_explicit(&snapshot->state, memory_order_acquire) != SNAPSHOT_REQUESTED)
        return;
    snapshot->values = statusGpio;
    snapshot->sound = sound;
    atomic_store_explicit(&snapshot->state, SNAPSHOT_READY, memory_order_release);
}

VvvfValues getDisplayVvvfGpio(VvvfSoundFunction *sound)
{
    atomic_store_explicit(&displaySnapshot.state, SNAPSHOT_REQUESTED, memory_order_release);
    while (atomic_load_explicit(&displaySnapshot.state, memory_order_acquire) != SNAPSHOT_READY) {}
    *sound = displaySnapshot.sound;
    return displaySnapshot.values;
}

void requestStatusVvvfGpio(){
    /* A read immediately following a mascon command must observe that command. */
    while (atomic_load_explicit(&updateStatusGpio, memory_order_acquire)) {}
    atomic_store_explicit(&masconSnapshot.state, SNAPSHOT_REQUESTED, memory_order_release);
}

bool canGetStatusVvvfGpio(){
    return atomic_load_explicit(&masconSnapshot.state, memory_order_acquire) == SNAPSHOT_READY;
}

VvvfValues getStatusVvvfGpio(){
    return masconSnapshot.values;
}

void setStatusVvvfGpio(VvvfValues *want){
    while (atomic_load_explicit(&updateStatusGpio, memory_order_acquire)) {}
    pendingStatus = *want;
    atomic_store_explicit(&updateStatusGpio, true, memory_order_release);
}

VvvfSoundFunction getSoundVvvfGpio(){
    return atomic_load_explicit(&activeSound, memory_order_relaxed);
}
void setSoundVvvfGpio(VvvfSoundFunction _sound){
    atomic_store_explicit(&requestedSound, _sound, memory_order_relaxed);
}

uint64_t calcTimeElapse = 0;

void taskCalculationPhases(void *param)
{
    uint64_t _s = 0;
    PhaseStatus outputU = 0, outputV = 0, outputW = 0;
    PhaseStatus preOutputU = 0, preOutputV = 0, preOutputW = 0;
    VvvfSoundFunction soundGpio = NULL;
    while (true)
    {
        _s = timer_getTickCount64();
        
        VvvfSoundFunction sound = atomic_load_explicit(&requestedSound, memory_order_relaxed);
        if (sound != soundGpio) {
            soundGpio = sound;
            atomic_store_explicit(&activeSound, soundGpio, memory_order_relaxed);
        }
        if (atomic_load_explicit(&updateStatusGpio, memory_order_acquire)) {
            statusGpio = pendingStatus;
            atomic_store_explicit(&updateStatusGpio, false, memory_order_release);
        }

        double _t = calcTimeElapse / 1000000.0;
        statusGpio.saw_time += _t;
        statusGpio.sin_time += _t;
        statusGpio.generation_current_time = timer_getTickCount64() / 1000000.0;
        int level = calculatePhases(&outputU, &outputV, &outputW, &statusGpio, soundGpio);

        /* The installed half bridges cannot drive the midpoint of a 3-level
         * profile. The display still renders it; keep all six physical gates off. */
        if (level != 2)
            outputU = outputV = outputW = PHASE_MIDDLE;

        if(abs(outputU - preOutputU) == 2) { preOutputU = outputU; outputU = 1; }
        if(abs(outputV - preOutputV) == 2) { preOutputV = outputV; outputV = 1; }
        if(abs(outputW - preOutputW) == 2) { preOutputW = outputW; outputW = 1; }

        if(!statusGpio.mascon_off){
            if(statusGpio.free_run){
                double freq_change = statusGpio.free_freq_change * _t;
                double final_freq = statusGpio.wave_stat + freq_change;
                if (statusGpio.sin_angle_freq * M_1_2PI <= final_freq)
                {
                    statusGpio.wave_stat = statusGpio.sin_angle_freq * M_1_2PI;
                    statusGpio.free_run = false;
                }
                else
                {
                    statusGpio.wave_stat = final_freq;
                    statusGpio.free_run = true;
                }
            }
        }
        else
        {
            double freq_change = statusGpio.free_freq_change * _t;
            double final_freq = statusGpio.wave_stat - freq_change;
            statusGpio.wave_stat = final_freq > 0 ? final_freq : 0;
            statusGpio.free_run = true;
        }

        publishSnapshot(&masconSnapshot, soundGpio);
        publishSnapshot(&displaySnapshot, soundGpio);

        while(timer_getTickCount64() - _s < 5);
        setPhasePinStatus(outputU, outputV, outputW);
        calcTimeElapse = timer_getTickCount64() - _s;
    }
}
