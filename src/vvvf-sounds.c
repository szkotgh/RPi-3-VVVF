#include <float.h>
#include <stdbool.h>
#include <stddef.h>

#include "pi-values.h"
#include "vvvf-sounds.h"

/* Translated from the four VVVF-Simulator YAML profiles. Frequencies are Hz,
 * jerk rates are Hz/s and carrier update intervals are seconds. */
#define COUNT_OF(array) (sizeof(array) / sizeof((array)[0]))

enum {
    NORMAL = 1, POWER_ON = 2, POWER_OFF = 4,
    HOLD_ON = 8, HOLD_OFF = 16,
    ALL_STATES = NORMAL | POWER_ON | POWER_OFF
};

typedef struct {
    double start_hz, start_amp, end_hz, end_amp;
    double cutoff, maximum;
    bool inverse;
    double curve_rate;
} AmplitudeCurve;

typedef struct {
    double from_hz, from_value, to_hz, to_value;
} CarrierRamp;

typedef struct {
    double from_hz;
    double rotate_from, rotate_below;  /* -1 means no rotation bound */
    unsigned states;
    PulseMode pulse;
    AmplitudeCurve normal, power_on, power_off;
    CarrierFreq carrier;
    const CarrierRamp *ramp;
    bool musical_carrier;
} Pattern;

typedef struct {
    const Pattern *patterns;
    size_t count;
    double min_hz, on_rate, off_rate, on_max_hz, off_max_hz;
} Direction;

/* The simulator evaluates moving carriers on the whole selected band, even
 * when its last fraction of a Hz extends beyond the interpolation endpoint. */
static double carrierHz(const Pattern *pattern, const VvvfValues *status,
                        double control_hz)
{
    if (pattern->musical_carrier) {
        /* N1000 acceleration table. All nine notes have FreeRunStuckAtHere. */
        static const struct { double from_hz, carrier_hz; } notes[] = {
            {5.6, 400}, {5.0, 350}, {4.3, 311}, {3.4, 294}, {2.7, 262},
            {2.0, 233}, {1.5, 223}, {0.5, 196}, {-1, 175}
        };
        for (size_t i = 0; i < COUNT_OF(notes); ++i) {
            if (control_hz > notes[i].from_hz ||
                (status->free_run && status->sin_angle_freq >= notes[i].from_hz * M_2PI))
                return notes[i].carrier_hz;
        }
    }
    if (pattern->ramp) {
        const CarrierRamp *r = pattern->ramp;
        double position = (control_hz - r->from_hz) / (r->to_hz - r->from_hz);
        return r->from_value + position * (r->to_value - r->from_value);
    }
    return pattern->carrier.base_freq;
}

static double amplitudeAt(const AmplitudeCurve *curve, double hz)
{
    double value;
    /* An inherited end frequency can coincide with the start at a band
     * boundary. Allow for the radians/s round trip and use the endpoint. */
    double tolerance = 2 * DBL_EPSILON * (1 + curve->start_hz);
    if (curve->end_hz - curve->start_hz <= tolerance || hz >= curve->end_hz)
        value = curve->end_amp;
    else if (hz <= curve->start_hz || curve->start_amp == curve->end_amp)
        value = curve->start_amp;
    else {
        double t = (hz - curve->start_hz) / (curve->end_hz - curve->start_hz);
        if (curve->inverse) {
            double start = curve->start_amp + curve->curve_rate;
            double end = curve->end_amp + curve->curve_rate;
            value = 1.0 / ((1.0 - t) / start + t / end) - curve->curve_rate;
        } else {
            value = curve->start_amp + t * (curve->end_amp - curve->start_amp);
        }
    }
    if (value < curve->cutoff) return 0;
    if (curve->maximum > 0 && value > curve->maximum) return curve->maximum;
    return value;
}

static double freeRunAmplitude(const Pattern *pattern, bool power_off,
                               double max_hz, double control_hz, double rotation_hz)
{
    AmplitudeCurve curve = power_off ? pattern->power_off : pattern->power_on;
    /* -1 endpoints inherit the normal curve at the motor's rotation frequency. */
    if (curve.end_hz == -1) {
        curve.end_hz = rotation_hz < max_hz ? rotation_hz : max_hz;
        if (curve.end_hz > pattern->normal.end_hz) curve.end_hz = pattern->normal.end_hz;
    }
    if (curve.end_amp == -1 || curve.start_amp == -1) {
        double normal = amplitudeAt(&pattern->normal, rotation_hz);
        if (curve.end_amp == -1) curve.end_amp = normal;
        if (curve.start_amp == -1) curve.start_amp = normal;
    }
    return amplitudeAt(&curve, control_hz);
}

static const Pattern *selectPattern(const Direction *direction, const VvvfValues *status,
                                    double control_hz)
{
    unsigned state = NORMAL;
    if (status->free_run) state = status->mascon_off ? POWER_OFF : POWER_ON;
    unsigned hold = status->mascon_off ? HOLD_OFF : HOLD_ON;
    for (size_t i = 0; i < direction->count; ++i) {
        const Pattern *p = &direction->patterns[i];
        if (!(p->states & state)) continue;
        if (p->rotate_from >= 0 && status->sin_angle_freq < p->rotate_from * M_2PI) continue;
        if (p->rotate_below >= 0 && status->sin_angle_freq >= p->rotate_below * M_2PI) continue;
        if (control_hz >= p->from_hz ||
            (status->free_run && (p->states & hold) && status->sin_angle_freq > p->from_hz * M_2PI))
            return p;
    }
    return NULL;
}

static void calculateProfile(const Direction *direction, int level,
                             VvvfValues *status, PwmCalculateValues *pwm)
{
    *pwm = (PwmCalculateValues){
        .level = level, .dipolar = -1, .min_freq = direction->min_hz, .none = true
    };
    double rotation_hz = status->sin_angle_freq / M_2PI;
    double max_hz = status->mascon_off ? direction->off_max_hz : direction->on_max_hz;
    status->free_freq_change = status->mascon_off ? direction->off_rate : direction->on_rate;
    if (status->free_run && status->wave_stat > max_hz)
        status->wave_stat = status->mascon_off ? max_hz : rotation_hz;

    double control_hz = status->wave_stat;
    /* The Pi's mascon integrates wave_stat. Apply the electrical minimum locally
     * so it cannot advance the motor's speed command at low speed. */
    if (!status->free_run && control_hz > 0 && control_hz < direction->min_hz)
        control_hz = direction->min_hz;
    const Pattern *pattern = selectPattern(direction, status, control_hz);
    if (!pattern) {
        if (status->free_run) status->wave_stat = status->mascon_off ? 0 : rotation_hz;
        return;
    }
    pwm->pulse_mode = pattern->pulse;
    pwm->carrier_freq = pattern->carrier;
    pwm->carrier_freq.base_freq = carrierHz(pattern, status, control_hz);
    pwm->amplitude = status->free_run
        ? freeRunAmplitude(pattern, status->mascon_off, max_hz, control_hz, rotation_hz)
        : amplitudeAt(&pattern->normal, control_hz);
    pwm->none = control_hz <= 0 || pwm->amplitude <= 0;
    if (status->mascon_off && pwm->amplitude == 0) status->wave_stat = 0;
}

/* Pattern rows are ordered by descending control frequency, retaining YAML
 * order for equal thresholds. The first line of a row gives the control
 * threshold, rotation bounds, eligible states and pulse. The next three curves
 * are normal, power-on and power-off, each {start Hz, amplitude, end Hz, amplitude}. */

/* JRE_E231_HITACHI.yaml */
static const Pattern e231Power[] = {
    {67, -1, -1, ALL_STATES, {P_1, Default},
     {0, 0, 65, 1}, {0, 0, -1, -1}, {0, 0, -1, -1}},
    {60, -1, -1, ALL_STATES, {P_3, Alt1},
     {60, 0.7, 68, 1}, {60, 0, -1, -1}, {60, 0, -1, -1}},
    {49, -1, -1, ALL_STATES, {Async, Default},
     {0, 0, 65, 1.37471264}, {0, 0, -1, -1}, {0, 0, -1, -1},
     .ramp = &(const CarrierRamp){49, 710, 60, 1750}},
    {23, -1, -1, ALL_STATES, {Async, Default},
     {0, 0, 65, 1.37471264}, {0, 0, -1, -1}, {0, 0, -1, -1},
     .ramp = &(const CarrierRamp){23, 1045, 48.9, 710}},
    {0, -1, -1, ALL_STATES, {Async, Default},
     {0, 0, 65, 1.37471264}, {0, 0, -1, -1}, {0, 0, -1, -1},
     .carrier = {1045, 0, 0}},
};
static const Pattern e231Brake[] = {
    {73, -1, -1, ALL_STATES, {P_1, Default},
     {0, 0, 73, 1}, {0, 0, -1, -1}, {0, 0, -1, -1}},
    {67, -1, -1, ALL_STATES, {P_3, Alt1},
     {67, 0.7, 75, 0.85}, {67, 0, -1, -1}, {67, 0, -1, -1}},
    {56, -1, -1, ALL_STATES, {Async, Default},
     {0, 0, 73, 1.4}, {0, 0, -1, -1}, {0, 0, -1, -1},
     .ramp = &(const CarrierRamp){56, 700, 67, 1600}},
    {29, -1, -1, ALL_STATES, {Async, Default},
     {0, 0, 73, 1.4}, {0, 0, -1, -1}, {0, 0, -1, -1},
     .ramp = &(const CarrierRamp){29, 1045, 55.9, 700}},
    {0, -1, -1, ALL_STATES, {Async, Default},
     {0, 0, 73, 1.4}, {0, 0, -1, -1}, {0, 0, -1, -1},
     .carrier = {1045, 0, 0}},
};

static void e231Sound(VvvfValues *status, PwmCalculateValues *pwm)
{
    /* patterns, count, minimum Hz, jerk on/off (Hz/s), maximum control Hz on/off */
    static const Direction directions[] = {
        {e231Power, COUNT_OF(e231Power), -1, 50, 70, 67, 67},
        {e231Brake, COUNT_OF(e231Brake), -1, 50, 70, 73, 73},
    };
    calculateProfile(&directions[status->brake], 2, status, pwm);
}

/* JRE_209_MITSUBISHI.yaml uses three levels. Use its pulse schedule and
 * modulation curves with the board's two-level PWM for both display and GPIO. */
static const Pattern jre209Power[] = {
    {53, -1, -1, ALL_STATES | HOLD_ON | HOLD_OFF, {P_1, Default},
     .normal = {53, 0.473, 66, 3, .inverse = true},
     .power_on = {0, 0.0001, -1, -1, .cutoff = 0.01, .inverse = true, .curve_rate = 0.8},
     .power_off = {0, 0.0001, -1, -1, .cutoff = 0.1, .inverse = true, .curve_rate = 0.06}},
    {45, -1, -1, ALL_STATES | HOLD_ON | HOLD_OFF, {P_3, Default},
     {45, 0.695, 53, 0.84}, {0, 0, -1, -1}, {0, 0, -1, -1, .cutoff = 0.3}},
    {29, -1, -1, ALL_STATES | HOLD_ON | HOLD_OFF, {P_9, Default},
     {29, 0.494, 59, 1}, {0, 0, -1, -1}, {0, 0, -1, -1, .cutoff = 0.3}},
    {19, -1, -1, ALL_STATES | HOLD_ON | HOLD_OFF, {P_21, Default},
     {0, 0, 59, 1}, {0, 0, -1, -1}, {0, 0, -1, -1, .cutoff = 0.3}},
    {9, -1, -1, ALL_STATES | HOLD_ON | HOLD_OFF, {P_33, Default},
     {9, 0.167, 59, 1}, {0, 0, -1, -1}, {0, 0, -1, -1, .cutoff = 0.3}},
    {0, -1, -1, ALL_STATES | HOLD_ON | HOLD_OFF, {P_57, Default},
     {0, 0.02, 59, 1}, {0, 0, -1, -1}, {0, 0, -1, -1, .cutoff = 0.3}},
};
static const Pattern jre209Brake[] = {
    {60, -1, -1, ALL_STATES | HOLD_ON | HOLD_OFF, {P_1, Default},
     .normal = {60, 0.6, 72, 3, .inverse = true},
     .power_on = {0, 0.001, -1, -1, .cutoff = 0.01, .inverse = true, .curve_rate = 0.8},
     .power_off = {0, 0.0001, -1, -1, .cutoff = 0.1, .inverse = true, .curve_rate = 0.06}},
    {49, -1, -1, ALL_STATES | HOLD_ON | HOLD_OFF, {P_3, Default},
     {0, 0, 68.5, 1}, {0, 0, -1, -1}, {0, 0, -1, -1, .cutoff = 0.3}},
    {40, -1, -1, ALL_STATES | HOLD_ON | HOLD_OFF, {P_9, Default},
     {40, 0.725, 50, 0.805}, {0, 0, -1, -1}, {0, 0, -1, -1, .cutoff = 0.3}},
    {19, -1, -1, ALL_STATES | HOLD_ON | HOLD_OFF, {P_21, Default},
     {0, 0, 40, 0.715}, {0, 0, -1, -1}, {0, 0, -1, -1, .cutoff = 0.3}},
    {7, -1, -1, ALL_STATES | HOLD_ON | HOLD_OFF, {P_33, Default},
     {0, 0, 19, 0.34}, {0, 0, -1, -1}, {0, 0, -1, -1, .cutoff = 0.3}},
};

static void jre209Sound(VvvfValues *status, PwmCalculateValues *pwm)
{
    /* patterns, count, minimum Hz, jerk on/off (Hz/s), maximum control Hz on/off */
    static const Direction directions[] = {
        {jre209Power, COUNT_OF(jre209Power), 2, 60, 30, 53, 53},
        {jre209Brake, COUNT_OF(jre209Brake), -1, 60, 30, 60, 60},
    };
    calculateProfile(&directions[status->brake], 2, status, pwm);
}

/* N1000_SIEMENS_GTO.yaml */
static const Pattern n1000Power[] = {
    {80, -1, -1, ALL_STATES, {P_1, Default},
     {0, 1, 10, 1, .maximum = 1.25}, {0, 1, -1, -1, .maximum = 1.25}, {0, 1, -1, -1, .maximum = 1.25}},
    {59, -1, -1, ALL_STATES, {CHM_3, Alt1},
     {59, 1.239, 80, 1.706578125, .maximum = 1.25}, {59, 1.239, -1, -1, .maximum = 1.25}, {59, 1.239, -1, -1, .maximum = 1.25}},
    {57, -1, -1, POWER_ON | POWER_OFF | HOLD_ON | HOLD_OFF, {P_3, Default},
     {0, 0, 59, 0.978, .maximum = 1.25}, {0, 0, 59, 0.978, .maximum = 1.25}, {0, 0, 59, 0.978, .maximum = 1.25}},
    {57, -1, -1, ALL_STATES, {CHM_5, Alt1},
     .normal = {57, 1.19446875, 59, 1.239, .maximum = 1.25},
     .power_on = {57, 1.19446875, 59, 1.239, .maximum = 1.25},
     .power_off = {57, 1.19446875, 59, 1.239, .maximum = 1.25}},
    {53.5, -1, -1, ALL_STATES, {CHM_7, Alt2},
     .normal = {53.5, 1.1165390625, 57, 1.19446875, .maximum = 1.25},
     .power_on = {53.5, 1.1165390625, 57, 1.19446875, .maximum = 1.25},
     .power_off = {53.5, 1.1165390625, 57, 1.19446875, .maximum = 1.25}},
    {43.5, -1, -1, ALL_STATES, {CHM_7, Default},
     .normal = {43.5, 0.94083090378835, 53.5, 1.13412536442935, .maximum = 1.25},
     .power_on = {43.5, 0.94083090378835, 53.5, 1.13412536442935, .maximum = 1.25},
     .power_off = {43.5, 0.94083090378835, 53.5, 1.13412536442935, .maximum = 1.25}},
    {36.7, -1, -1, ALL_STATES, {CHM_9, Default},
     .normal = {36.7, 0.80939067055247, 43.5, 0.94083090378835, .maximum = 1.25},
     .power_on = {36.7, 0.80939067055247, 43.5, 0.94083090378835, .maximum = 1.25},
     .power_off = {36.7, 0.80939067055247, 43.5, 0.94083090378835, .maximum = 1.25}},
    {30, -1, -1, ALL_STATES, {CHM_11, Default},
     .normal = {30, 0.679883381923, 36.7, 0.80939067055247, .maximum = 1.25},
     .power_on = {30, 0.679883381923, 36.7, 0.80939067055247, .maximum = 1.25},
     .power_off = {30, 0.679883381923, 36.7, 0.80939067055247, .maximum = 1.25}},
    {27, -1, -1, ALL_STATES, {CHM_13, Default},
     .normal = {27, 0.6218950437307, 30, 0.679883381923, .maximum = 1.25},
     .power_on = {27, 0.6218950437307, 30, 0.679883381923, .maximum = 1.25},
     .power_off = {27, 0.6218950437307, 30, 0.679883381923, .maximum = 1.25}},
    {24, -1, -1, ALL_STATES, {CHM_15, Default},
     .normal = {24, 0.5639067055384, 27, 0.6218950437307, .maximum = 1.25},
     .power_on = {24, 0.5639067055384, 27, 0.6218950437307, .maximum = 1.25},
     .power_off = {24, 0.5639067055384, 27, 0.6218950437307, .maximum = 1.25}},
    {-1, 24, -1, POWER_ON | POWER_OFF, {Async_THI, Default},
     {0, 0.1, 24, 0.5639067055384, .maximum = 1.25}, {0, 0, -1, -1, .maximum = 1.25}, {0, 0, -1, -1, .maximum = 1.25},
     .carrier = {400, 0, 0}},
    {-1, -1, 24, ALL_STATES, {Async_THI, Default},
     .normal = {0, 0.1, 24, 0.5639067055384, .maximum = 1.25},
     .power_on = {0, 0, -1, -1, .maximum = 1.25},
     .power_off = {0, 0, -1, -1, .maximum = 1.25}, .musical_carrier = true},
};
static const Pattern n1000Brake[] = {
    {79.5, -1, -1, ALL_STATES, {P_1, Default},
     {60, 1, 72, 1, .maximum = 1.25}, {0, 1, 1, 1}, {0, 1, 1, 1}},
    {70.7, -1, -1, ALL_STATES, {CHM_3, Alt1},
     .normal = {70.7, 1.2102388, 79.5, 1.372722, .maximum = 1.25},
     .power_on = {70.7, 1.2102388, 79.5, 1.372722, .cutoff = 0.498, .maximum = 1.25},
     .power_off = {70.7, 1.2102388, 79.5, 1.372722, .cutoff = 0.498, .maximum = 1.25}},
    {63.35, -1, -1, POWER_ON | POWER_OFF | HOLD_ON | HOLD_OFF, {P_3, Default},
     {0, 0, 70.7, 0.957, .maximum = 1.25}, {0, 0, 70.7, 0.978, .maximum = 1.25}, {0, 0, 70.7, 0.978, .maximum = 1.25}},
    {63.35, -1, -1, ALL_STATES, {CHM_5, Alt1},
     .normal = {63.35, 1.0745284, 70.7, 1.2102388, .maximum = 1.25},
     .power_on = {63.35, 1.0745284, 70.7, 1.2102388, .maximum = 1.25},
     .power_off = {63.35, 1.0745284, 70.7, 1.2102388, .maximum = 1.25}},
    {56.84, -1, -1, ALL_STATES, {CHM_7, Alt2},
     .normal = {56.84, 0.95432776, 63.35, 1.0745284, .maximum = 1.25},
     .power_on = {56.84, 0.95432776, 63.35, 1.0745284, .maximum = 1.25},
     .power_off = {56.84, 0.95432776, 63.35, 1.0745284, .maximum = 1.25}},
    {53.5, -1, -1, ALL_STATES, {CHM_7, Default},
     .normal = {53.5, 0.8898727059955, 56.84, 0.93918438521092, .maximum = 1.25},
     .power_on = {53.5, 0.8898727059955, 56.84, 0.93918438521092, .maximum = 1.25},
     .power_off = {53.5, 0.8898727059955, 56.84, 0.93918438521092, .maximum = 1.25}},
    {41, -1, -1, ALL_STATES, {CHM_7, Default},
     .normal = {41, 0.653700980401, 53.5, 0.8225122549135, .maximum = 1.25},
     .power_on = {41, 0.653700980401, 53.5, 0.8225122549135, .maximum = 1.25},
     .power_off = {41, 0.653700980401, 53.5, 0.8225122549135, .maximum = 1.25}},
    {34.5, -1, -1, ALL_STATES, {CHM_9, Default},
     .normal = {34.5, 0.5659191176545, 41, 0.653700980401, .maximum = 1.25},
     .power_on = {34.5, 0.5659191176545, 41, 0.653700980401, .maximum = 1.25},
     .power_off = {34.5, 0.5659191176545, 41, 0.653700980401, .maximum = 1.25}},
    {28.9, -1, -1, ALL_STATES, {CHM_11, Default},
     .normal = {28.9, 0.4902916666729, 34.5, 0.5659191176545, .maximum = 1.25},
     .power_on = {28.9, 0.4902916666729, 34.5, 0.5659191176545, .maximum = 1.25},
     .power_off = {28.9, 0.4902916666729, 34.5, 0.5659191176545, .maximum = 1.25}},
    {24.9, -1, -1, ALL_STATES, {CHM_13, Default},
     .normal = {24.9, 0.4362720588289, 28.9, 0.4902916666729, .maximum = 1.25},
     .power_on = {24.9, 0.4362720588289, 28.9, 0.4902916666729, .maximum = 1.25},
     .power_off = {24.9, 0.4362720588289, 28.9, 0.4902916666729, .maximum = 1.25}},
    {22.4, -1, -1, ALL_STATES, {CHM_15, Default},
     .normal = {22.4, 0.4025098039264, 24.9, 0.4362720588289, .maximum = 1.25},
     .power_on = {22.4, 0.4025098039264, 24.9, 0.4362720588289, .maximum = 1.25},
     .power_off = {22.4, 0.4025098039264, 24.9, 0.4362720588289, .maximum = 1.25}},
    {4, -1, 22.4, ALL_STATES, {Async_THI, Default},
     .normal = {4, 0.154019607844, 22.4, 0.4025098039264, .maximum = 1.25},
     .power_on = {0, 0, -1, -1, .maximum = 1.25},
     .power_off = {0, 0, -1, -1, .maximum = 1.25},
     .carrier = {400, 0, 0}},
    {-1, 22.4, -1, POWER_ON | POWER_OFF, {Async_THI, Default},
     .normal = {4, 0.154019607844, 22.4, 0.4025098039264, .maximum = 1.25},
     .power_on = {0, 0, -1, -1, .maximum = 1.25},
     .power_off = {0, 0, -1, -1, .maximum = 1.25},
     .carrier = {400, 0, 0}},
};

static void n1000Sound(VvvfValues *status, PwmCalculateValues *pwm)
{
    /* patterns, count, minimum Hz, jerk on/off (Hz/s), maximum control Hz on/off */
    static const Direction directions[] = {
        {n1000Power, COUNT_OF(n1000Power), 0, 60, 100, 80, 80},
        {n1000Brake, COUNT_OF(n1000Brake), 0, 60, 100, 79.5, 79.5},
    };
    calculateProfile(&directions[status->brake], 2, status, pwm);
}

/* OSAKA-METRO_10A_HITACHI_IGBT.yaml */
static const Pattern osaka10aPower[] = {
    {45, -1, -1, ALL_STATES, {P_1, Default},
     {46, 0.101, 46.1, 0.102}, {-1, -1, -1, -1}, {-1, -1, -1, -1}},
    {43, -1, 45, ALL_STATES, {P_3, Alt1},
     {43, 0.833, 45.999, 0.992}, {43, 0.833, 45, 0.939}, {43, 0.833, 45, 0.939}},
    {43, 45, -1, POWER_OFF, {P_3, Alt1},
     {43, 0.833, 45.999, 0.992}, {43, 0.833, 45, 0.939}, {43, 0.833, 45, 0.939}},
    {36.5, -1, 45, ALL_STATES, {Async, Default},
     {36.5, 1.008, 42.999, 1.628}, {36.5, 1.008, -1, -1}, {36.5, 1.008, -1, -1},
     .ramp = &(const CarrierRamp){36.5, 675, 43, 1500}},
    {36.5, 45, -1, POWER_OFF, {Async, Default},
     {36.5, 1.008, 42.999, 1.628}, {36.5, 1.008, -1, -1}, {36.5, 1.008, -1, -1},
     .ramp = &(const CarrierRamp){36.5, 675, 43, 1500}},
    {32, 45, -1, POWER_ON, {P_3, Alt1},
     {32, 0.833, 45, 0.92}, {32, 0.833, 45, 0.939}, {43, 0.833, 45, 0.939}},
    {20, 45, -1, POWER_ON, {Async, Default},
     {20, 1.008, 32, 1.628}, {20, 1.008, -1, -1}, {36.5, 1.008, -1, -1},
     .ramp = &(const CarrierRamp){20, 675, 32, 1500}},
    {0, -1, 45, ALL_STATES, {Async, Default},
     {0, 0, 36.499, 1.008}, {0, 0, -1, -1}, {0, 0, -1, -1, .cutoff = 0.1},
     .carrier = {675, 150, 0.0002}},
    {0, 45, -1, POWER_ON, {Async, Default},
     {0, 0, 20, 1.008}, {0, 0, 20, 1.008}, {0, 0, -1, -1, .cutoff = 0.1},
     .carrier = {675, 150, 0.0002}},
    {0, 45, -1, POWER_OFF, {Async, Default},
     {0, 0, 36.499, 1.008}, {0, 0, -1, -1}, {0, 0, 36.499, -1, .cutoff = 0.1},
     .carrier = {675, 150, 0.0002}},
};
static const Pattern osaka10aBrake[] = {
    {67, -1, -1, ALL_STATES, {P_1, Default},
     {70, 0.101, 70.1, 0.001}, {-1, -1, -1, -1}, {-1, -1, -1, -1}},
    {63, -1, 67, ALL_STATES, {P_3, Alt1},
     {63, 0.794, 69.999, 0.992}, {63, 0.794, 67, 0.872}, {63, 0.794, 67, 0.872}},
    {63, 67, -1, POWER_OFF, {P_3, Alt1},
     {63, 0.794, 69.999, 0.992}, {63, 0.794, 67, 0.872}, {63, 0.794, 67, 0.872}},
    {53.5, -1, 67, ALL_STATES, {Async, Default},
     {53.5, 0.97, 62.999, 1.348}, {53.5, 0.97, -1, -1}, {53.5, 0.97, -1, -1},
     .ramp = &(const CarrierRamp){53.5, 675, 63, 1440}},
    {53.5, 67, -1, POWER_OFF, {Async, Default},
     {53.5, 0.97, 62.999, 1.348}, {53.5, 0.97, -1, -1}, {53.5, 0.97, -1, -1},
     .ramp = &(const CarrierRamp){53.5, 675, 63, 1440}},
    {48, 67, -1, POWER_ON, {P_3, Alt1},
     {48, 0.794, 67, 0.9}, {48, 0.794, 67, 0.92}, {63, 0.794, 67, 0.872}},
    {30, 67, -1, POWER_ON, {Async, Default},
     {30, 0.97, 48, 1.348}, {30, 0.97, -1, -1}, {53.5, 0.97, -1, -1},
     .ramp = &(const CarrierRamp){30, 675, 48, 1440}},
    {4.5, -1, 67, ALL_STATES, {Async, Default},
     {4.5, 0.081, 53.499, 0.97}, {4.5, 0.081, -1, -1}, {4.5, 0.081, -1, -1},
     .carrier = {675, 150, 0.0002}},
    {3.5, -1, 67, NORMAL, {Async, Default},
     {3.5, 0.063, 4.499, 0.08}, {3.5, 0.063, -1, -1}, {3.5, 0.063, -1, -1}, .carrier = {.interval = 0.0002},
     .ramp = &(const CarrierRamp){3.5, 222, 4.5, 250}},
    {0, 67, -1, POWER_OFF, {Async, Default},
     {0, 0, 53.499, 0.97}, {0, 0, -1, -1}, {0, 0, -1, -1},
     .carrier = {675, 150, 0.0002}},
    {0, 67, -1, POWER_ON, {Async, Default},
     {0, 0, 30, 0.97}, {0, 0, 30, 0.97}, {0, 0, -1, -1},
     .carrier = {675, 150, 0.0002}},
    {0, -1, 67, POWER_ON | POWER_OFF, {Async, Default},
     {0, 0, 53.499, 0.97}, {0, 0, -1, -1}, {0, 0, -1, -1},
     .carrier = {675, 150, 0.0002}},
};

static void osaka10aSound(VvvfValues *status, PwmCalculateValues *pwm)
{
    /* patterns, count, minimum Hz, jerk on/off (Hz/s), maximum control Hz on/off */
    static const Direction directions[] = {
        {osaka10aPower, COUNT_OF(osaka10aPower), -1, 50, 60, 45.5, 45.5},
        {osaka10aBrake, COUNT_OF(osaka10aBrake), -1, 50, 60, 67, 67},
    };
    calculateProfile(&directions[status->brake], 2, status, pwm);
}

VvvfSoundFunction vvvf_sounds[] = {
    e231Sound, jre209Sound, n1000Sound, osaka10aSound
};
const char *vvvf_sound_names[] = {
    "JRE E231 HITACHI",
    "JRE 209 MITSUBISHI (2L)",
    "N1000 SIEMENS GTO",
    "OSAKA METRO 10A HITACHI IGBT"
};
_Static_assert(COUNT_OF(vvvf_sounds) == COUNT_OF(vvvf_sound_names), "Each sound needs a name");
const int vvvf_sounds_len = COUNT_OF(vvvf_sounds);

const char *getVvvfSoundName(VvvfSoundFunction sound)
{
    for (int i = 0; i < vvvf_sounds_len; ++i)
        if (vvvf_sounds[i] == sound) return vvvf_sound_names[i];
    return "UNKNOWN";
}
