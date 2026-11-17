#include <stdint.h>
#include <stdatomic.h>
#include <string.h>
#include <math.h>
#include "rpi-smartstart.h"
#include "QA7.h"
#include "xRTOS.h"
#include "task.h"
#include "windows.h"
#include "semaphore.h"

#include "vvvf-hardware.h"
#include "vvvf-sounds.h"
#include "vvvf-struct.h"
#include "vvvf-main.h"
#include "pi-values.h"

/* Waveform trace colours inside the graph panel. */
#define COLOR_WAVEFORM 0xFFCB
#define COLOR_WAVEFORM_BASE 0x9492

/*
 * Notch readout colours, by stage. RGB565, so each is the requested 24 bit
 * colour with the low 3/2/3 bits dropped.
 */
#define COLOR_NOTCH_EB 0xF800			/* #FF0000 */
#define COLOR_NOTCH_BRAKE 0xFC00		/* #FF8000 */
#define COLOR_NOTCH_NEUTRAL 0x07E0		/* #00FF00 */
#define COLOR_NOTCH_POWER 0x001F		/* #0000FF */

/* Toggle period of the EMO lamp, half of a full on/off cycle. */
#define LAMP_EMO_BLINK_US 1000000ULL

/*
 * Mascon notches, indexed by the 4 bit value from readMasconValue()
 * (mascon_1 = bit0 ... mascon_4 = bit3, so 0 ~ 15).
 *
 *   0       EB      emergency brake
 *   1 ~ 8   B8 ~ B1 service brake, 1 is the strongest
 *   9       N       neutral, coasts on the sound profile's jerk setting
 *   10      P0      powered but holding speed, not neutral
 *   11 ~ 15 P1 ~ P5 power
 *
 * freqRate is [Hz / sec]: negative decelerates, positive accelerates, zero
 * holds. Each ceiling below is split evenly over its notches, so P<n> is
 * MASCON_MAX_ACCEL * n / 5 and B<n> is MASCON_MAX_BRAKE * n / 8. The cart is
 * light, so braking sits well above power and EB a step above that. Retune
 * the three constants rather than the table.
 */
#define MASCON_MAX_ACCEL 4.0
#define MASCON_MAX_BRAKE (MASCON_MAX_ACCEL * 3.0)
#define MASCON_EB_BRAKE (MASCON_MAX_ACCEL * 5.0)

typedef struct
{
	const char *name;
	uint16_t colour;	// notch readout colour on the display
	double freqRate;	// [Hz / sec]
	bool neutral;		// hand the frequency over to the sound profile
	bool brake;			// ignored while neutral
} MasconNotch;

static const MasconNotch mascon_notches[16] = {
	{ "EB", COLOR_NOTCH_EB,      -MASCON_EB_BRAKE,               false, true  },
	{ "B8", COLOR_NOTCH_BRAKE,   -MASCON_MAX_BRAKE * 8.0 / 8.0,  false, true  },
	{ "B7", COLOR_NOTCH_BRAKE,   -MASCON_MAX_BRAKE * 7.0 / 8.0,  false, true  },
	{ "B6", COLOR_NOTCH_BRAKE,   -MASCON_MAX_BRAKE * 6.0 / 8.0,  false, true  },
	{ "B5", COLOR_NOTCH_BRAKE,   -MASCON_MAX_BRAKE * 5.0 / 8.0,  false, true  },
	{ "B4", COLOR_NOTCH_BRAKE,   -MASCON_MAX_BRAKE * 4.0 / 8.0,  false, true  },
	{ "B3", COLOR_NOTCH_BRAKE,   -MASCON_MAX_BRAKE * 3.0 / 8.0,  false, true  },
	{ "B2", COLOR_NOTCH_BRAKE,   -MASCON_MAX_BRAKE * 2.0 / 8.0,  false, true  },
	{ "B1", COLOR_NOTCH_BRAKE,   -MASCON_MAX_BRAKE * 1.0 / 8.0,  false, true  },
	{ "N",  COLOR_NOTCH_NEUTRAL,  0.0,                           true,  false },
	{ "P0", COLOR_NOTCH_POWER,    0.0,                           false, false },
	{ "P1", COLOR_NOTCH_POWER,    MASCON_MAX_ACCEL * 1.0 / 5.0,  false, false },
	{ "P2", COLOR_NOTCH_POWER,    MASCON_MAX_ACCEL * 2.0 / 5.0,  false, false },
	{ "P3", COLOR_NOTCH_POWER,    MASCON_MAX_ACCEL * 3.0 / 5.0,  false, false },
	{ "P4", COLOR_NOTCH_POWER,    MASCON_MAX_ACCEL * 4.0 / 5.0,  false, false },
	{ "P5", COLOR_NOTCH_POWER,    MASCON_MAX_ACCEL * 5.0 / 5.0,  false, false },
};

#define MASCON_NOTCH_EB 0

/*
 * Published by taskMascon for the display: the notch actually in force, which
 * is not the notch on the throttle while EMO is latched.
 */
static atomic_char activeNotch = MASCON_NOTCH_EB;
static atomic_bool emoLatched = false;

// Notch label for the display, e.g. "B7" / "N" / "P3".
const char *getMasconNotchName(char value)
{
	if (value < 0 || value > 15) return "??";
	return mascon_notches[(int)value].name;
}

// Notch colour for the display: EB red, brake amber, neutral green, power blue.
uint16_t getMasconNotchColour(char value)
{
	if (value < 0 || value > 15) return COLOR_DASH_TEXT;
	return mascon_notches[(int)value].colour;
}

void taskMascon(void *param)
{
	const char vvvf_sound_len = vvvf_sounds_len;
	char current_vvvf_sound = 0;
	uint64_t _dt = 0, _s;
	while (1)
	{
		_s = timer_getTickCount64();

		char masconValue = readMasconValue();
		if (masconValue < 0) masconValue = 0;
		if (masconValue > 15) masconValue = 15;

		// The emergency stop line is normally closed: HIGH while healthy. It
		// latches the moment it goes LOW, whether that is the button being hit
		// or the loop being cut, and only a reboot clears it. From then on the
		// throttle is ignored and EB is held.
		if (!readEmoValue()) atomic_store_explicit(&emoLatched, true, memory_order_relaxed);
		char effectiveNotch = atomic_load_explicit(&emoLatched, memory_order_relaxed) ? MASCON_NOTCH_EB : masconValue;
		const MasconNotch *notch = &mascon_notches[(int)effectiveNotch];
		atomic_store_explicit(&activeNotch, effectiveNotch, memory_order_relaxed);

		requestStatusVvvfGpio();
		while (!canGetStatusVvvfGpio())
			;
		VvvfValues gpioStatus = getStatusVvvfGpio();

		// Reverse may only flip while the machine is stopped and the throttle
		// is physically at EB, so the field never reverses under load.
		if (gpioStatus.sin_angle_freq == 0 && gpioStatus.wave_stat == 0 && masconValue == MASCON_NOTCH_EB)
		{
			bool wantReverse = readRevsValue() ? true : false;
			if (wantReverse != getReverseVvvfGpio()) setReverseVvvfGpio(wantReverse);
		}

		if (!readButtonR() && gpioStatus.sin_angle_freq == 0)
		{
			current_vvvf_sound++;
			if (current_vvvf_sound >= vvvf_sound_len)
				current_vvvf_sound = 0;
			setSoundVvvfGpio(vvvf_sounds[(int)(current_vvvf_sound)]);
			while (!readButtonR())
				;
		}

		double _changedFreq = gpioStatus.wave_stat + notch->freqRate * _dt / 1000000.0;
		if (_changedFreq < 0)
			_changedFreq = 0;
		if (_changedFreq > 100)
			_changedFreq = 100;

		if (notch->neutral)
			gpioStatus.mascon_off = true;	// keep brake as it was, like the old neutral
		else
		{
			gpioStatus.brake = notch->brake;
			gpioStatus.mascon_off = false;
		}

		double gpioStatusOldAngleFreq = gpioStatus.sin_angle_freq;
		if (!gpioStatus.free_run)
		{
			gpioStatus.wave_stat = _changedFreq;
			gpioStatus.sin_angle_freq = _changedFreq * M_2PI;
			if (gpioStatus.allow_sine_time_change)
				gpioStatus.sin_time *= ((gpioStatus.sin_angle_freq == 0) ? 1 : (gpioStatusOldAngleFreq / gpioStatus.sin_angle_freq));
		}

		setStatusVvvfGpio(&gpioStatus);

		while (timer_getTickCount64() - _s < 5000)
			;
		_dt = timer_getTickCount64() - _s;
	}
}
#ifdef ENABLE_DISPLAY

enum {
	DISPLAY_WIDTH = 640, DISPLAY_HEIGHT = 360,
	VOLT_SAMPLES = 5000, GRAPH_SCALE = 40, DISPLAY_PERIOD_US = 50000
};

static uint16_t screenBuffer[DISPLAY_WIDTH * DISPLAY_HEIGHT];
static double voltWeights[VOLT_SAMPLES];

/* Keep the previous image in ordinary cached RAM. After the boot screen,
 * only changed RGB565 pixels need a store to the noncached GPU framebuffer. */
static void presentScreenBuffer(void)
{
	static uint16_t previous[DISPLAY_WIDTH * DISPLAY_HEIGHT];
	static bool presented;
	uintptr_t address = frameBufferAddress & ~0xC0000000u;
	if (!address || frameBufferPitch < DISPLAY_WIDTH * sizeof(uint16_t)) return;

	for (int y = 0; y < DISPLAY_HEIGHT; ++y) {
		volatile uint16_t *row = (volatile uint16_t *)(address + y * frameBufferPitch);
		int offset = y * DISPLAY_WIDTH;
		if (presented && memcmp(&screenBuffer[offset], &previous[offset], DISPLAY_WIDTH * sizeof(uint16_t)) == 0)
			continue;
		for (int x = 0; x < DISPLAY_WIDTH; ++x) {
			uint16_t pixel = screenBuffer[offset + x];
			if (!presented || pixel != previous[offset + x]) {
				row[x] = pixel;
				previous[offset + x] = pixel;
			}
		}
	}
	presented = true;
}

typedef struct {
	VvvfValues status;
	VvvfSoundFunction sound;
	PwmCalculateValues pwm;
	double resolvedControlHz;
	bool controlOnly;
} DisplaySignal;

static void resolveDisplayPwm(DisplaySignal *signal)
{
	if (signal->controlOnly && signal->resolvedControlHz == signal->status.wave_stat) return;
	/* A free-run profile may change wave_stat. In that case the next sample
	 * needs a new lookup, even though this is still the same display frame. */
	signal->resolvedControlHz = signal->status.wave_stat;
	signal->sound(&signal->status, &signal->pwm);
}

static signed char sampleDisplayVoltage(DisplaySignal *signal, double time)
{
	signal->status.sin_time = time;
	signal->status.saw_time = time;
	signal->status.generation_current_time = time;
	resolveDisplayPwm(signal);
	PhaseStatus u, v;
	calculatePwmPhases(&u, &v, NULL, &signal->status, &signal->pwm);
	return (signed char)u - (signed char)v;
}

static void drawWaveform(const signed char *samples)
{
	/* U and V each range from 0 to 2, so the line voltage spans ±2. */
	for (int y = DASH_GRAPH_BASE_Y - 2 * GRAPH_SCALE; y <= DASH_GRAPH_BASE_Y + 2 * GRAPH_SCALE; ++y) {
		uint16_t *row = screenBuffer + y * DISPLAY_WIDTH + DASH_GRAPH_X;
		uint16_t colour = y == DASH_GRAPH_BASE_Y ? COLOR_WAVEFORM_BASE : COLOR_DASH_PANEL;
		for (int x = 0; x < DASH_GRAPH_W; ++x) row[x] = colour;
	}
	for (int x = 0; x < DASH_GRAPH_W; ++x) {
		int y = DASH_GRAPH_BASE_Y + samples[x] * GRAPH_SCALE;
		int next = x + 1 < DASH_GRAPH_W ? DASH_GRAPH_BASE_Y + samples[x + 1] * GRAPH_SCALE : y;
		int top = y < next ? y : next;
		int bottom = y > next ? y : next;
		for (int line = top; line <= bottom; ++line)
			screenBuffer[line * DISPLAY_WIDTH + DASH_GRAPH_X + x] = COLOR_WAVEFORM;
	}
}

static void updateWaveform(DisplaySignal *signal)
{
	signed char samples[DASH_GRAPH_W];
	for (int i = 0; i < DASH_GRAPH_W; ++i)
		samples[i] = -sampleDisplayVoltage(signal, (double)i / DASH_GRAPH_W / 30.0);

	bool running = signal->status.v_sin_angle_freq != 0;
	double voltage = 0;
	if (running) {
		/* One period on the same 5,000-point Fourier grid as the dashboard scale. */
		for (int i = 0; i < VOLT_SAMPLES; ++i) {
			double time = (double)i / (VOLT_SAMPLES * signal->status.v_sin_angle_freq * M_1_2PI);
			voltage += sampleDisplayVoltage(signal, time) * voltWeights[i];
		}
		voltage /= 1.1026577908425 * VOLT_SAMPLES;
		voltage *= 100;
	}

	double freqHz = signal->status.v_sin_angle_freq * M_1_2PI;
	resolveDisplayPwm(signal);
	windowUpdateFreq(screenBuffer, freqHz);
	windowUpdateVolt(screenBuffer, voltage, running);
	windowUpdatePuls(screenBuffer, signal->pwm.pulse_mode.pulse_name, signal->pwm.carrier_freq.base_freq, running && !signal->pwm.none);

	drawWaveform(samples);
}

static bool sameDisplayControls(const VvvfValues *previous, const VvvfValues *current)
{
	return previous->sin_angle_freq == current->sin_angle_freq
	    && previous->wave_stat == current->wave_stat && previous->brake == current->brake
	    && previous->free_run == current->free_run && previous->mascon_off == current->mascon_off;
}

void taskDisplay(void *param)
{
	VvvfValues previousControls = {0};
	VvvfSoundFunction previousSound = NULL;
	int previousNotch = -1;
	bool previousEmo = false, previousReverse = false;
	for (int i = 0; i < VOLT_SAMPLES; ++i)
		voltWeights[i] = sin(M_2PI * i / VOLT_SAMPLES);
	windowInitializeDashboard(screenBuffer);

	while (1)
	{
		uint64_t frameStart = timer_getTickCount64();
		DisplaySignal signal;
		signal.status = getDisplayVvvfGpio(&signal.sound);
		signal.status.allow_random_freq_move = false;
		signal.controlOnly = vvvfSoundUsesControlStateOnly(signal.sound);
		signal.resolvedControlHz = NAN;

		char notch = atomic_load_explicit(&activeNotch, memory_order_relaxed);
		bool reverse = getReverseVvvfGpio();
		bool emo = atomic_load_explicit(&emoLatched, memory_order_relaxed) && ((frameStart / LAMP_EMO_BLINK_US) & 1ULL);
		bool soundChanged = signal.sound != previousSound;
		bool waveformChanged = soundChanged || reverse != previousReverse || !signal.controlOnly
			|| !sameDisplayControls(&previousControls, &signal.status);
		bool lampsChanged = emo != previousEmo || reverse != previousReverse;
		if (waveformChanged) {
			previousControls = signal.status;
			updateWaveform(&signal);
		}
		if (notch != previousNotch)
			windowUpdateNotch(screenBuffer, getMasconNotchName(notch), getMasconNotchColour(notch));
		if (lampsChanged) windowUpdateLamps(screenBuffer, emo, reverse);
		if (soundChanged) {
			int soundIndex = 0;
			for (int i = 0; i < vvvf_sounds_len; ++i)
				if (vvvf_sounds[i] == signal.sound) soundIndex = i;
			windowUpdateSoundBar(screenBuffer, getVvvfSoundName(signal.sound), soundIndex + 1, vvvf_sounds_len);
		}
		if (waveformChanged || lampsChanged || notch != previousNotch)
			presentScreenBuffer();
		previousSound = signal.sound;
		previousNotch = notch;
		previousEmo = emo;
		previousReverse = reverse;
		uint64_t elapsed = timer_getTickCount64() - frameStart;
		if (elapsed < DISPLAY_PERIOD_US) timer_wait(DISPLAY_PERIOD_US - elapsed);
	}
}
#endif

void noTask(void *param)
{
	while (1)
	{
		__asm__("nop;\n");
	}
}

void flickTask(void *param)
{
	while (1)
	{
		Flash_Debug1();
		timer_wait(2000);
	}
}

void main(void)
{
	Flash_LED();
	initializeVvvfHardware(); // Initialize vvvf relating pins
	setSoundVvvfGpio(vvvf_sounds[0]);

#ifdef ENABLE_DISPLAY
	bool displayReady = initializeWindow(DISPLAY_WIDTH, DISPLAY_HEIGHT, 800, 480, 16);
	if (displayReady) {
		windowDrawBootScreen(screenBuffer, "starting... vvvf tasks");
		presentScreenBuffer();
	}
#endif

	xRTOS_Init(); // Initialize the xRTOS system .. done before any other xRTOS call
#ifdef ENABLE_DISPLAY
	xTaskCreate(0, displayReady ? taskDisplay : noTask, "Disp", 1024, NULL, 1, NULL);
#else
	xTaskCreate(0, noTask, "DispNop", 1024, NULL, 1, NULL);
#endif
	xTaskCreate(1, taskCalculationPhases, "Vvvf", 1024, NULL, 1, NULL);
	xTaskCreate(2, taskMascon, "Mascon", 1024, NULL, 1, NULL);
	xTaskCreate(3, noTask, "3-nop", 1024, NULL, 1, NULL);

	Flash_LED();
	timer_wait(5000000);
	Flash_LED();

	xTaskStartScheduler();
}
