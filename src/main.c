#include <stdint.h>
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

#include "dma.h"

void screenDMA(uint32_t source_addr, uint32_t dest_addr, uint32_t length)
{
	static DMA_ControlBlock dma_control_block = {0};
	DMA_CH_0.control_status |= 1 << 31; /* set reset */
	dma_control_block.transfer_information = 0;
	dma_control_block.transfer_information |= 1 << 26 | 1 << 9 | 1 << 8 | 1 << 5 | 1 << 4 | 1 << 0; /* no wide bursts, src increment, dest increment, interrupt bit */
	dma_control_block.source_addr = source_addr;
	dma_control_block.dest_addr = dest_addr;
	dma_control_block.next_control_block = 0;
	dma_control_block.stride = 0;
	dma_control_block.transfer_length = length;
	dma_control_block.dummy0 = 0;
	dma_control_block.dummy1 = 0;
	DMA_CH_0.control_status |= 1 << 29;
	DMA_CH_0.control_block_addr = (uint32_t)&dma_control_block | 0xC0000000;
	DMA_CH_0.control_status |= 1 << 0;
	while (!(DMA_CH_0.control_status & (1 << 2)))
		;
	DMA_CH_0.control_status |= 1 << 2;
}

void initializeDMA()
{
	DMA_ENABLE |= 1 << 0; /* enable DMA ch 0 */
}

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
 * is not the notch on the throttle while EMO is latched. Single bytes, written
 * by one task and read by another, so no locking is needed.
 */
static volatile char activeNotch = MASCON_NOTCH_EB;
static volatile bool emoLatched = false;
static volatile bool reverseActive = false;

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
		if (!readEmoValue()) emoLatched = true;
		char effectiveNotch = emoLatched ? MASCON_NOTCH_EB : masconValue;
		const MasconNotch *notch = &mascon_notches[(int)effectiveNotch];
		activeNotch = effectiveNotch;

		requestStatusVvvfGpio();
		while (!canGetStatusVvvfGpio())
			;
		VvvfValues gpioStatus = getStatusVvvfGpio();

		// Reverse may only flip while the machine is stopped and the throttle
		// is physically at EB, so the field never reverses under load.
		if (gpioStatus.sin_angle_freq == 0 && gpioStatus.wave_stat == 0 && masconValue == MASCON_NOTCH_EB)
		{
			bool wantReverse = readRevsValue() ? true : false;
			if (wantReverse != reverseActive)
			{
				reverseActive = wantReverse;
				setReverseVvvfGpio(wantReverse);
			}
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

int screenBufferRenderPos = 0;
uint16_t screenBuffer[640*360];
char screenBufferWait = 0;

void processScreenBufferRender(){
	if(screenBufferWait == 0) return;
	*(uint16_t *)((frameBufferAddress & ~0xC0000000) + screenBufferRenderPos * 2) = screenBuffer[screenBufferRenderPos];
	screenBufferRenderPos ++;
	if(screenBufferRenderPos >= 640*360) {
		screenBufferRenderPos = 0;
		screenBufferWait = 0;
	}
}

void taskDisplay(void *param)
{
	signed char buff[DASH_GRAPH_W];

	windowInitializeDashboard(screenBuffer);

	while (1)
	{
		requestStatusVvvfGpio();
		while (!canGetStatusVvvfGpio())
			;
		VvvfValues displayStatus = getStatusVvvfGpio();
		VvvfSoundFunction gpioSound = (VvvfSoundFunction)getSoundVvvfGpio();
		displayStatus.sin_time = 0;
		displayStatus.saw_time = 0;
		displayStatus.allow_random_freq_move = false;

		for (int i = 0; i < DASH_GRAPH_W; i++)
		{
			displayStatus.sin_time = (double)i / DASH_GRAPH_W / 30.0;
			displayStatus.saw_time = (double)i / DASH_GRAPH_W / 30.0;
			displayStatus.generation_current_time = (double)i / DASH_GRAPH_W / 30.0;
			PhaseStatus U, V;
			calculatePhases(&U, &V, 0, &displayStatus, gpioSound);
			buff[i] = (signed char)V - (signed char)U;
		}

		// VOLT is the fundamental's amplitude from a one period Fourier probe;
		// meaningless while stopped (the probe would divide by zero), so the
		// box shows "---" instead, as does PULS.
		bool running = displayStatus.v_sin_angle_freq != 0;
		double b_1 = 0;
		if (running)
		{
			displayStatus.sin_time = 0;
			displayStatus.saw_time = 0;
			for (int i = 0; i < 5000; i++)
			{
				displayStatus.sin_time = (double)i / (5000 * displayStatus.v_sin_angle_freq * M_1_2PI);
				displayStatus.saw_time = (double)i / (5000 * displayStatus.v_sin_angle_freq * M_1_2PI);
				displayStatus.generation_current_time = (double)i / (5000 * displayStatus.v_sin_angle_freq * M_1_2PI);

				PhaseStatus U, V;
				calculatePhases(&U, &V, 0, &displayStatus, gpioSound);
				signed char val = (signed char)U - (signed char)V;
				b_1 += val * sin(M_2PI * i / 5000);
			}
			b_1 /= 1.1026577908425 * 5000;
			b_1 *= 100;
		}

		// The PULS box wants the pwm the sound profile resolves at the current
		// frequency: carrier Hz when async, pulse count when synchronous. The
		// profile may scribble on the status it is given, hence the copy.
		VvvfValues pwmStatus = displayStatus;
		PwmCalculateValues pwm;
		gpioSound(&pwmStatus, &pwm);

		windowUpdateFreq(screenBuffer, displayStatus.v_sin_angle_freq * M_1_2PI);
		windowUpdateVolt(screenBuffer, b_1, running);
		windowUpdatePuls(screenBuffer, pwm.pulse_mode.pulse_name, pwm.carrier_freq.base_freq, running && !pwm.none);

		// Notch readout. This is the notch actually in force, which is EB while
		// EMO is latched no matter where the throttle sits.
		windowUpdateNotch(screenBuffer, getMasconNotchName(activeNotch), getMasconNotchColour(activeNotch));

		// State lamps. EMO blinks once a second while latched so a tripped stop
		// cannot be mistaken for a lamp that merely failed on; REVS just lights.
		bool emoLit = emoLatched && ((timer_getTickCount64() / LAMP_EMO_BLINK_US) & 1ULL);
		windowUpdateLamps(screenBuffer, emoLit, reverseActive);

		int soundIndex = 0;
		for (int i = 0; i < vvvf_sounds_len; i++)
			if (vvvf_sounds[i] == gpioSound) soundIndex = i;
		windowUpdateSoundBar(screenBuffer, getVvvfSoundName(gpioSound), soundIndex + 1, vvvf_sounds_len);

		for (int x = 0; x < DASH_GRAPH_W; x++)
		{
			int col = DASH_GRAPH_X + x;

			// Background Fill
			for (int y = DASH_GRAPH_Y; y < DASH_GRAPH_Y + DASH_GRAPH_H; y++)
				screenBuffer[col + 640 * y] = COLOR_DASH_PANEL;

			// Base Line
			screenBuffer[col + 640 * DASH_GRAPH_BASE_Y] = COLOR_WAVEFORM_BASE;

			// Draw graph on x
			int y_diff = (buff[x] - buff[(x + 1) == DASH_GRAPH_W ? x : x + 1]) * 40;
			int abs_y_diff = y_diff < 0 ? -y_diff : y_diff;
			int y_start = buff[x] * 40 + DASH_GRAPH_BASE_Y;

			while (abs_y_diff >= 0)
			{
				screenBuffer[col + 640 * (y_start + abs_y_diff * (y_diff > 0 ? -1 : 1))] = COLOR_WAVEFORM;
				abs_y_diff--;
			}
		}

		screenBufferWait = 1;
		timer_wait(50000);
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
	initializeWindow(640, 360, 800, 480, 16); // Auto resolution console, message to screen
	initializeDMA();
	windowDrawBootScreen(screenBuffer, "starting... vvvf tasks");
	screenDMA((uint32_t)screenBuffer, frameBufferAddress & ~0xC0000000, 640 * 360 * 2);
#endif

	xRTOS_Init(); // Initialize the xRTOS system .. done before any other xRTOS call
#ifdef ENABLE_DISPLAY
	xTaskCreate(0, taskDisplay, "Disp", 1024, NULL, 1, NULL);
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
