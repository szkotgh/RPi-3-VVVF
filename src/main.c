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
#include "icon.h"
#include "character.h"
#include "Font8x16.h"

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

void DrawRoundRect(uint16_t *buff, int R, int x, int y, int width, int height, uint16_t col)
{
	{ // Fill easy area.
		int _fill_x = width - 2 * R;
		int _fill_y = height;
		while (_fill_y > 0)
		{
			_fill_y--;
			_fill_x = width - 2 * R;
			while (_fill_x > 0)
			{
				_fill_x--;
				buff[(y + _fill_y) * frameBufferWidth + x + R + _fill_x] = col;
			}
		}
	}

	for (int i = 0; i <= R; i++)
	{ // ROUND of 20 px
		// LEFT TOP
		int circleL = (int)round(sqrt(-i * i + 2 * R * i));
		int circleR = (int)round(sqrt(R * R - i * i));

		int _fill_y = height - 2 * (R - circleL);
		while (_fill_y > 0)
		{
			_fill_y--;
			buff[(y + R - circleL + _fill_y) * frameBufferWidth + x + i] = col;
		}

		_fill_y = height - 2 * (R - circleR);
		while (_fill_y > 0)
		{
			_fill_y--;
			buff[(y + R - circleR + _fill_y) * frameBufferWidth + x + width - R + i] = col;
		}
	}
}

void DrawMonoCharacter(uint16_t *buff, long *data, int width, int height, int x, int y, uint16_t col, uint16_t fill)
{
	int _t = 0;
	while (_t < width * height)
	{
		int _index = _t / 32;
		int _bit = _t % 32;
		int _x = _t % width;
		int _y = _t / width;
		if (((data[_index] << _bit) & 0x80000000))
		{
			buff[x + _x + frameBufferWidth * (y + _y)] = col;
		}
		else
		{
			if (fill != 0){
				buff[x + _x + frameBufferWidth * (y + _y)] = fill;
			}
		}
		_t++;
	}
}

/*
 * Draws ASCII text with the 8x16 BitFont, scaled up by `scale`.
 * Pass fill = 0 to leave the glyph background untouched.
 *
 * BitFont packs the 16 row bytes of a glyph into 4 uint32 words, most
 * significant byte first, so the row byte cannot be read by casting the
 * word array to uint8_t on this little endian target.
 */
void DrawText(uint16_t *buff, const char *text, int x, int y, int scale, uint16_t col, uint16_t fill)
{
	for (const char *p = text; *p != 0; p++, x += 8 * scale)
	{
		if (x < 0 || x + 8 * scale > (int)frameBufferWidth) continue;
		if (y < 0 || y + 16 * scale > (int)frameBufferHeight) continue;

		for (int row = 0; row < 16; row++)
		{
			uint32_t word = BitFont[(unsigned char)*p * 4 + (row >> 2)];
			uint8_t bits = (uint8_t)(word >> (24 - 8 * (row & 3)));

			for (int bit = 0; bit < 8; bit++)
			{
				uint16_t c = (bits & (0x80 >> bit)) ? col : fill;
				if (c == 0) continue;

				for (int sy = 0; sy < scale; sy++)
					for (int sx = 0; sx < scale; sx++)
						buff[x + bit * scale + sx + frameBufferWidth * (y + row * scale + sy)] = c;
			}
		}
	}
}

void DrawNumber(uint16_t *buff, int value, int x, int y, int width, int height, int max_digit, bool zero_fill, bool center, uint16_t col, uint16_t fill, uint16_t fill_space)
{
	int original_x = x;
	int digit = zero_fill ? max_digit : (value <= 0 ? 1 : (log10(value) + 1));
	if (center)
	{
		x += (width - character_num_width * digit) / 2;
		y += (height - character_num_height) / 2;
	}

	if (fill_space != 0)
	{
		int temp_x = 0, temp_y = y;
		while (temp_y < y + character_num_height)
		{
			temp_x = original_x;
			while (temp_x < x)
			{
				buff[temp_x + frameBufferWidth * (temp_y)] = fill_space;
				temp_x++;
			}
			temp_x = x + character_num_width * digit;
			while (temp_x < original_x + width)
			{
				buff[temp_x + frameBufferWidth * (temp_y)] = fill_space;
				temp_x++;
			}
			temp_y++;
		}
	}

	for (int i = 0; i < digit; i++)
	{
		DrawMonoCharacter(buff, (long *)character_num[value % 10], character_num_width, character_num_height, x + (digit - i - 1) * character_num_width, y, col, fill);
		value /= 10;
	}
}

#define COLOR_BLANK 0x2965
#define COLOR_BLANK_TEXT 0xE71C
#define COLOR_WAVEFORM 0xFFCB
#define COLOR_WAVEFORM_BASE 0x9492
#define COLOR_WAVEFORM_FILL 0xE75F
#define COLOR_WAVEFORM_BACKGROUND COLOR_BLANK
#define COLOR_KEY COLOR_WAVEFORM
#define COLOR_KEY_TEXT_BACK COLOR_BLANK
#define COLOR_KEY_TEXT 0xfe46
#define COLOR_KEY_TEXT_SUB COLOR_BLANK_TEXT

/* Mascon notch readout, in the gap between the unit glyphs (which end at
 * x = 510) and the right edge of the key box (x = 634). */
#define NOTCH_LABEL_X 532
#define NOTCH_LABEL_Y 212
#define NOTCH_LABEL_SCALE 2
#define NOTCH_AREA_X 512
#define NOTCH_AREA_Y 246
#define NOTCH_AREA_W 120
#define NOTCH_AREA_H 84
#define NOTCH_SCALE 5
#define COLOR_KEY_BORDER 0xFFFF

void initializeGUI(uint16_t *buff)
{
	int _x = frameBufferWidth;
	int _y = frameBufferHeight;

	while (_x > 0)
	{
		_x--;
		_y = frameBufferHeight;
		while (_y > 0)
		{
			_y--;
			buff[_x + frameBufferWidth * _y] = COLOR_BLANK;
		}
	}

	DrawRoundRect(buff, 20, 5, 201, frameBufferWidth - 10, 140, COLOR_KEY_BORDER);
	DrawRoundRect(buff, 20, 6, 202, frameBufferWidth - 12, 138, COLOR_KEY_TEXT_BACK);
	DrawMonoCharacter(buff, (long *)character_voltage, 190, 80, 10, 200, COLOR_KEY_TEXT_SUB, 0x00);
	DrawMonoCharacter(buff, (long *)character_frequency, 190, 80, 10, 260, COLOR_KEY_TEXT_SUB, 0x00);
	DrawMonoCharacter(buff, (long *)character_percent, 40, 20, 470, 250, COLOR_KEY_TEXT_SUB, 0x00);
	DrawMonoCharacter(buff, (long *)character_hz, 40, 20, 470, 310, COLOR_KEY_TEXT_SUB, 0x00);
	// The label itself is redrawn every frame by taskDisplay, since it doubles
	// as the EMO / REV indicator.
}

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
 * holds. Power keeps the old full notch ceiling of 4 Hz/s split over P1~P5.
 * The cart is light, so braking doubles that ceiling over B1~B8 and EB
 * triples it.
 */
#define MASCON_MAX_ACCEL 4.0
#define MASCON_MAX_BRAKE (MASCON_MAX_ACCEL * 2.0)
#define MASCON_EB_BRAKE (MASCON_MAX_ACCEL * 3.0)

typedef struct
{
	const char *name;
	double freqRate;	// [Hz / sec]
	bool neutral;		// hand the frequency over to the sound profile
	bool brake;			// ignored while neutral
} MasconNotch;

static const MasconNotch mascon_notches[16] = {
	{ "EB", -MASCON_EB_BRAKE,               false, true  },
	{ "B8", -MASCON_MAX_BRAKE * 8.0 / 8.0,  false, true  },
	{ "B7", -MASCON_MAX_BRAKE * 7.0 / 8.0,  false, true  },
	{ "B6", -MASCON_MAX_BRAKE * 6.0 / 8.0,  false, true  },
	{ "B5", -MASCON_MAX_BRAKE * 5.0 / 8.0,  false, true  },
	{ "B4", -MASCON_MAX_BRAKE * 4.0 / 8.0,  false, true  },
	{ "B3", -MASCON_MAX_BRAKE * 3.0 / 8.0,  false, true  },
	{ "B2", -MASCON_MAX_BRAKE * 2.0 / 8.0,  false, true  },
	{ "B1", -MASCON_MAX_BRAKE * 1.0 / 8.0,  false, true  },
	{ "N",   0.0,                           true,  false },
	{ "P0",  0.0,                           false, false },
	{ "P1",  MASCON_MAX_ACCEL * 1.0 / 5.0,  false, false },
	{ "P2",  MASCON_MAX_ACCEL * 2.0 / 5.0,  false, false },
	{ "P3",  MASCON_MAX_ACCEL * 3.0 / 5.0,  false, false },
	{ "P4",  MASCON_MAX_ACCEL * 4.0 / 5.0,  false, false },
	{ "P5",  MASCON_MAX_ACCEL * 5.0 / 5.0,  false, false },
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
	int max_i = frameBufferWidth;
	signed char buff[max_i];
	static int waveFormImgDispX = 0;
	

	initializeGUI(screenBuffer);

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

		for (int i = 0; i < max_i; i++)
		{
			displayStatus.sin_time = (double)i / max_i / 30.0;
			displayStatus.saw_time = (double)i / max_i / 30.0;
			displayStatus.generation_current_time = (double)i / max_i / 30.0;
			PhaseStatus U, V;
			calculatePhases(&U, &V, 0, &displayStatus, gpioSound);
			buff[i] = (signed char)V - (signed char)U;
		}

		displayStatus.sin_time = 0;
		displayStatus.saw_time = 0;
		double b_1 = 0;
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

		DrawNumber(screenBuffer, (int)(round(displayStatus.v_sin_angle_freq * M_1_2PI)), 200, 261, 270, 80, 3, false, true, COLOR_KEY_TEXT, COLOR_KEY_TEXT_BACK, COLOR_KEY_TEXT_BACK); // BOX-1 CONTENT
		DrawNumber(screenBuffer, (int)b_1, 200, 201, 270, 80, 3, false, true, COLOR_KEY_TEXT, COLOR_KEY_TEXT_BACK, COLOR_KEY_TEXT_BACK);				  // BOX-2 CONTENT

		// Notch readout. This is the notch actually in force, which is EB while
		// EMO is latched no matter where the throttle sits.
		const char *notchName = getMasconNotchName(activeNotch);
		for (int _ny = NOTCH_AREA_Y; _ny < NOTCH_AREA_Y + NOTCH_AREA_H; _ny++)
			for (int _nx = NOTCH_AREA_X; _nx < NOTCH_AREA_X + NOTCH_AREA_W; _nx++)
				screenBuffer[_nx + frameBufferWidth * _ny] = COLOR_KEY_TEXT_BACK;

		int notchWidth = (notchName[1] != 0 ? 2 : 1) * 8 * NOTCH_SCALE;
		DrawText(screenBuffer, notchName,
			NOTCH_AREA_X + (NOTCH_AREA_W - notchWidth) / 2,
			NOTCH_AREA_Y + (NOTCH_AREA_H - 16 * NOTCH_SCALE) / 2,
			NOTCH_SCALE, COLOR_KEY_TEXT, 0x00);

		// The label above the notch doubles as the state indicator: EMO wins
		// over REV, since a latched emergency stop is the thing to see first.
		const char *label = emoLatched ? "EMO" : (reverseActive ? "REV" : "NOTCH");
		uint16_t labelColour = emoLatched ? COLOR_KEY_TEXT : COLOR_KEY_TEXT_SUB;
		for (int _ly = NOTCH_LABEL_Y; _ly < NOTCH_LABEL_Y + 16 * NOTCH_LABEL_SCALE; _ly++)
			for (int _lx = NOTCH_AREA_X; _lx < NOTCH_AREA_X + NOTCH_AREA_W; _lx++)
				screenBuffer[_lx + frameBufferWidth * _ly] = COLOR_KEY_TEXT_BACK;

		int labelWidth = 0;
		while (label[labelWidth] != 0) labelWidth++;
		labelWidth *= 8 * NOTCH_LABEL_SCALE;
		DrawText(screenBuffer, label,
			NOTCH_AREA_X + (NOTCH_AREA_W - labelWidth) / 2, NOTCH_LABEL_Y,
			NOTCH_LABEL_SCALE, labelColour, 0x00);

		waveFormImgDispX = 0;
		while (waveFormImgDispX < max_i)
		{
			// Background Fill
			int bg_fill = 0;
			while (bg_fill <= 200)
			{
				screenBuffer[0 + waveFormImgDispX + 640 * (0 + bg_fill)] = COLOR_WAVEFORM_BACKGROUND;
				bg_fill++;
			}

			// Base Line
			screenBuffer[0 + waveFormImgDispX + 640 * (100)] = COLOR_WAVEFORM_BASE;

			// Draw graph on x = i
			int y_diff = (buff[waveFormImgDispX] - buff[(waveFormImgDispX + 1) == max_i ? waveFormImgDispX : waveFormImgDispX + 1]) * 40;
			int abs_y_diff = y_diff < 0 ? -y_diff : y_diff;
			int y_start = buff[waveFormImgDispX] * 40 + 100;

			// int y_fill = buff[waveFormImgDispX] * 40;
			// int y_fill_dir = buff[waveFormImgDispX] > 0 ? -1 : 1;
			// y_fill *= y_fill_dir;
			// while (y_fill < 0)
			// {
			// 	y_fill++;
			// 	screenBuffer[0 + waveFormImgDispX + 640 * (100 + y_fill * y_fill_dir)] = COLOR_WAVEFORM_FILL;
			// }

			while (abs_y_diff >= 0)
			{
				screenBuffer[0 + waveFormImgDispX + 640 * (y_start + abs_y_diff * (y_diff > 0 ? -1 : 1))] = COLOR_WAVEFORM;
				abs_y_diff--;
			}

			waveFormImgDispX++;
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
	screenDMA((uint32_t)boot_img, frameBufferAddress & ~0xC0000000, 640 * 360 * 2);
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
