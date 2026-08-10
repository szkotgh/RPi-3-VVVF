#include <stdbool.h>			// C standard unit needed for bool and true/false
#include <stdint.h>				// C standard unit needed for uint8_t, uint32_t, etc
#include <math.h>				// Needed for sqrt and round (lamp circles)
#include "rpi-smartstart.h"
#include "Font8x16.h"
#include "windows.h"			// This units header

uint32_t frameBufferAddress = 0x00;
uint32_t frameBufferWidth = 0x00;
uint32_t frameBufferHeight = 0x00;
uint32_t frameBufferPitch = 0x00;
uint32_t frameBufferDepth = 0x00;

bool initializeWindow (int Width,										// Screen width request (Use 0 if you wish autodetect width)
					  int Height,									// Screen height request (Use 0 if you wish autodetect height)
					  int PhysicalWidth,
					  int PhysicalHeight,
					  int Depth)									// Screen colour depth request (Use 0 if you wish autodetect colour depth)
{
	uint32_t buffer[23];
	if ((Width == 0) || (Height == 0)) {							// Has auto width or height been requested
		if (mailbox_tag_message(&buffer[0], 5,
			MAILBOX_TAG_GET_PHYSICAL_WIDTH_HEIGHT,
			8, 0, 0, 0)) {											// Get current width and height of screen
			if (Width == 0) Width = buffer[3];						// Width passed in as zero set set Screenwidth variable
			if (Height == 0) Height = buffer[4];					// Height passed in as zero set set ScreenHeight variable
		} else return false;										// For some reason get screen physical failed
	}
	if (Depth == 0) {												// Has auto colour depth been requested
		if (mailbox_tag_message(&buffer[0], 4,
			MAILBOX_TAG_GET_COLOUR_DEPTH, 4, 4, 0)) {				// Get current colour depth of screen
			Depth = buffer[3];										// Depth passed in as zero set set current screen colour depth
		} else return false;										// For some reason get screen depth failed
	}
	if (!mailbox_tag_message(&buffer[0], 23,
		MAILBOX_TAG_SET_PHYSICAL_WIDTH_HEIGHT, 8, 8, PhysicalWidth, PhysicalHeight,
		MAILBOX_TAG_SET_VIRTUAL_WIDTH_HEIGHT, 8, 8, Width, Height,
		MAILBOX_TAG_SET_COLOUR_DEPTH, 4, 4, Depth,
		MAILBOX_TAG_ALLOCATE_FRAMEBUFFER, 8, 4, 16, 0,
		MAILBOX_TAG_GET_PITCH, 4, 0, 0))							// Attempt to set the requested settings
		return false;												// The requesting settings failed so return the failure

	frameBufferAddress = buffer[17];
	frameBufferPitch = buffer[22];									// Transfer the line pitch
	frameBufferWidth = Width;											// Transfer the screen width
	frameBufferHeight = Height;											// Transfer the screen height
	frameBufferDepth = Depth;										// Transfer the screen depth

	return true;													// Return successful
}

/* ==================== instrument panel ==================== */

/* Right hand column: four value boxes stacked over the lamp panel. */
#define DASH_BOX_X 424
#define DASH_BOX_W 208
#define DASH_BOX_H 42
#define DASH_BOX_FREQ_Y 8
#define DASH_BOX_VOLT_Y 60
#define DASH_BOX_PULS_Y 112
#define DASH_BOX_NOTCH_Y 164
#define DASH_LABEL_X (DASH_BOX_X + 12)
/* Values are left aligned on a shared column so "---" and digits line up. */
#define DASH_VALUE_X (DASH_BOX_X + 104)
#define DASH_VALUE_X1 (DASH_BOX_X + DASH_BOX_W - 8)

#define DASH_LAMP_PANEL_Y 216
#define DASH_LAMP_PANEL_H 136
#define DASH_LAMP_R 26
#define DASH_LAMP_CY (DASH_LAMP_PANEL_Y + 55)
#define DASH_LAMP_EMO_CX (DASH_BOX_X + 69)
#define DASH_LAMP_REVS_CX (DASH_BOX_X + 139)
#define DASH_LAMP_LABEL_Y (DASH_LAMP_CY + DASH_LAMP_R + 10)

/* Bottom bar under the graph: sound profile name and its index. */
#define DASH_BAR_X 8
#define DASH_BAR_Y 316
#define DASH_BAR_W 408
#define DASH_BAR_H 36

/* Lamps: dark red when off so the round window still reads as a lamp. */
#define COLOR_DASH_LAMP_OFF 0x4800				/* #4A0000 */
#define COLOR_DASH_LAMP_ON 0xF800				/* #FF0000 */

static void fillRect(uint16_t *buff, int x, int y, int w, int h, uint16_t col)
{
	for (int _y = y; _y < y + h; _y++)
		for (int _x = x; _x < x + w; _x++)
			buff[_x + frameBufferWidth * _y] = col;
}

static void drawCircleFilled(uint16_t *buff, int cx, int cy, int r, uint16_t col)
{
	for (int dy = -r; dy <= r; dy++)
	{
		int dx = (int)round(sqrt((double)(r * r - dy * dy)));
		for (int _x = cx - dx; _x <= cx + dx; _x++)
			buff[_x + frameBufferWidth * (cy + dy)] = col;
	}
}

static int strLen(const char *text)
{
	int len = 0;
	while (text[len] != 0) len++;
	return len;
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

static int fmtUnsigned(char *out, unsigned int value)
{
	char tmp[10];
	int n = 0;
	do { tmp[n++] = '0' + value % 10; value /= 10; } while (value != 0);
	for (int i = 0; i < n; i++) out[i] = tmp[n - 1 - i];
	out[n] = 0;
	return n;
}

/* Non negative value as "12.3" with `decimals` digits after the point. */
static int fmtFixed(char *out, double value, int decimals)
{
	unsigned int scale = 1;
	for (int i = 0; i < decimals; i++) scale *= 10;
	if (value < 0) value = 0;
	unsigned int v = (unsigned int)(value * scale + 0.5);
	int len = fmtUnsigned(out, v / scale);
	if (decimals > 0)
	{
		out[len++] = '.';
		unsigned int frac = v % scale;
		for (int i = decimals - 1; i >= 0; i--) { out[len + i] = '0' + frac % 10; frac /= 10; }
		len += decimals;
		out[len] = 0;
	}
	return len;
}

/* Value area repaint: value at scale 2, optional small unit on its baseline. */
static void drawBoxValue(uint16_t *buff, int boxY, const char *value, const char *unit, uint16_t colour)
{
	fillRect(buff, DASH_VALUE_X, boxY + 4, DASH_VALUE_X1 - DASH_VALUE_X, DASH_BOX_H - 8, COLOR_DASH_PANEL);
	DrawText(buff, value, DASH_VALUE_X, boxY + 5, 2, colour, 0x00);
	if (unit != 0)
		DrawText(buff, unit, DASH_VALUE_X + strLen(value) * 16 + 6, boxY + 18, 1, COLOR_DASH_TEXT, 0x00);
}

void windowDrawBootScreen(uint16_t *buff, const char *status)
{
	fillRect(buff, 0, 0, frameBufferWidth, frameBufferHeight, COLOR_DASH_BACK);
	DrawText(buff, "E-TRAIN PROJECT", (frameBufferWidth - 15 * 24) / 2, 126, 3, COLOR_DASH_TEXT, 0x00);
	DrawText(buff, "2026 DYHS. MAKE", (frameBufferWidth - 15 * 8) / 2, 185, 1, COLOR_DASH_TEXT, 0x00);
	DrawText(buff, status, (frameBufferWidth - strLen(status) * 8) / 2, 336, 1, COLOR_DASH_TEXT, 0x00);
}

void windowInitializeDashboard(uint16_t *buff)
{
	fillRect(buff, 0, 0, frameBufferWidth, frameBufferHeight, COLOR_DASH_BACK);

	fillRect(buff, DASH_GRAPH_X, DASH_GRAPH_Y, DASH_GRAPH_W, DASH_GRAPH_H, COLOR_DASH_PANEL);
	fillRect(buff, DASH_BAR_X, DASH_BAR_Y, DASH_BAR_W, DASH_BAR_H, COLOR_DASH_PANEL);

	static const struct { int y; const char *label; } boxes[4] = {
		{ DASH_BOX_FREQ_Y, "FREQ" },
		{ DASH_BOX_VOLT_Y, "VOLT" },
		{ DASH_BOX_PULS_Y, "PULS" },
		{ DASH_BOX_NOTCH_Y, "NOTCH" },
	};
	for (int i = 0; i < 4; i++)
	{
		fillRect(buff, DASH_BOX_X, boxes[i].y, DASH_BOX_W, DASH_BOX_H, COLOR_DASH_PANEL);
		DrawText(buff, boxes[i].label, DASH_LABEL_X, boxes[i].y + 5, 2, COLOR_DASH_TEXT, 0x00);
	}

	fillRect(buff, DASH_BOX_X, DASH_LAMP_PANEL_Y, DASH_BOX_W, DASH_LAMP_PANEL_H, COLOR_DASH_PANEL);
	DrawText(buff, "EMO", DASH_LAMP_EMO_CX - 12, DASH_LAMP_LABEL_Y, 1, COLOR_DASH_TEXT, 0x00);
	DrawText(buff, "REVS", DASH_LAMP_REVS_CX - 16, DASH_LAMP_LABEL_Y, 1, COLOR_DASH_TEXT, 0x00);
	windowUpdateLamps(buff, false, false);
}

void windowUpdateFreq(uint16_t *buff, double freqHz)
{
	char text[12];
	fmtFixed(text, freqHz, 1);
	drawBoxValue(buff, DASH_BOX_FREQ_Y, text, "Hz", COLOR_DASH_TEXT);
}

void windowUpdateVolt(uint16_t *buff, double percent, bool valid)
{
	char text[12];
	if (!valid)
	{
		drawBoxValue(buff, DASH_BOX_VOLT_Y, "---", 0, COLOR_DASH_TEXT);
		return;
	}
	fmtFixed(text, percent, 1);
	drawBoxValue(buff, DASH_BOX_VOLT_Y, text, "%", COLOR_DASH_TEXT);
}

/*
 * Synchronous pulse mode to its pulse count, -1 when it has none (async) or
 * the mode is not one this display knows how to name.
 */
static int pulseModeCount(PulseModeNames mode)
{
	if (mode == P_1 || mode == SP_1) return 1;
	if (mode == P_Wide_3 || mode == SP_Wide_3) return 3;
	if (mode == P_10 || mode == SP_10) return 10;
	if (mode == P_12 || mode == SP_12) return 12;
	if (mode == P_18 || mode == SP_18) return 18;
	if (mode >= P_3 && mode <= P_61) return 3 + 2 * ((int)mode - (int)P_3);
	if (mode >= SP_3 && mode <= SP_61) return 3 + 2 * ((int)mode - (int)SP_3);
	return -1;
}

void windowUpdatePuls(uint16_t *buff, PulseModeNames mode, double carrierHz, bool valid)
{
	char text[12];
	if (!valid)
	{
		drawBoxValue(buff, DASH_BOX_PULS_Y, "---", 0, COLOR_DASH_TEXT);
		return;
	}
	if (mode == Async || mode == Async_THI)
		fmtFixed(text, carrierHz, 1);				/* async: carrier frequency [Hz] */
	else
	{
		int count = pulseModeCount(mode);
		if (count < 0)
		{
			drawBoxValue(buff, DASH_BOX_PULS_Y, "---", 0, COLOR_DASH_TEXT);
			return;
		}
		int len = fmtUnsigned(text, count);			/* sync: pulse count, e.g. "15P" */
		text[len] = 'P';
		text[len + 1] = 0;
	}
	drawBoxValue(buff, DASH_BOX_PULS_Y, text, 0, COLOR_DASH_TEXT);
}

void windowUpdateNotch(uint16_t *buff, const char *name, uint16_t colour)
{
	drawBoxValue(buff, DASH_BOX_NOTCH_Y, name, 0, colour);
}

void windowUpdateLamps(uint16_t *buff, bool emoLit, bool revsLit)
{
	drawCircleFilled(buff, DASH_LAMP_EMO_CX, DASH_LAMP_CY, DASH_LAMP_R,
		emoLit ? COLOR_DASH_LAMP_ON : COLOR_DASH_LAMP_OFF);
	drawCircleFilled(buff, DASH_LAMP_REVS_CX, DASH_LAMP_CY, DASH_LAMP_R,
		revsLit ? COLOR_DASH_LAMP_ON : COLOR_DASH_LAMP_OFF);
}

void windowUpdateSoundBar(uint16_t *buff, const char *name, int index, int total)
{
	fillRect(buff, DASH_BAR_X, DASH_BAR_Y, DASH_BAR_W, DASH_BAR_H, COLOR_DASH_PANEL);
	DrawText(buff, name, DASH_BAR_X + 10, DASH_BAR_Y + 10, 1, COLOR_DASH_TEXT, 0x00);

	char text[24];
	int len = fmtUnsigned(text, index < 0 ? 0 : index);
	text[len++] = '/';
	len += fmtUnsigned(text + len, total < 0 ? 0 : total);
	DrawText(buff, text, DASH_BAR_X + DASH_BAR_W - 10 - len * 8, DASH_BAR_Y + 10, 1, COLOR_DASH_TEXT, 0x00);
}
