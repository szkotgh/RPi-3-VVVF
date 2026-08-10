#ifndef _WINDOWS_H_
#define _WINDOWS_H_

#include <stdbool.h>
#include <stdint.h>
#include "vvvf-struct.h"

extern uint32_t frameBufferAddress;
extern uint32_t frameBufferWidth;
extern uint32_t frameBufferHeight;
extern uint32_t frameBufferPitch;
extern uint32_t frameBufferDepth;

bool initializeWindow(int Width,										// Screen width request (Use 0 if you wish autodetect width)
	int Height,									// Screen height request (Use 0 if you wish autodetect height)
	int PhysicalWidth,
	int PhysicalHeight,
	int Depth);									// Screen colour depth request (Use 0 if you wish autodetect colour depth)

/*
 * Instrument panel layout, on the 640x360 RGB565 screen:
 *
 *   +--------------------------+  +-----------+
 *   |                          |  | FREQ  box |
 *   |                          |  | VOLT  box |
 *   |       VVVF graph         |  | PULS  box |
 *   |                          |  | NOTCH box |
 *   |                          |  +-----------+
 *   |                          |  | EMO  REVS |
 *   +--------------------------+  |   lamps   |
 *   | sound name           1/3 |  |           |
 *   +--------------------------+  +-----------+
 *
 * The graph panel geometry is public because the waveform itself is rendered
 * by taskDisplay, column by column, straight into the screen buffer.
 */
#define DASH_GRAPH_X 8
#define DASH_GRAPH_Y 8
#define DASH_GRAPH_W 408
#define DASH_GRAPH_H 300
#define DASH_GRAPH_BASE_Y (DASH_GRAPH_Y + DASH_GRAPH_H / 2)

#define COLOR_DASH_BACK 0x0000					/* #000000 screen background */
#define COLOR_DASH_PANEL 0x18C3					/* #1A1A1A panel plates */
#define COLOR_DASH_TEXT 0xFFFF					/* #FFFFFF labels and values */

/* 8x16 BitFont text, scaled up by `scale`. Pass fill = 0 for transparent. */
void DrawText(uint16_t *buff, const char *text, int x, int y, int scale, uint16_t col, uint16_t fill);

/* Boot splash: E-TRAIN PROJECT title card with a status line at the bottom. */
void windowDrawBootScreen(uint16_t *buff, const char *status);

/* Static dashboard chrome: panels, labels, lamps in their off state. */
void windowInitializeDashboard(uint16_t *buff);

/* Per frame value updates. Each repaints only its own box. */
void windowUpdateFreq(uint16_t *buff, double freqHz);
void windowUpdateVolt(uint16_t *buff, double percent, bool valid);
void windowUpdatePuls(uint16_t *buff, PulseModeNames mode, double carrierHz, bool valid);
void windowUpdateNotch(uint16_t *buff, const char *name, uint16_t colour);
void windowUpdateLamps(uint16_t *buff, bool emoLit, bool revsLit);
void windowUpdateSoundBar(uint16_t *buff, const char *name, int index, int total);
#endif
