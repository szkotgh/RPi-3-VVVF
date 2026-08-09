#ifndef VVVF_WAVE_H
#define VVVF_WAVE_H

#include "vvvf-struct.h"

typedef void (* VvvfSoundFunction)(VvvfValues *, PwmCalculateValues *);
extern VvvfSoundFunction vvvf_sounds[];
extern const char *vvvf_sound_names[];   /* display name, same index as vvvf_sounds */
extern const int vvvf_sounds_len;

/* Resolve a sound function (e.g. from getSoundVvvfGpio()) back to its
 * display name. Returns "UNKNOWN" when the pointer is not in vvvf_sounds. */
const char *getVvvfSoundName(VvvfSoundFunction sound);

#endif
