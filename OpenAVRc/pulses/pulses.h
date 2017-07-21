 /*
 **************************************************************************
 *                                                                        *
 *                 ____                ___ _   _____                      *
 *                / __ \___  ___ ___  / _ | | / / _ \____                 *
 *               / /_/ / _ \/ -_) _ \/ __ | |/ / , _/ __/                 *
 *               \____/ .__/\__/_//_/_/ |_|___/_/|_|\__/                  *
 *                   /_/                                                  *
 *                                                                        *
 *              This file is part of the OpenAVRc project.                *
 *                                                                        *
 *                         Based on code(s) named :                       *
 *             OpenTx - https://github.com/opentx/opentx                  *
 *             Deviation - https://www.deviationtx.com/                   *
 *                                                                        *
 *                Only AVR code here for visibility ;-)                   *
 *                                                                        *
 *   OpenAVRc is free software: you can redistribute it and/or modify     *
 *   it under the terms of the GNU General Public License as published by *
 *   the Free Software Foundation, either version 2 of the License, or    *
 *   (at your option) any later version.                                  *
 *                                                                        *
 *   OpenAVRc is distributed in the hope that it will be useful,          *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of       *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        *
 *   GNU General Public License for more details.                         *
 *                                                                        *
 *       License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html          *
 *                                                                        *
 **************************************************************************
*/


#ifndef PULSES_COMMON_H
#define PULSES_COMMON_H

#define SCHEDULE_MIXER_END(delay) nextMixerEndTime = getTmr16KHz() + (delay) - 2*16 // 2ms


enum ModuleFlag {
  MODULE_NORMAL_MODE,
  MODULE_RANGECHECK,
  MODULE_BIND,
  // MODULE_OFF, // will need an EEPROM conversion
};


#if defined(DSM2)
#define DSM2_BIND_TIMEOUT      255         // 255*11ms
extern uint8_t dsm2BindTimer;
#endif

#define IS_PPM_PROTOCOL(protocol)          (protocol<=PROTO_PPMSIM)

#if defined(PXX)
#define IS_PXX_PROTOCOL(protocol)          (protocol==PROTO_PXX)
#else
#define IS_PXX_PROTOCOL(protocol)          (0)
#endif

#if defined(DSM2)
#define IS_DSM2_PROTOCOL(protocol)         (protocol>=PROTO_DSM2_LP45 && protocol<=PROTO_DSM2_DSMX)
#else
#define IS_DSM2_PROTOCOL(protocol)         (0)
#endif

#if defined(DSM2_SERIAL)
#define IS_DSM2_SERIAL_PROTOCOL(protocol)  (IS_DSM2_PROTOCOL(protocol))
#else
#define IS_DSM2_SERIAL_PROTOCOL(protocol)  (0)
#endif

#if defined(SPIMODULES)
#define IS_SPIMODULES_PROTOCOL(protocol)  (protocol==PROTO_SPIMODULE)
#else
#define IS_SPIMODULES_PROTOCOL(protocol)  (0)
#endif

#if defined(MULTIMODULE)
	#define IS_MULTIMODULE_PROTOCOL(protocol)  (protocol==PROTO_MULTIMODULE)
	#define MULTIMODULE_BAUDRATE 100000
	#define MAX_PULSES_TRANSITIONS 300

	#define CROSSFIRE_FRAME_MAXLEN         64
	#define CROSSFIRE_CHANNELS_COUNT       16
    #define pulse_duration_t             uint16_t
    #define trainer_pulse_duration_t     uint16_t
	void putDsm2Flush();

	PACK(struct Dsm2SerialPulsesData {
  uint8_t  pulses[64];
  uint8_t * ptr;
  uint8_t  serialByte ;
  uint8_t  serialBitCount;
  uint16_t _alignment;
});
	
	template<class T> struct PpmPulsesData {
  T pulses[20];
  T * ptr;
};
  PACK(struct CrossfirePulsesData {
		uint8_t pulses[CROSSFIRE_FRAME_MAXLEN];
	});
	PACK(struct PxxTimerPulsesData {
  pulse_duration_t pulses[200];
  pulse_duration_t * ptr;
  uint16_t rest;
  uint16_t pcmCrc;
  uint32_t pcmOnesCount;
});
PACK(struct Dsm2TimerPulsesData {
  pulse_duration_t pulses[MAX_PULSES_TRANSITIONS];
  pulse_duration_t * ptr;
  uint16_t rest;
  uint8_t index;
});

	union ModulePulsesData {
	#if defined(PPM_PIN_SERIAL)
	PxxSerialPulsesData pxx;
	Dsm2SerialPulsesData dsm2;
	#else
	PxxTimerPulsesData pxx;
	Dsm2TimerPulsesData dsm2;
	#endif
	#if defined(PPM_PIN_UART)
		PxxUartPulsesData pxx_uart;
	#endif
	PpmPulsesData<pulse_duration_t> ppm;
	CrossfirePulsesData crossfire;
	} __ALIGNED;

/* The __ALIGNED keyword is required to align the struct inside the modulePulsesData below,
 * which is also defined to be __DMA  (which includes __ALIGNED) aligned.
 * Arrays in C/C++ are always defined to be *contiguously*. The first byte of the second element is therefore always
 * sizeof(ModulePulsesData). __ALIGNED is required for sizeof(ModulePulsesData) to be a multiple of the alignment.
 */

	extern ModulePulsesData modulePulsesData;
	union TrainerPulsesData {
  PpmPulsesData<trainer_pulse_duration_t> ppm;
};

#else
	#define IS_MULTIMODULE_PROTOCOL(protocol)  (0)
#endif

#include "pulses_avr.h"

#endif
