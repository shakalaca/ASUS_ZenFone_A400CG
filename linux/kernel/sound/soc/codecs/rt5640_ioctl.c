/*
 * rt5640_ioctl.h  --  RT5640 ALSA SoC audio driver IO control
 *
 * Copyright 2012 Realtek Microelectronics
 * Author: Bard <bardliao@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/spi/spi.h>
#include <sound/soc.h>
#include "rt56xx_ioctl.h"
#include "rt5640_ioctl.h"
#include "rt5640.h"
#include <linux/HWVersion.h>
#if (CONFIG_SND_SOC_RT5642_MODULE | CONFIG_SND_SOC_RT5642)
#include "rt5640-dsp.h"
#endif

extern int Read_PROJ_ID(void);
static int PROJ_ID;

// Jericho eq
hweq_t ME302C_hweq_param[] = {
        {/* NORMAL */
                {0},
                {0},
                0x0000,
        },
        {/*SPK MULTIMEDIA  */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x0000, 0xc10e, 0x1f2c, 0xf805, 0xc149, 0x1f1f, 0xf854, 0xe904, 0x1c10, 0x0000, 0xe904, 0x1c10, 0x0000, 0x0436, 0x0000, 0x1f59, 0x00a3, 0x1f5a},
                0x0046,
        },
        {/* HP MULTIMEDIA*/
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x2000, 0x0000, 0x2000},
                0x0040,
        },
        {/* SPK voip */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x2000, 0x0000, 0x2000},
                0x0000,
        },
        {/* HP voip*/
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x2000, 0x0000, 0x2000},
                0x0000,
        },
};

hweq_t ME372CG_hweq_param[] = {
	{/* NORMAL */
		{0},
		{0},
		0x0000,
	},
	{/*SPK MULTIMEDIA  */
		{0xa0,	0xa1,	0xa2,	0xa3,	0xa4,	0xa5,	0xa6,	0xa7,	0xa8,	0xa9,	0xaa,	0xab,	0xac,	0xad,	0xae,	0xaf,	0xb0,	0xb1,	0xb2},
		{0x1c10,0x0000,	0xc883,	0x1c10,	0x0000,	0xd588,	0x1c10,	0x0000,	0xe904,	0x1c10,	0x0000, 0xe904,	0x1c10,	0x0000,	0x0436,	0x0000,	0x1f8e,	0x0070,	0x1f8f},
		0x0040,
	},
	{/* HP MULTIMEDIA*/
		{0xa0,	0xa1,	0xa2,	0xa3,	0xa4,	0xa5,	0xa6,	0xa7,	0xa8,	0xa9,	0xaa,	0xab,	0xac,	0xad,	0xae,	0xaf,	0xb0,	0xb1,	0xb2},
		{0x1c10,0x01f4,	0xc5e9,	0x1a98,	0x1d2c,	0xc882,	0x1c10,	0x01f4,	0xe904,	0x1c10,	0x01f4, 0xe904,	0x1c10,	0x01f4,	0x1c10,	0x01f4,	0x2000,	0x0000,	0x2000},
		0x0040,
	},
	{/* SPK voip */
		{0xa0,	0xa1,	0xa2,	0xa3,	0xa4,	0xa5,	0xa6,	0xa7,	0xa8,	0xa9,	0xaa,	0xab,	0xac,	0xad,	0xae,	0xaf,	0xb0,	0xb1,	0xb2},
		{0x1c10,0x01f4,	0xc5e9,	0x1a98,	0x1d2c,	0xc882,	0x1c10,	0x01f4,	0xe904,	0x1c10,	0x01f4, 0xe904,	0x1c10,	0x01f4,	0x1c10,	0x01f4,	0x2000,	0x0000,	0x2000},
		0x0000,
	},
	{/* HP voip*/
		{0xa0,	0xa1,	0xa2,	0xa3,	0xa4,	0xa5,	0xa6,	0xa7,	0xa8,	0xa9,	0xaa,	0xab,	0xac,	0xad,	0xae,	0xaf,	0xb0,	0xb1,	0xb2},
		{0x1c10,0x01f4,	0xc5e9,	0x1a98,	0x1d2c,	0xc882,	0x1c10,	0x01f4,	0xe904,	0x1c10,	0x01f4, 0xe904,	0x1c10,	0x01f4,	0x1c10,	0x01f4,	0x2000,	0x0000,	0x2000},
		0x0000,
	},
};

hweq_t TX201LA_hweq_param[] = {
        {/* NORMAL */
                {0},
                {0},
                0x0000,
        },
        {/*SPK MULTIMEDIA  */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x0000, 0xc10e, 0x1f2c, 0xf805, 0xc149, 0x1f1f, 0xf854, 0xe904, 0x1c10, 0x0000, 0xe904, 0x1c10, 0x0000, 0x0436, 0x0000, 0x1f77, 0x0086, 0x1f78},
                0x0046,
        },
        {/* HP MULTIMEDIA*/
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x2000, 0x0000, 0x2000},
                0x0040,
        },
        {/* SPK voip */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x2000, 0x0000, 0x2000},
                0x0000,
        },
        {/* HP voip*/
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x2000, 0x0000, 0x2000},
                0x0000,
        },
};

hweq_t ME175CG_hweq_param[] = {
        {/* NORMAL */
                {0},
                {0},
                0x0000,
        },
        {/*SPK MULTIMEDIA  */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x2000, 0x0000, 0x2000},
                0x0000,
        },
        {/* HP MULTIMEDIA*/
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x2000, 0x0000, 0x2000},
                0x0000,
        },
        {/* SPK voip */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x2000, 0x0000, 0x2000},
                0x0000,
        },
        {/* HP voip*/
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x2000, 0x0000, 0x2000},
                0x0000,
        },
};

hweq_t PF400CG_hweq_param[] = {
        {/* NONE */
                {0},
                {0},
                0x0000,
        },
        {/* Playback(a12 speaker)  */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                //{0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x1d0b, 0x02b4, 0x1d2b},
                {0x1c10,0x01f4, 0xc243, 0x1e45, 0xfb54, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xf3ec, 0x0ead, 0xf5ae, 0x0436, 0x0000, 0x1f68, 0x0094, 0x1f69},
                0x0052,
        },
        {/* Playback(headphone) */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x2000, 0x0000, 0x2000},
                0x0000,
        },
        {/* Playback(p72g speaker) */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x1d0b, 0x02b4, 0x1d2b},
                0x0000,
        },
        {/* RingTone(a12 speaker) */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc243, 0x1e45, 0xfb54, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xf3ec, 0x0ead, 0xf5ae, 0x0436, 0x0000, 0x1f68, 0x0094, 0x1f69},
                0x0000,
        },
        {/* RingTone(headphone) */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x2000, 0x0000, 0x2000},
                0x0000,
        },
        {/* RingTone(p72g speaker) */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc243, 0x1e45, 0xfb54, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xf3ec, 0x0ead, 0xf5ae, 0x0436, 0x0000, 0x1f68, 0x0094, 0x1f69},
                0x0000,
        },
};

hweq_t A400CG_hweq_param[] = {
        {/* NONE */
                {0},
                {0},
                0x0000,
        },
        {/* Playback(a12 speaker)  */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                //{0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x1d0b, 0x02b4, 0x1d2b},
                {0x1c10,0x01f4, 0xc243, 0x1e45, 0xfb54, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xf3ec, 0x0ead, 0xf5ae, 0x0436, 0x0000, 0x1f68, 0x0094, 0x1f69},
                0x0052,
        },
        {/* Playback(headphone) */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x2000, 0x0000, 0x2000},
                0x0000,
        },
        {/* Playback(p72g speaker) */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x1d0b, 0x02b4, 0x1d2b},
                0x0000,
        },
        {/* RingTone(a12 speaker) */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc243, 0x1e45, 0xfb54, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xf3ec, 0x0ead, 0xf5ae, 0x0436, 0x0000, 0x1f68, 0x0094, 0x1f69},
                0x0052,
        },
        {/* RingTone(headphone) */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x2000, 0x0000, 0x2000},
                0x0000,
        },
        {/* RingTone(p72g speaker) */
                {0xa0,  0xa1,   0xa2,   0xa3,   0xa4,   0xa5,   0xa6,   0xa7,   0xa8,   0xa9,   0xaa,   0xab,   0xac,   0xad,   0xae,   0xaf,   0xb0,   0xb1,   0xb2},
                {0x1c10,0x01f4, 0xc243, 0x1e45, 0xfb54, 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0xf3ec, 0x0ead, 0xf5ae, 0x0436, 0x0000, 0x1f68, 0x0094, 0x1f69},
                0x0052,
        },
};

hweq_t PF400CG_drc_agc_param[] = {
	{/*Off*/
		{0xb4,   0xb5,   0xb6},
		{0x0206, 0x1f00, 0x0000},
		0x0000
	},
	{/*Recording(a12 board mic)*/
		{0xb4,   0xb5,   0xb6},
		{0xe226, 0x33b2, 0xa557},
		0x0000
	},
	{/*Recording(headset mic)*/
		{0xb4,   0xb5,   0xb6},
		{0xe226, 0x33b2, 0xa557},
		0x0000
	},
	{/*Recording(p72g board mic)*/
		{0xb4,   0xb5,   0xb6},
		{0xe226, 0x33b1, 0xa557},
		0x0000
	},
	{/*Ringtone(a12 ringtone)*/
		{0xb4,   0xb5,   0xb6},
		{0x6006, 0x1fe8, 0x0000},
		0x0000
	},
	{/*Ringtone(headphone ringtone)*/
		{0xb4,   0xb5,   0xb6},
		{0x0206, 0x1f00, 0x0000},
		0x0000
	},
	{/*Ringtone(p72g ringtone)*/
		{0xb4,   0xb5,   0xb6},
		{0x460c, 0x1fa6, 0x0000},
		0x0000
	},
};

hweq_t A400CG_drc_agc_param[] = {
        {/*Off*/
                {0xb4,   0xb5,   0xb6},
                {0x0206, 0x1f00, 0x0000},
                0x0000
        },
        {/*Recording(a12 board mic)*/
                {0xb4,   0xb5,   0xb6},
                {0xe226, 0x33b2, 0xa557},
                0x0000
        },
        {/*Recording(headset mic)*/
                {0xb4,   0xb5,   0xb6},
                {0xe226, 0x33b2, 0xa557},
                0x0000
        },
        {/*Recording(p72g board mic)*/
                {0xb4,   0xb5,   0xb6},
                {0xe226, 0x33b1, 0xa557},
                0x0000
        },
        {/*Ringtone(a12 ringtone)*/
                {0xb4,   0xb5,   0xb6},
                {0x6006, 0x1fe8, 0x0000},
                0x0000
        },
        {/*Ringtone(headphone ringtone)*/
                {0xb4,   0xb5,   0xb6},
                {0x0206, 0x1f00, 0x0000},
                0x0000
        },
        {/*Ringtone(p72g ringtone)*/
                {0xb4,   0xb5,   0xb6},
                {0x460c, 0x1fa6, 0x0000},
                0x0000
        },
};
#define RT5640_HWEQ_LEN ARRAY_SIZE(A400CG_hweq_param)
#define RT5640_DRC_AGC_LEN ARRAY_SIZE(A400CG_drc_agc_param)

int rt5640_update_eqmode(
	struct snd_soc_codec *codec, int mode, int device)
{
	struct rt56xx_ops *ioctl_ops = rt56xx_get_ioctl_ops();
	int i;
	static int eqmode, index;
	hweq_t *proj_hweq_param;
	
	pr_info("%s :mode is %d, device %d\n", __func__, mode, device);   
	if(codec == NULL ||  mode >= RT5640_HWEQ_LEN)
		return -EINVAL;
	if(mode == eqmode)
		return 0;

        PROJ_ID = Read_PROJ_ID();
        if (PROJ_ID == PROJ_ID_ME372CG || PROJ_ID == PROJ_ID_ME372CL) {
		proj_hweq_param = ME372CG_hweq_param;
	} else if (PROJ_ID == PROJ_ID_GEMINI) {
		proj_hweq_param = TX201LA_hweq_param;
	} else if (PROJ_ID == PROJ_ID_ME302C) {
		proj_hweq_param = ME302C_hweq_param;
	} else if (PROJ_ID == PROJ_ID_ME175CG) {
                proj_hweq_param = ME175CG_hweq_param;
        } else if (PROJ_ID == PROJ_ID_PF400CG) {
                proj_hweq_param = PF400CG_hweq_param;
        } else if (PROJ_ID == PROJ_ID_A400CG) {
                proj_hweq_param = A400CG_hweq_param;
        }
        
	if (mode == NONE) {
		index = 0;
	} else if (mode == PLAYBACK) {
		if (device == A12_SPEAKER) index = 1;
		else if (device == HEADPHONE) index = 2;
		else if (device == P72G_SPEAKER) index = 3;
	} else if (mode == RINGTONE) {
		if (device == A12_SPEAKER) index = 4;
		else if (device == HEADPHONE) index = 5;
		else if (device == P72G_SPEAKER) index = 6;
	} else {
		pr_err("%s : Not support mode %d device %d\n", __func__, mode, device);
		index = 0;
	}

	for(i = 0; i <= EQ_REG_NUM; i++) {
		if(proj_hweq_param[index].reg[i])
			ioctl_ops->index_write(codec, proj_hweq_param[index].reg[i],
					proj_hweq_param[index].value[i]);
		else
			break;
	} 
	snd_soc_update_bits(codec, RT5640_EQ_CTRL2, RT5640_EQ_CTRL_MASK,
			proj_hweq_param[mode].ctrl);
        snd_soc_update_bits(codec, RT5640_EQ_CTRL1,
                RT5640_EQ_UPD, RT5640_EQ_UPD);
        snd_soc_update_bits(codec, RT5640_EQ_CTRL1, RT5640_EQ_UPD, 0);

	eqmode = mode;

	return 0;
}

int rt5640_update_drc_agc_mode(struct snd_soc_codec *codec, int mode, int device)
{
	pr_info("%s(): mode %d device %d\n", __func__, mode, device);
	int i, index;
	if (mode == NONE) {
		index = 0;
	} /*else if (mode == RECORD) {
		if (device == A12_MIC) index = 1;
		else if (device == HEADSET_MIC) index = 2;
		else if (device == P72G_MIC) index = 3;
	} */else if (mode == RINGTONE) {
		if (device == A12_SPEAKER) index = 4;
		else if (device == HEADPHONE) index = 5;
		else if (device == P72G_SPEAKER) index = 6;
	} else {
		pr_err("%s : Not support mode %d device %d\n", __func__, mode, device);
		index = 0;
	}
		
        if (PROJ_ID == PROJ_ID_PF400CG) {
  	  for(i = 0; i < 3; i++) {
		if(PF400CG_drc_agc_param[index].reg[i])
			snd_soc_write(codec, PF400CG_drc_agc_param[index].reg[i],
					PF400CG_drc_agc_param[index].value[i]);
	  }
        } else if (PROJ_ID == PROJ_ID_A400CG) {
          for(i = 0; i < 3; i++) {
                if(A400CG_drc_agc_param[index].reg[i])
                        snd_soc_write(codec, A400CG_drc_agc_param[index].reg[i],
                                        A400CG_drc_agc_param[index].value[i]);
          }
        }
}

void set_drc_agc_enable(struct snd_soc_codec *codec, int enable, int path)
{
	snd_soc_update_bits(codec, RT5640_DRC_AGC_1, RT5640_DRC_AGC_P_MASK |
		RT5640_DRC_AGC_MASK | RT5640_DRC_AGC_UPD,
		enable << RT5640_DRC_AGC_SFT | path << RT5640_DRC_AGC_P_SFT |
		1 << RT5640_DRC_AGC_UPD_BIT);
}

void set_drc_agc_parameters(struct snd_soc_codec *codec, int attack_rate,
			int sample_rate, int recovery_rate, int limit_level)
{
	snd_soc_update_bits(codec, RT5640_DRC_AGC_3, RT5640_DRC_AGC_TAR_MASK,
				limit_level << RT5640_DRC_AGC_TAR_SFT);
	snd_soc_update_bits(codec, RT5640_DRC_AGC_1, RT5640_DRC_AGC_AR_MASK |
		RT5640_DRC_AGC_R_MASK | RT5640_DRC_AGC_UPD |
		RT5640_DRC_AGC_RC_MASK, attack_rate << RT5640_DRC_AGC_AR_SFT |
		sample_rate << RT5640_DRC_AGC_R_SFT |
		recovery_rate << RT5640_DRC_AGC_RC_SFT |
		0x1 << RT5640_DRC_AGC_UPD_BIT);
}

void set_digital_boost_gain(struct snd_soc_codec *codec,
			int post_gain, int pre_gain)
{
	snd_soc_update_bits(codec, RT5640_DRC_AGC_2,
		RT5640_DRC_AGC_POB_MASK | RT5640_DRC_AGC_PRB_MASK,
		post_gain << RT5640_DRC_AGC_POB_SFT |
		pre_gain << RT5640_DRC_AGC_PRB_SFT);
	snd_soc_update_bits(codec, RT5640_DRC_AGC_1,
		RT5640_DRC_AGC_UPD, 1 << RT5640_DRC_AGC_UPD_BIT);
}

void set_noise_gate(struct snd_soc_codec *codec, int noise_gate_en,
	int noise_gate_hold_en, int compression_gain, int noise_gate_th)
{
	snd_soc_update_bits(codec, RT5640_DRC_AGC_3,
		RT5640_DRC_AGC_NGB_MASK | RT5640_DRC_AGC_NG_MASK |
		RT5640_DRC_AGC_NGH_MASK | RT5640_DRC_AGC_NGT_MASK,
		noise_gate_en << RT5640_DRC_AGC_NG_SFT |
		noise_gate_hold_en << RT5640_DRC_AGC_NGH_SFT |
		compression_gain << RT5640_DRC_AGC_NGB_SFT |
		noise_gate_th << RT5640_DRC_AGC_NGT_SFT);
	snd_soc_update_bits(codec, RT5640_DRC_AGC_1,
		RT5640_DRC_AGC_UPD, 1 << RT5640_DRC_AGC_UPD_BIT);
}

void set_drc_agc_compression(struct snd_soc_codec *codec,
		int compression_en, int compression_ratio)
{
	snd_soc_update_bits(codec, RT5640_DRC_AGC_2,
		RT5640_DRC_AGC_CP_MASK | RT5640_DRC_AGC_CPR_MASK,
		compression_en << RT5640_DRC_AGC_CP_SFT |
		compression_ratio << RT5640_DRC_AGC_CPR_SFT);
	snd_soc_update_bits(codec, RT5640_DRC_AGC_1,
		RT5640_DRC_AGC_UPD, 1 << RT5640_DRC_AGC_UPD_BIT);
}

void get_drc_agc_enable(struct snd_soc_codec *codec, int *enable, int *path)
{
	unsigned int reg = snd_soc_read(codec, RT5640_DRC_AGC_1);

	*enable = (reg & RT5640_DRC_AGC_MASK) >> RT5640_DRC_AGC_SFT;
	*path = (reg & RT5640_DRC_AGC_P_MASK) >> RT5640_DRC_AGC_P_SFT;
}

void get_drc_agc_parameters(struct snd_soc_codec *codec, int *attack_rate,
		int *sample_rate, int *recovery_rate, int *limit_level)
{
	unsigned int reg = snd_soc_read(codec, RT5640_DRC_AGC_3);

	*limit_level = (reg & RT5640_DRC_AGC_TAR_MASK) >>
			RT5640_DRC_AGC_TAR_SFT;
	reg = snd_soc_read(codec, RT5640_DRC_AGC_1);
	*attack_rate = (reg & RT5640_DRC_AGC_AR_MASK) >> RT5640_DRC_AGC_AR_SFT;
	*sample_rate = (reg & RT5640_DRC_AGC_R_MASK) >> RT5640_DRC_AGC_R_SFT;
	*recovery_rate = (reg & RT5640_DRC_AGC_RC_MASK) >>
				RT5640_DRC_AGC_RC_SFT;
}

void get_digital_boost_gain(struct snd_soc_codec *codec,
			int *post_gain, int *pre_gain)
{
	unsigned int reg = snd_soc_read(codec, RT5640_DRC_AGC_2);

	*post_gain = (reg & RT5640_DRC_AGC_POB_MASK) >> RT5640_DRC_AGC_POB_SFT;
	*pre_gain = (reg & RT5640_DRC_AGC_PRB_MASK) >> RT5640_DRC_AGC_PRB_SFT;
}

void get_noise_gate(struct snd_soc_codec *codec, int *noise_gate_en,
	int *noise_gate_hold_en, int *compression_gain, int *noise_gate_th)
{
	unsigned int reg = snd_soc_read(codec, RT5640_DRC_AGC_3);

	printk("get_noise_gate reg=0x%04x\n",reg);
	*noise_gate_en = (reg & RT5640_DRC_AGC_NG_MASK) >>
				RT5640_DRC_AGC_NG_SFT;
	*noise_gate_hold_en = (reg & RT5640_DRC_AGC_NGH_MASK) >>
				RT5640_DRC_AGC_NGH_SFT;
	*compression_gain = (reg & RT5640_DRC_AGC_NGB_MASK) >>
				RT5640_DRC_AGC_NGB_SFT;
	*noise_gate_th = (reg & RT5640_DRC_AGC_NGT_MASK) >>
				RT5640_DRC_AGC_NGT_SFT;
}

void get_drc_agc_compression(struct snd_soc_codec *codec,
		int *compression_en, int *compression_ratio)
{
	unsigned int reg = snd_soc_read(codec, RT5640_DRC_AGC_2);

	*compression_en = (reg & RT5640_DRC_AGC_CP_MASK) >>
				RT5640_DRC_AGC_CP_SFT;
	*compression_ratio = (reg & RT5640_DRC_AGC_CPR_MASK) >>
				RT5640_DRC_AGC_CPR_SFT;
}

int rt5640_ioctl_common(struct snd_hwdep *hw, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct snd_soc_codec *codec = hw->private_data;
	struct rt56xx_cmd __user *_rt56xx = (struct rt56xx_cmd *)arg;
	struct rt56xx_cmd rt56xx;
	struct rt56xx_ops *ioctl_ops = rt56xx_get_ioctl_ops();
	int *buf, mask1 = 0, mask2 = 0;
	static int eq_mode;

	if (copy_from_user(&rt56xx, _rt56xx, sizeof(rt56xx))) {
		dev_err(codec->dev,"copy_from_user faild\n");
		return -EFAULT;
	}
	dev_dbg(codec->dev, "%s(): rt56xx.number=%d, cmd=%d\n",
			__func__, rt56xx.number, cmd);
	buf = kmalloc(sizeof(*buf) * rt56xx.number, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;
	if (copy_from_user(buf, rt56xx.buf, sizeof(*buf) * rt56xx.number)) {
		goto err;
	}

	switch (cmd) {
	case RT_SET_CODEC_HWEQ_IOCTL:
		if (eq_mode == *buf)
			break;
		eq_mode = *buf;
	//	rt5640_update_eqmode(codec, eq_mode);
		break;

	case RT_GET_CODEC_ID:
		*buf = snd_soc_read(codec, RT5640_VENDOR_ID2);
		if (copy_to_user(rt56xx.buf, buf, sizeof(*buf) * rt56xx.number))
			goto err;
		break;

	case RT_SET_CODEC_SPK_VOL_IOCTL:
		if(*(buf) <= 0x27) {
			snd_soc_update_bits(codec, RT5640_SPK_VOL,
				RT5640_L_VOL_MASK | RT5640_R_VOL_MASK,
				*(buf) << RT5640_L_VOL_SFT |
				*(buf) << RT5640_R_VOL_SFT);
		}
		break;

	case RT_SET_CODEC_MIC_GAIN_IOCTL:
		if(*(buf) <= 0x8) {
			snd_soc_update_bits(codec, RT5640_IN1_IN2,
				RT5640_BST_MASK1, *(buf) << RT5640_BST_SFT1);
			snd_soc_update_bits(codec, RT5640_IN3_IN4,
				RT5640_BST_MASK2, *(buf) << RT5640_BST_SFT2);
		}
		break;

	case RT_SET_CODEC_3D_SPK_IOCTL:
		if(rt56xx.number < 4)
			break;
		if (NULL == ioctl_ops->index_update_bits)
			break;

		mask1 = 0;
		if(*buf != -1)
			mask1 |= RT5640_3D_SPK_MASK;
		if(*(buf + 1) != -1)
			mask1 |= RT5640_3D_SPK_M_MASK;
		if(*(buf + 2) != -1)
			mask1 |= RT5640_3D_SPK_CG_MASK;
		if(*(buf + 3) != -1)
			mask1 |= RT5640_3D_SPK_SG_MASK;
		ioctl_ops->index_update_bits(codec, RT5640_3D_SPK, mask1,
			*(buf) << RT5640_3D_SPK_SFT |
			*(buf + 1) << RT5640_3D_SPK_M_SFT |
			*(buf + 2) << RT5640_3D_SPK_CG_SFT |
			*(buf + 3) << RT5640_3D_SPK_SG_SFT);
		break;

	case RT_SET_CODEC_MP3PLUS_IOCTL:
		if(rt56xx.number < 5)
			break;
		mask1 = mask2 = 0;
		if(*buf != -1)
			mask1 |= RT5640_M_MP3_MASK;
		if(*(buf + 1) != -1)
			mask1 |= RT5640_EG_MP3_MASK;
		if(*(buf + 2) != -1)
			mask2 |= RT5640_OG_MP3_MASK;
		if(*(buf + 3) != -1)
			mask2 |= RT5640_HG_MP3_MASK;
		if(*(buf + 4) != -1)
			mask2 |= RT5640_MP3_WT_MASK;

		snd_soc_update_bits(codec, RT5640_MP3_PLUS1, mask1,
			*(buf) << RT5640_M_MP3_SFT |
			*(buf + 1) << RT5640_EG_MP3_SFT);
		snd_soc_update_bits(codec, RT5640_MP3_PLUS2, mask2,
			*(buf + 2) << RT5640_OG_MP3_SFT |
			*(buf + 3) << RT5640_HG_MP3_SFT |
			*(buf + 4) << RT5640_MP3_WT_SFT);
		break;
	case RT_SET_CODEC_3D_HEADPHONE_IOCTL:
		if(rt56xx.number < 4)
			break;
		if (NULL == ioctl_ops->index_update_bits)
			break;

		mask1 = 0;
		if(*buf != -1)
			mask1 |= RT5640_3D_HP_MASK;
		if(*(buf + 1) != -1)
			mask1 |= RT5640_3D_BT_MASK;
		if(*(buf + 2) != -1)
			mask1 |= RT5640_3D_1F_MIX_MASK;
		if(*(buf + 3) != -1)
			mask1 |= RT5640_3D_HP_M_MASK;

		snd_soc_update_bits(codec, RT5640_3D_HP, mask1,
			*(buf)<<RT5640_3D_HP_SFT |
			*(buf + 1) << RT5640_3D_BT_SFT |
			*(buf + 2) << RT5640_3D_1F_MIX_SFT |
			*(buf + 3) << RT5640_3D_HP_M_SFT);
		if(*(buf + 4) != -1)
			ioctl_ops->index_update_bits(codec,
					0x59, 0x1f, *(buf+4));
		break;

	case RT_SET_CODEC_BASS_BACK_IOCTL:
		if(rt56xx.number < 3)
			break;
		mask1 = 0;
		if(*buf != -1)
			mask1 |= RT5640_BB_MASK;
		if(*(buf + 1) != -1)
			mask1 |= RT5640_BB_CT_MASK;
		if(*(buf + 2) != -1)
			mask1 |= RT5640_G_BB_BST_MASK;

		snd_soc_update_bits(codec, RT5640_BASE_BACK, mask1,
			*(buf) << RT5640_BB_SFT |
			*(buf + 1) << RT5640_BB_CT_SFT |
			*(buf + 2) << RT5640_G_BB_BST_SFT);
		break;

	case RT_SET_CODEC_DIPOLE_SPK_IOCTL:
		if(rt56xx.number < 2)
			break;
		if (NULL == ioctl_ops->index_update_bits)
			break;

		mask1 = 0;
		if(*buf != -1)
			mask1 |= RT5640_DP_SPK_MASK;
		if(*(buf + 1) != -1)
			mask1 |= RT5640_DP_ATT_MASK;

		ioctl_ops->index_update_bits(codec, RT5640_DIP_SPK_INF,
			mask1, *(buf) << RT5640_DP_SPK_SFT |
			*(buf + 1) << RT5640_DP_ATT_SFT );
		break;

	case RT_SET_CODEC_DRC_AGC_ENABLE_IOCTL:
		if(rt56xx.number < 2)
			break;
		set_drc_agc_enable(codec, *(buf), *(buf + 1));
		break;

	case RT_SET_CODEC_DRC_AGC_PAR_IOCTL:
		if(rt56xx.number < 4)
			break;
		set_drc_agc_parameters(codec, *(buf), *(buf + 1),
				*(buf + 2), *(buf + 3));
		break;

	case RT_SET_CODEC_DIGI_BOOST_GAIN_IOCTL:
		if(rt56xx.number < 2)
			break;
		set_digital_boost_gain(codec, *(buf), *(buf + 1));
		break;

	case RT_SET_CODEC_NOISE_GATE_IOCTL:
		if(rt56xx.number < 4)
			break;
		set_noise_gate(codec, *(buf), *(buf + 1),
				*(buf + 2), *(buf + 3));
		break;

	case RT_SET_CODEC_DRC_AGC_COMP_IOCTL:
		if(rt56xx.number < 2)
			break;
		set_drc_agc_compression(codec, *(buf), *(buf + 1));
		break;

	case RT_SET_CODEC_WNR_ENABLE_IOCTL:
		if (NULL == ioctl_ops->index_update_bits)
			break;

		ioctl_ops->index_update_bits(codec, RT5640_WND_1,
			RT5640_WND_MASK, *(buf) << RT5640_WND_SFT );
		break;

	case RT_GET_CODEC_DRC_AGC_ENABLE_IOCTL:
		if(rt56xx.number < 2)
			break;
		get_drc_agc_enable(codec, (buf), (buf + 1));
		if (copy_to_user(rt56xx.buf, buf, sizeof(*buf) * rt56xx.number))
			goto err;
		break;

	case RT_GET_CODEC_DRC_AGC_PAR_IOCTL:
		if(rt56xx.number < 4)
			break;
		get_drc_agc_parameters(codec, (buf), (buf + 1),
				(buf + 2), (buf + 3));
		if (copy_to_user(rt56xx.buf, buf,
			sizeof(*buf) * rt56xx.number))
			goto err;
		break;

	case RT_GET_CODEC_DIGI_BOOST_GAIN_IOCTL:
		if(rt56xx.number < 2)
			break;
		get_digital_boost_gain(codec, (buf), (buf + 1));
		if (copy_to_user(rt56xx.buf, buf,
			sizeof(*buf) * rt56xx.number))
			goto err;
		break;

	case RT_GET_CODEC_NOISE_GATE_IOCTL:
		if(rt56xx.number < 4)
			break;
		get_noise_gate(codec, (buf), (buf + 1), (buf + 2), (buf + 3));
		if (copy_to_user(rt56xx.buf, buf,
			sizeof(*buf) * rt56xx.number))
			goto err;
		break;

	case RT_GET_CODEC_DRC_AGC_COMP_IOCTL:
		if(rt56xx.number < 2)
			break;
		get_drc_agc_compression(codec, (buf), (buf + 1));
		if (copy_to_user(rt56xx.buf, buf,
			sizeof(*buf) * rt56xx.number))
			goto err;
		break;

	case RT_GET_CODEC_SPK_VOL_IOCTL:
		*buf = (snd_soc_read(codec, RT5640_SPK_VOL) & RT5640_L_VOL_MASK)
			>> RT5640_L_VOL_SFT;
		if (copy_to_user(rt56xx.buf, buf, sizeof(*buf) * rt56xx.number))
			goto err;
		break;

	case RT_GET_CODEC_MIC_GAIN_IOCTL:
		*buf = (snd_soc_read(codec, RT5640_IN1_IN2) & RT5640_BST_MASK1)
			>> RT5640_BST_SFT1;
		if (copy_to_user(rt56xx.buf, buf, sizeof(*buf) * rt56xx.number))
			goto err;
		break;
#if (CONFIG_SND_SOC_RT5642_MODULE | CONFIG_SND_SOC_RT5642)
	case RT_READ_CODEC_DSP_IOCTL:
	case RT_WRITE_CODEC_DSP_IOCTL:
	case RT_GET_CODEC_DSP_MODE_IOCTL:
		return rt56xx_dsp_ioctl_common(hw, file, cmd, arg);
#endif
	case RT_GET_CODEC_HWEQ_IOCTL:
	case RT_GET_CODEC_3D_SPK_IOCTL:
	case RT_GET_CODEC_MP3PLUS_IOCTL:
	case RT_GET_CODEC_3D_HEADPHONE_IOCTL:
	case RT_GET_CODEC_BASS_BACK_IOCTL:
	case RT_GET_CODEC_DIPOLE_SPK_IOCTL:
	default:
		break;
	}

	kfree(buf);
	return 0;

err:
	kfree(buf);
	return -EFAULT;
}
EXPORT_SYMBOL_GPL(rt5640_ioctl_common);
