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

#ifndef __RT5640_IOCTL_H__
#define __RT5640_IOCTL_H__

#include <sound/hwdep.h>
#include <linux/ioctl.h>
 
enum {
	NONE_DEVICE,
	A12_SPEAKER,
	HEADPHONE,
	P72G_SPEAKER,
	A12_MIC,
	P72G_MIC,
	HEADSET_MIC,
	DEVICE_NUM,
};

enum {
	NONE,
	PLAYBACK,
	RECORD,
	VOIP,
	RINGTONE,
	VR,
	USE_MODE_NUM,
};

#define EQ_REG_NUM 19
typedef struct  hweq_s {
	unsigned int reg[EQ_REG_NUM];
	unsigned int value[EQ_REG_NUM];
	unsigned int ctrl;
} hweq_t;

int rt5640_ioctl_common(struct snd_hwdep *hw, struct file *file,
			unsigned int cmd, unsigned long arg);
int rt5640_update_eqmode(
	struct snd_soc_codec *codec, int mode, int device);
int rt5640_update_drc_agc_mode(
	struct snd_soc_codec *codec, int mode, int device);

#endif /* __RT5640_IOCTL_H__ */
