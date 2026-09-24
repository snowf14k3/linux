/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _RAPHAEL_SENSOR_H
#define _RAPHAEL_SENSOR_H

#include <linux/types.h>
#include <media/v4l2-cci.h>

struct raphael_sensor_reg {
	u32 reg;
	u64 val;
	u32 delay_us;
};

struct raphael_sensor_reg_list {
	const struct raphael_sensor_reg *regs;
	unsigned int num_regs;
};

struct raphael_sensor_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	u32 fps_x100;
	u64 pixel_rate;
	s64 link_freq;
	unsigned int link_freq_index;
	struct raphael_sensor_reg_list settings;
};

struct raphael_sensor_variant {
	const char *name;
	u16 chip_id_register;
	u16 chip_id;
	u16 frame_length_register;
	u16 vts_max;
	u16 exposure_register;
	u8 exposure_data_bytes;
	u8 exposure_shift;
	u16 analogue_gain_register;
	u16 gain_min;
	u16 gain_max;
	u16 gain_default;
	u8 exposure_min;
	u8 exposure_margin;
	bool custom1_supply;
	u32 mbus_code;
	const s64 *link_frequencies;
	unsigned int num_link_frequencies;
	const struct raphael_sensor_mode *modes;
	unsigned int num_modes;
	struct raphael_sensor_reg_list init;
	struct raphael_sensor_reg_list stream_on;
	struct raphael_sensor_reg_list stream_off;
	struct raphael_sensor_reg_list group_hold_on;
	struct raphael_sensor_reg_list group_hold_off;
};

extern const struct raphael_sensor_variant raphael_imx586;
extern const struct raphael_sensor_variant raphael_ov8856;
extern const struct raphael_sensor_variant raphael_s5k3l6;
extern const struct raphael_sensor_variant raphael_s5k3t2;

#endif
