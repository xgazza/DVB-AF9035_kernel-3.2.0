/*
 * Afatech AF9033 demodulator driver
 *
 * Copyright (C) 2008 Afatech
 * Copyright (C) 2009 Antti Palosaari <crope@iki.fi>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *
 * Thanks to TerraTec for a support received.
 */

#include "dvb_frontend.h"
#include <linux/slab.h>         /* for kzalloc/kfree */
#include <linux/version.h>
#include "af9033_priv.h"
#include "af9033.h"
#include "af9033_reg.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,3,0)) || ((defined V4L2_VERSION) && (V4L2_VERSION >= 197120))
/* all DVB frontend drivers now work directly with the DVBv5
 * structure. This warrants that all drivers will be
 * getting/setting frontend parameters on a consistent way, in
 * order to avoid copying data from/to the DVBv3 structs
 * without need.
 */
#define V4L2_ONLY_DVB_V5
#endif

static int af9033_debug;
module_param_named(debug, af9033_debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off frontend debugging (default:off).");
static int af9033_snrdb;
module_param_named(snrdb, af9033_snrdb, int, 0644);
MODULE_PARM_DESC(snrdb, "Turn on/off SNR output as dBx10 (default:off).");

struct af9033_state {
	struct i2c_adapter *i2c;
	struct dvb_frontend frontend;
	struct af9033_config config;

	u16 signal_strength;
	u32 ber;
	u32 ucblocks;
	u16 snr;
	u32 frequency;
	unsigned long next_statistics_check;
};

static u8 regmask[8] = {0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x7f, 0xff};

/* write multiple registers */
static int af9033_write_regs(struct af9033_state *state, u8 mbox, u16 reg,
	u8 *val, u8 len)
{
	u8 buf[3+len];
	struct i2c_msg msg = {
		.addr = state->config.demod_address,
		.flags = 0,
		.len = sizeof(buf),
		.buf = buf };

	buf[0] = mbox;
	buf[1] = reg >> 8;
	buf[2] = reg & 0xff;
	memcpy(&buf[3], val, len);

	if (i2c_transfer(state->i2c, &msg, 1) != 1) {
		warn("I2C write failed reg:%04x len:%d", reg, len);
		return -EREMOTEIO;
	}
	return 0;
}

/* read multiple registers */
static int af9033_read_regs(struct af9033_state *state, u8 mbox, u16 reg,
	u8 *val, u8 len)
{
	u8 obuf[3] = {mbox, reg >> 8, reg & 0xff};
	struct i2c_msg msg[2] = {
		{
			.addr = state->config.demod_address,
			.flags = 0,
			.len = sizeof(obuf),
			.buf = obuf
		}, {
			.addr = state->config.demod_address,
			.flags = I2C_M_RD,
			.len = len,
			.buf = val
		}
	};

	if (i2c_transfer(state->i2c, msg, 2) != 2) {
		warn("I2C read failed reg:%04x", reg);
		return -EREMOTEIO;
	}
	return 0;
}

/* write single register */
static int af9033_write_reg(struct af9033_state *state, u8 mbox, u16 reg,
	u8 val)
{
	return af9033_write_regs(state, mbox, reg, &val, 1);
}

/* read single register */
static int af9033_read_reg(struct af9033_state *state, u8 mbox, u16 reg,
	u8 *val)
{
	return af9033_read_regs(state, mbox, reg, val, 1);
}

/* write single register bits */
static int af9033_write_reg_bits(struct af9033_state *state, u8 mbox, u16 reg,
	u8 pos, u8 len, u8 val)
{
	int ret;
	u8 tmp, mask;

	ret = af9033_read_reg(state, mbox, reg, &tmp);
	if (ret)
		return ret;

	mask = regmask[len - 1] << pos;
	tmp = (tmp & ~mask) | ((val << pos) & mask);

	return af9033_write_reg(state, mbox, reg, tmp);
}

/* read single register bits */
static int af9033_read_reg_bits(struct af9033_state *state, u8 mbox, u16 reg,
	u8 pos, u8 len, u8 *val)
{
	int ret;
	u8 tmp;

	ret = af9033_read_reg(state, mbox, reg, &tmp);
	if (ret)
		return ret;

	*val = (tmp >> pos) & regmask[len - 1];
	return 0;
}

static u32 af913_div(u32 a, u32 b, u32 x)
{
	u32 r = 0, c = 0, i;
	deb_info("%s: a:%d b:%d x:%d\n", __func__, a, b, x);

	if (a > b) {
		c = a / b;
		a = a - c * b;
	}

	for (i = 0; i < x; i++) {
		if (a >= b) {
			r += 1;
			a -= b;
		}
		a <<= 1;
		r <<= 1;
	}
	r = (c << (u32)x) + r;

	deb_info("%s: a:%d b:%d x:%d r:%d r:%x\n", __func__, a, b, x, r, r);
	return r;
}

#ifdef V4L2_ONLY_DVB_V5
static int af9033_set_coeff(struct af9033_state *state, u32 bw)
#else
static int af9033_set_coeff(struct af9033_state *state, fe_bandwidth_t bw)
#endif
{
	int ret = 0;
	u8 tmp, i = 0;
	u8 buf[36];
	u32 uninitialized_var(coeff1_2048nu);
	u32 uninitialized_var(coeff1_4096nu);
	u32 uninitialized_var(coeff1_8191nu);
	u32 uninitialized_var(coeff1_8192nu);
	u32 uninitialized_var(coeff1_8193nu);
	u32 uninitialized_var(coeff2_2k);
	u32 uninitialized_var(coeff2_4k);
	u32 uninitialized_var(coeff2_8k);
	u16 uninitialized_var(bfsfcw_fftindex_ratio);
	u16 uninitialized_var(fftindex_bfsfcw_ratio);
	deb_info("%s: adc_clock:%d bw:%d\n", __func__,
		state->config.adc_clock, bw);

#ifdef V4L2_ONLY_DVB_V5
#define BANDWIDTH_5_MHZ 5000000
#define BANDWIDTH_6_MHZ 6000000
#define BANDWIDTH_7_MHZ 7000000
#define BANDWIDTH_8_MHZ 8000000
#endif

	switch (state->config.adc_clock) {
	case 20156250:
		switch (bw) {
#if 0 /* keep */
		case BANDWIDTH_5_MHZ:
			coeff1_2048nu = 0x02449b5c;
			coeff1_4096nu = 0x01224dae;
			coeff1_8191nu = 0x00912b60;
			coeff1_8192nu = 0x009126d7;
			coeff1_8193nu = 0x0091224e;
			coeff2_2k = 0x01224dae;
			coeff2_4k = 0x009126d7;
			coeff2_8k = 0x0048936b;
			bfsfcw_fftindex_ratio = 0x0387;
			fftindex_bfsfcw_ratio = 0x0122;
			break;
#endif
		case BANDWIDTH_6_MHZ:
			coeff1_2048nu = 0x02b8ba6e;
			coeff1_4096nu = 0x015c5d37;
			coeff1_8191nu = 0x00ae340d;
			coeff1_8192nu = 0x00ae2e9b;
			coeff1_8193nu = 0x00ae292a;
			coeff2_2k = 0x015c5d37;
			coeff2_4k = 0x00ae2e9b;
			coeff2_8k = 0x0057174e;
			bfsfcw_fftindex_ratio = 0x02f1;
			fftindex_bfsfcw_ratio = 0x015c;
			break;
		case BANDWIDTH_7_MHZ:
			coeff1_2048nu = 0x032cd980;
			coeff1_4096nu = 0x01966cc0;
			coeff1_8191nu = 0x00cb3cba;
			coeff1_8192nu = 0x00cb3660;
			coeff1_8193nu = 0x00cb3007;
			coeff2_2k = 0x01966cc0;
			coeff2_4k = 0x00cb3660;
			coeff2_8k = 0x00659b30;
			bfsfcw_fftindex_ratio = 0x0285;
			fftindex_bfsfcw_ratio = 0x0196;
			break;
		case BANDWIDTH_8_MHZ:
			coeff1_2048nu = 0x03a0f893;
			coeff1_4096nu = 0x01d07c49;
			coeff1_8191nu = 0x00e84567;
			coeff1_8192nu = 0x00e83e25;
			coeff1_8193nu = 0x00e836e3;
			coeff2_2k = 0x01d07c49;
			coeff2_4k = 0x00e83e25;
			coeff2_8k = 0x00741f12;
			bfsfcw_fftindex_ratio = 0x0234;
			fftindex_bfsfcw_ratio = 0x01d0;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case 20187500:
		switch (bw) {
#if 0 /* keep */
		case BANDWIDTH_5_MHZ:
			coeff1_2048nu = 0x0243b546;
			coeff1_4096nu = 0x0121daa3;
			coeff1_8191nu = 0x0090f1d9;
			coeff1_8192nu = 0x0090ed51;
			coeff1_8193nu = 0x0090e8ca;
			coeff2_2k = 0x0121daa3;
			coeff2_4k = 0x0090ed51;
			coeff2_8k = 0x004876a9;
			bfsfcw_fftindex_ratio = 0x0388;
			fftindex_bfsfcw_ratio = 0x0122;
			break;
#endif
		case BANDWIDTH_6_MHZ:
			coeff1_2048nu = 0x02b7a654;
			coeff1_4096nu = 0x015bd32a;
			coeff1_8191nu = 0x00adef04;
			coeff1_8192nu = 0x00ade995;
			coeff1_8193nu = 0x00ade426;
			coeff2_2k = 0x015bd32a;
			coeff2_4k = 0x00ade995;
			coeff2_8k = 0x0056f4ca;
			bfsfcw_fftindex_ratio = 0x02f2;
			fftindex_bfsfcw_ratio = 0x015c;
			break;
		case BANDWIDTH_7_MHZ:
			coeff1_2048nu = 0x032b9761;
			coeff1_4096nu = 0x0195cbb1;
			coeff1_8191nu = 0x00caec30;
			coeff1_8192nu = 0x00cae5d8;
			coeff1_8193nu = 0x00cadf81;
			coeff2_2k = 0x0195cbb1;
			coeff2_4k = 0x00cae5d8;
			coeff2_8k = 0x006572ec;
			bfsfcw_fftindex_ratio = 0x0286;
			fftindex_bfsfcw_ratio = 0x0196;
			break;
		case BANDWIDTH_8_MHZ:
			coeff1_2048nu = 0x039f886f;
			coeff1_4096nu = 0x01cfc438;
			coeff1_8191nu = 0x00e7e95b;
			coeff1_8192nu = 0x00e7e21c;
			coeff1_8193nu = 0x00e7dadd;
			coeff2_2k = 0x01cfc438;
			coeff2_4k = 0x00e7e21c;
			coeff2_8k = 0x0073f10e;
			bfsfcw_fftindex_ratio = 0x0235;
			fftindex_bfsfcw_ratio = 0x01d0;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case 20250000:
		switch (bw) {
#if 0 /* keep */
		case BANDWIDTH_5_MHZ:
			coeff1_2048nu = 0x0241eb3b;
			coeff1_4096nu = 0x0120f59e;
			coeff1_8191nu = 0x00907f53;
			coeff1_8192nu = 0x00907acf;
			coeff1_8193nu = 0x0090764b;
			coeff2_2k = 0x0120f59e;
			coeff2_4k = 0x00907acf;
			coeff2_8k = 0x00483d67;
			bfsfcw_fftindex_ratio = 0x038b;
			fftindex_bfsfcw_ratio = 0x0121;
			break;
#endif
		case BANDWIDTH_6_MHZ:
			coeff1_2048nu = 0x02b580ad;
			coeff1_4096nu = 0x015ac057;
			coeff1_8191nu = 0x00ad6597;
			coeff1_8192nu = 0x00ad602b;
			coeff1_8193nu = 0x00ad5ac1;
			coeff2_2k = 0x015ac057;
			coeff2_4k = 0x00ad602b;
			coeff2_8k = 0x0056b016;
			bfsfcw_fftindex_ratio = 0x02f4;
			fftindex_bfsfcw_ratio = 0x015b;
			break;
		case BANDWIDTH_7_MHZ:
			coeff1_2048nu = 0x03291620;
			coeff1_4096nu = 0x01948b10;
			coeff1_8191nu = 0x00ca4bda;
			coeff1_8192nu = 0x00ca4588;
			coeff1_8193nu = 0x00ca3f36;
			coeff2_2k = 0x01948b10;
			coeff2_4k = 0x00ca4588;
			coeff2_8k = 0x006522c4;
			bfsfcw_fftindex_ratio = 0x0288;
			fftindex_bfsfcw_ratio = 0x0195;
			break;
		case BANDWIDTH_8_MHZ:
			coeff1_2048nu = 0x039cab92;
			coeff1_4096nu = 0x01ce55c9;
			coeff1_8191nu = 0x00e7321e;
			coeff1_8192nu = 0x00e72ae4;
			coeff1_8193nu = 0x00e723ab;
			coeff2_2k = 0x01ce55c9;
			coeff2_4k = 0x00e72ae4;
			coeff2_8k = 0x00739572;
			bfsfcw_fftindex_ratio = 0x0237;
			fftindex_bfsfcw_ratio = 0x01ce;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case 20583333:
		switch (bw) {
#if 0 /* keep */
		case BANDWIDTH_5_MHZ:
			coeff1_2048nu = 0x02388f54;
			coeff1_4096nu = 0x011c47aa;
			coeff1_8191nu = 0x008e2846;
			coeff1_8192nu = 0x008e23d5;
			coeff1_8193nu = 0x008e1f64;
			coeff2_2k = 0x011c47aa;
			coeff2_4k = 0x008e23d5;
			coeff2_8k = 0x004711ea;
			bfsfcw_fftindex_ratio = 0x039a;
			fftindex_bfsfcw_ratio = 0x011c;
			break;
#endif
		case BANDWIDTH_6_MHZ:
			coeff1_2048nu = 0x02aa4598;
			coeff1_4096nu = 0x015522cc;
			coeff1_8191nu = 0x00aa96bb;
			coeff1_8192nu = 0x00aa9166;
			coeff1_8193nu = 0x00aa8c12;
			coeff2_2k = 0x015522cc;
			coeff2_4k = 0x00aa9166;
			coeff2_8k = 0x005548b3;
			bfsfcw_fftindex_ratio = 0x0300;
			fftindex_bfsfcw_ratio = 0x0155;
			break;
		case BANDWIDTH_7_MHZ:
			coeff1_2048nu = 0x031bfbdc;
			coeff1_4096nu = 0x018dfdee;
			coeff1_8191nu = 0x00c7052f;
			coeff1_8192nu = 0x00c6fef7;
			coeff1_8193nu = 0x00c6f8bf;
			coeff2_2k = 0x018dfdee;
			coeff2_4k = 0x00c6fef7;
			coeff2_8k = 0x00637f7b;
			bfsfcw_fftindex_ratio = 0x0293;
			fftindex_bfsfcw_ratio = 0x018e;
			break;
		case BANDWIDTH_8_MHZ:
			coeff1_2048nu = 0x038db21f;
			coeff1_4096nu = 0x01c6d910;
			coeff1_8191nu = 0x00e373a3;
			coeff1_8192nu = 0x00e36c88;
			coeff1_8193nu = 0x00e3656d;
			coeff2_2k = 0x01c6d910;
			coeff2_4k = 0x00e36c88;
			coeff2_8k = 0x0071b644;
			bfsfcw_fftindex_ratio = 0x0240;
			fftindex_bfsfcw_ratio = 0x01c7;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case 20416667:
		switch (bw) {
#if 0 /* keep */
		case BANDWIDTH_5_MHZ:
			coeff1_2048nu = 0x023d337f;
			coeff1_4096nu = 0x011e99c0;
			coeff1_8191nu = 0x008f515a;
			coeff1_8192nu = 0x008f4ce0;
			coeff1_8193nu = 0x008f4865;
			coeff2_2k = 0x011e99c0;
			coeff2_4k = 0x008f4ce0;
			coeff2_8k = 0x0047a670;
			bfsfcw_fftindex_ratio = 0x0393;
			fftindex_bfsfcw_ratio = 0x011f;
			break;
#endif
		case BANDWIDTH_6_MHZ:
			coeff1_2048nu = 0x02afd765;
			coeff1_4096nu = 0x0157ebb3;
			coeff1_8191nu = 0x00abfb39;
			coeff1_8192nu = 0x00abf5d9;
			coeff1_8193nu = 0x00abf07a;
			coeff2_2k = 0x0157ebb3;
			coeff2_4k = 0x00abf5d9;
			coeff2_8k = 0x0055faed;
			bfsfcw_fftindex_ratio = 0x02fa;
			fftindex_bfsfcw_ratio = 0x0158;
			break;
		case BANDWIDTH_7_MHZ:
			coeff1_2048nu = 0x03227b4b;
			coeff1_4096nu = 0x01913da6;
			coeff1_8191nu = 0x00c8a518;
			coeff1_8192nu = 0x00c89ed3;
			coeff1_8193nu = 0x00c8988e;
			coeff2_2k = 0x01913da6;
			coeff2_4k = 0x00c89ed3;
			coeff2_8k = 0x00644f69;
			bfsfcw_fftindex_ratio = 0x028d;
			fftindex_bfsfcw_ratio = 0x0191;
			break;
		case BANDWIDTH_8_MHZ:
			coeff1_2048nu = 0x03951f32;
			coeff1_4096nu = 0x01ca8f99;
			coeff1_8191nu = 0x00e54ef7;
			coeff1_8192nu = 0x00e547cc;
			coeff1_8193nu = 0x00e540a2;
			coeff2_2k = 0x01ca8f99;
			coeff2_4k = 0x00e547cc;
			coeff2_8k = 0x0072a3e6;
			bfsfcw_fftindex_ratio = 0x023c;
			fftindex_bfsfcw_ratio = 0x01cb;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case 20480000:
		switch (bw) {
#if 0 /* keep */
		case BANDWIDTH_5_MHZ:
			coeff1_2048nu = 0x023b6db7;
			coeff1_4096nu = 0x011db6db;
			coeff1_8191nu = 0x008edfe5;
			coeff1_8192nu = 0x008edb6e;
			coeff1_8193nu = 0x008ed6f7;
			coeff2_2k = 0x011db6db;
			coeff2_4k = 0x008edb6e;
			coeff2_8k = 0x00476db7;
			bfsfcw_fftindex_ratio = 0x0396;
			fftindex_bfsfcw_ratio = 0x011e;
			break;
#endif
		case BANDWIDTH_6_MHZ:
			coeff1_2048nu = 0x02adb6db;
			coeff1_4096nu = 0x0156db6e;
			coeff1_8191nu = 0x00ab7312;
			coeff1_8192nu = 0x00ab6db7;
			coeff1_8193nu = 0x00ab685c;
			coeff2_2k = 0x0156db6e;
			coeff2_4k = 0x00ab6db7;
			coeff2_8k = 0x0055b6db;
			bfsfcw_fftindex_ratio = 0x02fd;
			fftindex_bfsfcw_ratio = 0x0157;
			break;
		case BANDWIDTH_7_MHZ:
			coeff1_2048nu = 0x03200000;
			coeff1_4096nu = 0x01900000;
			coeff1_8191nu = 0x00c80640;
			coeff1_8192nu = 0x00c80000;
			coeff1_8193nu = 0x00c7f9c0;
			coeff2_2k = 0x01900000;
			coeff2_4k = 0x00c80000;
			coeff2_8k = 0x00640000;
			bfsfcw_fftindex_ratio = 0x028f;
			fftindex_bfsfcw_ratio = 0x0190;
			break;
		case BANDWIDTH_8_MHZ:
			coeff1_2048nu = 0x03924925;
			coeff1_4096nu = 0x01c92492;
			coeff1_8191nu = 0x00e4996e;
			coeff1_8192nu = 0x00e49249;
			coeff1_8193nu = 0x00e48b25;
			coeff2_2k = 0x01c92492;
			coeff2_4k = 0x00e49249;
			coeff2_8k = 0x00724925;
			bfsfcw_fftindex_ratio = 0x023d;
			fftindex_bfsfcw_ratio = 0x01c9;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case 20500000:
		switch (bw) {
#if 0 /* keep */
		case BANDWIDTH_5_MHZ:
			coeff1_2048nu = 0x023adeff;
			coeff1_4096nu = 0x011d6f80;
			coeff1_8191nu = 0x008ebc36;
			coeff1_8192nu = 0x008eb7c0;
			coeff1_8193nu = 0x008eb34a;
			coeff2_2k = 0x011d6f80;
			coeff2_4k = 0x008eb7c0;
			coeff2_8k = 0x00475be0;
			bfsfcw_fftindex_ratio = 0x0396;
			fftindex_bfsfcw_ratio = 0x011d;
			break;
#endif
		case BANDWIDTH_6_MHZ:
			coeff1_2048nu = 0x02ad0b99;
			coeff1_4096nu = 0x015685cc;
			coeff1_8191nu = 0x00ab4840;
			coeff1_8192nu = 0x00ab42e6;
			coeff1_8193nu = 0x00ab3d8c;
			coeff2_2k = 0x015685cc;
			coeff2_4k = 0x00ab42e6;
			coeff2_8k = 0x0055a173;
			bfsfcw_fftindex_ratio = 0x02fd;
			fftindex_bfsfcw_ratio = 0x0157;
			break;
		case BANDWIDTH_7_MHZ:
			coeff1_2048nu = 0x031f3832;
			coeff1_4096nu = 0x018f9c19;
			coeff1_8191nu = 0x00c7d44b;
			coeff1_8192nu = 0x00c7ce0c;
			coeff1_8193nu = 0x00c7c7ce;
			coeff2_2k = 0x018f9c19;
			coeff2_4k = 0x00c7ce0c;
			coeff2_8k = 0x0063e706;
			bfsfcw_fftindex_ratio = 0x0290;
			fftindex_bfsfcw_ratio = 0x0190;
			break;
		case BANDWIDTH_8_MHZ:
			coeff1_2048nu = 0x039164cb;
			coeff1_4096nu = 0x01c8b266;
			coeff1_8191nu = 0x00e46056;
			coeff1_8192nu = 0x00e45933;
			coeff1_8193nu = 0x00e45210;
			coeff2_2k = 0x01c8b266;
			coeff2_4k = 0x00e45933;
			coeff2_8k = 0x00722c99;
			bfsfcw_fftindex_ratio = 0x023e;
			fftindex_bfsfcw_ratio = 0x01c9;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case 20625000:
		switch (bw) {
#if 0 /* keep */
		case BANDWIDTH_5_MHZ:
			coeff1_2048nu = 0x02376948;
			coeff1_4096nu = 0x011bb4a4;
			coeff1_8191nu = 0x008ddec1;
			coeff1_8192nu = 0x008dda52;
			coeff1_8193nu = 0x008dd5e3;
			coeff2_2k = 0x011bb4a4;
			coeff2_4k = 0x008dda52;
			coeff2_8k = 0x0046ed29;
			bfsfcw_fftindex_ratio = 0x039c;
			fftindex_bfsfcw_ratio = 0x011c;
			break;
#endif
		case BANDWIDTH_6_MHZ:
			coeff1_2048nu = 0x02a8e4bd;
			coeff1_4096nu = 0x0154725e;
			coeff1_8191nu = 0x00aa3e81;
			coeff1_8192nu = 0x00aa392f;
			coeff1_8193nu = 0x00aa33de;
			coeff2_2k = 0x0154725e;
			coeff2_4k = 0x00aa392f;
			coeff2_8k = 0x00551c98;
			bfsfcw_fftindex_ratio = 0x0302;
			fftindex_bfsfcw_ratio = 0x0154;
			break;
		case BANDWIDTH_7_MHZ:
			coeff1_2048nu = 0x031a6032;
			coeff1_4096nu = 0x018d3019;
			coeff1_8191nu = 0x00c69e41;
			coeff1_8192nu = 0x00c6980c;
			coeff1_8193nu = 0x00c691d8;
			coeff2_2k = 0x018d3019;
			coeff2_4k = 0x00c6980c;
			coeff2_8k = 0x00634c06;
			bfsfcw_fftindex_ratio = 0x0294;
			fftindex_bfsfcw_ratio = 0x018d;
			break;
		case BANDWIDTH_8_MHZ:
			coeff1_2048nu = 0x038bdba6;
			coeff1_4096nu = 0x01c5edd3;
			coeff1_8191nu = 0x00e2fe02;
			coeff1_8192nu = 0x00e2f6ea;
			coeff1_8193nu = 0x00e2efd2;
			coeff2_2k = 0x01c5edd3;
			coeff2_4k = 0x00e2f6ea;
			coeff2_8k = 0x00717b75;
			bfsfcw_fftindex_ratio = 0x0242;
			fftindex_bfsfcw_ratio = 0x01c6;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	default:
		err("invalid xtal freq");
		return -EINVAL;
	}
	if (ret) {
		err("invalid bandwidth");
		return ret;
	}

#ifdef V4L2_ONLY_DVB_V5
#undef BANDWIDTH_5_MHZ
#undef BANDWIDTH_6_MHZ
#undef BANDWIDTH_7_MHZ
#undef BANDWIDTH_8_MHZ
#endif

	/* adc multiplier */
	ret = af9033_read_reg(state, OFDM, api_adcx2, &tmp);
	if (ret)
		return ret;

	if (tmp == 1) {
		coeff1_2048nu /= 2;
		coeff1_4096nu /= 2;
		coeff1_8191nu /= 2;
		coeff1_8192nu /= 2;
		coeff1_8193nu /= 2 ;
		coeff2_2k /= 2;
		coeff2_4k /= 2;
		coeff2_8k /= 2;
	}

	buf[i++] = (u8) ((coeff1_2048nu         & 0x03000000) >> 24);
	buf[i++] = (u8) ((coeff1_2048nu         & 0x00ff0000) >> 16);
	buf[i++] = (u8) ((coeff1_2048nu         & 0x0000ff00) >> 8);
	buf[i++] = (u8) ((coeff1_2048nu         & 0x000000ff));
	buf[i++] = (u8) ((coeff1_4096nu         & 0x03000000) >> 24);
	buf[i++] = (u8) ((coeff1_4096nu         & 0x00ff0000) >> 16);
	buf[i++] = (u8) ((coeff1_4096nu         & 0x0000ff00) >> 8);
	buf[i++] = (u8) ((coeff1_4096nu         & 0x000000ff));
	buf[i++] = (u8) ((coeff1_8191nu         & 0x03000000) >> 24);
	buf[i++] = (u8) ((coeff1_8191nu         & 0x00ff0000) >> 16);
	buf[i++] = (u8) ((coeff1_8191nu         & 0x0000ff00) >> 8);
	buf[i++] = (u8) ((coeff1_8191nu         & 0x000000ff));
	buf[i++] = (u8) ((coeff1_8192nu         & 0x03000000) >> 24);
	buf[i++] = (u8) ((coeff1_8192nu         & 0x00ff0000) >> 16);
	buf[i++] = (u8) ((coeff1_8192nu         & 0x0000ff00) >> 8);
	buf[i++] = (u8) ((coeff1_8192nu         & 0x000000ff));
	buf[i++] = (u8) ((coeff1_8193nu         & 0x03000000) >> 24);
	buf[i++] = (u8) ((coeff1_8193nu         & 0x00ff0000) >> 16);
	buf[i++] = (u8) ((coeff1_8193nu         & 0x0000ff00) >> 8);
	buf[i++] = (u8) ((coeff1_8193nu         & 0x000000ff));
	buf[i++] = (u8) ((coeff2_8k             & 0x01000000) >> 24);
	buf[i++] = (u8) ((coeff2_8k             & 0x00ff0000) >> 16);
	buf[i++] = (u8) ((coeff2_8k             & 0x0000ff00) >> 8);
	buf[i++] = (u8) ((coeff2_8k             & 0x000000ff));
	buf[i++] = (u8) ((coeff2_2k             & 0x01000000) >> 24);
	buf[i++] = (u8) ((coeff2_2k             & 0x00ff0000) >> 16);
	buf[i++] = (u8) ((coeff2_2k             & 0x0000ff00) >> 8);
	buf[i++] = (u8) ((coeff2_2k             & 0x000000ff));
	buf[i++] = (u8) ((coeff2_4k             & 0x01000000) >> 24);
	buf[i++] = (u8) ((coeff2_4k             & 0x00ff0000) >> 16);
	buf[i++] = (u8) ((coeff2_4k             & 0x0000ff00) >> 8);
	buf[i++] = (u8) ((coeff2_4k             & 0x000000ff));
	buf[i++] = (u8) ((bfsfcw_fftindex_ratio &     0x00ff));
	buf[i++] = (u8) ((bfsfcw_fftindex_ratio &     0xff00) >> 8);
	buf[i++] = (u8) ((fftindex_bfsfcw_ratio &     0x00ff));
	buf[i++] = (u8) ((fftindex_bfsfcw_ratio &     0xff00) >> 8);

	deb_info("%s: coeff:", __func__);
	debug_dump(buf, sizeof(buf), deb_info);

	/* program */
	return af9033_write_regs(state, OFDM, api_cfoe_NS_2048_coeff1_25_24,
		buf, sizeof(buf));
}

static int af9033_set_crystal_ctrl(struct af9033_state *state)
{
	u8 buf[4];
	u32 crystal_cw;
	deb_info("%s: crystal_clock:%d\n", __func__,
		state->config.crystal_clock);

	crystal_cw = af913_div(state->config.crystal_clock, 1000000ul, 19ul);

	buf[0] = (u8) ((crystal_cw & 0x000000ff));
	buf[1] = (u8) ((crystal_cw & 0x0000ff00) >> 8);
	buf[2] = (u8) ((crystal_cw & 0x00ff0000) >> 16);
	buf[3] = (u8) ((crystal_cw & 0xff000000) >> 24);

	deb_info("%s: crystal_cw:", __func__);
	debug_dump(buf, sizeof(buf), deb_info);

	/* program */
	return af9033_write_regs(state, OFDM, api_crystal_clk_7_0, buf,
		sizeof(buf));
}

static int af9033_set_adc_ctrl(struct af9033_state *state)
{
	u8 buf[3];
	u32 adc_cw;
	deb_info("%s: adc_clock:%d\n", __func__, state->config.adc_clock);

	adc_cw = af913_div(state->config.adc_clock, 1000000ul, 19ul);

	buf[0] = (u8) ((adc_cw & 0x000000ff));
	buf[1] = (u8) ((adc_cw & 0x0000ff00) >> 8);
	buf[2] = (u8) ((adc_cw & 0x00ff0000) >> 16);

	deb_info("%s: adc_cw:", __func__);
	debug_dump(buf, sizeof(buf), deb_info);

	/* program */
	return af9033_write_regs(state, OFDM, p_reg_f_adc_7_0, buf,
		sizeof(buf));
}

static int af9033_set_freq_ctrl(struct af9033_state *state)
{
	int ret;
	u8 buf[3], tmp;
	u32 adc_freq, freq_cw;
	s8 bfs_spec_inv;
	int if_sample_freq;

	bfs_spec_inv = state->config.rf_spec_inv ? -1 : 1;

	adc_freq       = state->config.adc_clock;
	if_sample_freq = state->config.if_freq;

	while (if_sample_freq > (adc_freq / 2))
		if_sample_freq = if_sample_freq - adc_freq;

	if (if_sample_freq >= 0)
		bfs_spec_inv = bfs_spec_inv * (-1);
	else
		if_sample_freq = if_sample_freq * (-1);

	freq_cw = af913_div(if_sample_freq, adc_freq, 23ul);

	if (bfs_spec_inv == -1)
		freq_cw *= -1;

	/* adc multiplier */
	ret = af9033_read_reg(state, OFDM, api_adcx2, &tmp);
	if (ret)
		return ret;

	if (tmp == 1)
		freq_cw /= 2;

	buf[0] = (u8) ((freq_cw & 0x000000ff));
	buf[1] = (u8) ((freq_cw & 0x0000ff00) >> 8);
	buf[2] = (u8) ((freq_cw & 0x007f0000) >> 16);

	deb_info("%s: freq_cw:", __func__);
	debug_dump(buf, sizeof(buf), deb_info);

	/* program */
	return af9033_write_regs(state, OFDM, api_bfs_fcw_7_0, buf,
		sizeof(buf));
}
static void af9033_release(struct dvb_frontend *fe)
{
	struct af9033_state *state = fe->demodulator_priv;
	kfree(state);
}

static int af9033_init(struct dvb_frontend *fe)
{
	struct af9033_state *state = fe->demodulator_priv;
	int ret, i, len;
	u8 tmp0, tmp1;
	struct regdesc *init;
	deb_info("%s\n", __func__);

	/* power on */
	ret = af9033_write_reg_bits(state, OFDM, p_reg_afe_mem0, 3, 1, 0);
	if (ret)
		goto error;

	ret = af9033_write_reg(state, OFDM, api_suspend_flag, 0);
	if (ret)
		goto error;

	/* tell to the firmware type of the tuner */
	ret = af9033_write_reg(state, LINK, p_reg_link_ofsm_dummy_15_8,
		state->config.tuner);
	if (ret)
		goto error;

	/* set read-update bit for constellation */
	ret = af9033_write_reg_bits(state, OFDM, p_reg_feq_read_update,
		reg_feq_read_update_pos, reg_feq_read_update_len, 1);
	if (ret)
		goto error;

	/* enable FEC monitor */
	ret = af9033_write_reg_bits(state, OFDM, p_fec_vtb_rsd_mon_en,
		fec_vtb_rsd_mon_en_pos, fec_vtb_rsd_mon_en_len, 1);
	if (ret)
		goto error;

	/* program crystal control */
	ret = af9033_set_crystal_ctrl(state);
	if (ret)
		goto error;

	/* program ADC control */
	ret = af9033_set_adc_ctrl(state);
	if (ret)
		goto error;

	/* enable DVB-T interrupt */
	ret = af9033_write_reg_bits(state, LINK, p_reg_dvbt_inten,
		reg_dvbt_inten_pos, reg_dvbt_inten_len, 1);
	if (ret)
		goto error;

	/* enable DVB-T mode */
	ret = af9033_write_reg_bits(state, LINK, p_reg_dvbt_en,
		reg_dvbt_en_pos, reg_dvbt_en_len, 1);
	if (ret)
		goto error;

	/* set dca_upper_chip */
	ret = af9033_write_reg_bits(state, OFDM, p_reg_dca_upper_chip,
		reg_dca_upper_chip_pos, reg_dca_upper_chip_len, 0);
	if (ret)
		goto error;

	ret = af9033_write_reg_bits(state, LINK, p_reg_top_hostb_dca_upper,
		reg_top_hostb_dca_upper_pos, reg_top_hostb_dca_upper_len, 0);
	if (ret)
		goto error;

	ret = af9033_write_reg_bits(state, LINK, p_reg_top_hosta_dca_upper,
		reg_top_hosta_dca_upper_pos, reg_top_hosta_dca_upper_len, 0);
	if (ret)
		goto error;

	/* set dca_lower_chip */
	ret = af9033_write_reg_bits(state, OFDM, p_reg_dca_lower_chip,
		reg_dca_lower_chip_pos, reg_dca_lower_chip_len, 0);
	if (ret)
		goto error;

	ret = af9033_write_reg_bits(state, LINK, p_reg_top_hostb_dca_lower,
		reg_top_hostb_dca_lower_pos, reg_top_hostb_dca_lower_len, 0);
	if (ret)
		goto error;

	ret = af9033_write_reg_bits(state, LINK, p_reg_top_hosta_dca_lower,
		reg_top_hosta_dca_lower_pos, reg_top_hosta_dca_lower_len, 0);
	if (ret)
		goto error;

	/* set phase latch */
	ret = af9033_write_reg_bits(state, OFDM, p_reg_dca_platch,
		reg_dca_platch_pos, reg_dca_platch_len, 0);
	if (ret)
		goto error;

	/* set fpga latch */
	ret = af9033_write_reg(state, OFDM, p_reg_dca_fpga_latch, 0);
	if (ret)
		goto error;

	/* set stand alone */
	ret = af9033_write_reg_bits(state, OFDM, p_reg_dca_stand_alone,
		reg_dca_stand_alone_pos, reg_dca_stand_alone_len, 1);
	if (ret)
		goto error;

	/* set DCA enable */
	ret = af9033_write_reg_bits(state, OFDM, p_reg_dca_en, reg_dca_en_pos,
		reg_dca_en_len, 0);
	if (ret)
		goto error;

	/* load OFSM settings */
	deb_info("%s: load ofsm settings\n", __func__);
	len = ARRAY_SIZE(ofsm_init);
	init = ofsm_init;
	for (i = 0; i < len; i++) {
		ret = af9033_write_reg(state, OFDM, init[i].addr, init[i].val);
		if (ret)
			goto error;
	}

	/* load tuner specific settings */
	deb_info("%s: load tuner specific settings\n", __func__);
	switch (state->config.tuner) {
	case AF9033_TUNER_TUA9001:
		len = ARRAY_SIZE(tuner_init_tua9001);
		init = tuner_init_tua9001;
		break;
	case AF9033_TUNER_MXL5007t:
		len = ARRAY_SIZE(tuner_init_mxl5007t);
		init = tuner_init_mxl5007t;
		break;
        case AF9033_TUNER_TDA18218:
                len = ARRAY_SIZE(tuner_init_tda18218);
                init = tuner_init_tda18218;
                break;
	default:
		len = 0;
		init = NULL;
		break;
	}
	for (i = 0; i < len; i++) {
		ret = af9033_write_reg(state, OFDM, init[i].addr, init[i].val);
		if (ret)
			goto error;
	}

	/* set H/W MPEG2 locked detection **/
	ret = af9033_write_reg(state, LINK, p_reg_top_lock3_out, 1);
	if (ret)
		goto error;

	/* set registers for driving power */
	ret = af9033_write_reg(state, LINK, p_reg_top_padmiscdr2, 1);
	if (ret)
		goto error;

	/* et registers for driving power */
	ret = af9033_write_reg(state, LINK, p_reg_top_padmiscdr4, 0);
	if (ret)
		goto error;

	/* set registers for driving power */
	ret = af9033_write_reg(state, LINK, p_reg_top_padmiscdr8, 0);
	if (ret)
		goto error;

	/* set TS mode */
	deb_info("%s: setting ts mode\n", __func__);
	tmp0 = 0; /* parallel mode */
	tmp1 = 0; /* serial mode */
	switch (state->config.output_mode) {
	case AF9033_TS_MODE_PARALLEL:
		tmp0 = 1;
		break;
	case AF9033_TS_MODE_SERIAL:
		tmp1 = 1;
		break;
	case AF9033_TS_MODE_USB:
		/* usb mode for AF9035 */
	default:
		break;
	}
	ret = af9033_write_reg_bits(state, OFDM, p_mp2if_mpeg_par_mode,
		mp2if_mpeg_par_mode_pos, mp2if_mpeg_par_mode_len, tmp0);
	if (ret)
		goto error;
	ret = af9033_write_reg_bits(state, OFDM, p_mp2if_mpeg_ser_mode,
		mp2if_mpeg_ser_mode_pos, mp2if_mpeg_ser_mode_len, tmp1);
	if (ret)
		goto error;

	if (state->config.output_mode == AF9033_TS_MODE_SERIAL) {
		ret = af9033_write_reg_bits(state, LINK, p_reg_top_hostb_mpeg_ser_mode,
			reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 1);
		if (ret)
			goto error;
	}
error:
	if (ret)
		deb_info("%s: failed:%d\n", __func__, ret);

	return ret;
}

static int af9033_sleep(struct dvb_frontend *fe)
{
	struct af9033_state *state = fe->demodulator_priv;
	int ret;
	u8 tmp, i;
	deb_info("%s\n", __func__);

	ret = af9033_write_reg(state, OFDM, api_suspend_flag, 1);
	if (ret)
		goto error;

	ret = af9033_write_reg(state, OFDM, api_trigger_ofsm, 0);
	if (ret)
		goto error;

	for (i = 0; i < 150; i++) {
		ret = af9033_read_reg(state, OFDM, api_suspend_flag, &tmp);
		if (ret)
			goto error;
		if (!tmp)
			break;
		msleep(10);
	}
	if (tmp) {
		deb_info("%s: power off time outs\n", __func__);
		return -ETIMEDOUT;
	}

	ret = af9033_write_reg_bits(state, OFDM, p_reg_afe_mem0, 3, 1, 1);
	if (ret)
		goto error;

	/* fixed current leakage (?) */
	if (state->config.output_mode != AF9033_TS_MODE_USB) {
		/* enable parallel TS */
		ret = af9033_write_reg_bits(state, LINK,
			p_reg_top_hosta_mpeg_ser_mode,
			reg_top_hosta_mpeg_ser_mode_pos,
			reg_top_hosta_mpeg_ser_mode_len, 0);
		if (ret)
			goto error;

		ret = af9033_write_reg_bits(state, LINK,
			p_reg_top_hosta_mpeg_par_mode,
			reg_top_hosta_mpeg_par_mode_pos,
			reg_top_hosta_mpeg_par_mode_len, 1);
	}

error:
	if (ret)
		deb_info("%s: failed:%d\n", __func__, ret);

	return ret;
}

#ifdef V4L2_ONLY_DVB_V5
static int af9033_set_frontend(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *params = &fe->dtv_property_cache;
	struct af9033_state *state = fe->demodulator_priv;
	int ret;
	u8 tmp;
	deb_info("%s: freq:%d bw:%d\n", __func__, params->frequency,
		params->bandwidth_hz);

	state->frequency = params->frequency;

	/* program tuner */
	if (fe->ops.tuner_ops.set_params)
		fe->ops.tuner_ops.set_params(fe);

	/* program CFOE coefficients */
	ret = af9033_set_coeff(state, params->bandwidth_hz);
	if (ret)
		goto error;

	/* program frequency control */
	ret = af9033_set_freq_ctrl(state);
	if (ret)
		goto error;

	/* program bandwidth */
	switch (params->bandwidth_hz) {
	case 6000000:
		tmp = 0;
		break;
	case 7000000:
		tmp = 1;
		break;
	case 8000000:
		tmp = 2;
		break;
#if 0 /* keep */
	case 5000000:
		tmp = 3;
		break;
#endif
	default:
		deb_info("%s: invalid bandwidth\n", __func__);
		return -EINVAL;
	}
	ret = af9033_write_reg_bits(state, OFDM, g_reg_bw, reg_bw_pos,
		reg_bw_len, tmp);
	if (ret)
		goto error;

	/* clear easy mode flag */
	ret = af9033_write_reg(state, OFDM, api_Training_Mode, 0x00);
	if (ret)
		goto error;

	/* clear empty channel flag */
	ret = af9033_write_reg(state, OFDM, api_empty_channel_status, 0x00);
	if (ret)
		goto error;

	/* clear MPEG2 lock flag */
	ret = af9033_write_reg_bits(state, OFDM, r_mp2if_sync_byte_locked,
		mp2if_sync_byte_locked_pos, mp2if_sync_byte_locked_len, 0x00);
	if (ret)
		goto error;

	/* set frequency band
	    174 -  230 MHz VHF     band = 0x00
	    350 -  900 MHz UHF     band = 0x01
	   1670 - 1680 MHz L-BAND  band = 0x02
	   otherwise               band = 0xff */
	/* TODO: are both min/max ranges really required... */
	if ((state->frequency >= 174000000) && (state->frequency <= 230000000))
		tmp = 0x00; /* VHF */
	else if ((state->frequency >= 350000000) && (state->frequency <= 900000000))
		tmp = 0x01; /* UHF */
	else if ((state->frequency >= 1670000000) && (state->frequency <= 1680000000))
		tmp = 0x02; /* L-BAND */
	else
		tmp = 0xff;

	ret = af9033_write_reg(state, OFDM, api_FreBand, tmp);
	if (ret)
		goto error;

	/* trigger ofsm */
	ret = af9033_write_reg(state, OFDM, api_trigger_ofsm, 0);
	if (ret)
		goto error;
error:
	if (ret)
		deb_info("%s: failed:%d\n", __func__, ret);

	return ret;
}
#else
static int af9033_set_frontend(struct dvb_frontend *fe,
	struct dvb_frontend_parameters *params)
{
	struct af9033_state *state = fe->demodulator_priv;
	int ret;
	u8 tmp;
	deb_info("%s: freq:%d bw:%d\n", __func__, params->frequency,
		params->u.ofdm.bandwidth);

	state->frequency = params->frequency;

	/* program tuner */
	if (fe->ops.tuner_ops.set_params)
		fe->ops.tuner_ops.set_params(fe, params);

	/* program CFOE coefficients */
	ret = af9033_set_coeff(state, params->u.ofdm.bandwidth);
	if (ret)
		goto error;

	/* program frequency control */
	ret = af9033_set_freq_ctrl(state);
	if (ret)
		goto error;

	/* program bandwidth */
	switch (params->u.ofdm.bandwidth) {
	case BANDWIDTH_6_MHZ:
		tmp = 0;
		break;
	case BANDWIDTH_7_MHZ:
		tmp = 1;
		break;
	case BANDWIDTH_8_MHZ:
		tmp = 2;
		break;
#if 0 /* keep */
	case BANDWIDTH_5_MHZ:
		tmp = 3;
		break;
#endif
	default:
		deb_info("%s: invalid bandwidth\n", __func__);
		return -EINVAL;
	}
	ret = af9033_write_reg_bits(state, OFDM, g_reg_bw, reg_bw_pos,
		reg_bw_len, tmp);
	if (ret)
		goto error;

	/* clear easy mode flag */
	ret = af9033_write_reg(state, OFDM, api_Training_Mode, 0x00);
	if (ret)
		goto error;

	/* clear empty channel flag */
	ret = af9033_write_reg(state, OFDM, api_empty_channel_status, 0x00);
	if (ret)
		goto error;

	/* clear MPEG2 lock flag */
	ret = af9033_write_reg_bits(state, OFDM, r_mp2if_sync_byte_locked,
		mp2if_sync_byte_locked_pos, mp2if_sync_byte_locked_len, 0x00);
	if (ret)
		goto error;

	/* set frequency band
	    174 -  230 MHz VHF     band = 0x00
	    350 -  900 MHz UHF     band = 0x01
	   1670 - 1680 MHz L-BAND  band = 0x02
	   otherwise               band = 0xff */
	/* TODO: are both min/max ranges really required... */
	if ((state->frequency >= 174000000) && (state->frequency <= 230000000))
		tmp = 0x00; /* VHF */
	else if ((state->frequency >= 350000000) && (state->frequency <= 900000000))
		tmp = 0x01; /* UHF */
	else if ((state->frequency >= 1670000000) && (state->frequency <= 1680000000))
		tmp = 0x02; /* L-BAND */
	else
		tmp = 0xff;

	ret = af9033_write_reg(state, OFDM, api_FreBand, tmp);
	if (ret)
		goto error;

	/* trigger ofsm */
	ret = af9033_write_reg(state, OFDM, api_trigger_ofsm, 0);
	if (ret)
		goto error;
error:
	if (ret)
		deb_info("%s: failed:%d\n", __func__, ret);

	return ret;
}
#endif

static int af9033_get_tune_settings(struct dvb_frontend *fe,
	struct dvb_frontend_tune_settings *fesettings)
{
	fesettings->min_delay_ms = 800;
	fesettings->step_size = 0;
	fesettings->max_drift = 0;

	return 0;
}

#ifdef V4L2_ONLY_DVB_V5
static int af9033_get_frontend(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct af9033_state *state = fe->demodulator_priv;
	int ret;
	u8 buf[8];
	deb_info("%s\n", __func__);
#define TRANSMISSION_MODE  (g_reg_tpsd_txmod - g_reg_tpsd_txmod)
#define GUARD_INTERVAL     (g_reg_tpsd_gi    - g_reg_tpsd_txmod)
#define HIERARCHY          (g_reg_tpsd_hier  - g_reg_tpsd_txmod)
#define CONSTELLATION      (g_reg_tpsd_const - g_reg_tpsd_txmod)
#define BANDWIDTH          (g_reg_bw         - g_reg_tpsd_txmod)
#define PRIORITY           (g_reg_dec_pri    - g_reg_tpsd_txmod)
#define CODE_RATE_HP       (g_reg_tpsd_hpcr  - g_reg_tpsd_txmod)
#define CODE_RATE_LP       (g_reg_tpsd_lpcr  - g_reg_tpsd_txmod)

	/* read all needed registers */
	ret = af9033_read_regs(state, OFDM, g_reg_tpsd_txmod, buf, sizeof(buf));
	if (ret)
		goto error;

	deb_info("%s: ", __func__);
	debug_dump(buf, sizeof(buf), deb_info);

	switch ((buf[CONSTELLATION] >> 0) & 3) {
	case 0:
		p->modulation = QPSK;
		break;
	case 1:
		p->modulation = QAM_16;
		break;
	case 2:
		p->modulation = QAM_64;
		break;
	}

	switch ((buf[TRANSMISSION_MODE] >> 0) & 3) {
	case 0:
		p->transmission_mode = TRANSMISSION_MODE_2K;
		break;
	case 1:
		p->transmission_mode = TRANSMISSION_MODE_8K;
		break;
#if 0 /* keep */
	case 2:
		p->transmission_mode = TRANSMISSION_MODE_4K;
		break;
#endif
	}

	switch ((buf[GUARD_INTERVAL] >> 0) & 3) {
	case 0:
		p->guard_interval = GUARD_INTERVAL_1_32;
		break;
	case 1:
		p->guard_interval = GUARD_INTERVAL_1_16;
		break;
	case 2:
		p->guard_interval = GUARD_INTERVAL_1_8;
		break;
	case 3:
		p->guard_interval = GUARD_INTERVAL_1_4;
		break;
	}

	switch ((buf[HIERARCHY] >> 0) & 7) {
	case 0:
		p->hierarchy = HIERARCHY_NONE;
		break;
	case 1:
		p->hierarchy = HIERARCHY_1;
		break;
	case 2:
		p->hierarchy = HIERARCHY_2;
		break;
	case 3:
		p->hierarchy = HIERARCHY_4;
		break;
	}

	switch ((buf[CODE_RATE_HP] >> 0) & 7) {
	case 0:
		p->code_rate_HP = FEC_1_2;
		break;
	case 1:
		p->code_rate_HP = FEC_2_3;
		break;
	case 2:
		p->code_rate_HP = FEC_3_4;
		break;
	case 3:
		p->code_rate_HP = FEC_5_6;
		break;
	case 4:
		p->code_rate_HP = FEC_7_8;
		break;
	case 5:
		p->code_rate_HP = FEC_NONE;
		break;
	}

	switch ((buf[CODE_RATE_LP] >> 0) & 7) {
	case 0:
		p->code_rate_LP = FEC_1_2;
		break;
	case 1:
		p->code_rate_LP = FEC_2_3;
		break;
	case 2:
		p->code_rate_LP = FEC_3_4;
		break;
	case 3:
		p->code_rate_LP = FEC_5_6;
		break;
	case 4:
		p->code_rate_LP = FEC_7_8;
		break;
	case 5:
		p->code_rate_LP = FEC_NONE;
		break;
	}

	switch ((buf[BANDWIDTH] >> 0) & 3) {
	case 0:
		p->bandwidth_hz = 6000000;
		break;
	case 1:
		p->bandwidth_hz = 7000000;
		break;
	case 2:
		p->bandwidth_hz = 8000000;
		break;
#if 0 /* keep */
	case 3:
		p->bandwidth_hz = 5000000;
		break;
#endif
	}

	p->inversion = INVERSION_AUTO;
	p->frequency = state->frequency;

error:
	if (ret)
		deb_info("%s: failed:%d\n", __func__, ret);

	return ret;
}
#else
static int af9033_get_frontend(struct dvb_frontend *fe,
	struct dvb_frontend_parameters *p)
{
	struct af9033_state *state = fe->demodulator_priv;
	int ret;
	u8 buf[8];
	deb_info("%s\n", __func__);
#define TRANSMISSION_MODE  (g_reg_tpsd_txmod - g_reg_tpsd_txmod)
#define GUARD_INTERVAL     (g_reg_tpsd_gi    - g_reg_tpsd_txmod)
#define HIERARCHY          (g_reg_tpsd_hier  - g_reg_tpsd_txmod)
#define CONSTELLATION      (g_reg_tpsd_const - g_reg_tpsd_txmod)
#define BANDWIDTH          (g_reg_bw         - g_reg_tpsd_txmod)
#define PRIORITY           (g_reg_dec_pri    - g_reg_tpsd_txmod)
#define CODE_RATE_HP       (g_reg_tpsd_hpcr  - g_reg_tpsd_txmod)
#define CODE_RATE_LP       (g_reg_tpsd_lpcr  - g_reg_tpsd_txmod)

	/* read all needed registers */
	ret = af9033_read_regs(state, OFDM, g_reg_tpsd_txmod, buf, sizeof(buf));
	if (ret)
		goto error;

	deb_info("%s: ", __func__);
	debug_dump(buf, sizeof(buf), deb_info);

	switch ((buf[CONSTELLATION] >> 0) & 3) {
	case 0:
		p->u.ofdm.constellation = QPSK;
		break;
	case 1:
		p->u.ofdm.constellation = QAM_16;
		break;
	case 2:
		p->u.ofdm.constellation = QAM_64;
		break;
	}

	switch ((buf[TRANSMISSION_MODE] >> 0) & 3) {
	case 0:
		p->u.ofdm.transmission_mode = TRANSMISSION_MODE_2K;
		break;
	case 1:
		p->u.ofdm.transmission_mode = TRANSMISSION_MODE_8K;
		break;
#if 0 /* keep */
	case 2:
		p->u.ofdm.transmission_mode = TRANSMISSION_MODE_4K;
		break;
#endif
	}

	switch ((buf[GUARD_INTERVAL] >> 0) & 3) {
	case 0:
		p->u.ofdm.guard_interval = GUARD_INTERVAL_1_32;
		break;
	case 1:
		p->u.ofdm.guard_interval = GUARD_INTERVAL_1_16;
		break;
	case 2:
		p->u.ofdm.guard_interval = GUARD_INTERVAL_1_8;
		break;
	case 3:
		p->u.ofdm.guard_interval = GUARD_INTERVAL_1_4;
		break;
	}

	switch ((buf[HIERARCHY] >> 0) & 7) {
	case 0:
		p->u.ofdm.hierarchy_information = HIERARCHY_NONE;
		break;
	case 1:
		p->u.ofdm.hierarchy_information = HIERARCHY_1;
		break;
	case 2:
		p->u.ofdm.hierarchy_information = HIERARCHY_2;
		break;
	case 3:
		p->u.ofdm.hierarchy_information = HIERARCHY_4;
		break;
	}

	switch ((buf[CODE_RATE_HP] >> 0) & 7) {
	case 0:
		p->u.ofdm.code_rate_HP = FEC_1_2;
		break;
	case 1:
		p->u.ofdm.code_rate_HP = FEC_2_3;
		break;
	case 2:
		p->u.ofdm.code_rate_HP = FEC_3_4;
		break;
	case 3:
		p->u.ofdm.code_rate_HP = FEC_5_6;
		break;
	case 4:
		p->u.ofdm.code_rate_HP = FEC_7_8;
		break;
	case 5:
		p->u.ofdm.code_rate_HP = FEC_NONE;
		break;
	}

	switch ((buf[CODE_RATE_LP] >> 0) & 7) {
	case 0:
		p->u.ofdm.code_rate_LP = FEC_1_2;
		break;
	case 1:
		p->u.ofdm.code_rate_LP = FEC_2_3;
		break;
	case 2:
		p->u.ofdm.code_rate_LP = FEC_3_4;
		break;
	case 3:
		p->u.ofdm.code_rate_LP = FEC_5_6;
		break;
	case 4:
		p->u.ofdm.code_rate_LP = FEC_7_8;
		break;
	case 5:
		p->u.ofdm.code_rate_HP = FEC_NONE;
		break;
	}

	switch ((buf[BANDWIDTH] >> 0) & 3) {
	case 0:
		p->u.ofdm.bandwidth = BANDWIDTH_6_MHZ;
		break;
	case 1:
		p->u.ofdm.bandwidth = BANDWIDTH_7_MHZ;
		break;
	case 2:
		p->u.ofdm.bandwidth = BANDWIDTH_8_MHZ;
		break;
#if 0 /* keep */
	case 3:
		p->u.ofdm.bandwidth = BANDWIDTH_5_MHZ;
		break;
#endif
	}

	p->inversion = INVERSION_AUTO;
	p->frequency = state->frequency;

error:
	if (ret)
		deb_info("%s: failed:%d\n", __func__, ret);

	return ret;
}
#endif

static int af9033_update_ber_ucblocks(struct dvb_frontend *fe)
{
	struct af9033_state *state = fe->demodulator_priv;
	int ret;
	u8 buf[3];
	u32 error_bit_count = 0;
	u32 total_bit_count = 0;
	u16 abort_packet_count = 0;

	/* don't update ber / ucblocks unnecessary often */
	if (time_before(jiffies, state->next_statistics_check))
		return 0;

	/* set minimum ber / ucblocks update interval */
	state->next_statistics_check = jiffies + msecs_to_jiffies(500);

	state->ber = 0;

	/* no need to check ber / ucblocks in case of no lock */
	ret = af9033_read_reg_bits(state, OFDM,
		r_mp2if_sync_byte_locked, mp2if_sync_byte_locked_pos,
		mp2if_sync_byte_locked_len, buf);
	if (ret)
		goto error;
	if (!buf[0])
		goto exit;

	/* get abort packet count */
	ret = af9033_read_regs(state, OFDM, api_rsd_abort_packet_cnt_7_0, buf,
		sizeof(buf) - 1);
	if (ret)
		goto error;

	abort_packet_count = (buf[1] << 8) + buf[0];

	/* get error bit count */
	ret = af9033_read_regs(state, OFDM, api_rsd_bit_err_cnt_7_0, buf,
		sizeof(buf));
	if (ret)
		goto error;

	error_bit_count = (buf[2] << 16) + (buf[1] << 8) + buf[0];
	error_bit_count = error_bit_count - abort_packet_count * 8 * 8;

	/* get used RSD counting period (it is 10000 by defaut) */
	ret = af9033_read_regs(state, OFDM, api_r_rsd_packet_unit_7_0, buf,
		sizeof(buf) - 1);
	if (ret)
		goto error;

	total_bit_count = (buf[1] << 8) + buf[0];
	total_bit_count = total_bit_count - abort_packet_count;
	total_bit_count = total_bit_count * 204 * 8;

	if (total_bit_count)
		state->ber = error_bit_count * 1000000000 / total_bit_count;

	state->ucblocks += abort_packet_count;

	deb_info("%s: err bits:%d total bits:%d abort count:%d\n", __func__,
		error_bit_count, total_bit_count, abort_packet_count);

error:
	if (ret)
		deb_info("%s: failed:%d\n", __func__, ret);
exit:
	return ret;
}

static int af9033_update_snr(struct dvb_frontend *fe)
{
	struct af9033_state *state = fe->demodulator_priv;
	int ret;
	u8 buf[3], i, len;
	u32 snr_val;
	struct snr_table *uninitialized_var(snr_table);

	/* read snr registers */
	ret = af9033_read_regs(state, OFDM, api_qnt_vbc_err_7_0, buf,
		sizeof(buf));
	if (ret)
		goto error;
	snr_val = (buf[2] << 16) + (buf[1] << 8) + buf[0];

	/* read current constellation */
	ret = af9033_read_reg_bits(state, OFDM, g_reg_tpsd_const,
		reg_tpsd_const_pos, reg_tpsd_const_len, &buf[0]);
	if (ret)
		goto error;

	switch (buf[0]) {
	case 0:
		len = ARRAY_SIZE(qpsk_snr_table);
		snr_table = qpsk_snr_table;
		break;
	case 1:
		len = ARRAY_SIZE(qam16_snr_table);
		snr_table = qam16_snr_table;
		break;
	case 2:
		len = ARRAY_SIZE(qam64_snr_table);
		snr_table = qam64_snr_table;
		break;
	default:
		len = 0;
	}

	/* get snr from lookup table */
	for (i = 0; i < len; i++) {
		if (snr_val < snr_table[i].val) {
			state->snr = snr_table[i].snr * 10;
			break;
		}
	}
	deb_info("%s: snr_val:%x snr:%x\n", __func__, snr_val, state->snr);

	if (len && !af9033_snrdb)
		state->snr = (0xffff / (snr_table[len - 1].snr * 10)) * state->snr;

error:
	return ret;
}

static int af9033_update_signal_strength(struct dvb_frontend *fe)
{
	struct af9033_state *state = fe->demodulator_priv;
	int ret;
	u8 strength;

	/* read signal strength from 0-100 scale */
	ret = af9033_read_reg(state, OFDM, api_signal_strength, &strength);
	if (ret)
		goto error;

	/* scale value to 0x0000-0xffff */
	state->signal_strength = strength * 0xffff / 100;

error:
	return ret;
}

static int af9033_read_status(struct dvb_frontend *fe, fe_status_t *status)
{
	struct af9033_state *state = fe->demodulator_priv;
	int ret = 0;
	u8 tmp;
	*status = 0;

	/* empty channel; 0:no result, 1:signal, 2:empty */
	ret = af9033_read_reg(state, OFDM, api_empty_channel_status, &tmp);
	if (ret)
		goto error;
	if (tmp == 0x01) /* have signal */
		*status |= FE_HAS_SIGNAL;

	if (tmp != 0x02) {
		/* TPS lock */
		ret = af9033_read_reg_bits(state, OFDM, p_fd_tpsd_lock,
			fd_tpsd_lock_pos, fd_tpsd_lock_len, &tmp);
		if (ret)
			goto error;
		if (tmp)
			*status |= FE_HAS_VITERBI | FE_HAS_CARRIER;

		/* MPEG2 lock */
		ret = af9033_read_reg_bits(state, OFDM,
			r_mp2if_sync_byte_locked, mp2if_sync_byte_locked_pos,
			mp2if_sync_byte_locked_len, &tmp);
		if (ret)
			goto error;
		if (tmp)
			*status |= FE_HAS_SYNC | FE_HAS_LOCK;
	}

	/* update ber / ucblocks */
	ret = af9033_update_ber_ucblocks(fe);

error:
	if (ret)
		deb_info("%s: failed:%d\n", __func__, ret);

	return ret;
}

static int af9033_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct af9033_state *state = fe->demodulator_priv;
	int ret;
	deb_info("%s\n", __func__);
	ret = af9033_update_ber_ucblocks(fe);
	*ber = state->ber;
	return ret;
}

static int af9033_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct af9033_state *state = fe->demodulator_priv;
	int ret;
	deb_info("%s\n", __func__);
	ret = af9033_update_signal_strength(fe);
	if (ret)
		goto error;
	ret = af9033_update_ber_ucblocks(fe);
	*strength = state->signal_strength;
error:
	return ret;
}

static int af9033_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct af9033_state *state = fe->demodulator_priv;
	int ret;
	deb_info("%s\n", __func__);
	ret = af9033_update_snr(fe);
	if (ret)
		goto error;
	ret = af9033_update_ber_ucblocks(fe);
	*snr = state->snr;
error:
	return ret;
}

static int af9033_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	struct af9033_state *state = fe->demodulator_priv;
	int ret;
	deb_info("%s\n", __func__);
	ret = af9033_update_ber_ucblocks(fe);
	*ucblocks = state->ucblocks;
	return ret;
}

static int af9033_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct af9033_state *state = fe->demodulator_priv;
	deb_info("%s: enable:%d\n", __func__, enable);

	return af9033_write_reg_bits(state, LINK, p_reg_bypass_host2tuner,
		reg_bypass_host2tuner_pos, reg_bypass_host2tuner_len, enable);
}

static struct dvb_frontend_ops af9033_ops;

struct dvb_frontend *af9033_attach(const struct af9033_config *config,
	struct i2c_adapter *i2c)
{
	int ret;
	struct af9033_state *state = NULL;
	u8 buf[8];
	deb_info("%s:\n", __func__);

	/* allocate memory for the internal state */
	state = kzalloc(sizeof(struct af9033_state), GFP_KERNEL);
	if (state == NULL)
		goto error;

	/* setup the state */
	state->i2c = i2c;
	memcpy(&state->config, config, sizeof(struct af9033_config));

	/* firmware version */
	ret = af9033_read_regs(state, LINK, 0x83e9, &buf[0], sizeof(buf) / 2);
	if (ret)
		goto error;

	ret = af9033_read_regs(state, OFDM, 0x4191, &buf[4], sizeof(buf) / 2);
	if (ret)
		goto error;

	info("firmware version: LINK:%d.%d.%d.%d OFDM:%d.%d.%d.%d",
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

	/* settings for mp2if */
	if (state->config.output_mode == AF9033_TS_MODE_USB) {
		/* split 15 PSB to 1K + 1K and enable flow control */
		ret = af9033_write_reg_bits(state, OFDM, p_reg_mp2if2_half_psb,
			reg_mp2if2_half_psb_pos, reg_mp2if2_half_psb_len, 0);
		if (ret)
			goto error;
		ret = af9033_write_reg_bits(state, OFDM, p_reg_mp2if_stop_en,
			reg_mp2if_stop_en_pos, reg_mp2if_stop_en_len, 1);
	} else {
		/* AF9033 set mpeg to full speed */
		ret = af9033_write_reg_bits(state, OFDM, p_reg_mpeg_full_speed,
			reg_mpeg_full_speed_pos, reg_mpeg_full_speed_len, 0);
		if (ret)
			goto error;
		ret = af9033_write_reg_bits(state, OFDM, p_reg_mp2if_stop_en,
			reg_mp2if_stop_en_pos, reg_mp2if_stop_en_len, 0);
	}
	if (ret)
		goto error;

	/* set to 0 as open drain for tuner i2c */
	ret = af9033_write_reg(state, LINK, p_reg_top_padodpu, 0);
	if (ret)
		goto error;

	/* set to 0 as push pull for tuner AGC */
	ret = af9033_write_reg(state, LINK, p_reg_top_agc_od, 0);
	if (ret)
		goto error;

	/* create dvb_frontend */
	memcpy(&state->frontend.ops, &af9033_ops,
		sizeof(struct dvb_frontend_ops));
	state->frontend.demodulator_priv = state;

	return &state->frontend;
error:
	kfree(state);
	return NULL;
}
EXPORT_SYMBOL(af9033_attach);

static struct dvb_frontend_ops af9033_ops = {
#ifdef V4L2_ONLY_DVB_V5
	.delsys = { SYS_DVBT },
#endif
	.info = {
		.name = "Afatech AF9033 DVB-T",
#ifndef V4L2_ONLY_DVB_V5
		.type = FE_OFDM,
#endif
		.frequency_min =  44250000,
		.frequency_max = 867250000,
		.frequency_stepsize = 62500,
		.frequency_tolerance = 0,
		.caps =
			FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
			FE_CAN_FEC_5_6 | FE_CAN_FEC_7_8 | FE_CAN_FEC_AUTO |
			FE_CAN_QPSK | FE_CAN_QAM_16 |
			FE_CAN_QAM_64 | FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO |
			FE_CAN_HIERARCHY_AUTO |
			FE_CAN_RECOVER |
			FE_CAN_MUTE_TS
	},

	.release = af9033_release,

	.init = af9033_init,
	.sleep = af9033_sleep,

	.set_frontend = af9033_set_frontend,
	.get_tune_settings = af9033_get_tune_settings,

	.get_frontend = af9033_get_frontend,

	.read_status = af9033_read_status,
	.read_ber = af9033_read_ber,
	.read_signal_strength = af9033_read_signal_strength,
	.read_snr = af9033_read_snr,
	.read_ucblocks = af9033_read_ucblocks,

	.i2c_gate_ctrl = af9033_i2c_gate_ctrl,
};

MODULE_AUTHOR("Antti Palosaari <crope@iki.fi>");
MODULE_DESCRIPTION("Afatech AF9033 DVB-T demodulator driver");
MODULE_LICENSE("GPL");
