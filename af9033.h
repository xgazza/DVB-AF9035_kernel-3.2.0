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

#ifndef AF9033_H
#define AF9033_H

#include <linux/dvb/frontend.h>

enum af9033_ts_mode {
	AF9033_TS_MODE_PARALLEL,
	AF9033_TS_MODE_SERIAL,
	AF9033_TS_MODE_USB, /* only for AF9035 */
};

enum af9033_tuner {
	AF9033_TUNER_TUA9001    = 0x27, /* Infineon TUA 9001 */
	AF9033_TUNER_FC0011     = 0x28, /* Fitipower FC0011 */
	AF9033_TUNER_MXL5007t   = 0xa0, /* Maxlinear MXL5007t */
        AF9033_TUNER_TDA18218   = 0xa1, /* NXP TDA 18218HN */
};

/* clock setting table:
 =================================
 adc_clock  crystal_clock  Xtal
 =================================
 20480000   20480000      FPGA
 16384000   20480000      16.38MHz
 20480000   20480000      20.48MHz
 36000000   20250000      36.00MHz
 30000000   20156250      30.00MHz
 26000000   20583333      26.00MHz
 28000000   20416667      28.00MHz
 32000000   20500000      32.00MHz
 34000000   20187500      34.00MHz
 24000000   20500000      24.00MHz
 22000000   20625000      22.00MHz
 12000000   20250000      12.00MHz
*/

struct af9033_config {
	/* demodulator's I2C address */
	u8 demod_address;

	u8 tuner_address;

	/* xtal clock Hz */
	u32 crystal_clock;

	/* ADC clock Hz */
	u32 adc_clock;

	/* tuner ID */
	u8 tuner;

	/* intermediate frequency Hz */
	u32 if_freq;

	/* TS data output mode */
	u8 output_mode:2;

	/* RF spectrum inversion */
	u8 rf_spec_inv:1;
};


#if  defined(DETACHED_TERRATEC_MODULES) || \
     defined(CONFIG_DVB_AF9033) || \
	(defined(CONFIG_DVB_AF9033_MODULE) && defined(MODULE))
extern struct dvb_frontend *af9033_attach(const struct af9033_config *config,
	struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend *af9033_attach(
const struct af9033_config *config, struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif /* CONFIG_DVB_AF9033 */

#endif /* AF9033_H */
