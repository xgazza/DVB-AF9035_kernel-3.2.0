/*
 * Infineon TUA 9001 silicon tuner driver
 *
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
 */

#include <linux/slab.h>         /* for kzalloc/kfree */
#include <linux/version.h>
#include "tua9001.h"
#include "tua9001_priv.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,3,0)) || ((defined V4L2_VERSION) && (V4L2_VERSION >= 197120))
/* all DVB frontend drivers now work directly with the DVBv5
 * structure. This warrants that all drivers will be
 * getting/setting frontend parameters on a consistent way, in
 * order to avoid copying data from/to the DVBv3 structs
 * without need.
 */
#define V4L2_ONLY_DVB_V5
#endif

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug");

/* write register */
static int tua9001_writereg(struct tua9001_priv *priv, u8 reg, u16 val)
{
	u8 buf[3] = {reg, val >> 8, val & 0xff};
	struct i2c_msg msg = { .addr = priv->cfg->i2c_address,
		.flags = 0, .buf = buf, .len = 3 };

	if (i2c_transfer(priv->i2c, &msg, 1) != 1) {
		err("I2C write failed, reg:%02x", reg);
		return -EREMOTEIO;
	}
	return 0;
}

static int tua9001_release(struct dvb_frontend *fe)
{
	kfree(fe->tuner_priv);
	fe->tuner_priv = NULL;
	return 0;
}

static int tua9001_init(struct dvb_frontend *fe)
{
	struct tua9001_priv *priv = fe->tuner_priv;
	int ret = 0;
	u8 i;
	struct regdesc data[] = {
		{0x1e, 0x6512},
		{0x25, 0xb888},
		{0x39, 0x5460},
		{0x3b, 0x00c0},
		{0x3a, 0xf000},
		{0x08, 0x0000},
		{0x32, 0x0030},
		{0x41, 0x703a},
		{0x40, 0x1c78},
		{0x2c, 0x1c00},
		{0x36, 0xc013},
		{0x37, 0x6f18},
		{0x27, 0x0008},
		{0x2a, 0x0001},
		{0x34, 0x0a40},
	};

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1); /* open i2c-gate */

	for (i = 0; i < ARRAY_SIZE(data); i++) {
		ret = tua9001_writereg(priv, data[i].reg, data[i].val);
		if (ret)
			break;
	}

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0); /* close i2c-gate */

	if (ret)
		deb_info("%s: failed:%d\n", __func__, ret);

	return ret;
}

#ifdef V4L2_ONLY_DVB_V5
static int tua9001_set_params(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *params = &fe->dtv_property_cache;
	struct tua9001_priv *priv = fe->tuner_priv;
	int ret;
	u16 val;
	u32 freq;
	u8 i;
	struct regdesc data[2];

	switch (params->bandwidth_hz) {
#if 0
	case 5000000:
		val  = 0x3000;
		break;
#endif
	case 6000000:
		val  = 0x2000;
		break;
	case 7000000:
		val  = 0x1000;
		break;
	case 8000000:
	default:
		val  = 0x0000;
		break;
	}

	data[0].reg = 0x04;
	data[0].val = val;

freq = params->frequency;

#define OFFSET 150000000
freq = freq - OFFSET;
freq  = freq/1000;
freq  = 48 * freq;
freq  = freq/1000;

val = freq;

	data[1].reg = 0x1f;
	data[1].val = val;


	deb_info("%s: freq:%d bw:%d freq tuner:%d val:%d\n", __func__,
		params->frequency, params->bandwidth_hz, priv->frequency,
		val);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1); /* open i2c-gate */

	for (i = 0; i < ARRAY_SIZE(data); i++) {
		ret = tua9001_writereg(priv, data[i].reg, data[i].val);
		if (ret)
			break;
	}

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0); /* close i2c-gate */

	if (ret)
		deb_info("%s: failed:%d\n", __func__, ret);

	return ret;
}
#else
static int tua9001_set_params(struct dvb_frontend *fe,
	struct dvb_frontend_parameters *params)
{
	struct tua9001_priv *priv = fe->tuner_priv;
	int ret;
	u16 val;
	u32 freq;
	u8 i;
	struct regdesc data[2];

	switch (params->u.ofdm.bandwidth) {
#if 0
	case BANDWIDTH_5_MHZ:
		val  = 0x3000;
		break;
#endif
	case BANDWIDTH_6_MHZ:
		val  = 0x2000;
		break;
	case BANDWIDTH_7_MHZ:
		val  = 0x1000;
		break;
	case BANDWIDTH_8_MHZ:
	default:
		val  = 0x0000;
		break;
	}

	data[0].reg = 0x04;
	data[0].val = val;

freq = params->frequency;

#define OFFSET 150000000
freq = freq - OFFSET;
freq  = freq/1000;
freq  = 48 * freq;
freq  = freq/1000;

val = freq;

	data[1].reg = 0x1f;
	data[1].val = val;


	deb_info("%s: freq:%d bw:%d freq tuner:%d val:%d\n", __func__,
		params->frequency, params->u.ofdm.bandwidth, priv->frequency,
		val);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1); /* open i2c-gate */

	for (i = 0; i < ARRAY_SIZE(data); i++) {
		ret = tua9001_writereg(priv, data[i].reg, data[i].val);
		if (ret)
			break;
	}

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0); /* close i2c-gate */

	if (ret)
		deb_info("%s: failed:%d\n", __func__, ret);

	return ret;
}
#endif

static int tua9001_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct tua9001_priv *priv = fe->tuner_priv;
	*frequency = priv->frequency;
	return 0;
}

static const struct dvb_tuner_ops tua9001_tuner_ops = {
	.info = {
		.name           = "Infineon TUA 9001",

		.frequency_min  = 170000000,
		.frequency_max  = 860000000,
		.frequency_step = 0,
	},

	.release       = tua9001_release,
	.init          = tua9001_init,

	.set_params    = tua9001_set_params,

	.get_frequency = tua9001_get_frequency,
};

struct dvb_frontend * tua9001_attach(struct dvb_frontend *fe,
	struct i2c_adapter *i2c, struct tua9001_config *cfg)
{
	struct tua9001_priv *priv = NULL;

	priv = kzalloc(sizeof(struct tua9001_priv), GFP_KERNEL);
	if (priv == NULL)
		return NULL;

	priv->cfg = cfg;
	priv->i2c = i2c;

	info("Infineon TUA 9001 successfully attached.");

	memcpy(&fe->ops.tuner_ops, &tua9001_tuner_ops,
		sizeof(struct dvb_tuner_ops));

	fe->tuner_priv = priv;
	return fe;
}
EXPORT_SYMBOL(tua9001_attach);

MODULE_DESCRIPTION("Infineon TUA 9001 silicon tuner driver");
MODULE_AUTHOR("Antti Palosaari <crope@iki.fi>");
MODULE_LICENSE("GPL");
