/*
 * Afatech AF9035 DVB USB driver
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

#ifndef AF9035_H
#define AF9035_H
#include "af9033_reg.h"

#define DVB_USB_LOG_PREFIX "af9035"
#include "dvb-usb.h"

#define deb_info(args...) dprintk(dvb_usb_af9035_debug, 0x01, args)
#define deb_rc(args...)   dprintk(dvb_usb_af9035_debug, 0x02, args)
#define deb_xfer(args...) dprintk(dvb_usb_af9035_debug, 0x04, args)
#define deb_reg(args...)  dprintk(dvb_usb_af9035_debug, 0x08, args)
#define deb_i2c(args...)  dprintk(dvb_usb_af9035_debug, 0x10, args)
#define deb_fw(args...)   dprintk(dvb_usb_af9035_debug, 0x20, args)

#define AF9035_USB_TIMEOUT 2000

#define LINK 0x00
#define OFDM 0x80

#define TS_MODE_SINGLE   0
#define TS_MODE_DCA_PIP  1
#define TS_MODE_DCA      2 /* any other value than 0, 1, 3 (?) */
#define TS_MODE_PIP      3

#define TS_PACKET_SIZE            188
#define TS_USB20_PACKET_COUNT     348
#define TS_USB20_FRAME_SIZE       (TS_PACKET_SIZE*TS_USB20_PACKET_COUNT)
#define TS_USB11_PACKET_COUNT      21
#define TS_USB11_FRAME_SIZE       (TS_PACKET_SIZE*TS_USB11_PACKET_COUNT)
#define TS_USB20_MAX_PACKET_SIZE  512
#define TS_USB11_MAX_PACKET_SIZE   64

/* EEPROM locations */
#define GANY_ONLY 0x42f5
#define EEPROM_FLB_OFS  8
#define EEPROM_BASE_ADDR  (GANY_ONLY + EEPROM_FLB_OFS)
#define EEPROM_SHIFT      (0x10)

#define EEPROM_IR_MODE    (EEPROM_BASE_ADDR+0x10)   /* 00:disabled, 01:HID */
#define EEPROM_SELSUSPEND (EEPROM_BASE_ADDR+0x28)   /* selective suspend mode */
#define EEPROM_TS_MODE    (EEPROM_BASE_ADDR+0x28+1) /* 0:one ts, 1:dual ts */
#define EEPROM_2WIREADDR  (EEPROM_BASE_ADDR+0x28+2) /* 2nd demod I2C addr */
#define EEPROM_SUSPEND    (EEPROM_BASE_ADDR+0x28+3) /* suspend mode */
#define EEPROM_IR_TYPE    (EEPROM_BASE_ADDR+0x28+4) /* 0:NEC, 1:RC6 */

#define EEPROM_SAW_BW1    (EEPROM_BASE_ADDR+0x28+5)
#define EEPROM_XTAL1      (EEPROM_BASE_ADDR+0x28+6)
#define EEPROM_SPECINV1   (EEPROM_BASE_ADDR+0x28+7)
#define EEPROM_TUNER_ID1  (EEPROM_BASE_ADDR+0x30+4)
#define EEPROM_IFFREQL1   (EEPROM_BASE_ADDR+0x30)
#define EEPROM_IFFREQH1   (EEPROM_BASE_ADDR+0x30+1)
#define EEPROM_IF1L1      (EEPROM_BASE_ADDR+0x30+2)
#define EEPROM_IF1H1      (EEPROM_BASE_ADDR+0x30+3)

#define EEPROM_SAW_BW2    (EEPROM_BASE_ADDR+EEPROM_SHIFT+0x28+5)
#define EEPROM_XTAL2      (EEPROM_BASE_ADDR+EEPROM_SHIFT+0x28+6)
#define EEPROM_SPECINV2   (EEPROM_BASE_ADDR+EEPROM_SHIFT+0x28+7)
#define EEPROM_TUNER_ID2  (EEPROM_BASE_ADDR+EEPROM_SHIFT+0x30+4)
#define EEPROM_IFFREQL2   (EEPROM_BASE_ADDR+EEPROM_SHIFT+0x30)
#define EEPROM_IFFREQH2   (EEPROM_BASE_ADDR+EEPROM_SHIFT+0x30+1)
#define EEPROM_IF1L2      (EEPROM_BASE_ADDR+EEPROM_SHIFT+0x30+2)
#define EEPROM_IF1H2      (EEPROM_BASE_ADDR+EEPROM_SHIFT+0x30+3)

struct af9035_clock {
	u32 crystal;
	u32 adc;
};

static struct af9035_clock clock_table[] = {
	{20480000, 20480000},  /*     FPGA */
	{16384000, 20480000},  /* 16.38MHz */
	{20480000, 20480000},  /* 20.48MHz */
	{36000000, 20250000},  /* 36.00MHz */
	{30000000, 20156250},  /* 30.00MHz */
	{26000000, 20583333},  /* 26.00MHz */
	{28000000, 20416667},  /* 28.00MHz */
	{32000000, 20500000},  /* 32.00MHz */
	{34000000, 20187500},  /* 34.00MHz */
	{24000000, 20500000},  /* 24.00MHz */
	{22000000, 20625000},  /* 22.00MHz */
	{12000000, 20250000},  /* 12.00MHz */
};

struct af9035_req {
	u8  cmd;
	u8  mbox;
	u8  wlen;
	u8  *wbuf;
	u8  rlen;
	u8  *rbuf;
};

/* USB commands */
#define CMD_REG_DEMOD_READ          0x00
#define CMD_REG_DEMOD_WRITE         0x01
#define CMD_REG_TUNER_READ          0x02
#define CMD_REG_TUNER_WRITE         0x03
#define CMD_REG_EEPROM_READ         0x04
#define CMD_REG_EEPROM_WRITE        0x05
#define CMD_VAR_READ                0x08
#define CMD_VAR_WRITE               0x09

#define CMD_DATA_READ               0x06

#define CMD_PLATFORM_GET            0x0A
#define CMD_PLATFORM_SET            0x0B
#define CMD_IP_CACHE                0x0D
#define CMD_IP_ADD                  0x0E
#define CMD_IP_REMOVE               0x0F
#define CMD_PID_ADD                 0x10
#define CMD_PID_REMOVE              0x11
/* get SI/PSI table for specific PID "once" */
#define CMD_SIPSI_GET               0x12
#define CMD_SIPSI_MPE_RESET         0x13
#define CMD_H_PID_ADD               0x15
#define CMD_H_PID_REMOVE            0x16
#define CMD_ABORT                   0x17
#define CMD_IR_GET                  0x18
#define CMD_IR_SET                  0x19
#define CMD_FW_DOWNLOAD_BEGIN       0x24
#define CMD_FW_DOWNLOAD             0x21
#define CMD_FW_DOWNLOAD_END         0x25
#define CMD_QUERYINFO               0x22
#define CMD_BOOT                    0x23
#define CMD_REBOOT                  0x23
#define CMD_RUN_CODE                0x26
#define CMD_SCATTER_READ            0x28
#define CMD_SCATTER_WRITE           0x29
#define CMD_GENERIC_READ            0x2A
#define CMD_GENERIC_WRITE           0x2B

#define CMD_SERVICES_GET            0x83
#define CMD_COMPONENT_ADD           0x86
#define CMD_COMPONENT_REMOVE        0x87
#define CMD_FIG_ADD                 0x88
#define CMD_FIG_REMOVE              0x89

/* this Linux driver does not implement usage of "short command" at all */
#define CMD_SHORT_REG_DEMOD_READ    0x02
#define CMD_SHORT_REG_DEMOD_WRITE   0X03
#define CMD_SHORT_REG_TUNER_READ    0x04
#define CMD_SHORT_REG_TUNER_WRITE   0X05

struct af9035_config {
	u8 dual_mode:1;
	u16 mt2060_if1[2];
};

struct af9035_segment {
#define SEGMENT_FW_DOWNLOAD 0
#define SEGMENT_ROM_COPY    1
#define SEGMENT_DIRECT_CMD  2
	u8 type;
	u32 len;
};

struct af9035_firmware_header {
#define SEGMENT_MAX_COUNT 50
	u8 segment_count;
	struct af9035_segment segment[SEGMENT_MAX_COUNT];
};

#endif
