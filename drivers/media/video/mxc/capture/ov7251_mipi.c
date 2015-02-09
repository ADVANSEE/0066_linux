/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <mach/mipi_csi2.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"
#include <linux/proc_fs.h>

#define OV7251_VOLTAGE_ANALOG               2800000
#define OV7251_VOLTAGE_DIGITAL_CORE         1500000
#define OV7251_VOLTAGE_DIGITAL_IO           1800000

#define MIN_FPS 100
#define MAX_FPS 100
#define DEFAULT_FPS 100

#define OV7251_XCLK_MIN 6000000
#define OV7251_XCLK_MAX 24000000

#define OV7251_CHIP_ID_HIGH_BYTE	0x300A
#define OV7251_CHIP_ID_LOW_BYTE		0x300B
#define OV7251_CHIP_ID_REV_BYTE		0x3029


enum ov7251_mode {
	ov7251_mode_MIN = 0,
	ov7251_mode_VGA_640_480 = 0,
	ov7251_mode_MAX = 0,
	ov7251_mode_INIT = 0xff, /*only for sensor init*/
};

enum ov7251_frame_rate {
	ov7251_100_fps
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ov7251_mode_info {
	enum ov7251_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

#define	CMD_MAIN	0x3022
#define	CMD_ACK		0x3023
#define	CMD_PARA0	0x3024
#define	CMD_PARA1	0x3025
#define	CMD_PARA2	0x3026
#define	CMD_PARA3	0x3027
#define	CMD_PARA4	0x3028
#define	FW_STATUS	0x3029

/*!
 * Maintains the information on the current state of the sensor.
 */
static struct sensor_data ov7251_data;

static struct reg_value ov7251_init_setting_100fps_VGA[] = {
 
/*   
    // revision 1F RAW 8
    {0x0103, 0x01,0,0},{0x0100, 0x00,0,0},{0x3005, 0x00,0,0},
    {0x3012, 0xc0,0,0},{0x3013, 0xd2,0,0},{0x3014, 0x04,0,0},
    {0x3016, 0x10,0,0},{0x3017, 0x00,0,0},{0x3018, 0x00,0,0},
    {0x301a, 0x00,0,0},{0x301b, 0x00,0,0},{0x301c, 0x00,0,0},
    {0x3023, 0x05,0,0},{0x3037, 0xf0,0,0},{0x3098, 0x04,0,0},
    {0x3099, 0x28,0,0},{0x309a, 0x05,0,0},{0x309b, 0x04,0,0},
    {0x30b0, 0x08,0,0},{0x30b1, 0x01,0,0},{0x30b3, 0x32,0,0},
    {0x30b4, 0x03,0,0},{0x30b5, 0x05,0,0},{0x3106, 0xda,0,0},
    {0x3500, 0x00,0,0},{0x3501, 0x1f,0,0},{0x3502, 0x80,0,0},
    {0x3503, 0x07,0,0},{0x3509, 0x10,0,0},{0x350b, 0x10,0,0},
    {0x3600, 0x1c,0,0},{0x3602, 0x62,0,0},{0x3620, 0xb7,0,0},
    {0x3622, 0x04,0,0},{0x3626, 0x21,0,0},{0x3627, 0x30,0,0},
    {0x3630, 0x44,0,0},{0x3631, 0x35,0,0},{0x3634, 0x60,0,0},
    {0x3636, 0x00,0,0},{0x3662, 0x03,0,0},{0x3663, 0x70,0,0},
    {0x3664, 0xf0,0,0},{0x3666, 0x0a,0,0},{0x3669, 0x1a,0,0},
    {0x366a, 0x00,0,0},{0x366b, 0x50,0,0},{0x3673, 0x01,0,0},
    {0x3674, 0xff,0,0},{0x3675, 0x03,0,0},{0x3705, 0xc1,0,0},
    {0x3709, 0x40,0,0},{0x373c, 0x08,0,0},{0x3742, 0x00,0,0},
    {0x3757, 0xb3,0,0},{0x3788, 0x00,0,0},{0x37a8, 0x01,0,0},
    {0x37a9, 0xc0,0,0},{0x3800, 0x00,0,0},{0x3801, 0x04,0,0},
    {0x3802, 0x00,0,0},{0x3803, 0x04,0,0},{0x3804, 0x02,0,0},
    {0x3805, 0x8b,0,0},{0x3806, 0x01,0,0},{0x3807, 0xeb,0,0},
    {0x3808, 0x02,0,0},{0x3809, 0x80,0,0},{0x380a, 0x01,0,0},
    {0x380b, 0xe0,0,0},{0x380c, 0x03,0,0},{0x380d, 0xa0,0,0},
    {0x380e, 0x02,0,0},{0x380f, 0x0a,0,0},{0x3810, 0x00,0,0},
    {0x3811, 0x04,0,0},{0x3812, 0x00,0,0},{0x3813, 0x05,0,0},
    {0x3814, 0x11,0,0},{0x3815, 0x11,0,0},{0x3820, 0x40,0,0},
    {0x3821, 0x00,0,0},{0x382f, 0x0e,0,0},{0x3832, 0x00,0,0},
    {0x3833, 0x05,0,0},{0x3834, 0x00,0,0},{0x3835, 0x0c,0,0},
    {0x3837, 0x00,0,0},{0x3b80, 0x00,0,0},{0x3b81, 0xa5,0,0},
    {0x3b82, 0x10,0,0},{0x3b83, 0x00,0,0},{0x3b84, 0x08,0,0},
    {0x3b85, 0x00,0,0},{0x3b86, 0x01,0,0},{0x3b87, 0x00,0,0},
    {0x3b88, 0x00,0,0},{0x3b89, 0x00,0,0},{0x3b8a, 0x00,0,0},
    {0x3b8b, 0x05,0,0},{0x3b8c, 0x00,0,0},{0x3b8d, 0x00,0,0},
    {0x3b8e, 0x00,0,0},{0x3b8f, 0x1a,0,0},{0x3b94, 0x05,0,0},
    {0x3b95, 0xf2,0,0},{0x3b96, 0x40,0,0},{0x3c00, 0x89,0,0},
    {0x3c01, 0x63,0,0},{0x3c02, 0x01,0,0},{0x3c03, 0x00,0,0},
    {0x3c04, 0x00,0,0},{0x3c05, 0x03,0,0},{0x3c06, 0x00,0,0},
    {0x3c07, 0x06,0,0},{0x3c0c, 0x01,0,0},{0x3c0d, 0xd0,0,0},
    {0x3c0e, 0x02,0,0},{0x3c0f, 0x0a,0,0},{0x4001, 0x42,0,0},
    {0x4004, 0x04,0,0},{0x4005, 0x00,0,0},{0x404e, 0x01,0,0},
    {0x4300, 0xff,0,0},{0x4301, 0x00,0,0},{0x4501, 0x48,0,0},
    {0x4600, 0x00,0,0},{0x4601, 0x4e,0,0},{0x4801, 0x0f,0,0},
    {0x4806, 0x0f,0,0},{0x4819, 0xaa,0,0},{0x4823, 0x3e,0,0},
    {0x4837, 0x19,0,0},{0x4a0d, 0x00,0,0},{0x4a47, 0x7f,0,0},
    {0x4a49, 0xf0,0,0},{0x4a4b, 0x30,0,0},{0x5000, 0x85,0,0},
    {0x5001, 0x80,0,0},{0x0100, 0x01,0,0}
 */ 

// revision 1D RAW 8
// gain = 16
// exposure = 9.576ms
  
    {0x0103, 0x01,0,0},{0x0100, 0x00,0,0},{0x3005, 0x00,0,0},
    {0x3012, 0xc0,0,0},{0x3013, 0xd2,0,0},{0x3014, 0x04,0,0},
    {0x3016, 0xf0,0,0},{0x3017, 0xf0,0,0},{0x3018, 0xf0,0,0},
    {0x301a, 0xf0,0,0},{0x301b, 0xf0,0,0},{0x301c, 0xf0,0,0},
    {0x3023, 0x07,0,0},{0x3037, 0xf0,0,0},{0x3098, 0x04,0,0},
    {0x3099, 0x28,0,0},{0x309a, 0x05,0,0},{0x309b, 0x04,0,0},
    {0x30b0, 0x08,0,0},{0x30b1, 0x01,0,0},{0x30b3, 0x32,0,0},
    {0x30b4, 0x03,0,0},{0x30b5, 0x05,0,0},{0x3106, 0x12,0,0},
    {0x3500, 0x00,0,0},{0x3501, 0x1f,0,0},{0x3502, 0x80,0,0}, 
    {0x3503, 0x07,0,0},{0x3509, 0x10,0,0},{0x350b, 0x10,0,0},
    {0x3600, 0x1c,0,0},{0x3602, 0x62,0,0},{0x3620, 0xb7,0,0},
    {0x3622, 0x04,0,0},{0x3626, 0x21,0,0},{0x3627, 0x30,0,0},
    {0x3634, 0x41,0,0},{0x3636, 0x00,0,0},{0x3662, 0x03,0,0},
    {0x3664, 0xf0,0,0},{0x3669, 0x1a,0,0},{0x366a, 0x00,0,0},
    {0x366b, 0x50,0,0},{0x3705, 0xc1,0,0},{0x3709, 0x40,0,0},
    {0x373c, 0x08,0,0},{0x3742, 0x00,0,0},{0x3788, 0x00,0,0},
    {0x37a8, 0x01,0,0},{0x37a9, 0xc0,0,0},{0x3800, 0x00,0,0},
    {0x3801, 0x04,0,0},{0x3802, 0x00,0,0},{0x3803, 0x04,0,0},
    {0x3804, 0x02,0,0},{0x3805, 0x8b,0,0},{0x3806, 0x01,0,0},
    {0x3807, 0xeb,0,0},{0x3808, 0x02,0,0},{0x3809, 0x80,0,0},
    {0x380a, 0x01,0,0},{0x380b, 0xe0,0,0},{0x380c, 0x03,0,0},
    {0x380d, 0xa0,0,0},{0x380e, 0x02,0,0},{0x380f, 0x04,0,0},
    {0x3810, 0x00,0,0},{0x3811, 0x04,0,0},{0x3812, 0x00,0,0},
    {0x3813, 0x05,0,0},{0x3814, 0x11,0,0},{0x3815, 0x11,0,0},
    {0x3820, 0x40,0,0},{0x3821, 0x00,0,0},{0x382f, 0xc4,0,0},
    {0x3832, 0xff,0,0},{0x3833, 0xff,0,0},{0x3834, 0x00,0,0},
    {0x3835, 0x05,0,0},{0x3837, 0x00,0,0},{0x3b80, 0x00,0,0},
    {0x3b81, 0xa5,0,0},{0x3b82, 0x10,0,0},{0x3b83, 0x00,0,0},
    {0x3b84, 0x08,0,0},{0x3b85, 0x00,0,0},{0x3b86, 0x01,0,0},
    {0x3b87, 0x00,0,0},{0x3b88, 0x00,0,0},{0x3b89, 0x00,0,0},
    {0x3b8a, 0x00,0,0},{0x3b8b, 0x05,0,0},{0x3b8c, 0x00,0,0},
    {0x3b8d, 0x00,0,0},{0x3b8e, 0x00,0,0},{0x3b8f, 0x1a,0,0},
    {0x3b94, 0x05,0,0},{0x3b95, 0xf2,0,0},{0x3b96, 0x40,0,0},
    {0x3c00, 0x89,0,0},{0x3c01, 0xab,0,0},{0x3c02, 0x01,0,0},
    {0x3c03, 0x00,0,0},{0x3c04, 0x00,0,0},{0x3c05, 0x03,0,0},
    {0x3c06, 0x00,0,0},{0x3c07, 0x05,0,0},{0x3c0c, 0x00,0,0},
    {0x3c0d, 0x00,0,0},{0x3c0e, 0x00,0,0},{0x3c0f, 0x00,0,0},
    {0x4001, 0xc2,0,0},{0x4004, 0x04,0,0},{0x4005, 0x20,0,0},
    {0x404e, 0x01,0,0},{0x4300, 0xff,0,0},{0x4301, 0x00,0,0},
    {0x4600, 0x00,0,0},{0x4601, 0x4e,0,0},
    {0x4800, 0x64,0,0},
    {0x4801, 0x0f,0,0},
    {0x4806, 0x0f,0,0},{0x4819, 0xaa,0,0},{0x4823, 0x3e,0,0},
    {0x4837, 0x19,0,0},{0x4a0d, 0x00,0,0},{0x5000, 0x85,0,0},
    {0x5001, 0x80,0,0},
    {0x0100, 0x01,0,0}


};


static struct ov7251_mode_info ov7251_mode_info_data[ov7251_mode_MAX + 1] = {
	{
		ov7251_mode_VGA_640_480,640,480,
		ov7251_init_setting_100fps_VGA,
		ARRAY_SIZE(ov7251_init_setting_100fps_VGA),
	},
};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;
static struct fsl_mxc_camera_platform_data *camera_plat;

static int ov7251_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ov7251_remove(struct i2c_client *client);

static s32 ov7251_read_reg(u16 reg, u8 *val);
static s32 ov7251_write_reg(u16 reg, u8 val);

static const struct i2c_device_id ov7251_id[] = {
	{"ov7251_mipi", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov7251_id);

static struct i2c_driver ov7251_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ov7251_mipi",
		  },
	.probe  = ov7251_probe,
	.remove = ov7251_remove,
	.id_table = ov7251_id,
};


static s32 update_device_addr(struct sensor_data *sensor)
{
	int ret;
	u8 buf[4];
	unsigned reg = 0x3100;
	unsigned default_addr = 0x3c;
	struct i2c_msg msg;

	if (sensor->i2c_client->addr == default_addr)
		return 0;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = sensor->i2c_client->addr << 1;
	msg.addr = default_addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;


	ret = i2c_transfer(sensor->i2c_client->adapter, &msg, 1);
	if (ret < 0)
		pr_debug("%s: ov7251_mipi ret=%d\n", __func__, ret);
	return ret;
}

static s32 ov7251_write_reg(u16 reg, u8 val)
{
	u8 au8Buf[3] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (i2c_master_send(ov7251_data.i2c_client, au8Buf, 3) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}
	pr_debug("reg=%x,val=%x\n", reg, val);
	return 0;
}

static s32 ov7251_read_reg(u16 reg, u8 *val)
{
	u8 au8RegBuf[2] = {0};
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (2 != i2c_master_send(ov7251_data.i2c_client, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (1 != i2c_master_recv(ov7251_data.i2c_client, &u8RdVal, 1)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;
	pr_debug("%s:write reg=%x / val=%x\n",__func__, reg, *val);

	return u8RdVal;
}

static s32 power_control(struct sensor_data *sensor, int on)
{
	struct fsl_mxc_camera_platform_data *plat = camera_plat;

	if (sensor->on != on) {
		if (plat->pwdn)
			plat->pwdn(on ? 0 : 1);
		sensor->on = on;
	}
	return 0;
}

void OV7251_stream_on(void)
{
	ov7251_write_reg(0x0100, 0x01);
}

void OV7251_stream_off(void)
{
	ov7251_write_reg(0x4202, 0x00);
}

static void ov7251_set_virtual_channel(int channel)
{
	u8 channel_id;

	ov7251_read_reg(0x4814, &channel_id);
	channel_id &= ~(3 << 6);
	ov7251_write_reg(0x4814, channel_id | (channel << 6));
}

/* download ov7251 settings to sensor through i2c */
static int ov7251_download_firmware(struct reg_value *pModeSetting, s32 ArySize)
{
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int i, retval = 0;

	for (i = 0; i < ArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = ov7251_read_reg(RegAddr, &RegVal);
			if (retval < 0)
				goto err;

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov7251_write_reg(RegAddr, Val);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			msleep(Delay_ms);
	}
err:
	return retval;
}


/* if sensor changes inside scaling or subsampling
 * change mode directly
 * */
static int ov7251_change_mode_direct(enum ov7251_mode mode)
{
	struct reg_value *pModeSetting = NULL;
	s32 ArySize = 0;
	int retval = 0;

	/* check if the input mode and frame rate is valid */
	pModeSetting = ov7251_mode_info_data[mode].init_data_ptr;
	ArySize = ov7251_mode_info_data[mode].init_data_size;

	ov7251_data.pix.width = ov7251_mode_info_data[mode].width;
	ov7251_data.pix.height = ov7251_mode_info_data[mode].height;

	if (ov7251_data.pix.width == 0 || ov7251_data.pix.height == 0 ||
		pModeSetting == NULL || ArySize == 0)
		return -EINVAL;

	OV7251_stream_off();

	/* Write capture setting */
	retval = ov7251_download_firmware(pModeSetting, ArySize);
	if (retval < 0)
		goto err;

	OV7251_stream_on();


err:
	return retval;
}

static int ov7251_init_mode( enum ov7251_mode mode)
{
	struct reg_value *pModeSetting = NULL;
	s32 ArySize = 0;
	int retval = 0;
	void *mipi_csi2_info;
	u32 mipi_reg, msec_wait4stable = 0;

	if ((mode > ov7251_mode_MAX || mode < ov7251_mode_MIN)
		&& (mode != ov7251_mode_INIT)) {
		pr_err("Wrong ov7251 mode detected!\n");
		return -1;
	}

	mipi_csi2_info = mipi_csi2_get_info();

	/* initial mipi dphy */
	if (mipi_csi2_info) {
		if (!mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_enable(mipi_csi2_info);

		if (mipi_csi2_get_status(mipi_csi2_info)) {
			mipi_csi2_set_lanes(mipi_csi2_info);

			/*Only reset MIPI CSI2 HW at sensor initialize*/
			if (mode == ov7251_mode_INIT)
			{
			      pr_debug("mipi_csi2_reset\n");
			      mipi_csi2_reset(mipi_csi2_info);
			}
			if (ov7251_data.pix.pixelformat == V4L2_PIX_FMT_GREY)
			{
				pr_debug("mipi_csi2_set_datatype\n");
				mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_RAW8);
			}
			else
				pr_err("currently this sensor format can not be supported!\n");
		} else {
			pr_err("Can not enable mipi csi2 driver!\n");
			return -1;
		}
	} else {
		printk(KERN_ERR "Fail to get mipi_csi2_info!\n");
		return -1;
	}

	if (mode == ov7251_mode_INIT) {
		pModeSetting = ov7251_init_setting_100fps_VGA;
		ArySize = ARRAY_SIZE(ov7251_init_setting_100fps_VGA);

		ov7251_data.pix.width = 640;
		ov7251_data.pix.height = 480;
		retval = ov7251_download_firmware(pModeSetting, ArySize);
	} else {
		/* change inside subsampling or scaling
		 * download firmware directly */
		retval = ov7251_change_mode_direct(mode);
	}
	if (retval < 0)
		goto err;
	
	pr_debug("ov7251 set virtual channel\n");
	ov7251_set_virtual_channel(ov7251_data.virtual_channel);

	/* add delay to wait for sensor stable */
	pr_debug("wait for sensor stable\n");
	msleep(msec_wait4stable);

	if (mipi_csi2_info) {
		unsigned int i;

		i = 0;

		/* wait for mipi sensor ready */
		pr_debug("wait for mipi sensor ready\n");
		mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
		while ((mipi_reg == 0x200) && (i < 10)) {
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
			i++;
			msleep(10);
		}

		if (i >= 10) {
			pr_err("mipi csi2 can not receive sensor clk!\n");
			return -1;
		}

		i = 0;
		pr_debug("wait for mipi stable\n");
		/* wait for mipi stable */
		mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
		while ((mipi_reg != 0x0) && (i < 10)) {
			mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
			i++;
			msleep(10);
		}

		if (i >= 10) {
			pr_err("mipi csi2 can not reveive data correctly!\n");
			return -1;
		}
		pr_debug("mipi receive data correctly\n");
	}
err:
	return retval;
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = ov7251_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", ov7251_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = OV7251_XCLK_MIN;
	p->u.bt656.clock_max = OV7251_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;

	if (on && !sensor->on) {
		if (io_regulator)
			if (regulator_enable(io_regulator) != 0)
				return -EIO;
		if (core_regulator)
			if (regulator_enable(core_regulator) != 0)
				return -EIO;
		if (gpo_regulator)
			if (regulator_enable(gpo_regulator) != 0)
				return -EIO;
		if (analog_regulator)
			if (regulator_enable(analog_regulator) != 0)
				return -EIO;
		/* Make sure power on */
		power_control(sensor, 1);

	} else if (!on && sensor->on) {
		if (analog_regulator)
			regulator_disable(analog_regulator);
		if (core_regulator)
			regulator_disable(core_regulator);
		if (io_regulator)
			regulator_disable(io_regulator);
		if (gpo_regulator)
			regulator_disable(gpo_regulator);

		power_control(sensor, 0);
	}
	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum ov7251_frame_rate frame_rate;
	int ret = 0;

	/* Make sure power on */
	power_control(sensor, 1);

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 100)
			frame_rate = ov7251_100_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}
		ret = ov7251_init_mode(	(u32)a->parm.capture.capturemode);
		if (ret < 0)
			return ret;

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	f->fmt.pix = sensor->pix;

	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = ov7251_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = ov7251_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = ov7251_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = ov7251_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = ov7251_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = ov7251_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = ov7251_data.ae_mode;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;
	u8 gain;
	u32 exposure;

	pr_debug("In ov7251:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_AUTO_FOCUS_START:
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		if( vc->value <= 20 ) // 20 µs
		    exposure = 1;
		  else if(  vc->value >= 1000000 ) // 1000000 µs
		    exposure = 1000000;
		  else
		    exposure = vc->value / 20;
	  
		ov7251_write_reg (0x3500,((exposure & 0xf000) >>12));
		ov7251_write_reg (0x3501,((exposure & 0x0ff0) >> 4));
		ov7251_write_reg (0x3502,((exposure & 0x000f) << 4));  
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		  if( vc->value <= 1 )
		    gain = 1;
		  else if(  vc->value >= 255 )
		    gain = 255;
		  else
		    gain = vc->value;
		ov7251_write_reg (0x350b,gain);
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > ov7251_mode_MAX)
		return -EINVAL;

	fsize->pixel_format = ov7251_data.pix.pixelformat;
	fsize->discrete.width = ov7251_mode_info_data[fsize->index].width;
	fsize->discrete.height = ov7251_mode_info_data[fsize->index].height;
	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
		"ov7251_mipi_camera");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{

	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	if (fmt->index > ov7251_mode_MAX)
		return -EINVAL;

	fmt->pixelformat = ov7251_data.pix.pixelformat;

	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct sensor_data *sensor = s->priv;
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	int ret;
	enum ov7251_frame_rate frame_rate;
	void *mipi_csi2_info;

	ov7251_data.on = true;

	/* mclk */
	tgt_xclk = ov7251_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)OV7251_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)OV7251_XCLK_MIN);
	ov7251_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
//	set_mclk_rate(&ov7251_data.mclk, ov7251_data.mclk_source);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	if (tgt_fps == 100)
		frame_rate = ov7251_100_fps;
	else
		return -EINVAL; /* Only support 15fps or 30fps now. */

	mipi_csi2_info = mipi_csi2_get_info();

	/* enable mipi csi2 */
	if (mipi_csi2_info)
		mipi_csi2_enable(mipi_csi2_info);
	else {
		printk(KERN_ERR "Fail to get mipi_csi2_info!\n");
		return -EPERM;
	}

	ret = ov7251_init_mode(ov7251_mode_INIT);

	return ret;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	void *mipi_csi2_info;

	mipi_csi2_info = mipi_csi2_get_info();

	/* disable mipi csi2 */
	if (mipi_csi2_info)
		if (mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_disable(mipi_csi2_info);

	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc ov7251_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*) ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*) ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*) ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func*) ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_s_fmt_cap}, */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *) ioctl_g_chip_ident},

};

static struct v4l2_int_slave ov7251_slave = {
	.ioctls = ov7251_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ov7251_ioctl_desc),
};

static struct v4l2_int_device ov7251_int_device = {
	.module = THIS_MODULE,
	.name = "ov7251_mipi",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ov7251_slave,
	},
};

static int write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char localbuf[256];
	if( count < sizeof(localbuf) ){
		if(copy_from_user(localbuf,buffer,count)){
			printk(KERN_ERR "Error reading user buf\n" );
		} else {
			int addr0 ;
			int value ;
			int numScanned ;
			if(2 == (numScanned = sscanf(localbuf,"%04x %02x", &addr0, &value)) ){
				if( (0xFFFF >= addr0) && (0xff >= value) ){
					s32 rval ;

					rval = ov7251_write_reg(addr0,value);
                                        if (rval < 0)
						pr_err("%s, write reg 0x%x failed: %d\n", __func__, addr0, rval);
					else
                                                pr_err("ov7251[%04x] = %02x\n", addr0,value);
				}
				else
					printk(KERN_ERR "Invalid data: %s\n", localbuf);
			} else if(1 == numScanned){
				if(0xFFFF > addr0){
					s32 rval;
					u8 value;
					rval = ov7251_read_reg(addr0,&value);
					if (0 == rval) {
						pr_err("ov7251[%04x] == 0x%02x\n", addr0,value);
					} else {
						pr_err("%s, read reg 0x%x failed: %d\n", __func__, addr0, rval);
					}
				}
			}
			else
				printk(KERN_ERR "Invalid data: %s\n", localbuf);
		}
	}

	return count ;
}

/*!
 * ov7251 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov7251_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sensor_data *sensor = &ov7251_data;
	int retval;
	struct fsl_mxc_camera_platform_data *plat_data = client->dev.platform_data;
	u8 chip_id_high, chip_id_low, chip_rev;
        struct proc_dir_entry *pde;

	/* Set initial values for the sensor struct. */
	memset(&ov7251_data, 0, sizeof(ov7251_data));

	sensor->mipi_camera = 1;
	ov7251_data.mclk = 24000000; /* 6 - 54 MHz, typical 24MHz */
	ov7251_data.mclk = plat_data->mclk;
	ov7251_data.mclk_source = plat_data->mclk_source;
	ov7251_data.ipu = plat_data->ipu;
	ov7251_data.csi = plat_data->csi;
	ov7251_data.io_init = plat_data->io_init;
	sensor->virtual_channel = sensor->csi | (sensor->ipu << 1);

	ov7251_data.i2c_client = client;
	ov7251_data.pix.pixelformat = V4L2_PIX_FMT_GREY;
	ov7251_data.pix.width = 640;
	ov7251_data.pix.height = 480;
	ov7251_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	ov7251_data.streamcap.capturemode = 0;
	ov7251_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ov7251_data.streamcap.timeperframe.numerator = 1;
	camera_plat = plat_data;

	if (plat_data->io_regulator) {
		io_regulator = regulator_get(&client->dev,
					     plat_data->io_regulator);
		if (!IS_ERR(io_regulator)) {
			regulator_set_voltage(io_regulator,
					      OV7251_VOLTAGE_DIGITAL_IO,
					      OV7251_VOLTAGE_DIGITAL_IO);
			retval = regulator_enable(io_regulator);
			if (retval) {
				pr_err("%s:io set voltage error\n", __func__);
				goto err1;
			} else {
				dev_dbg(&client->dev,
					"%s:io set voltage ok\n", __func__);
			}
		} else
			io_regulator = NULL;
	}

	if (plat_data->core_regulator) {
		core_regulator = regulator_get(&client->dev,
					       plat_data->core_regulator);
		if (!IS_ERR(core_regulator)) {
			regulator_set_voltage(core_regulator,
					      OV7251_VOLTAGE_DIGITAL_CORE,
					      OV7251_VOLTAGE_DIGITAL_CORE);
			retval = regulator_enable(core_regulator);
			if (retval) {
				pr_err("%s:core set voltage error\n", __func__);
				goto err2;
			} else {
				dev_dbg(&client->dev,
					"%s:core set voltage ok\n", __func__);
			}
		} else
			core_regulator = NULL;
	}

	if (plat_data->analog_regulator) {
		analog_regulator = regulator_get(&client->dev,
						 plat_data->analog_regulator);
		if (!IS_ERR(analog_regulator)) {
			regulator_set_voltage(analog_regulator,
					      OV7251_VOLTAGE_ANALOG,
					      OV7251_VOLTAGE_ANALOG);
			retval = regulator_enable(analog_regulator);
			if (retval) {
				pr_err("%s:analog set voltage error\n",
					__func__);
				goto err3;
			} else {
				dev_dbg(&client->dev,
					"%s:analog set voltage ok\n", __func__);
			}
		} else
			analog_regulator = NULL;
	}

	if (plat_data->io_init)
		plat_data->io_init();

	if (plat_data->lock)
		plat_data->lock();
	power_control(&ov7251_data, 1);
	update_device_addr(&ov7251_data);
	if (plat_data->unlock)
		plat_data->unlock();

	retval = ov7251_read_reg(OV7251_CHIP_ID_HIGH_BYTE, &chip_id_high);
	if (retval < 0 || chip_id_high != 0x77) {
		pr_warning("camera ov7251_mipi is not found\n");
		retval = -ENODEV;
		goto err4;
	}
	retval = ov7251_read_reg(OV7251_CHIP_ID_LOW_BYTE, &chip_id_low);
	if (retval < 0 || chip_id_low != 0x50) {
		pr_warning("camera ov7251_mipi is not found\n");
		retval = -ENODEV;
		goto err4;
	}

	retval = ov7251_read_reg(OV7251_CHIP_ID_REV_BYTE, &chip_rev);
	if (retval < 0 || chip_rev != 0x50) {
		pr_warning("camera ov7251_mipi revison is not good\n");
		retval = -ENODEV;
		goto err4;
	}
	
	
	
	power_control(&ov7251_data, 0);

	ov7251_int_device.priv = &ov7251_data;
	retval = v4l2_int_device_register(&ov7251_int_device);

	pde = create_proc_entry("driver/ov7251", 0, 0);
	if( pde ) {
		pde->write_proc = write_proc ;
		pde->data = &ov7251_int_device ;
	}
	else
		printk( KERN_ERR "Error creating ov7251 proc entry\n" );

	pr_info("camera ov7251_mipi is found\n");
	return retval;

err4:
	if (analog_regulator) {
		regulator_disable(analog_regulator);
		regulator_put(analog_regulator);
	}
err3:
	if (core_regulator) {
		regulator_disable(core_regulator);
		regulator_put(core_regulator);
	}
err2:
	if (io_regulator) {
		regulator_disable(io_regulator);
		regulator_put(io_regulator);
	}
err1:
	return retval;
}

/*!
 * ov7251 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov7251_remove(struct i2c_client *client)
{
	pr_err("%s\n",__func__);
	remove_proc_entry("driver/ov7251", NULL);

	v4l2_int_device_unregister(&ov7251_int_device);

	if (gpo_regulator) {
		regulator_disable(gpo_regulator);
		regulator_put(gpo_regulator);
	}

	if (analog_regulator) {
		regulator_disable(analog_regulator);
		regulator_put(analog_regulator);
	}

	if (core_regulator) {
		regulator_disable(core_regulator);
		regulator_put(core_regulator);
	}

	if (io_regulator) {
		regulator_disable(io_regulator);
		regulator_put(io_regulator);
	}

	return 0;
}

/*!
 * ov7251 init function
 * Called by insmod ov7251_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int ov7251_init(void)
{
	u8 err;
	
	pr_err("%s\n",__func__);
	
	err = i2c_add_driver(&ov7251_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d\n",
			__func__, err);

	return err;
}

/*!
 * OV7251 cleanup function
 * Called on rmmod ov7251_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit ov7251_clean(void)
{
	pr_err("%s\n",__func__);	
	i2c_del_driver(&ov7251_i2c_driver);
}

module_init(ov7251_init);
module_exit(ov7251_clean);

MODULE_AUTHOR("Advansee SARL.");
MODULE_DESCRIPTION("OV7251 MIPI Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
