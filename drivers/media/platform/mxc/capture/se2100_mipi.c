/*
 * Copyright (C) 2011-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2017 Neuroptics. All Rights Reserved.
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
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <linux/mipi_csi2.h>
#include <linux/miscdevice.h>
#include <media/v4l2-chip-ident.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"


//static int se2100_download_firmware(struct reg_value *pModeSetting, s32 ArySize);


#define se2100_XCLK_MIN 	6000000
#define se2100_XCLK_MAX 	24000000
#define SE2100_MODE_MAX 	0

#define SE2100_I2C_DEVICE_ADDR		0x6e
static const struct v4l2_frmsize_discrete se2100_frmsizes[SE2100_MODE_MAX + 1] = {
    {  640, 480 },
};
struct mipi_csi2_info {
    bool            mipi_en;
    int             ipu_id;
    unsigned int    csi_id;
    unsigned int    v_channel;
    unsigned int    lanes;
    unsigned int    datatype;
    struct clk      *cfg_clk;
    struct clk      *dphy_clk;
    struct clk      *pixel_clk;
    void __iomem    *mipi_csi2_base;
    struct platform_device  *pdev;

    struct mutex mutex_lock;
};

struct se2100 {
	//struct v4l2_subdev subdev;
	struct i2c_client *i2c_client;


	u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;
	int csi;
	int ipu;
	int vc;
	atomic_t open_excl;
};

static struct sensor_data se2100_data;
static int pwdn_gpio, pwr_en_gpio,mclk_en_gpio;
static struct se2100* se2100_misc_wa = NULL;

struct reg_value {
	u16 u8RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

static int se2100_probe(struct i2c_client *adapter, const struct i2c_device_id *device_id);
static int se2100_remove(struct i2c_client *client);
static int se2100_suspend(struct device *dev);
static int se2100_resume(struct device *dev);

static s32 se2100_read_reg(u8 reg, u8 *val);
static s32 se2100_write_reg(u8 reg, u8 val);


static struct reg_value se2100_barcode[] = {
	{0x15, 0x12, 0, 0}, 
	{0x12, 0x00, 0, 0}, {0x3a, 0x20, 0, 0}, {0x2b, 0x10, 0, 0},
	{0x01, 0x15, 0, 0}, {0x02, 0x23, 0, 0}, {0x9d, 0x20, 0, 0},
	{0x23, 0x55, 0, 0}, {0x5d, 0xb3, 0, 0}, {0xbf, 0x08, 0, 0},
	{0xca, 0x10, 0, 0}, {0x64, 0x00, 0, 0}, {0xbb, 0x10, 0, 0},
	{0x20, 0x09, 0, 0}, {0x21, 0x4f, 0, 0}, {0x3e, 0x83, 0, 0},
	{0x2f, 0x04, 0, 0}, {0x16, 0xa3, 0, 0}, {0x13, 0x08, 0, 0},
	{0x87, 0x29, 0, 0}, {0x8a, 0x08, 0, 0}, {0x8b, 0x7d, 0, 0},
	{0x8c, 0x00, 0, 0}, {0x8d, 0xf8, 0, 0}, {0x24, 0x59, 0, 0},
	{0x97, 0x4c, 0, 0}, {0x98, 0x2c, 0, 0}, {0x82, 0x2a, 0, 0},
	{0x83, 0x54, 0, 0}, {0x86, 0x88, 0, 0}, {0x89, 0xfb, 0, 0},
	{0x94, 0x82, 0, 0}, {0x96, 0xb3, 0, 0}, {0x13, 0x09, 0, 0},
	{0x6c, 0xc2, 0, 0}, {0x27, 0x98, 0, 0}, {0x71, 0x00, 0, 0},
	{0x7e, 0x84, 0, 0}, {0x61, 0xf2, 0, 0}, {0xd9, 0x25, 0, 0},
	{0xdf, 0x26, 0, 0}, {0x03, 0xa0, 0, 0}, {0x70, 0x00, 0, 0},
	{0xf1, 0x6b, 0, 0},
};


static struct reg_value se2100_barcode_cam[] = {
	{0x15, 0x12, 0, 0}, {0x09, 0x01, 0, 0},
	{0x3a, 0x20, 0, 0}, {0x1e, 0x40, 0, 0},
	{0x1b, 0x0e, 0, 0}, {0x2a, 0x00, 0, 0}, {0x2b, 0x10, 0, 0},
	{0x92, 0x09, 0, 0}, {0x93, 0x00, 0, 0}, {0x8a, 0x96, 0, 0},
	{0x8b, 0x7d, 0, 0}, {0x13, 0x00, 0, 0},
	{0x01, 0x15, 0, 0}, {0x02, 0x23, 0, 0}, {0x9d, 0x20, 0, 0},
	{0x8c, 0x02, 0, 0}, {0x8d, 0xee, 0, 0}, {0x13, 0x07, 0, 0},
	{0x5d, 0xb3, 0, 0}, {0xbf, 0x08, 0, 0}, {0xc3, 0x08, 0, 0},
	{0xca, 0x10, 0, 0},
	{0x62, 0x00, 0, 0},
	{0x63, 0x00, 0, 0},
	{0xb9, 0x00, 0, 0},
	{0x64, 0x00, 0, 0},
	{0xbb, 0x10, 0, 0},
	{0x08, 0x20, 0, 0},
	{0x20, 0x09, 0, 0},
	{0x21, 0x4f, 0, 0},
	{0x3e, 0x83, 0, 0},
	{0x2f, 0x04, 0, 0},
	{0x16, 0xa3, 0, 0},
	{0x6c, 0xc2, 0, 0},
	{0x27, 0x98, 0, 0},
	{0x71, 0x0f, 0, 0},
	{0x7e, 0x84, 0, 0},
	{0x7f, 0x3c, 0, 0},
	{0x60, 0xe5, 0, 0},
	{0x61, 0xf2, 0, 0},
	{0x6d, 0xc0, 0, 0},
	{0xd9, 0x25, 0, 0},
	{0xdf, 0x26, 0, 0},
	{0x17, 0x00, 0, 0},
	{0x18, 0xa0, 0, 0},
	{0x19, 0x00, 0, 0},
	{0x1a, 0x78, 0, 0},
	{0x03, 0xa0, 0, 0},
	{0x4a, 0x0c, 0, 0},
	{0xda, 0x00, 0, 0},
	{0xdb, 0xa2, 0, 0},
	{0xdc, 0x00, 0, 0},
	{0xdd, 0x7a, 0, 0},
	{0xde, 0x00, 0, 0},
	{0x34, 0x1d, 0, 0},
	{0x36, 0x45, 0, 0},
	{0x6e, 0x20, 0, 0},
	{0xbc, 0x0d, 0, 0},
	{0x35, 0x30, 0, 0},
	{0x65, 0x2a, 0, 0},
	{0x66, 0x2a, 0, 0},
	{0xbd, 0xf4, 0, 0},
	{0xbe, 0x44, 0, 0},
	{0x9b, 0xf4, 0, 0},
	{0x9c, 0x44, 0, 0},
	{0x37, 0xf4, 0, 0},
	{0x38, 0x44, 0, 0},
	{0x70, 0x0b, 0, 0},
	{0x73, 0x27, 0, 0},
	{0x79, 0x24, 0, 0},
	{0x7a, 0x12, 0, 0},
	{0x75, 0x8a, 0, 0},
	{0x76, 0x98, 0, 0},
	{0x77, 0x2a, 0, 0},
	{0x7b, 0x58, 0, 0},
	{0x7d, 0x00, 0, 0},
	{0x13, 0x07, 0, 0},
	{0x24, 0x4a, 0, 0},
	{0x25, 0x88, 0, 0},
	{0x97, 0x3c, 0, 0},
	{0x98, 0x8a, 0, 0},
	{0x80, 0x92, 0, 0},
	{0x81, 0x00, 0, 0},
	{0x82, 0x2a, 0, 0},
	{0x83, 0x54, 0, 0},
	{0x84, 0x39, 0, 0},
	{0x85, 0x5d, 0, 0},
	{0x86, 0x88, 0, 0},
	{0x89, 0x63, 0, 0},
	{0x94, 0x82, 0, 0},
	{0x96, 0xb3, 0, 0},
	{0x9a, 0x50, 0, 0},
	{0x99, 0x10, 0, 0},
	{0x9f, 0x64, 0, 0},
	{0x39, 0x9c, 0, 0},
	{0x3f, 0x9c, 0, 0},
	{0x90, 0x20, 0, 0},
	{0x91, 0xd0, 0, 0},
	{0x40, 0x3b, 0, 0},
	{0x41, 0x36, 0, 0},
	{0x42, 0x2b, 0, 0},
	{0x43, 0x1d, 0, 0},
	{0x44, 0x1a, 0, 0},
	{0x45, 0x14, 0, 0},
	{0x46, 0x11, 0, 0},
	{0x47, 0x0f, 0, 0},
	{0x48, 0x0e, 0, 0},
	{0x49, 0x0d, 0, 0},
	{0x4b, 0x0c, 0, 0},
	{0x4c, 0x0b, 0, 0},
	{0x4e, 0x0a, 0, 0},
	{0x4f, 0x09, 0, 0},
	{0x50, 0x09, 0, 0},
	{0x5a, 0x56, 0, 0},
	{0x51, 0x13, 0, 0},
	{0x52, 0x05, 0, 0},
	{0x53, 0x91, 0, 0},
	{0x54, 0x72, 0, 0},
	{0x57, 0x96, 0, 0},
	{0x58, 0x35, 0, 0},
	{0x5a, 0xd6, 0, 0},
	{0x51, 0x28, 0, 0},
	{0x52, 0x35, 0, 0},
	{0x53, 0x9e, 0, 0},
	{0x54, 0x7d, 0, 0},
	{0x57, 0x50, 0, 0},
	{0x58, 0x15, 0, 0},
	{0x5c, 0x26, 0, 0},
	{0x6a, 0x81, 0, 0},
	{0x23, 0x55, 0, 0},
	{0xa1, 0x31, 0, 0},
	{0xa2, 0x0d, 0, 0},
	{0xa3, 0x27, 0, 0},
	{0xa4, 0x0a, 0, 0},
	{0xa5, 0x2c, 0, 0},
	{0xa6, 0x04, 0, 0},
	{0xa7, 0x1a, 0, 0},
	{0xa8, 0x18, 0, 0},
	{0xa9, 0x13, 0, 0},
	{0xaa, 0x18, 0, 0},
	{0xab, 0x1c, 0, 0},
	{0xac, 0x3c, 0, 0},
	{0xad, 0xf0, 0, 0},
	{0xae, 0x57, 0, 0},
	{0xd0, 0x92, 0, 0},
	{0xd1, 0x00, 0, 0},
	{0xd2, 0x58, 0, 0},
	{0xc5, 0xaa, 0, 0},
	{0xc6, 0x88, 0, 0},
	{0xc7, 0x10, 0, 0},
	{0xc8, 0x0d, 0, 0},
	{0xc9, 0x10, 0, 0},
	{0xd3, 0x09, 0, 0},
	{0xd4, 0x24, 0, 0},
	{0xee, 0x30, 0, 0},
	{0xb0, 0xe0, 0, 0},
	{0xb3, 0x48, 0, 0},
	{0xb4, 0xe3, 0, 0},
	{0xb1, 0xf0, 0, 0},
	{0xb2, 0xa0, 0, 0},
	{0xb4, 0x63, 0, 0},
	{0xb1, 0xb0, 0, 0},
	{0xb2, 0xa0, 0, 0},
	{0x55, 0x00, 0, 0},
	{0x56, 0x40, 0, 0},
	{0xf1, 0x00, 0, 0},
};


static s32 se2100_write_reg(u8 reg, u8 val)
{
	u8 au8Buf[2] = {0};

	au8Buf[0] = reg;
	au8Buf[1] = val;

	//register u8 RegAddr = 0;
	u8 RegVal = 0;
	int retval = 0;


	//printk("se2100 write client %p\n", se2100_data.i2c_client);
	if (i2c_master_send(se2100_data.i2c_client, au8Buf, 2) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}
	//pr_err("%s:write sucess :reg=%x,val=%x\n",__func__, reg, val);

	retval = se2100_read_reg(reg, &RegVal);
	
	return 0;
}

static s32 se2100_read_reg(u8 reg, u8 *val)
{
	u8 au8RegBuf[2] = {0};
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg;
	//printk("se2100 read client %p\n", se2100_data.i2c_client);

	if (1 != i2c_master_send(se2100_data.i2c_client, au8RegBuf, 1)) {
		pr_err("%s:read reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (1 != i2c_master_recv(se2100_data.i2c_client, &u8RdVal, 1)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;
	//printk("%s:read success:reg=%x,val=%x\n",__func__, reg, u8RdVal);

	return u8RdVal;
}

static void reset_regs(void)
{
	se2100_write_reg(0x12, 0x80);  //resetting the registers
	mdelay(1);
	se2100_write_reg(0x09, 0x01);  //disabling suspend mode
}

static int se2100_download_firmware(struct reg_value *pModeSetting, s32 ArySize)
{
	register u32 Delay_ms = 0;
	register u8 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int i, retval = 0;

	for (i = 0; i < ArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u8RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = se2100_read_reg(RegAddr, &RegVal);
			if (retval < 0)
				goto err;

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = se2100_write_reg(RegAddr, Val);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			msleep(Delay_ms);
	}
err:
	return retval;
}

static int barcode_resume(void)
{
	s32 ArySize = 0;
	int retval = 0;
	struct reg_value *pModeSetting = NULL;
	
	//printk(KERN_INFO"resumeeeeee\n");

	pModeSetting = se2100_barcode;
	ArySize = ARRAY_SIZE(se2100_barcode);
	retval = se2100_download_firmware(pModeSetting, ArySize);
	
	return retval;

}

static int barcode_suspend(void)
{
	int retval = 0;
	//printk(KERN_INFO"suspendinggggg\n");
	gpio_set_value(pwdn_gpio,0);
	udelay(10);
	retval = se2100_write_reg(0x09, 0x81);  //enabling suspend mode
	udelay(10);
	gpio_set_value(pwdn_gpio,1);
	return retval;
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
	
	sensor->on = on;		
	//printk("in %s\n",__func__);
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
	//printk(KERN_INFO" %s entereedddd \n" , __func__);

    switch (a->type) {
        /* This is the only case currently handled. */
        case V4L2_BUF_TYPE_VIDEO_CAPTURE:
            memset(a, 0, sizeof(*a));
            a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            cparm->capability = sensor->streamcap.capability;
            cparm->capturemode = sensor->streamcap.capturemode;
            cparm->timeperframe = sensor->streamcap.timeperframe;
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
    int ret = 0;
	//printk(KERN_INFO" %s entereedddd \n" , __func__);

    switch (a->type) {
        /* This is the only case currently handled. */
        case V4L2_BUF_TYPE_VIDEO_CAPTURE:

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
static int ioctl_g_fmt_cap(struct v4l2_int_device *s,
			  struct v4l2_format *f)
{
    struct sensor_data *sensor = s->priv;
	//printk(KERN_INFO" %s entereedddd \n" , __func__);

    f->fmt.pix.pixelformat =se2100_data.pix.pixelformat ;
    f->fmt.pix.width = se2100_data.pix.width;
    f->fmt.pix.height = se2100_data.pix.height;
    f->fmt.pix.bytesperline = se2100_data.pix.bytesperline;
    f->fmt.pix.sizeimage =  se2100_data.pix.sizeimage;
    return 0;
}

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	
	//printk(KERN_INFO" %s entereedddd \n" , __func__);
    if (s == NULL) {
        pr_err("   ERROR!! no slave device set!\n");
        return -1;
    }

    memset(p, 0, sizeof(*p));
    p->u.bt656.clock_curr = se2100_data.mclk;
    pr_debug("   clock_curr=mclk=%d\n", se2100_data.mclk);
    p->if_type = V4L2_IF_TYPE_BT656;
    p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
    p->u.bt656.clock_min = se2100_XCLK_MIN;
    p->u.bt656.clock_max = se2100_XCLK_MAX;
    p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

    return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	//printk(KERN_INFO" %s entereedddd \n" , __func__);

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
	int ret = 0;
	u32 lanes;
	u32 mipi_reg;
	u32 mipi_reg_err1;
	u32 mipi_reg_err2;
	unsigned int laa;

	gpio_set_value(pwdn_gpio,0);

	struct mipi_csi2_info *mipi_csi2_info;
	//printk(KERN_INFO" %s entereedddd \n" , __func__);

	se2100_data.on = true;

	/* mclk */
	tgt_xclk = se2100_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)se2100_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)se2100_XCLK_MIN);
	se2100_data.mclk = tgt_xclk;
	pr_debug("Setting mclk to %d MHz\n", tgt_xclk / 1000000);
//	clk_set_rate(se2100_data.sensor_clk, se2100_data.mclk);

	mipi_csi2_info = mipi_csi2_get_info();

	/* enable mipi csi2 */
	if (mipi_csi2_info) {
		mipi_csi2_enable(mipi_csi2_info);
	} else {
		printk(KERN_ERR "Fail to get mipi_csi2_info!\n");
		return -EPERM;
	}

    	if (!mipi_csi2_get_status(mipi_csi2_info))
     	   mipi_csi2_enable(mipi_csi2_info);

	if (!mipi_csi2_get_status(mipi_csi2_info)) {
		pr_err("Can not enable mipi csi2 driver!\n");
		return -1;
	}

	// se2100 is single lane
	mipi_csi2_info->lanes = 1;

	lanes = mipi_csi2_set_lanes(mipi_csi2_info);
	mipi_csi2_reset(mipi_csi2_info);
//	laa = mipi_csi2_read(mipi_csi2_info, MIPI_CSI2_N_LANES);
	//printk(KERN_ERR "%s() in %s: number of mipi lanes %d\n",__func__, __FILE__, (lanes + 1));

	if ((lanes + 1) != 1) {
		printk(KERN_ERR "%s() in %s: Unsupported, number of mipi lanes %d\n",
				__func__, __FILE__, (lanes + 1));
		return -1;
	}

	if (se2100_data.pix.pixelformat == V4L2_PIX_FMT_UYVY ) {
		mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_YUV422);
	} else
		pr_err("currently this sensor format can not be supported!\n");

//	ret = barcode_resume();//zebra changes 
//	msleep(5);

	if (mipi_csi2_info) {
		unsigned int i = 0;

		/* wait for mipi sensor ready */
		mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
		mipi_reg_err1= mipi_csi2_get_error1(mipi_csi2_info);
		mipi_reg_err2= mipi_csi2_get_error2(mipi_csi2_info);
		//pr_err("mipi csi2 reg vals-------- phy_status: %x  err1: %x   err2: %x\n",mipi_reg, mipi_reg_err1, mipi_reg_err2);

		while ((mipi_reg == 0x200) && (i <= 10)) {
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
			i++;
			msleep(10);
		}
		if (i >= 10) {
			pr_err("mipi csi2 can not receive sensor clk!\n");
			return -1;
		}

		i = 0;
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

#if 1//def DEBUG_REG
		mipi_csi2_get_error2(mipi_csi2_info); /*For debugging enable this */
#endif
	}

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

	//printk(KERN_INFO" %s entereedddd \n" , __func__);
    mipi_csi2_info = mipi_csi2_get_info();

    /* disable mipi csi2 */
    if (mipi_csi2_info)
        if (mipi_csi2_get_status(mipi_csi2_info))
            mipi_csi2_disable(mipi_csi2_info);

    return 0;
}


/*!
 * ioctl_s_fmt_cap - V4L2 sensor interface handler for ioctl_s_fmt_cap
 * 		     set camera output format and resolution format
 *
 * @s: pointer to standard V4L2 device structure
 * @arg: pointer to parameter, according this to set camera
 *
 * Returns 0 if set succeed, else return -1
 */
static int ioctl_s_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;
	u32 format = f->fmt.pix.pixelformat;
	int  ret = 0;
	//u16 data;
	u16 mipi_dt_id;
	//printk(KERN_INFO" %s entereedddd \n" , __func__);
	

	switch (format) {
	case V4L2_PIX_FMT_UYVY:			
	case V4L2_PIX_FMT_YUYV:			
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_GREY:
			se2100_data.pix.pixelformat = format;
	break;


	default:
		printk(KERN_INFO" %d fmt not support \n" ,format);
		pr_debug("case not supported\n");
		return -EINVAL;
	}

	return ret;
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
	//printk(KERN_INFO" %s entereedddd \n" , __func__);
	
	switch (fmt->type) {
		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
			if (fmt->index != 0)
				return -EINVAL;
			fmt->flags = 0;
			strlcpy(fmt->description, "UYVY (YUV 4:2:2), packed",
					sizeof(fmt->description));
			fmt->pixelformat = se2100_data.pix.pixelformat;
			break;
		default:
			return -EINVAL;
	}

    return 0;
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
	//printk(KERN_INFO" %s entereedddd \n" , __func__);
	if (fsize->index > SE2100_MODE_MAX)
		return -EINVAL;
	if (1)
	{
		fsize->type=V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = se2100_frmsizes[fsize->index].width;
		fsize->discrete.height = se2100_frmsizes[fsize->index].height;
		return 0;
	}
}


/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	int i, j, count;
	//printk(KERN_INFO" %s entereedddd \n" , __func__);

	if (fival->index < 0 )
		return -EINVAL;

	if (fival->pixel_format == 0 || fival->width == 0 || fival->height == 0) {
		pr_warning("Please assign pixelformat, width and height.\n");
		return -EINVAL;
	}

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator =30;
	return 0;
	
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */	
static struct v4l2_int_ioctl_desc se2100_ioctl_desc[] = {
    {vidioc_int_dev_init_num, (v4l2_int_ioctl_func*) ioctl_dev_init},
    {vidioc_int_dev_exit_num, ioctl_dev_exit},
    { vidioc_int_s_power_num,
	  (v4l2_int_ioctl_func *)ioctl_s_power },
    {vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*) ioctl_g_ifparm},
    
    {vidioc_int_enum_fmt_cap_num, (v4l2_int_ioctl_func*) ioctl_enum_fmt_cap},
    {vidioc_int_enum_framesizes_num,  (v4l2_int_ioctl_func *) ioctl_enum_framesizes},

    { vidioc_int_enum_frameintervals_num, (v4l2_int_ioctl_func *)ioctl_enum_frameintervals },

    {vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
    {vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_s_fmt_cap},
  
    {vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
    {vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},

};

static struct v4l2_int_slave se2100_slave = {
    .ioctls = se2100_ioctl_desc,
    .num_ioctls = ARRAY_SIZE(se2100_ioctl_desc),
};

static struct v4l2_int_device se2100_int_device = {
    .module = THIS_MODULE,
    .name = "se2100",
    .type = v4l2_int_type_slave,
    .u = {
        .slave = &se2100_slave,
    },
};


static int se2100_misc_open(struct inode* pINode, struct file* pFile)
{
	//printk("se2100_misc_open \n");
	if (atomic_inc_return(&se2100_data.open_excl) != 1 )
	{
		atomic_dec(&se2100_data.open_excl);
		return -EBUSY;
	}
	pFile->private_data = &se2100_data;
	return(0);
}


static int se2100_misc_release(struct inode* node, struct file* file)
{
	//printk("se2100_misc_release \n");
	atomic_dec(&se2100_data.open_excl);
	return(0);
}

static long se2100_misc_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
	struct se2100* se2100;
	struct i2c_client* client;
	struct i2c_rdwr_ioctl_data rdwr_data;
	struct i2c_msg msg;
	u8 __user* usr_data;
	int ret = 0;

	u8 au8Buf[2] = {0};	

	se2100 = file->private_data;


	//printk("se2100_misc_ioctl called\n");
	if ( (cmd != I2C_RDWR) || !arg ) {
		printk(KERN_INFO"se2100_misc_ioctl - false 1\n");
		return -EINVAL;
	}
		
	//client = v4l2_get_subdevdata(&se2100->subdev);
	client = se2100_data.i2c_client;

	if ( copy_from_user(&rdwr_data, (struct i2c_rdwr_ioctl_data __user*) arg, sizeof(rdwr_data)) ) {
		printk(KERN_INFO"se2100_misc_ioctl - false 2\n");
		return -EFAULT;
	}	

	if ( rdwr_data.nmsgs != 1 ) {
		printk(KERN_INFO"se2100_misc_ioctl - false 3\n");
		return -EINVAL;
	}
	
	if ( copy_from_user(&msg, rdwr_data.msgs, sizeof(struct i2c_msg)) ) {
		printk(KERN_INFO"se2100_misc_ioctl - false 4\n");
		return -EFAULT;
	}	


		// Only allow transfers to the SE2100, limit the size of the message and don't allow received length changes
/*	if ( (msg.addr != SE2100_I2C_DEVICE_ADDR) || (msg.len > 256) || (msg.flags & I2C_M_RECV_LEN) ) {
		printk(KERN_INFO"se2100_misc_ioctl - false 5\n");
		return -EINVAL;
	}
*/
	// Map the data buffer from user-space to kernel space
	// WA reuse same structure for message

	

	usr_data = (u8 __user*) msg.buf;
	msg.buf = memdup_user(usr_data, msg.len);
	if ( IS_ERR(msg.buf) )
	{
		printk(KERN_INFO"se2100_misc_ioctl - false 6\n");
		return(PTR_ERR(msg.buf));
	}

	//printk("msg.buf =%s\n",msg.buf);	
	ret = i2c_transfer(client->adapter, &msg, 1);
	//printk("ret = %d\n",ret);
	if ( (ret >= 0) && (msg.flags & I2C_M_RD) ) {
	//	printk("msg.flags =0x%x\n",msg.flags);	
		// Successful read, copy data to user-space
		if ( copy_to_user(usr_data, msg.buf, msg.len) ) {
			printk(KERN_INFO"se2100_misc_ioctl - false 7\n");
			ret = -EFAULT;
		}
	}
	
	kfree(msg.buf);
	return ret;
}

static const struct file_operations se2100_misc_fops =
{
	.owner = THIS_MODULE,
	.unlocked_ioctl = se2100_misc_ioctl,
	.open = se2100_misc_open,
	.release = se2100_misc_release,
};

static struct miscdevice se2100_misc_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sdl_console",
	.fops = &se2100_misc_fops,
};

static int se2100_suspend(struct device *dev)
{
	int retval = 0;
	//printk(KERN_INFO"suspendinggggg\n");
	/*revB timing changes for powerdown sequence*/
	gpio_set_value(pwdn_gpio,0);
	mdelay(1);
	retval = se2100_write_reg(0x09, 0x81);  //enabling suspend mode
	mdelay(10);
	gpio_set_value(pwdn_gpio,1);
	mdelay(1);
	gpio_set_value(mclk_en_gpio,0);
	mdelay(1);
	gpio_set_value(pwr_en_gpio,0);
	//return retval;
	return 0;

}

static int se2100_resume(struct device *dev)
{
	int ret=0;
	/*revB timing changes for powerup sequence*/
	/*reload all registers*/
	gpio_set_value(pwr_en_gpio,1);
	udelay(10);
	gpio_set_value(mclk_en_gpio,1);
	mdelay(10);
	gpio_set_value(pwdn_gpio,0);
	mdelay(2);
	reset_regs();
	ret=barcode_resume();
	//return ret;
	return 0;
}

static int se2100_probe(struct i2c_client *client, const struct i2c_device_id *device_id)
{
	int retval;
	struct device *dev = &client->dev;
	struct se2100 *se2100_data_misc;
	/* Set initial values for the sensor struct. */
	memset(&se2100_data, 0, sizeof(se2100_data));
#if 1
	/* request power down pin */
	mclk_en_gpio = of_get_named_gpio(dev->of_node, "mclk_en-gpios", 0);
	if (!gpio_is_valid(mclk_en_gpio)) {
		dev_err(dev, "no sensor pwdn pin available\n");
		return -ENODEV;
	}
	retval = devm_gpio_request_one(dev, mclk_en_gpio, GPIOF_OUT_INIT_HIGH,
					"se2100_mclk_en");

	/* request power down pin */
	pwdn_gpio = of_get_named_gpio(dev->of_node, "pwdn-gpios", 0);
	if (!gpio_is_valid(pwdn_gpio)) {
		dev_err(dev, "no sensor pwdn pin available\n");
		return -ENODEV;
	}
	retval = devm_gpio_request_one(dev, pwdn_gpio, GPIOF_OUT_INIT_HIGH,
					"se2100_pwdn");

	/* request power down pin */
	pwr_en_gpio = of_get_named_gpio(dev->of_node, "2p8_en-gpios", 0);
	if (!gpio_is_valid(pwr_en_gpio)) {
		dev_err(dev, "no sensor pwr_en pin available\n");
		return -ENODEV;
	}
	retval = devm_gpio_request_one(dev, pwr_en_gpio, GPIOF_OUT_INIT_HIGH,
					"se2100_2p8");
#endif
//	se2100_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
//	if (IS_ERR(se2100_data.sensor_clk)) {
//		/* assuming clock enabled by default */
//		se2100_data.sensor_clk = NULL;
//		dev_err(dev, "clock-frequency missing or invalid\n");
//		return PTR_ERR(se2100_data.sensor_clk);
//	}

	retval = of_property_read_u32(dev->of_node, "mclk",
			&(se2100_data.mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
			(u32 *) &(se2100_data.mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
			&(se2100_data.csi));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "ipu_id",
			&(se2100_data.ipu));
	if (retval) {
		dev_err(dev, "ipu id missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "vc",
			&(se2100_data.vc));
	if (retval) {
		dev_warn(dev, "virtual channel is missing or invalid..Defaults to 0..\n");
		se2100_data.vc = 0;
	}


	gpio_set_value(pwr_en_gpio,1);
	udelay(10);
	gpio_set_value(mclk_en_gpio,1);
//	clk_prepare_enable(se2100_data.sensor_clk);
	mdelay(10);
	se2100_data.i2c_client = client;
	se2100_data.pix.pixelformat = V4L2_PIX_FMT_UYVY;
    	se2100_data.pix.width = 640;
    	se2100_data.pix.height = 480;
	se2100_data.pix.bytesperline = 640 *2;
	 se2100_data.pix.sizeimage = 640 * 480 *2;
    	se2100_data.streamcap.capability = V4L2_MODE_HIGHQUALITY | V4L2_CAP_TIMEPERFRAME;
    	se2100_data.streamcap.capturemode = 0;
    	se2100_data.streamcap.timeperframe.denominator = 30;
    	se2100_data.streamcap.timeperframe.numerator = 1;

	gpio_set_value(pwdn_gpio,0);
	msleep(2);

	reset_regs();
	//printk("Camera-se2100 register loading started\n");
	retval = barcode_resume();//zebra changes 
	//printk("Camera-se2100 register loading finished\n");
	msleep(5);

//	clk_disable_unprepare(se2100_data.sensor_clk);
	se2100_int_device.priv = &se2100_data;
	retval = v4l2_int_device_register(&se2100_int_device);
	printk("Camera-se2100 MIPI Interface is found\n");

	se2100_misc_wa = se2100_data_misc;
	atomic_set(&se2100_misc_wa->open_excl, 0);
	misc_register(&se2100_misc_device);
	printk(KERN_INFO" device connected \n");

	pr_info("camera se2100 connected");
	return retval;
}
static int se2100_remove(struct i2c_client *client)
{

	int retval;
	retval = barcode_suspend();

    	v4l2_int_device_unregister(&se2100_int_device);
	
	misc_deregister(&se2100_misc_device);
	se2100_misc_wa = NULL;
	return 0;
}

static const struct i2c_device_id se2100_id[] = {
	{"se2100_mipi", 0},
	{},
};

static SIMPLE_DEV_PM_OPS(se2100_pm, se2100_suspend, se2100_resume);

MODULE_DEVICE_TABLE(i2c, se2100_id);

static struct i2c_driver se2100_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "se2100_mipi",
		  .pm = &se2100_pm,
		  },
	.probe  = se2100_probe,
	.remove = se2100_remove,
	.id_table = se2100_id,
};




module_i2c_driver(se2100_i2c_driver);

MODULE_AUTHOR("Neuroptics");
MODULE_DESCRIPTION("se2100 MIPI Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
