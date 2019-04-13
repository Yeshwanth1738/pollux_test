/*
 * Copyright (C) 2018 Neuroptics, Inc. All Rights Reserved.
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"
#include "py480_reg_settings.h"

#define PY480_VOLTAGE_ANALOG		3300000
#define PY480_VOLTAGE_ANALOG_PIX	3300000

#define MIN_FPS 			30
#define MAX_FPS 			60
#define DEFAULT_FPS 			30
#define PY480_XCLK_MIN 		68000000
#define PY480_XCLK_MAX 		68000000
#define PY480_CHIP_ID_REG	0x0000

#define PY480_REG_COARSE		199
#define PY480_REG_AGAIN			204
#define PY480_REG_DGAIN			205

#define PY480_REG_AE_ROI_X		256
#define PY480_REG_AE_ROI_Y		257

enum py480_mode {
	py480_mode_808x608 = 0,
	py480_mode_640x480 = 1,
	py480_mode_320x240 = 2,
	py480_mode_MAX	=2
};

enum py480_frame_rate {
	py480_30_fps,
	py480_60_fps
};

static int py480_framerates[] = { DEFAULT_FPS };

struct py480_mode_info {
	enum py480_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

struct py480 {
	struct v4l2_int_device *v4l2_int_device;
	struct spi_device *spi;
	struct v4l2_pix_format pix;
	const struct py480_datafmt *fmt;
	struct v4l2_captureparm streamcap;
	bool on;

	u32 regwidth;
	u32 buswidth;
	u16 reset_gpio;
	u16 *buf;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;
	int csi;

	struct v4l2_ctrl_handler ctrls;
	void (*io_init)(void);
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct sensor_data *py480_data;
static int rst_gpio, cam_pwr_en_gpio, pix_cam_en_gpio,five_v_en_gpio,one_p_eight_en_gpio,clk_en_gpio;
static u32 reg_width,bus_width;
bool py480_suspend_state;
static int last_capture_mode; 

#define DEF_WIDTH	808
#define DEF_HEIGHT	608

static struct py480_mode_info py480_mode_info_data[2][py480_mode_MAX + 1] = {
	{
		{
			py480_mode_808x608,
			808, 608,
			py480_reso_808x608_30,
			ARRAY_SIZE(py480_reso_808x608_30)
		}, {
			py480_mode_640x480,
			640, 480,
			py480_reso_640x480_30,
			ARRAY_SIZE(py480_reso_640x480_30)
		}, {
			py480_mode_320x240,
			320, 240,
			py480_reso_320x240_30,
			ARRAY_SIZE(py480_reso_320x240_30)
		},
	},
	{
		{
			py480_mode_808x608,
			808, 608,
			py480_reso_808x608_60,
			ARRAY_SIZE(py480_reso_808x608_60)
		}, {
			py480_mode_640x480,
			640, 480,
			py480_reso_640x480_60,
			ARRAY_SIZE(py480_reso_640x480_60)
		}, {
			py480_mode_320x240,
			320, 240,
			py480_reso_320x240_60,
			ARRAY_SIZE(py480_reso_320x240_60)
		},
	}
};

static int py480_probe(struct spi_device *spi);
static int py480_remove(struct spi_device *spi);
static int py480_suspend(struct device *dev);
static int py480_resume(struct device *dev);
static int py480_sysfs_suspend(bool state);


static const struct spi_device_id py480_id[] = {
	{"py480", 0},
	{},
};

static SIMPLE_DEV_PM_OPS(py480_pm, py480_suspend, py480_resume);

MODULE_DEVICE_TABLE(spi, py480_id);

static struct spi_driver py480_spi_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "py480",
		  .pm = &py480_pm,
		  },
	.probe  = py480_probe,
	.remove = py480_remove,
	.id_table = py480_id,
};


static inline void py480_reset(void)
{
	/* camera reset */
	gpio_set_value(rst_gpio, 0);
	udelay(10);
	gpio_set_value(rst_gpio, 1);
	udelay(10);	
}

static inline void py480_regulator_enable(void)
{
	/* camera enable regulator */
	gpio_set_value(cam_pwr_en_gpio, 1);
		
}

static inline void py480_regulator_enable_pix(void)
{
	/* imx6 pix enable regulator*/
	gpio_set_value(pix_cam_en_gpio, 1);

}

static inline void py480_regulator_enable_5v(void)
{
	/* imx6 5v enable regulator*/
	gpio_set_value(five_v_en_gpio, 1);
	udelay(10);

}

static inline void py480_regulator_enable_1p8(void)
{
	/* imx6 pix enable regulator*/
	gpio_set_value(one_p_eight_en_gpio, 0);
	udelay(10);
	gpio_set_value(one_p_eight_en_gpio, 1);
	udelay(10);

}

static inline void py480_regulator_enable_clk(void)
{
	/* imx6 pix enable regulator*/
	gpio_set_value(clk_en_gpio, 1);

}

static int py480_write_reg26(struct spi_device *spi,u32 *reg_val)
{
	struct spi_transfer  t = {
		.tx_buf = reg_val,
		.len = 4,
		.bits_per_word = 26,
	};

	struct spi_message m;
	
	if (!spi) {
		dev_err(NULL,
			"%s: spi is unexpectedly NULL\n", __func__);
		return -1;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(spi, &m);

}

static int py480_write_reg(struct spi_device *spi,u16 reg,u16 value)
{
	int ret;
	u32 tempval= (reg << 1) | 0x1;
	//printk("tempval =0x%x\n",tempval);

	tempval = (tempval << 16) | (value);

	ret = py480_write_reg26(spi,&tempval);
	return ret;
}

static int py480_read_reg(struct spi_device *spi,u16 *reg,u16 *value)
{
	u16 tempval= (*reg << 1) | 0x0;
	u32 newval = 0;
	int retval;
		
	struct spi_transfer   reg_t = {
		.tx_buf = &tempval,
		.len = 2,
		.bits_per_word = 10,
	};

	struct spi_transfer   value_t = {
		.rx_buf = &newval,
		.len = 4,
		.bits_per_word = 17,
	};

	struct spi_message m;

	//printk("tempval =%d\n",tempval);
	if (!spi) {
		dev_err(NULL,
			"%s: spi is unexpectedly NULL\n", __func__);
		return -1;
	}

	spi_message_init(&m);
	spi_message_add_tail(&reg_t, &m);
	spi_message_add_tail(&value_t, &m);
	retval=spi_sync(spi, &m);
	
	*value =(u16)newval;
	return retval;
}

#if 0
static int py480_write_reg(struct spi_device *spi,u16 reg,u16 value)
{
	int len = 2;
	u16 tempval = (reg << 1) | 0x1;
	
	struct spi_transfer   reg_t = {
		.tx_buf = &tempval,
		.len = len,
		.bits_per_word = 10,
	};

	struct spi_transfer   value_t = {
		.tx_buf = &value,
		.len = len,
		.bits_per_word = 16,
	};

	struct spi_message m;
	
	printk("tempval =%d\n",tempval);
	if (!spi) {
		dev_err(NULL,
			"%s: spi is unexpectedly NULL\n", __func__);
		return -1;
	}

	spi_message_init(&m);
	spi_message_add_tail(&reg_t, &m);
	spi_message_add_tail(&value_t, &m);
	return spi_sync(spi, &m);

}

#endif
/* download py480 settings to sensor through spi */
static int py480_download_firmware(struct reg_value *mode_setting, s32 arysize)
{
	u16 regaddr = 0;
	u16 val = 0;
	int i, retval = 0;

	for (i = 0; i < arysize; ++i, ++mode_setting) {
		regaddr = mode_setting->reg;
		val = mode_setting->val;

		retval = py480_write_reg(py480_data->spi,regaddr, val);
		if (retval < 0)
			goto err;

	}
err:
	return retval;
}

/* download py480 settings to sensor through spi */
static int py480_read_firmware(struct reg_value *mode_setting, s32 arysize)
{
	u16 regaddr = 0;
	u16 val = 0;
	int i, retval = 0;

	for (i = 0; i < arysize; ++i, ++mode_setting) {
		regaddr = mode_setting->reg;

		retval = py480_read_reg(py480_data->spi,&regaddr, &val);
		if (retval < 0)
			goto err;
		printk("regval=0x%x\n",val);

	}
err:
	return retval;
}

static int py480_init_mode(void)
{
	int retval = 0,retries =0;
	u16 regval = 0;
	u16 pllreg =24;
	
	/*Enabling Clock Management Part 1*/
	retval = py480_download_firmware(py480_enable_clk_management1, 
			ARRAY_SIZE(py480_enable_clk_management1));
	if (retval < 0)
		goto err;

	/*check pll to be locked */
	while((regval == 0) && (retries < 20))
	{
		retval = py480_read_reg(py480_data->spi,&pllreg,&regval);
		if (retval < 0)
			goto err;
		printk("regval=0x%x\n",regval);
		retries++;
		udelay(100);
		if (retries == 20)
			printk("pll lock error\n");
	}
	/*Enabling Clock Management Part 2*/
	retval = py480_download_firmware(py480_enable_clk_management2, 
			ARRAY_SIZE(py480_enable_clk_management2));
	if (retval < 0)
		goto err;
	/*Required register uploads*/
	retval = py480_download_firmware(py480_required_register_uploads, 
			ARRAY_SIZE(py480_required_register_uploads));
	if (retval < 0)
		goto err;

	/*soft power up*/
	retval = py480_download_firmware(py480_soft_power_up, 
			ARRAY_SIZE(py480_soft_power_up));
	if (retval < 0)
		goto err;

	py480_data->pix.width = DEF_WIDTH; 
	py480_data->pix.height = DEF_HEIGHT;
err:
	return retval;
}
static int py480_reg_read_back(void)
{
	int retval = 0,retries =0;
	u16 regval = 0;
	u16 pllreg =24;
	
	/*Enabling Clock Management Part 1*/
	retval = py480_read_firmware(py480_enable_clk_management1, 
			ARRAY_SIZE(py480_enable_clk_management1));
	if (retval < 0)
		goto err;

	/*Enabling Clock Management Part 2*/
	retval = py480_read_firmware(py480_enable_clk_management2, 
			ARRAY_SIZE(py480_enable_clk_management2));
	if (retval < 0)
		goto err;
	/*Required register uploads*/
	retval = py480_read_firmware(py480_required_register_uploads, 
			ARRAY_SIZE(py480_required_register_uploads));
	if (retval < 0)
		goto err;

	/*soft power up*/
	retval = py480_read_firmware(py480_soft_power_up, 
			ARRAY_SIZE(py480_soft_power_up));
	if (retval < 0)
		goto err;

err:
	return retval;
}

static int py480_change_mode(enum py480_frame_rate frame_rate,
			    enum py480_mode mode)
{
	struct reg_value *mode_setting = NULL;
	s32 arysize = 0;
	int retval = 0;

	if (mode > py480_mode_MAX || mode < 0) {
		pr_err("Wrong py480 mode detected!\n");
		return -EINVAL;
	}

	mode_setting = py480_mode_info_data[frame_rate][mode].init_data_ptr;
	arysize = py480_mode_info_data[frame_rate][mode].init_data_size;

	py480_data->pix.width = py480_mode_info_data[frame_rate][mode].width;
	py480_data->pix.height = py480_mode_info_data[frame_rate][mode].height;

	if (py480_data->pix.width == 0 || py480_data->pix.height == 0 ||
	    mode_setting == NULL || arysize == 0)
		return -EINVAL;

	/* set py480 to subsampling mode */
	retval = py480_download_firmware(mode_setting, arysize);

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
	p->u.bt656.clock_curr = py480_data->mclk;
	pr_debug("   clock_curr=mclk=%d\n", py480_data->mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = PY480_XCLK_MIN;
	p->u.bt656.clock_max = PY480_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	printk("in %s\n",__func__);
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

	sensor->on = on;
	if(on)
	{		
		gpio_set_value(clk_en_gpio, 1);
		mdelay(10);
		gpio_set_value(rst_gpio, 1);
		mdelay(1);
	}
	else
	{
		gpio_set_value(rst_gpio, 0);
		mdelay(1);
		gpio_set_value(clk_en_gpio, 0);
		mdelay(1);
	}
	printk("in %s\n",__func__);
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

	printk("in %s\n",__func__);
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
	enum py480_frame_rate frame_rate;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 30)
			frame_rate = py480_30_fps;
		else if (tgt_fps == 60)
			frame_rate = py480_60_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		ret = py480_change_mode(frame_rate,
				a->parm.capture.capturemode);
		if (ret < 0)
			return ret;

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode = a->parm.capture.capturemode;
		last_capture_mode = a->parm.capture.capturemode;

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

	printk("in %s\n",__func__);
	return ret;
}

/**
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int ioctl_try_fmt_cap(struct v4l2_int_device *s,
			     struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct sensor_data *sensor = s->priv;
	struct v4l2_pix_format *pix2 = &sensor->pix;

	pix->width = DEF_WIDTH;
	pix->height = DEF_HEIGHT;
	
	pix->pixelformat = V4L2_PIX_FMT_UYVY;
	pix->bytesperline = pix->width; 
	pix->colorspace =  V4L2_COLORSPACE_JPEG;
	pix->field = V4L2_FIELD_NONE;

	pix->sizeimage = pix->bytesperline * pix->height * 2;
	pix->priv = 0;
	*pix2 = *pix;
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

	switch (format) {
	case V4L2_PIX_FMT_UYVY:
		ret = ioctl_try_fmt_cap(s, f);
		if (ret)
			return ret;
		break;
	case V4L2_PIX_FMT_GREY:
		sensor->pix.pixelformat = format;
		sensor->pix.colorspace =  V4L2_COLORSPACE_JPEG;
		sensor->pix.field = V4L2_FIELD_NONE;
		break;
	default :
		pr_debug("case not supported\n");
		ret =-EINVAL;
		break;
	}

	printk("in %s\n",__func__);
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

	printk("in %s\n",__func__);
	f->fmt.pix = sensor->pix;

	return 0;
}

#define V4L2_CID_COARSE_INTEG_TIME	(V4L2_CID_USER_PY480_BASE | 0x0)
#define V4L2_CID_ANALOG_GAIN		(V4L2_CID_USER_PY480_BASE | 0x1)
#define V4L2_CID_DIGITAL_GAIN		(V4L2_CID_USER_PY480_BASE | 0x2)
#define V4L2_CID_TESTPATTERN		(V4L2_CID_USER_PY480_BASE | 0x3)

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
		vc->value = py480_data->brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = py480_data->hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = py480_data->contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = py480_data->saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = py480_data->red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = py480_data->blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = py480_data->ae_mode;
		break;
	default:
		ret = -EINVAL;
	}

	printk("in %s\n",__func__);
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

	pr_debug("In py480:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
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
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_COARSE_INTEG_TIME:
		retval = py480_write_reg(py480_data->spi,PY480_REG_COARSE, vc->value);
		break;
	case V4L2_CID_ANALOG_GAIN:
		retval = py480_write_reg(py480_data->spi,PY480_REG_AGAIN, vc->value);
		break;
	case V4L2_CID_DIGITAL_GAIN: // digital gain
		retval = py480_write_reg(py480_data->spi,PY480_REG_DGAIN, vc->value);
		break;
	case V4L2_CID_TESTPATTERN:
		retval = py480_write_reg(py480_data->spi,144, vc->value);
		break;
	default:
		retval = -EPERM;
		break;
	}

	printk("in %s\n",__func__);
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
	if (fsize->index > py480_mode_MAX)
		return -EINVAL;

	fsize->pixel_format = py480_data->pix.pixelformat;
	fsize->discrete.width =
			max(py480_mode_info_data[0][fsize->index].width,
			    py480_mode_info_data[1][fsize->index].width);
	fsize->discrete.height =
			max(py480_mode_info_data[0][fsize->index].height,
			    py480_mode_info_data[1][fsize->index].height);
	printk("in %s\n",__func__);
	return 0;
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

	if (fival->index < 0 || fival->index > py480_mode_MAX)
		return -EINVAL;

	if (fival->width == 0 || fival->height == 0 ||
	    fival->pixel_format == 0) {
		pr_warning("Please assign pixelformat, width and height.\n");
		return -EINVAL;
	}

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;

	count = 0;
	for (i = 0; i < ARRAY_SIZE(py480_mode_info_data); i++) {
		for (j = 0; j < (py480_mode_MAX + 1); j++) {
			if (fival->pixel_format == py480_data->pix.pixelformat
			 && fival->width == py480_mode_info_data[i][j].width
			 && fival->height == py480_mode_info_data[i][j].height
			 && py480_mode_info_data[i][j].init_data_ptr != NULL) {
				count++;
			}
			if (fival->index == (count - 1)) {
				fival->discrete.denominator =
						py480_framerates[i];
				return 0;
			}
		}
	}

	printk("in %s\n",__func__);
	return -EINVAL;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{

	printk("in %s\n",__func__);
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
	if (fmt->index > py480_mode_MAX)
		return -EINVAL;

	fmt->pixelformat = py480_data->pix.pixelformat;

	printk("in %s\n",__func__);
	return 0;
}

static int py480_set_clk_rate(void)
{
	u32 tgt_xclk;	/* target xclk */
	int ret;

	/* mclk */
	tgt_xclk = py480_data->mclk;
	tgt_xclk = min(tgt_xclk, (u32)PY480_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)PY480_XCLK_MIN);
	py480_data->mclk = tgt_xclk;

	printk("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	ret = clk_set_rate(py480_data->sensor_clk, py480_data->mclk);
	if (ret < 0)
		pr_err("set rate filed, rate=%d\n", py480_data->mclk);
	return ret;
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
	u32 tgt_fps;	/* target frames per secound */
	u32 tgt_xclk;	/* target xclk */
	enum py480_frame_rate frame_rate;
	int ret=0;
	u16 regval = 0;
	u16 pllreg = 0;

	py480_data->on = true;

	/*assertion of reset pin to 1*/
	gpio_set_value(rst_gpio, 1);
	mdelay(1);

	printk("in %s\n",__func__);
	/* mclk */
	tgt_xclk = py480_data->mclk;
	tgt_xclk = min(tgt_xclk, (u32)PY480_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)PY480_XCLK_MIN);
	py480_data->mclk = tgt_xclk;

	printk("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	ret = clk_set_rate(py480_data->sensor_clk, py480_data->mclk);
	if (ret < 0)
		pr_err("set rate filed, rate=%d\n", py480_data->mclk);

	/* Default camera frame rate is set in probe */
	tgt_fps = py480_data->streamcap.timeperframe.denominator /
		  py480_data->streamcap.timeperframe.numerator;

	if (tgt_fps == 30)
		frame_rate = py480_30_fps;
	else if (tgt_fps == 60)
		frame_rate = py480_60_fps;
	else
		return -EINVAL; /* Only support 30fps or 60fps now. */
	
	ret = py480_init_mode();
	if(ret < 0)
		return ret;

#if 1 
	ret = py480_download_firmware(py480_gain_10,ARRAY_SIZE(py480_gain_10));
	if(ret < 0)
		return ret;
	
	ret = py480_change_mode(frame_rate,py480_mode_808x608);
	if(ret < 0)
		return ret;
	last_capture_mode = 0;
	/*Enable sequencer*/
	pllreg = 192;
	regval = 0x0801;
	ret = py480_write_reg(py480_data->spi,pllreg, regval);
	if (ret < 0)
		goto err;
#endif
err:
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
	return 0;
}

/*!
 * ioctl_dbg_g_register - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_REGISTER ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */

static int ioctl_dbg_g_register(struct v4l2_int_device *s, struct v4l2_dbg_register *id)
{
	int retval;
	u16 reg =id->reg;
	u16 val;
	
	retval = py480_read_reg(py480_data->spi,&reg,&val);
	if (retval < 0)	
		return retval;
		
	id->val=val;
	printk("in %s\n",__func__);
	
	return 0;
}

/*!
 * ioctl_dbg_s_register - V4L2 sensor interface handler for
 *			VIDIOC_DBG_S_REGISTER ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_dbg_s_register(struct v4l2_int_device *s, struct v4l2_dbg_register *id)
{
	int retval;
	printk("in %s\n",__func__);
	retval = py480_write_reg(py480_data->spi,(u16)id->reg,(u16)id->val);	
	if (retval < 0)	
		return retval;

	return 0;
}


/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc py480_ioctl_desc[] = {
	{ vidioc_int_dev_init_num,
	  (v4l2_int_ioctl_func *)ioctl_dev_init },
	{ vidioc_int_dev_exit_num,
	  ioctl_dev_exit},
	{ vidioc_int_s_power_num,
	  (v4l2_int_ioctl_func *)ioctl_s_power },
	{ vidioc_int_g_ifparm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ifparm },
	{ vidioc_int_init_num,
	  (v4l2_int_ioctl_func *)ioctl_init },
	{ vidioc_int_enum_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap },
	{ vidioc_int_g_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },
	{ vidioc_int_s_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_s_fmt_cap },
	{ vidioc_int_g_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_parm },
	{ vidioc_int_s_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_s_parm },
	{ vidioc_int_g_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ vidioc_int_s_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_s_ctrl },
	{ vidioc_int_enum_framesizes_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_framesizes },
	{ vidioc_int_enum_frameintervals_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_frameintervals },
	{ vidioc_int_dbg_g_register_num,
	  (v4l2_int_ioctl_func *)ioctl_dbg_g_register },
	{ vidioc_int_dbg_s_register_num,
	  (v4l2_int_ioctl_func *)ioctl_dbg_s_register },
};

static struct v4l2_int_slave py480_slave = {
	.ioctls = py480_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(py480_ioctl_desc),
};

static struct v4l2_int_device py480_int_device = {
	.module = THIS_MODULE,
	.name = "py480",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &py480_slave,
	},
};

/*!
 * py480 SPI suspned function
 *
 * @param dev            struct device *
 * @return  Error code indicating success or failure
 */

static int py480_suspend(struct device *dev)
{

	struct sensor_data *sensor = dev_get_drvdata(dev);
	int ret;
	u16 regval = 0;
	u16 pllreg = 0;

	/*Disable sequencer*/
	pllreg = 192;
	regval = 0x0802;
	ret = py480_write_reg(py480_data->spi,pllreg, regval);
	if (ret < 0)
		return ret;

	/*soft power down*/	
	ret = py480_download_firmware(py480_soft_power_down,ARRAY_SIZE(py480_soft_power_down));
	if(ret < 0)
		return ret;

	/*disable clock management2*/	
	ret = py480_download_firmware(py480_disable_clock_management2,ARRAY_SIZE(py480_disable_clock_management2));
	if(ret < 0)
		return ret;

	/*disable clock management1*/	
	ret = py480_download_firmware(py480_disable_clock_management1,ARRAY_SIZE(py480_disable_clock_management1));
	if(ret < 0)
		return ret;

	/*revB timing changes for powerup and powerdown sequence*/
	/*assertion of reset pin to 0*/
	gpio_set_value(rst_gpio, 0);
	mdelay(1);
	gpio_set_value(clk_en_gpio, 0);
	mdelay(1);
	gpio_set_value(pix_cam_en_gpio, 0);
	mdelay(1);
	gpio_set_value(cam_pwr_en_gpio, 0);
	mdelay(1);
	gpio_set_value(five_v_en_gpio, 0);
	mdelay(1);
	printk(KERN_INFO"py480 suspending\n");
	
	return 0;
}



/*!
 * py480 SPI sysfs suspned function
 *
 * @param dev            struct device *
 * @return  Error code indicating success or failure
 */

static int py480_sysfs_suspend(bool state)
{

	int ret,retries=0;
	u16 regval = 0;
	u16 pllreg = 0;
	enum py480_frame_rate frame_rate = py480_30_fps;
	/*revB timing changes for powerup and powerdown sequence*/
	if(state == 1)
	{
		/*assertion of reset pin to 0*/
		gpio_set_value(rst_gpio, 0);
		mdelay(1);
		gpio_set_value(clk_en_gpio, 0);
		mdelay(1);
		gpio_set_value(pix_cam_en_gpio, 0);
		mdelay(1);
		gpio_set_value(cam_pwr_en_gpio, 0);
		mdelay(1);
		gpio_set_value(five_v_en_gpio, 0);
		mdelay(1);
		printk(KERN_INFO"py480 suspending\n");
	}
	else if(state == 0)
	{
	 	regval = 0;
		 pllreg = 24;
		/*5v gpio*/
		gpio_set_value(five_v_en_gpio, 1);
		mdelay(1);
		/*camera power*/
		gpio_set_value(cam_pwr_en_gpio, 1);
		mdelay(1);
		/*pix clk*/
		gpio_set_value(pix_cam_en_gpio, 1);
		mdelay(1);
#if 0/*settings will be written by application when openening the camera node*/
		/*enable */
		gpio_set_value(clk_en_gpio, 1);
		mdelay(10);
		/*assertion of reset pin to 1*/
		gpio_set_value(rst_gpio, 1);
		mdelay(1);
		printk(KERN_INFO"py480 resuming\n");
 
		ret = py480_init_mode();
		if(ret < 0)
			return ret;

		ret = py480_download_firmware(py480_gain_10,ARRAY_SIZE(py480_gain_10));
			if(ret < 0)
		return ret;

		if(last_capture_mode == 0)	
			ret = py480_change_mode(frame_rate,py480_mode_808x608);
		else if(last_capture_mode == 1)
			ret = py480_change_mode(frame_rate,py480_mode_640x480);
		else if(last_capture_mode == 2)
			ret = py480_change_mode(frame_rate,py480_mode_320x240);

		if(ret < 0)
			return ret;
		/*Enable sequencer*/
		pllreg = 192;
		regval = 0x0801;
		ret = py480_write_reg(py480_data->spi,pllreg, regval);
		if (ret < 0)
			return ret;

#endif

	}
	return 0;
}

/*!
 * py480 SPI resume function
 *
 * @param dev            struct device *
 * @return  Error code indicating success or failure
 */

static int py480_resume(struct device *dev)
{
	enum py480_frame_rate frame_rate = py480_30_fps;
	int ret,retries=0;
	u16 regval = 0;
	u16 pllreg = 24;
	/*revB timing changes for powerup and powerdown sequence*/
	/*5v gpio*/
	gpio_set_value(five_v_en_gpio, 1);
	mdelay(1);
	/*camera power*/
	gpio_set_value(cam_pwr_en_gpio, 1);
	mdelay(1);
	/*pix clk*/
	gpio_set_value(pix_cam_en_gpio, 1);
	mdelay(1);
#if 0/*camera handle will be closed by application when entering into charging*/
	/*enable */
	gpio_set_value(clk_en_gpio, 1);
	mdelay(10);
	/*assertion of reset pin to 1*/
	gpio_set_value(rst_gpio, 1);
	mdelay(1);
	printk(KERN_INFO"py480 resuming\n");
	ret = py480_init_mode();
	if(ret < 0)
		return ret;

	ret = py480_download_firmware(py480_gain_10,ARRAY_SIZE(py480_gain_10));
	if(ret < 0)
		return ret;

	if(last_capture_mode == 0)	
		ret = py480_change_mode(frame_rate,py480_mode_808x608);
	else if(last_capture_mode == 1)
		ret = py480_change_mode(frame_rate,py480_mode_640x480);
	else if(last_capture_mode == 2)
		ret = py480_change_mode(frame_rate,py480_mode_320x240);
	
	if(ret < 0)
		return ret;

	/*Enable sequencer*/
	pllreg = 192;
	regval = 0x0801;
	ret = py480_write_reg(py480_data->spi,pllreg, regval);
	if (ret < 0)
		return ret;
#endif

	return 0;
}


static ssize_t py480_suspend_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	
	return sprintf(buf, "%u\n", py480_suspend_state);
}

static ssize_t py480_suspend_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc = -ENXIO;
	unsigned long data;

	rc = kstrtoul(buf, 0, &data);
	if (rc)
		return rc;

	py480_suspend_state = (bool)data;
	
	py480_sysfs_suspend(py480_suspend_state);
		
	rc =count;
	return rc;
}
static DEVICE_ATTR_RW(py480_suspend_state);

static struct attribute *suspend_attrs[] = {
	&dev_attr_py480_suspend_state.attr,
	NULL,
};

static struct attribute_group py480_attr_group = {
	.attrs = suspend_attrs,
};


/*!
 * py480 SPI probe function
 *
 * @param spi            struct spi_device *
 * @return  Error code indicating success or failure
 */
static int py480_probe(struct spi_device *spi)
{
	struct pinctrl *pinctrl;	
	int retval,error;
	u16 chip_id = 0;
	u16 val=0;

	py480_data= devm_kzalloc(&spi->dev, sizeof(*py480_data), GFP_KERNEL);
	if (!py480_data)
		return -ENOMEM;

	py480_data->spi = spi;

	/* py480 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(&spi->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&spi->dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}
	
	retval = of_property_read_u32(spi->dev.of_node, "regwidth", &reg_width);
	if (retval < 0) {
		return retval;
	}
	retval = of_property_read_u32(spi->dev.of_node, "buswidth", &bus_width);
	if (retval < 0) {
		return retval;
	}


	
	if (of_find_property(spi->dev.of_node, "rst-gpios", NULL)) {
		/* request reset pin */
		rst_gpio = of_get_named_gpio(spi->dev.of_node, "rst-gpios", 0);
		if (!gpio_is_valid(rst_gpio)) {
			dev_err(&spi->dev, "no sensor reset pin available\n");
			return -EINVAL;
		}
		retval = devm_gpio_request_one(&spi->dev, rst_gpio, GPIOF_OUT_INIT_LOW,
					"py480_reset");
		if (retval < 0)
			return retval;

	}

	if (of_find_property(spi->dev.of_node, "cam_en-gpios", NULL)) {
		/* request camera power enable pin */
		cam_pwr_en_gpio = of_get_named_gpio(spi->dev.of_node, "cam_en-gpios", 0);
		if (!gpio_is_valid(cam_pwr_en_gpio)) {
			dev_err(&spi->dev, "no sensor camera power enable pin available\n");
			return -EINVAL;
		}
		retval = devm_gpio_request_one(&spi->dev, cam_pwr_en_gpio, GPIOF_OUT_INIT_LOW,
					"py480_regulator_enable");
		if (retval < 0)
			return retval;
	}

	if (of_find_property(spi->dev.of_node, "pix_en-gpios", NULL)) {
		/* request pixel regulator enable pin */
		pix_cam_en_gpio = of_get_named_gpio(spi->dev.of_node, "pix_en-gpios", 0);
		if (!gpio_is_valid(pix_cam_en_gpio)) {
			dev_err(&spi->dev, "no sensor reset pin available\n");
			return -EINVAL;
		}
		retval = devm_gpio_request_one(&spi->dev, pix_cam_en_gpio, GPIOF_OUT_INIT_LOW,
					"py480_regulator_enable_pix");
		if (retval < 0)
			return retval;
	}

	if (of_find_property(spi->dev.of_node, "clk_en-gpios", NULL)) {
		/* request clk enable pin */
		clk_en_gpio = of_get_named_gpio(spi->dev.of_node, "clk_en-gpios", 0);
		if (!gpio_is_valid(clk_en_gpio)) {
			dev_err(&spi->dev, "no sensor clk pin available\n");
			return -EINVAL;
		}
		retval = devm_gpio_request_one(&spi->dev, clk_en_gpio, GPIOF_OUT_INIT_LOW,
					"py480_regulator_enable_clk");
		if (retval < 0)
			return retval;
	}

	if (of_find_property(spi->dev.of_node, "fivev_en-gpios", NULL)) {
		/* request clk enable pin */
		five_v_en_gpio = of_get_named_gpio(spi->dev.of_node, "fivev_en-gpios", 0);
		if (!gpio_is_valid(five_v_en_gpio)) {
			dev_err(&spi->dev, "no 5v pin available\n");
			return -EINVAL;
		}
		retval = devm_gpio_request_one(&spi->dev, five_v_en_gpio, GPIOF_OUT_INIT_HIGH,
					"py480_regulator_enable_clk");
		if (retval < 0)
			return retval;
	}


	py480_data->sensor_clk = devm_clk_get(&spi->dev, "csi_mclk");
	if (IS_ERR(py480_data->sensor_clk)) {
		dev_err(&spi->dev, "get mclk failed\n");
		return PTR_ERR(py480_data->sensor_clk);
	}

	retval = of_property_read_u32(spi->dev.of_node, "mclk",
					&(py480_data->mclk));
	if (retval) {
		dev_err(&spi->dev, "mclk frequency is invalid\n");
		return retval;
	}

	retval = of_property_read_u32(spi->dev.of_node, "mclk_source",
					(u32 *) &(py480_data->mclk_source));
	if (retval) {
		dev_err(&spi->dev, "mclk_source invalid\n");
		return retval;
	}

	retval = of_property_read_u32(spi->dev.of_node, "csi_id",
					&(py480_data->csi));
	if (retval) {
		dev_err(&spi->dev, "csi_id invalid\n");
		return retval;
	}

	retval = of_property_read_u32(spi->dev.of_node, "ipu_id",
					&(py480_data->ipu));
	if (retval) {
		dev_err(&spi->dev, "ipu_id invalid\n");
		return retval;
	}
	py480_regulator_enable();
	mdelay(1);
	py480_regulator_enable_pix();
	mdelay(1);
	py480_regulator_enable_clk();
	mdelay(10);
	
	clk_prepare_enable(py480_data->sensor_clk);

	py480_data->io_init = py480_reset;
	py480_data->spi = spi;
	py480_data->pix.pixelformat = V4L2_PIX_FMT_GREY;
	py480_data->pix.width = DEF_WIDTH;
	py480_data->pix.height = DEF_HEIGHT;
	py480_data->streamcap.capability = V4L2_MODE_HIGHQUALITY | V4L2_CAP_TIMEPERFRAME;
	py480_data->streamcap.capturemode = 0;
	py480_data->streamcap.timeperframe.denominator = DEFAULT_FPS;
	py480_data->streamcap.timeperframe.numerator = 1;

	py480_reset();
	mdelay(1);

	if (py480_data->spi && bus_width == 26) {
		py480_data->spi->bits_per_word = 26;
		retval = py480_data->spi->master->setup(py480_data->spi);
		if (retval) {
			dev_warn(&py480_data->spi->dev,
					"26-bit SPI not available\n");
			return retval;
		}
	}
	
	retval = py480_read_reg(py480_data->spi,&chip_id,&val);
	if (retval < 0 || val != 0x5004) {
		clk_disable_unprepare(py480_data->sensor_clk);
		pr_warning("camera py480 is not found\n");
		return -ENODEV;
	}
	printk("py480 camera sensor chip id: 0x%x\n", val);

	error = sysfs_create_group(&spi->dev.kobj, &py480_attr_group);
	if (error) {
		dev_err(&spi->dev, "Unable to export suspend state, error: %d\n",
			error);
		return error;
	}
	
	clk_disable_unprepare(py480_data->sensor_clk);
		
	py480_int_device.priv = py480_data;
	retval = v4l2_int_device_register(&py480_int_device);
	if (retval < 0){
		dev_err(&spi->dev, "%s--v4l2_int_device_register failed, ret=%d\n", __func__, retval);
		sysfs_remove_group(&spi->dev.kobj, &py480_attr_group);		
		return retval;
		
	}
	pr_info("camera py480 is found\n");
	return retval;
}

/*!
 * py480 SPI detach function
 *
 * @param spi            struct spi_device *
 * @return  Error code indicating success or failure
 */
static int py480_remove(struct spi_device *spi)
{
	sysfs_remove_group(&spi->dev.kobj, &py480_attr_group);		
	v4l2_int_device_unregister(&py480_int_device);
	printk("in %s\n",__func__);

	return 0;
}

module_spi_driver(py480_spi_driver);

MODULE_AUTHOR("Neuroptics");
MODULE_DESCRIPTION("PHYTHON480 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
