/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 *
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

#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

struct fb_hx8347g {
	struct spi_device	*spi;
	u32			regwidth;
	u32			buswidth;
	u16			reset_gpio;
	u8			*buf;
};

struct fb_hx8347g *fb_hx8347g_data;
int pwdn_gpio;
bool hx8347_suspend_state;
void fb_hx8347g_reset(int gpio)
{
	//gpio_set_value(gpio, 0);
	//mdelay(5);
	gpio_set_value(gpio, 1);
	//mdelay(1);
}

int fb_hx8347g_write_spi(struct spi_device *spi, void *buf, size_t len)
{
	struct spi_transfer t = {
		.tx_buf = buf,
		.len = len,
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


void fb_hx8347g_write_reg16_bus16(struct fb_hx8347g *fb_hx8347g, int len, ...)
{
	va_list args;
	int ret;
	int pad = 1;
	u16 *buf = (u16 *)fb_hx8347g->buf;

	if (len <= 0)
		return;

	va_start(args, len);
	*buf = 0x7400;
	*buf |= (u8)va_arg(args, unsigned int);

	ret = fb_hx8347g_write_spi(fb_hx8347g->spi, fb_hx8347g->buf, (len - pad) * sizeof(u16));
	if (ret < 0) {
		dev_err(&fb_hx8347g->spi->dev,
			"%s: write() failed and returned %d\n", __func__, ret);
		return;
	}

	buf = (u16 *)fb_hx8347g->buf;
	
	*buf = 0x7600;
	*buf |= (u8)va_arg(args, unsigned int);
	
	
	ret = fb_hx8347g_write_spi(fb_hx8347g->spi, fb_hx8347g->buf, (len - pad) * sizeof(u16));
	if (ret < 0) {
		dev_err(&fb_hx8347g->spi->dev,
			"%s: write() failed and returned %d\n", __func__, ret);
		return;
	}
	va_end(args);
}

#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__})/sizeof(int))

#define write_reg(fb_hx8347g, ...)  \
	fb_hx8347g_write_reg16_bus16(fb_hx8347g, NUMARGS(__VA_ARGS__), __VA_ARGS__)

static int fb_hx8347g_init_display_st(struct fb_hx8347g *fb_hx8347g)
{
	/* Driving ability setting */
	    write_reg(fb_hx8347g,234, 0x00);
	    write_reg(fb_hx8347g,235, 0x20);
	    write_reg(fb_hx8347g,236, 0x0C);
	    write_reg(fb_hx8347g,237, 0xC4);
	    write_reg(fb_hx8347g,232, 0x40);
	    write_reg(fb_hx8347g,233, 0x38);
	    write_reg(fb_hx8347g,241, 0x01);
	    write_reg(fb_hx8347g,242, 0x10);
	    write_reg(fb_hx8347g,39, 0xA3);
	
	    /* Adjust the Gamma Curve */
	    write_reg(fb_hx8347g,64, 0x01);
	    write_reg(fb_hx8347g,65, 0x00);
	    write_reg(fb_hx8347g,66, 0x00);
	    write_reg(fb_hx8347g,67, 0x10);
	    write_reg(fb_hx8347g,68, 0x0E);
	    write_reg(fb_hx8347g,69, 0x24);
	    write_reg(fb_hx8347g,70, 0x04);
	    write_reg(fb_hx8347g,71, 0x50);
	    write_reg(fb_hx8347g,72, 0x02);
	    write_reg(fb_hx8347g,73, 0x13);
	    write_reg(fb_hx8347g,74, 0x19);
	    write_reg(fb_hx8347g,75, 0x19);
	    write_reg(fb_hx8347g,76, 0x16);
	    write_reg(fb_hx8347g,80, 0x1B);
	    write_reg(fb_hx8347g,81, 0x31);
	    write_reg(fb_hx8347g,82, 0x2F);
	    write_reg(fb_hx8347g,83, 0x3F);
	    write_reg(fb_hx8347g,84, 0x3F);
	    write_reg(fb_hx8347g,85, 0x3E);
	    write_reg(fb_hx8347g,86, 0x2F);
	    write_reg(fb_hx8347g,87, 0x7B);
	    write_reg(fb_hx8347g,88, 0x09);
	    write_reg(fb_hx8347g,89, 0x06);
	    write_reg(fb_hx8347g,90, 0x06);
	    write_reg(fb_hx8347g,91, 0x0C);
	    write_reg(fb_hx8347g,92, 0x1D);
	    write_reg(fb_hx8347g,93, 0xCC);

	    /* Power voltage setting */
   	 write_reg(fb_hx8347g,27, 0x1B);
   	 write_reg(fb_hx8347g,26, 0x01);
    	write_reg(fb_hx8347g,36, 0x2F);
   	 write_reg(fb_hx8347g,37, 0x57);
	/*****VCOM offset ****/
  	  write_reg(fb_hx8347g,35, 0x86);

	/* Power On sequence ---------------------------------------------------------*/
 	 write_reg(fb_hx8347g,24, 0x36); /* Display frame rate = 70Hz RADJ = '0110' */
 	 write_reg(fb_hx8347g,25, 0x01); /* OSC_EN = 1 */
 	 write_reg(fb_hx8347g,28, 0x06); /* AP[2:0] = 111 */
 	 write_reg(fb_hx8347g,31, 0x90); /* GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0*/
 	 mdelay(10);
 	 /* 262k/65k color selection */
 	 write_reg(fb_hx8347g,23, 0x05); /* default 0x06 262k color,  0x05 65k color */
 	 /* SET PANEL */
 	 write_reg(fb_hx8347g,54, 0x09); /* SS_PANEL = 1, GS_PANEL = 0,REV_PANEL = 0, BGR_PANEL = 1 */
 
  	/* Display On */
 	 write_reg(fb_hx8347g,40, 0x38);
 	 mdelay(60);
	 write_reg(fb_hx8347g,40, 0x3C);

	/* Set GRAM Area - Partial Display Control */
 	write_reg(fb_hx8347g,1, 0x00); /* DP_STB = 0, DP_STB_S = 0, SCROLL = 0, */
	write_reg(fb_hx8347g, 0x16, 0xA0); //default MY(0) MX(0) MV(0) ML(0) BGR(0) // MY(1) MX(1) MV(0)
	write_reg(fb_hx8347g, 0X31, 0X03);
	write_reg(fb_hx8347g, 0X32, 0X08);

	/*set GRAM Area*/
	write_reg(fb_hx8347g, 0X06, 0X00);
	write_reg(fb_hx8347g, 0X07, 0X00);//column start
	write_reg(fb_hx8347g, 0X08, 0X00);
	write_reg(fb_hx8347g, 0X09, 0XEF);//column end
	write_reg(fb_hx8347g, 0X02, 0X00); 
	write_reg(fb_hx8347g, 0X03, 0X00);//row start
	write_reg(fb_hx8347g, 0X04, 0X01);
	write_reg(fb_hx8347g, 0X05, 0X3F);//row end


	write_reg(fb_hx8347g,6, 0);
	write_reg(fb_hx8347g,7, 0);
  	write_reg(fb_hx8347g,2, 0);
	write_reg(fb_hx8347g,3, 0);

	*fb_hx8347g->buf=0x22;
	fb_hx8347g_write_spi(fb_hx8347g->spi,fb_hx8347g->buf,8);
	return 0;
}
	
	
static int fb_hx8347g_init_display(struct fb_hx8347g *fb_hx8347g)
{

	printk("%s\n",__func__);
	/* driving ability */
	write_reg(fb_hx8347g, 0x2E, 0x89); //GDOFF
	write_reg(fb_hx8347g, 0x29, 0x8F); //RTN
	write_reg(fb_hx8347g, 0x2B, 0x02); //DUM
	write_reg(fb_hx8347g, 0xE2, 0x00); //VREF
	write_reg(fb_hx8347g, 0xE4, 0x01); //EQ
	write_reg(fb_hx8347g, 0xE5, 0x10); //EQ
	write_reg(fb_hx8347g, 0xE6, 0x01); //EQ
	write_reg(fb_hx8347g, 0xE7, 0x10); //EQ
	write_reg(fb_hx8347g, 0xE8, 0x4E); //OPON
	write_reg(fb_hx8347g, 0xF2, 0x00); //GEN
	write_reg(fb_hx8347g, 0xEA, 0x00); //PTBA
	write_reg(fb_hx8347g, 0xEB, 0x20); //PTBA
	write_reg(fb_hx8347g, 0xEC, 0x3C); //STBA	
	write_reg(fb_hx8347g, 0xED, 0xC8); //STBA
	write_reg(fb_hx8347g, 0xE9, 0x38); //OPON1
	write_reg(fb_hx8347g, 0xF1, 0x01); //OTPS1B
	printk("%s++\n",__func__);
#if 1
	/*Gamma 2.8 setting*/
	write_reg(fb_hx8347g, 0x40,0X00);
	write_reg(fb_hx8347g, 0x41,0X00);
	write_reg(fb_hx8347g, 0x42,0X00);
	write_reg(fb_hx8347g, 0x43,0X15);
	write_reg(fb_hx8347g, 0x44,0X13);
	write_reg(fb_hx8347g, 0x45,0X3F);
	write_reg(fb_hx8347g, 0x47,0X55);
	write_reg(fb_hx8347g, 0x48,0X00);
	write_reg(fb_hx8347g, 0x49,0X12);
	write_reg(fb_hx8347g, 0x4A,0X19);
	write_reg(fb_hx8347g, 0x4B,0X19);
	write_reg(fb_hx8347g, 0x4C,0X16);
	write_reg(fb_hx8347g, 0x50,0X00);
	write_reg(fb_hx8347g, 0x51,0X2C);
	write_reg(fb_hx8347g, 0x52,0X2A);
	write_reg(fb_hx8347g, 0x53,0X3F);
	write_reg(fb_hx8347g, 0x54,0X3F);
	write_reg(fb_hx8347g, 0x55,0X3F);
	write_reg(fb_hx8347g, 0x56,0X2A);
	write_reg(fb_hx8347g, 0x57,0X7E);
	write_reg(fb_hx8347g, 0x58,0X09);
	write_reg(fb_hx8347g, 0x59,0X06);
	write_reg(fb_hx8347g, 0x5A,0X06);
	write_reg(fb_hx8347g, 0x5B,0X0D);
	write_reg(fb_hx8347g, 0x5C,0X1F);
	write_reg(fb_hx8347g, 0x5D,0XFF);
#endif

	/* power voltage */
	write_reg(fb_hx8347g, 0x1B, 0x1A);
	write_reg(fb_hx8347g, 0x1A, 0x02);
	write_reg(fb_hx8347g, 0x24, 0x61);
	write_reg(fb_hx8347g, 0x25, 0x5C);

	/* VCOM offset */
	write_reg(fb_hx8347g, 0x23, 0x62); /* for flicker adjust */

	/* power on */
	write_reg(fb_hx8347g, 0x18, 0x36);//RADJ 70HZ
	write_reg(fb_hx8347g, 0x19, 0x01); /* start osc OSC_EN=1 */
	write_reg(fb_hx8347g, 0x1F, 0x88);/*GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0, STB=0*/
	mdelay(5);
	write_reg(fb_hx8347g, 0x1F, 0x80);/*GAS=1, VOMG=00, PON=0, DK=0, XDK=0, DVDH_TRI=0, STB=0*/
	mdelay(5);
	write_reg(fb_hx8347g, 0x1F, 0x90);/*GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0*/
	mdelay(5);
	write_reg(fb_hx8347g, 0x1F, 0xD4);/*GAS=1, VOMG=10, PON=1, DK=0, XDK=1, DDVDH_TRI=0, STB=0*/
	mdelay(5);

	/* color selection */
	write_reg(fb_hx8347g, 0x17, 0x05); //default 0x06 262k color // 0x05 65k color

	/*panel characteristic */
	write_reg(fb_hx8347g, 0x36, 0x09);//SS_P, GS_P,REV_P,BGR_P

	/*display on setting */
	write_reg(fb_hx8347g, 0x28, 0x38);//GON=1, DTE=1, D=1000
	mdelay(40);
	write_reg(fb_hx8347g, 0x28, 0x3C);//GON=1, DTE=1, D=1100
#if 1
	/*set GRAM Area*/
	write_reg(fb_hx8347g, 0X06, 0X00);
	write_reg(fb_hx8347g, 0X07, 0X00);//row start
	write_reg(fb_hx8347g, 0X08, 0X00);
	write_reg(fb_hx8347g, 0X09, 0xEF);//row end
	write_reg(fb_hx8347g, 0X02, 0X00); 
	write_reg(fb_hx8347g, 0X03, 0X00);//column start
	write_reg(fb_hx8347g, 0X04, 0X01);
	write_reg(fb_hx8347g, 0X05, 0X3F);//column end
#endif	 
	/*RGB Interface control*/	
	write_reg(fb_hx8347g, 0X31, 0X03);
	write_reg(fb_hx8347g, 0X32, 0X08);
	printk("fb_hx8347g spi\n");	 

	//*fb_hx8347g->buf=0x22;
	//fb_hx8347g_write_spi(fb_hx8347g->spi,fb_hx8347g->buf,8);
	return 0;
}

static int fb_hx8347g_suspend(struct device *dev)
{

	printk("%s\n",__func__);
	/*display off setting */
	write_reg(fb_hx8347g_data, 0x28, 0x38);//GON=1, DTE=1, D=1000
	mdelay(40);
	write_reg(fb_hx8347g_data, 0x28, 0x34);//GON=1, DTE=1, D=1100
	/*set standby*/
	write_reg(fb_hx8347g_data, 0x1F, 0x89);/*GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0, STB=1*/
	mdelay(5);
	/*oscillator disable*/
	write_reg(fb_hx8347g_data, 0x19, 0x00); /* start osc OSC_EN=0 */
	/* power off lvds chip */
	mdelay(1);
	gpio_set_value(pwdn_gpio,0);

	return 0;
}

static int fb_hx8347g_sysfs_suspend(bool state)
{

	if(state == 1)
	{
		/*display off setting */
		write_reg(fb_hx8347g_data, 0x28, 0x38);//GON=1, DTE=1, D=1000
		mdelay(40);
		write_reg(fb_hx8347g_data, 0x28, 0x34);//GON=1, DTE=1, D=1100
		/*set standby*/
		write_reg(fb_hx8347g_data, 0x1F, 0x89);/*GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0, STB=1*/
		mdelay(5);
		/*oscillator disable*/
		write_reg(fb_hx8347g_data, 0x19, 0x00); /* start osc OSC_EN=0 */
		/* power off lvds chip */
		mdelay(1);
		gpio_set_value(pwdn_gpio,0);
		printk("display suspending\n");
	}
	else if(state == 0)
	{
		/* power on lvds chip */
		gpio_set_value(pwdn_gpio,1);
		mdelay(1);
		/* power on */
		write_reg(fb_hx8347g_data, 0x18, 0x36);//RADJ 70HZ
		write_reg(fb_hx8347g_data, 0x19, 0x01); /* start osc OSC_EN=1 */
		write_reg(fb_hx8347g_data, 0x1F, 0x88);/*GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0, STB=0*/
		mdelay(5);
		write_reg(fb_hx8347g_data, 0x1F, 0x80);/*GAS=1, VOMG=00, PON=0, DK=0, XDK=0, DVDH_TRI=0, STB=0*/
		mdelay(5);
		write_reg(fb_hx8347g_data, 0x1F, 0x90);/*GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0*/
		mdelay(5);
		write_reg(fb_hx8347g_data, 0x1F, 0xD4);/*GAS=1, VOMG=10, PON=1, DK=0, XDK=1, DDVDH_TRI=0, STB=0*/
		mdelay(5);

		/*display on setting */
		write_reg(fb_hx8347g_data, 0x28, 0x38);//GON=1, DTE=1, D=1000
		mdelay(40);
		write_reg(fb_hx8347g_data, 0x28, 0x3C);//GON=1, DTE=1, D=1100
		printk("display resuming\n");

	}
	return 0;
}

static int fb_hx8347g_resume(struct device *dev)
{
	printk("%s\n",__func__);
	/* power on lvds chip */
	gpio_set_value(pwdn_gpio,1);
	mdelay(1);

	/* power on */
	write_reg(fb_hx8347g_data, 0x18, 0x36);//RADJ 70HZ
	write_reg(fb_hx8347g_data, 0x19, 0x01); /* start osc OSC_EN=1 */
	write_reg(fb_hx8347g_data, 0x1F, 0x88);/*GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0, STB=0*/
	mdelay(5);
	write_reg(fb_hx8347g_data, 0x1F, 0x80);/*GAS=1, VOMG=00, PON=0, DK=0, XDK=0, DVDH_TRI=0, STB=0*/
	mdelay(5);
	write_reg(fb_hx8347g_data, 0x1F, 0x90);/*GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0*/
	mdelay(5);
	write_reg(fb_hx8347g_data, 0x1F, 0xD4);/*GAS=1, VOMG=10, PON=1, DK=0, XDK=1, DDVDH_TRI=0, STB=0*/
	mdelay(5);

	/*display on setting */
	write_reg(fb_hx8347g_data, 0x28, 0x38);//GON=1, DTE=1, D=1000
	mdelay(40);
	write_reg(fb_hx8347g_data, 0x28, 0x3C);//GON=1, DTE=1, D=1100


	return 0;
}

static ssize_t hx8347_suspend_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	
	return sprintf(buf, "%u\n", hx8347_suspend_state);
}

static ssize_t hx8347_suspend_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc = -ENXIO;
	unsigned long data;

	rc = kstrtoul(buf, 0, &data);
	if (rc)
		return rc;

	hx8347_suspend_state = (bool)data;
	
	fb_hx8347g_sysfs_suspend(hx8347_suspend_state);
		
	rc =count;
	return rc;
}
static DEVICE_ATTR_RW(hx8347_suspend_state);

static struct attribute *suspend_attrs[] = {
	&dev_attr_hx8347_suspend_state.attr,
	NULL,
};

static struct attribute_group hx8347_attr_group = {
	.attrs = suspend_attrs,
};

static int fb_hx8347g_probe(struct spi_device *spi)
{
	struct fb_hx8347g *fb_hx8347g;
	void *buf = NULL;
	int ret;
	int gpio, flags,error;
	enum of_gpio_flags of_flags;

	fb_hx8347g = devm_kzalloc(&spi->dev, sizeof(*fb_hx8347g), GFP_KERNEL);
	if (!fb_hx8347g)
		return -ENOMEM;

	fb_hx8347g_data = fb_hx8347g; 

	fb_hx8347g->spi = spi;

	ret = of_property_read_u32(spi->dev.of_node, "regwidth", &fb_hx8347g->regwidth);
	if (ret < 0) {
		goto err1;
	}
	ret = of_property_read_u32(spi->dev.of_node, "buswidth", &fb_hx8347g->buswidth);
	if (ret < 0) {
		goto err1;
	}

	buf = devm_kzalloc(&spi->dev, 128, GFP_KERNEL);
	if (!buf)
		goto err1;

	fb_hx8347g->buf = buf;

	if (of_find_property(spi->dev.of_node, "reset-gpios", NULL)) {
		gpio = of_get_named_gpio_flags(spi->dev.of_node, "reset-gpios", 0, &of_flags);
		if (gpio == -EPROBE_DEFER) {
			ret = gpio;
			goto err1;
		}
		if (gpio < 0) {
			dev_err(&spi->dev,
				"failed to get '%s' from DT\n", "reset-gpios");
			ret = gpio;
			goto err1;
		}

		/* active low translates to initially low */
		flags = (of_flags & OF_GPIO_ACTIVE_LOW) ? GPIOF_OUT_INIT_LOW :
							GPIOF_OUT_INIT_HIGH;
		ret = devm_gpio_request_one(&spi->dev, gpio, flags,
						spi->dev.driver->name);
		if (ret) {
			dev_err(&spi->dev,
				"gpio_request_one('%s'=%d) failed with %d\n",
				"reset-gpios", gpio, ret);
			goto err1;
		}

		fb_hx8347g->reset_gpio = gpio;
		fb_hx8347g_reset(gpio);
	}

#if 1
	if (of_find_property(spi->dev.of_node, "pwdn-gpios", NULL)) {
		pwdn_gpio = of_get_named_gpio_flags(spi->dev.of_node, "pwdn-gpios", 0, &of_flags);
		if (pwdn_gpio == -EPROBE_DEFER) {
			ret = pwdn_gpio;
			goto err1;
		}
		if (pwdn_gpio < 0) {
			dev_err(&spi->dev,
				"failed to get '%s' from DT\n", "pwdn-gpios");
			ret = pwdn_gpio;
			goto err1;
		}

		/* active low translates to initially low */
		flags = (of_flags & OF_GPIO_ACTIVE_LOW) ? GPIOF_OUT_INIT_LOW :
							GPIOF_OUT_INIT_HIGH;
		ret = devm_gpio_request_one(&spi->dev, pwdn_gpio, flags,
						spi->dev.driver->name);
		if (ret) {
			dev_err(&spi->dev,
				"gpio_request_one('%s'=%d) failed with %d\n",
				"pwdn-gpios", pwdn_gpio, ret);
			goto err1;
		}
		gpio_set_value(pwdn_gpio,1);
	}
#endif
	/* 16-bit SPI setup */
	if (fb_hx8347g->spi && fb_hx8347g->buswidth == 16) {
		fb_hx8347g->spi->bits_per_word = 16;
		ret = fb_hx8347g->spi->master->setup(fb_hx8347g->spi);
		if (ret) {
			dev_warn(&fb_hx8347g->spi->dev,
					"16-bit SPI not available\n");
			goto err1;
		}
	}

	error = sysfs_create_group(&spi->dev.kobj, &hx8347_attr_group);
	if (error) {
		dev_err(&spi->dev, "Unable to export suspend state, error: %d\n",
			error);
		return error;
	}

	/* configure lcd */
	//fb_hx8347g_init_display(fb_hx8347g);//already configured in u-boot

	dev_info(&fb_hx8347g->spi->dev, "success\n");

	return 0;

err1:
	return ret;
}

static int fb_hx8347g_remove(struct spi_device *spi)
{
	sysfs_remove_group(&spi->dev.kobj, &hx8347_attr_group);	
	return 0;
}

static const struct spi_device_id fb_hx8347g_ids[] = {
	{ "fb_hx8347g" },
	{ },
};

static SIMPLE_DEV_PM_OPS(fb_hx8347g_pm, fb_hx8347g_suspend, fb_hx8347g_resume);

MODULE_DEVICE_TABLE(spi, fb_hx8347g_ids);

static struct spi_driver fb_hx8347g_driver = {
	.driver = {
		.name	= "fb_hx8347g",
		.owner	= THIS_MODULE,
		.pm	= &fb_hx8347g_pm, 
	},
	.id_table	= fb_hx8347g_ids,
	.probe	= fb_hx8347g_probe,
	.remove	= fb_hx8347g_remove,
};

module_spi_driver(fb_hx8347g_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("e-con Systems India Pvt Ltd");
MODULE_DESCRIPTION("SPI driver for HX8347G LCD Panel");
