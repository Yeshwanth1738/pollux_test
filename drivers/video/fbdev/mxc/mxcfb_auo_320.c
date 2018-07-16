/*
 * e-consystems India Pvt. Ltd.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/mipi_dsi.h>
#include <linux/mxcfb.h>
#include <linux/backlight.h>
#include <video/mipi_display.h>

#include "mipi_dsi.h"

#define AUO_ARS163_SINGLE_DATA_LANE	0x01
#define AUO_ARS163_MAX_DPHY_CLK		800
#define PANEL_ROTATE 			0x01

struct auo_reg {
	/* Address and register value */
	u8 data[35];
	int len;
	int delay;
};

struct auo_reg auo_reg_poweron_sequence[] = {
	{ { 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00 }, 6, 0 },
	{ { 0xBD, 0x03, 0x20, 0x14, 0x4B, 0x00 }, 6, 0 },
	{ { 0xBE, 0x01, 0x90, 0x14, 0x14, 0x01 }, 6, 0 },
	{ { 0xBF, 0x03, 0x20, 0x14, 0x4B, 0x00 }, 6, 0 },
	{ { 0xBB, 0x07, 0x07, 0x07 }, 4, 0 },
	{ { 0xC7, 0x40 }, 2, 0 },
	{ { 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x02 }, 6, 0 },
	{ { 0xEB, 0x02 }, 2, 0 },
	{ { 0xFE, 0x08, 0x50 }, 3, 0 },
	{ { 0xC3, 0xF2, 0x95, 0x04 }, 4, 0 },
	{ { 0xE9, 0x00, 0x36, 0x38 }, 4, 0 },
	{ { 0xCA, 0x04 }, 2, 0 },
	{ { 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01 }, 6, 0 },
	{ { 0xB0, 0x03, 0x03, 0x03 }, 4, 0 },
	{ { 0xB1, 0x05, 0x05, 0x05 }, 4, 0 },
	{ { 0xB2, 0x01, 0x01, 0x01 }, 4, 0 },
	{ { 0xB4, 0x07, 0x07, 0x07 }, 4, 0 },
	{ { 0xB5, 0x03, 0x03, 0x03 }, 4, 0 },
	{ { 0xB6, 0x55, 0x55, 0x55 }, 4, 0 },
	{ { 0xB7, 0x36, 0x36, 0x36 }, 4, 0 },
	{ { 0xB8, 0x23, 0x23, 0x23 }, 4, 0 },
	{ { 0xB9, 0x03, 0x03, 0x03 }, 4, 0 },
	{ { 0xBA, 0x03, 0x03, 0x03 }, 4, 0 },
	{ { 0xBE, 0x32, 0x30, 0x70 }, 4, 0 },
	{ { 0xCF, 0xFF, 0xD4, 0x95, 0xE8, 0x4F, 0x00, 0x04 }, 8, 0 },
	{ { 0x35, 0x01, }, 2, 0 },
#ifndef PANEL_ROTATE
	{ { 0x36, 0x00, }, 2, 0 },
#else
	{ { 0x36, 0x82, }, 2, 0 },
#endif
	{ { 0xC0, 0x20, }, 2, 0 },
	{ { 0xC2, 0x17, 0x17, 0x17, 0x17, 0x17, 0x0B }, 7, 10 },
	{ { 0x11 }, 1, 350 },
	{ { 0x29 }, 1, 10 },
};


static struct fb_videomode auo_lcd_modedb[] = {
	{
	 "AUO-320", 60, 320, 320, 37880,	//"TRULY-WVGA", 64, 480, 800, 37880,
	 8, 8,
	 6, 6,
	 8, 6,
	 FB_SYNC_OE_LOW_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,
	},
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch		= 0x0,
	.data_lane_num  = AUO_ARS163_SINGLE_DATA_LANE,
	.max_phy_clk    = AUO_ARS163_MAX_DPHY_CLK,
	.dpi_fmt		= MIPI_RGB888,	//888,
};

void mipid_auo_ar163_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data)
{
	*mode = &auo_lcd_modedb[0];
	*size = ARRAY_SIZE(auo_lcd_modedb);
	*data = &lcd_config;
}

int mipid_auo_ars163_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	//u8 buf[DSI_CMD_BUF_MAXSIZE];
	int paramlen, cnt  = 0;
	u8 data_type = 0;
	int err = 0;
	int datalen = 0;

	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI LCD AUO ARS163 setup.\n");
//	printk("%s ++\n",__func__);

	paramlen = ARRAY_SIZE(auo_reg_poweron_sequence);

//	printk("%s : paramlen = %d \n",__func__,paramlen);
#if 1	
	for(cnt =0; cnt < paramlen; cnt++)
	{
		if(auo_reg_poweron_sequence[cnt].len == 1)
		{
			data_type = MIPI_DSI_DCS_SHORT_WRITE;
			datalen = 0;
		}
		else if(auo_reg_poweron_sequence[cnt].len == 2 )
		{
			data_type = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
			datalen = 0;
		}
		else
		{
			data_type = MIPI_DSI_DCS_LONG_WRITE;
			datalen = auo_reg_poweron_sequence[cnt].len;
		}
		
//		printk("%s: data_type = 0x%x, data_first_byte = 0x%x, len = %d \n",__func__, data_type, auo_reg_poweron_sequence[cnt].data[0], datalen);
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, data_type,(u32 *)auo_reg_poweron_sequence[cnt].data, datalen);
//		printk("%s: err = %d \n",__func__,err);
		if(err < 0)
			break;
		if(auo_reg_poweron_sequence[cnt].delay > 0)
			mdelay(auo_reg_poweron_sequence[cnt].delay);
	}

	
#endif	
//	printk("%s --\n",__func__);
	
	return 0;
}

