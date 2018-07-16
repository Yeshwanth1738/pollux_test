/*
 * Copyright (C) 2016 e-Con Systems India Pvt. Ltd. All Rights Reserved.
 * Copyright (C) 2008-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <sound/soc.h>

#include "imx-audmux.h"

static int imx_audmux_config(int int_port, int ext_port)
{
	int ret = 0;
	unsigned int ptcr, pdcr;

	int_port--;
	ext_port--;

	/* Output port: 2, connected to SSI
	   PTCR2’s = 0
	   PDCR2’s RXDSEL[2:0] = Receiver data from Port 5
	 */

	ptcr = IMX_AUDMUX_V2_PTCR_SYN;
	pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port);

	ret = imx_audmux_v2_configure_port(int_port, ptcr, pdcr);
	if (ret) {
		pr_err("audmux internal port setup failed\n");
		return ret;
	}

	/* Input port: 5, connected to by chip
	   PTCR5’s RFSDIR and RCLKDIR = 0.
	 */
	ptcr = IMX_AUDMUX_V2_PTCR_SYN |
	       IMX_AUDMUX_V2_PTCR_TFSEL(int_port) | IMX_AUDMUX_V2_PTCR_TFSDIR |
	       IMX_AUDMUX_V2_PTCR_TCSEL(int_port) | IMX_AUDMUX_V2_PTCR_TCLKDIR;
	pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(int_port);

	ret = imx_audmux_v2_configure_port(ext_port, ptcr, pdcr);
	if (ret) {
		pr_err("audmux external port setup failed\n");
		return ret;
	}

	return 0;
}

static int imx_btlsr_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	u32 channels = params_channels(params);
	u32 rate = params_rate(params);
	u32 bclk = rate * channels * 32;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret) {
		dev_err(cpu_dai->dev, "failed to set dai fmt\n");
		return ret;
	}
/*
	ret = snd_soc_dai_set_tdm_slot(cpu_dai,
				       channels == 1 ? 0xfffffffe : 0xfffffffc,
				       channels == 1 ? 0xfffffffe : 0xfffffffc,
				       2, 32);
							 */
 ret = snd_soc_dai_set_tdm_slot(cpu_dai,
 			channels == 1 ? 1 : 0x3,
 			channels == 1 ? 1 : 0x3,
 			2, 32);
	if (ret) {
		dev_err(cpu_dai->dev, "failed to set dai tdm slot\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, bclk, SND_SOC_CLOCK_OUT);
	if (ret)
		dev_err(cpu_dai->dev, "failed to set sysclk\n");

	return ret;

}

static struct snd_soc_ops imx_btlsr_ops = {
	.hw_params	= imx_btlsr_hw_params,
};

static struct snd_soc_dai_link imx_dai_bt = {
	.name		= "imx-btlsr",
	.stream_name	= "imx-btlsr",
	.codec_dai_name = "bt-sco-pcm",
	.ops		= &imx_btlsr_ops,
};


static struct snd_soc_card snd_soc_card_bt = {
	.name		= "imx-audio-btlsr",
	.dai_link	= &imx_dai_bt,
	.num_links	= 1,
	.owner = THIS_MODULE,
};

static int imx_btlsr_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_card_bt;
	struct device_node *ssi_np, *np = pdev->dev.of_node;
	struct platform_device *ssi_pdev;
	struct device_node *bt_np;
	int int_port, ext_port, ret;

	pr_info("%s(): Initialzing BTLSR SCO Sound card\n", __func__);

	ret = of_property_read_u32(np, "mux-int-port", &int_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
		return ret;
	}

	ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
		return ret;
	}
	//imx_audmux_config(int int_port, int ext_port)
	imx_audmux_config(int_port, ext_port);

	ssi_np = of_parse_phandle(pdev->dev.of_node, "ssi-controller", 0);
	if (!ssi_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		return -EINVAL;
	}

	ssi_pdev = of_find_device_by_node(ssi_np);
	if (!ssi_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EINVAL;
		goto end;
	}

	bt_np = of_parse_phandle(pdev->dev.of_node, "bt-controller", 0);
	if (!bt_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto end;
	}

	card->dev = &pdev->dev;
	card->dai_link->cpu_dai_name = dev_name(&ssi_pdev->dev);
	card->dai_link->platform_of_node = ssi_np;
	card->dai_link->codec_of_node = bt_np;

	//Now start regestration process
	platform_set_drvdata(pdev, card);

	ret = snd_soc_register_card(card);
	if (ret)
		dev_err(&pdev->dev, "Failed to register card: %d\n", ret);

end:
	if (ssi_np)
		of_node_put(ssi_np);
	if (bt_np)
		of_node_put(bt_np);

	return ret;
}

static int imx_btlsr_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_card_bt;
	pr_info("%s(): Unregister BTLSR SCO Sound card\n", __func__);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id imx_btlsr_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-btlsr", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_btlsr_dt_ids);

static struct platform_driver imx_btlsr_driver = {
	.driver			= {
		.name		= "imx-btlsr",
		.pm		= &snd_soc_pm_ops,
		.of_match_table = imx_btlsr_dt_ids,
	},
	.probe			= imx_btlsr_probe,
	.remove			= imx_btlsr_remove,
};

module_platform_driver(imx_btlsr_driver);

/* Module information */
MODULE_AUTHOR("Mohamed Thalib H");
MODULE_DESCRIPTION("ALSA SoC i.MX BT LSR");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-audio-btlsr");
