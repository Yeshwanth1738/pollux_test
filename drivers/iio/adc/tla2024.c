/* SPDX-License-Identifier: GPL-2.0
 *
 * Texas Instruments TLA2021/TLA2022/TLA2024 12-bit ADC driver
 *
 * Copyright (C) 2018 Koninklijke Philips N.V.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/iio/iio.h>

#define TLA2024_DATA 0x00
#define DATA_RES_MASK GENMASK(15, 4)
#define DATA_RES_SHIFT 4

#define TLA2024_CONF 0x01
#define CONF_OS_MASK BIT(15)
#define CONF_OS_SHIFT 15
#define CONF_MUX_MASK GENMASK(14, 12)
#define CONF_MUX_SHIFT 12
#define CONF_PGA_MASK GENMASK(11, 9)
#define CONF_PGA_SHIFT 9
#define CONF_MODE_MASK BIT(8)
#define CONF_MODE_SHIFT 8
#define CONF_DR_MASK GENMASK(7, 5)
#define CONF_DR_SHIFT 5

#define TLA2024_CONV_RETRY 10

struct tla202x_model {
	unsigned int mux_available;
	unsigned int pga_available;
};

struct tla2024 {
	struct i2c_client *i2c;
	struct tla202x_model *model;
	struct mutex lock; /* protect i2c transfers */
	u16 conf_cache;
	u8 used_mux_channels;
};

struct tla2024_channel {
	int ainp;
	int ainn;
	char *datasheet_name;
	unsigned int differential;
};

static const struct tla2024_channel tla2024_all_channels[] = {
	{0, 1, "AIN0-AIN1", 1},
	{0, 3, "AIN0-AIN3", 1},
	{1, 3, "AIN1-AIN3", 1},
	{2, 3, "AIN2-AIN3", 1},
	{0, -1, "AIN0-GND", 0},
	{1, -1, "AIN1-GND", 0},
	{2, -1, "AIN2-GND", 0},
	{3, -1, "AIN3-GND", 0},
};

static inline int tla2024_find_chan_idx(int ainp_in, int ainn_in, u16 *idx)
{
	u16 i = 0;

	for (i = 0; i < ARRAY_SIZE(tla2024_all_channels); i++) {
		if ((tla2024_all_channels[i].ainp == ainp_in) &&
		    (tla2024_all_channels[i].ainn == ainn_in)) {
			*idx = i;
			return 0;
		}
	}
	return -EINVAL;
}

#define TLA202x_MODEL(_mux, _pga)		\
	{					\
		.mux_available = (_mux),	\
		.pga_available = (_pga),	\
	}

enum tla2024_model_id {
	TLA2024 = 0,
	TLA2022 = 1,
	TLA2021 = 2,
};

static struct tla202x_model tla202x_models[] = {
	TLA202x_MODEL(1, 1), // TLA2024
	TLA202x_MODEL(0, 1), // TLA2022
	TLA202x_MODEL(0, 0), // TLA2021
};

static const int tla2024_samp_freq_avail[] = {
	128, 250, 490, 920, 1600, 2400, 3300 };

static const int tla2024_pga_scale[] = {
	3000, 2000, 1000, 500, 250, 125 };

static const char * const tla2024_full_scale_range_avail[] = {
	"6144000", "4096000", "2048000", "1024000", "512000", "256000" };

static const char * const tla2024_conversion_mode_avail[] = {
				"Continuous",
				"Single-Shot" };

static int tla2024_get(struct tla2024 *priv, u8 addr, u16 mask,
		       u16 shift, u16 *val)
{
	int ret;
	u16 data;

	mutex_lock(&priv->lock);

	ret = i2c_smbus_read_word_swapped(priv->i2c, addr);
	if (ret < 0) {
		mutex_unlock(&priv->lock);
		return ret;
	}

	data = (u16)ret;
	if (addr == TLA2024_CONF)
		priv->conf_cache = data;

	*val = (mask & data) >> shift;
	mutex_unlock(&priv->lock);
	return 0;
}

static int tla2024_set(struct tla2024 *priv, u8 addr, u16 mask,
		       u16 shift, u16 val)
{
	int ret;
	u16 data;

	mutex_lock(&priv->lock);

	ret = i2c_smbus_read_word_swapped(priv->i2c, addr);
	if (ret < 0) {
		mutex_unlock(&priv->lock);
		return ret;
	}
	data = (u16)ret;
	data &= ~mask;
	data |= mask & (val << shift);

	ret = i2c_smbus_write_word_swapped(priv->i2c, addr, data);
	if (!ret && addr == TLA2024_CONF)
		priv->conf_cache = data;

	mutex_unlock(&priv->lock);
	return ret;
}

static int tla2024_get_conversion_mode(struct iio_dev *idev,
				       const struct iio_chan_spec *chan)
{
	struct tla2024 *priv = iio_priv(idev);
	int ret;
	u16 data;

	ret = tla2024_get(priv, TLA2024_CONF, CONF_MODE_MASK,
			  CONF_MODE_SHIFT, &data);
	if (ret < 0)
		return ret;

	return data;
}

static int tla2024_set_conversion_mode(struct iio_dev *idev,
				       const struct iio_chan_spec *chan,
				       unsigned int data)
{
	struct tla2024 *priv = iio_priv(idev);

	return tla2024_set(priv, TLA2024_CONF, CONF_MODE_MASK,
			   CONF_MODE_SHIFT, data);
}

static int tla2024_get_full_scale_range(struct iio_dev *idev,
					const struct iio_chan_spec *chan)
{
	struct tla2024 *priv = iio_priv(idev);
	int ret;
	u16 data;

	ret = tla2024_get(priv, TLA2024_CONF, CONF_PGA_MASK,
			  CONF_PGA_SHIFT, &data);
	if (ret < 0)
		return ret;

	return data;
}

static int tla2024_set_full_scale_range(struct iio_dev *idev,
					const struct iio_chan_spec *chan,
					unsigned int data)
{
	struct tla2024 *priv = iio_priv(idev);

	return tla2024_set(priv, TLA2024_CONF, CONF_PGA_MASK,
			   CONF_PGA_SHIFT, data);
}

static const struct iio_enum tla2024_conversion_mode = {
	.items = tla2024_conversion_mode_avail,
	.num_items = ARRAY_SIZE(tla2024_conversion_mode_avail),
	.get = tla2024_get_conversion_mode,
	.set = tla2024_set_conversion_mode,
};

static const struct iio_enum tla2024_full_scale_range = {
	.items = tla2024_full_scale_range_avail,
	.num_items = ARRAY_SIZE(tla2024_full_scale_range_avail),
	.get = tla2024_get_full_scale_range,
	.set = tla2024_set_full_scale_range,
};

#define TLA2024_IIO_ENUM_AVAIL(_name, _shared, _e)			\
{									\
	.name = (_name "_available"),					\
	.shared = (_shared),						\
	.read = iio_enum_available_read,				\
	.private = (uintptr_t)(_e),			\
}

static const struct iio_chan_spec_ext_info tla202x_ext_info_pga[] = {
	IIO_ENUM("conversion_mode", IIO_SHARED_BY_ALL,
		 &tla2024_conversion_mode),
	TLA2024_IIO_ENUM_AVAIL("conversion_mode", IIO_SHARED_BY_ALL,
			       &tla2024_conversion_mode),
	IIO_ENUM("full_scale_range", IIO_SHARED_BY_ALL,
		 &tla2024_full_scale_range),
	TLA2024_IIO_ENUM_AVAIL("full_scale_range", IIO_SHARED_BY_ALL,
			       &tla2024_full_scale_range),
	{},
};

static const struct iio_chan_spec_ext_info tla202x_ext_info[] = {
	IIO_ENUM("conversion_mode", IIO_SHARED_BY_ALL,
		 &tla2024_conversion_mode),
	TLA2024_IIO_ENUM_AVAIL("conversion_mode", IIO_SHARED_BY_ALL,
			       &tla2024_conversion_mode),
	{},
};

#if 0
static int tla2024_read_avail(struct iio_dev *idev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:

		*length = ARRAY_SIZE(tla2024_samp_freq_avail);
		*vals = tla2024_samp_freq_avail;

		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;
	}

	return -EINVAL;
}
#endif

static int tla2024_of_find_chan(struct tla2024 *priv, struct device_node *ch)
{
	u16 chan_idx = 0;
	u32 tmp_p, tmp_n;
	int ainp, ainn;
	int ret;

	ret = of_property_read_u32_index(ch, "single-ended", 0, &tmp_p);
	if (ret) {
		ret = of_property_read_u32_index(ch,
						 "differential", 0, &tmp_p);
		if (ret)
			return ret;

		ret = of_property_read_u32_index(ch,
						 "differential", 1, &tmp_n);
		if (ret)
			return ret;

		ainp = (int)tmp_p;
		ainn = (int)tmp_n;
	} else {
		ainp = (int)tmp_p;
		ainn = -1;
	}

	ret = tla2024_find_chan_idx(ainp, ainn, &chan_idx);
	if (ret < 0)
		return ret;

	// if model doesn"t have mux then only channel 0 is allowed
	if (!priv->model->mux_available && chan_idx)
		return -EINVAL;

	// if already used
	if ((priv->used_mux_channels) & (1 << chan_idx))
		return -EINVAL;

	return chan_idx;
}

static int tla2024_init_chan(struct iio_dev *idev, struct device_node *node,
			     struct iio_chan_spec *chan)
{
	struct tla2024 *priv = iio_priv(idev);
	u16 chan_idx = 0;
	int ret;

	ret = tla2024_of_find_chan(priv, node);
	if (ret < 0)
		return ret;

	chan_idx = (u16)ret;
	priv->used_mux_channels |= (1 << chan_idx);
	chan->type = IIO_VOLTAGE;
	chan->channel = tla2024_all_channels[chan_idx].ainp;
	chan->channel2 = tla2024_all_channels[chan_idx].ainn;
	chan->differential = tla2024_all_channels[chan_idx].differential;
	chan->extend_name = node->name;
	chan->datasheet_name = tla2024_all_channels[chan_idx].datasheet_name;
	chan->indexed = 1;
	chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
					BIT(IIO_CHAN_INFO_PROCESSED);
	chan->info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ);
//	chan->info_mask_shared_by_all_available =
//					BIT(IIO_CHAN_INFO_SAMP_FREQ);

	if (priv->model->pga_available)
		chan->ext_info = tla202x_ext_info_pga;
	else
		chan->ext_info = tla202x_ext_info;

	return 0;
}

static int tla2024_wait(struct tla2024 *priv)
{
	int ret = 0;
	u16 retry = TLA2024_CONV_RETRY;
	u16 status;

	do {
		if (!--retry)
			return -EIO;
		ret = tla2024_get(priv, TLA2024_CONF, CONF_OS_MASK,
				  CONF_OS_SHIFT, &status);
		if (ret < 0)
			return ret;
		if (!status)
			usleep_range(25, 1000);
	} while (!status);

	return ret;
}

static int tla2024_select_channel(struct tla2024 *priv,
				  struct iio_chan_spec const *chan)
{
	int ret = 0;
	int ainp = chan->channel;
	int ainn = chan->channel2;
	u16 tmp, chan_id = 0;

	tla2024_find_chan_idx(ainp, ainn, &chan_id);

	tmp = (priv->conf_cache & CONF_MUX_MASK) >> CONF_MUX_SHIFT;
	if (tmp != chan_id)
		ret = tla2024_set(priv, TLA2024_CONF, CONF_MUX_MASK,
				  CONF_MUX_SHIFT, chan_id);
	return ret;
}

static int tla2024_convert(struct tla2024 *priv)
{
	int ret = 0;

	if (priv->conf_cache & CONF_MODE_MASK) {
		ret = tla2024_set(priv, TLA2024_CONF, CONF_OS_MASK,
				  CONF_OS_SHIFT, 1);
		if (ret < 0)
			return ret;
		ret = tla2024_wait(priv);
	}
	return ret;
}

static int tla2024_read_raw(struct iio_dev *idev,
			    struct iio_chan_spec const *channel, int *val,
			    int *val2, long mask)
{
	struct tla2024 *priv = iio_priv(idev);
	int ret;
	u16 data, idx;
	s16 tmp;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = tla2024_select_channel(priv, channel);
		if (ret < 0)
			return ret;

		ret = tla2024_convert(priv);
		if (ret < 0)
			return ret;

		ret = tla2024_get(priv, TLA2024_DATA, DATA_RES_MASK,
				  DATA_RES_SHIFT, &data);
		if (ret < 0)
			return ret;

		*val = data;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		ret = tla2024_select_channel(priv, channel);
		if (ret < 0)
			return ret;

		ret = tla2024_convert(priv);
		if (ret < 0)
			return ret;

		ret = tla2024_get(priv, TLA2024_DATA, DATA_RES_MASK,
				  DATA_RES_SHIFT, &data);
		if (ret < 0)
			return ret;

		tmp = (s16)(data << DATA_RES_SHIFT);
		idx = (priv->conf_cache & CONF_PGA_MASK) >> CONF_PGA_SHIFT;
		*val = (tmp >> DATA_RES_SHIFT) * tla2024_pga_scale[idx];
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:

		ret = tla2024_get(priv, TLA2024_CONF, CONF_DR_MASK,
				  CONF_DR_SHIFT, &data);
		if (ret < 0)
			return ret;

		*val = tla2024_samp_freq_avail[data];
		return IIO_VAL_INT;

	default:
		break;
	}

	return -EINVAL;
}

static int tla2024_write_raw(struct iio_dev *idev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct tla2024 *priv = iio_priv(idev);
	u16 i;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:

		for (i = 0; i < ARRAY_SIZE(tla2024_samp_freq_avail); i++) {
			if (tla2024_samp_freq_avail[i] == val)
				break;
		}

		if (i == ARRAY_SIZE(tla2024_samp_freq_avail))
			return -EINVAL;

		return tla2024_set(priv, TLA2024_CONF, CONF_DR_MASK,
					CONF_DR_SHIFT, i);

	default:
		break;
	}

	return -EINVAL;
}

static int tla2024_of_chan_init(struct iio_dev *idev)
{
	struct device_node *node = idev->dev.of_node;
	struct device_node *child;
	struct iio_chan_spec *channels;
	int ret, i = 0, num_channels = 0;

	num_channels = of_get_available_child_count(node);
	if (!num_channels) {
		dev_err(&idev->dev, "no channels configured\n");
		return -ENODEV;
	}

	channels = devm_kcalloc(&idev->dev, num_channels,
				sizeof(struct iio_chan_spec), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	i = 0;
	for_each_available_child_of_node(node, child) {
		ret = tla2024_init_chan(idev, child, &channels[i]);
		if (ret) {
			of_node_put(child);
			return ret;
		}
		i++;
	}

	idev->channels = channels;
	idev->num_channels = num_channels;

	return 0;
}

static const struct iio_info tla2024_info = {
	.read_raw = tla2024_read_raw,
	.write_raw = tla2024_write_raw,
//	.read_avail = tla2024_read_avail,
};

static int tla2024_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *iio;
	struct tla2024 *priv;
	struct tla202x_model *model;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	model = &tla202x_models[id->driver_data];

	iio = devm_iio_device_alloc(&client->dev, sizeof(*priv));
	if (!iio)
		return -ENOMEM;

	priv = iio_priv(iio);
	priv->i2c = client;
	priv->model = model;
	mutex_init(&priv->lock);

	iio->dev.parent = &client->dev;
	iio->dev.of_node = client->dev.of_node;
	iio->name = client->name;
	iio->modes = INDIO_DIRECT_MODE;
	iio->info = &tla2024_info;

	ret = tla2024_of_chan_init(iio);
	if (ret < 0)
		return ret;

	ret = tla2024_set(priv, TLA2024_CONF, CONF_MODE_MASK,
			  CONF_MODE_SHIFT, 1);
	if (ret < 0)
		return ret;

	ret = iio_device_register(iio);

	return ret;
}

static int tla2024_remove(struct i2c_client *client)
{
	struct iio_dev *iio = i2c_get_clientdata(client);

	iio_device_unregister(iio);
	return 0;
}

static const struct i2c_device_id tla2024_id[] = {
	{ "tla2024", TLA2024 },
	{ "tla2022", TLA2022 },
	{ "tla2021", TLA2021 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tla2024_id);

static const struct of_device_id tla2024_of_match[] = {
	{ .compatible = "ti,tla2024" },
	{ .compatible = "ti,tla2022" },
	{ .compatible = "ti,tla2021" },
	{ }
};
MODULE_DEVICE_TABLE(of, tla2024_of_match);

static struct i2c_driver tla2024_driver = {
	.driver = {
		.name = "tla2024",
		.of_match_table = of_match_ptr(tla2024_of_match),
	},
	.probe = tla2024_probe,
	.remove = tla2024_remove,
	.id_table = tla2024_id,
};
module_i2c_driver(tla2024_driver);

MODULE_AUTHOR("Ibtsam Haq <ibtsam.haq@philips.com>");
MODULE_DESCRIPTION("Texas Instruments TLA2021/TLA2022/TLA2024 ADC driver");
MODULE_LICENSE("GPL v2");
