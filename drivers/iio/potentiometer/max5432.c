/*
 * Maxim Integrated MAX5432 digital potentiometer driver
 * base on drivers/iio/potentiometer/ds1803.c
 *
 * Datasheet: https://datasheets.maximintegrated.com/en/ds/MAX5432-MAX5435.pdf
 *
 * DEVID	#Wipers	#Positions	Resistor Opts (kOhm)	i2c address
 * max5432	1	256		10, 50, 100		0101xxx
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/export.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/of.h>

#define MAX5432_MAX_POS		255

#define MAX5432_VREG		0x11
#define MAX5432_NVREG	0x21

#define MAX5432_COPY_NV_TO_V	0x61
#define MAX5432_COPY_V_TO_NV	0x51

enum max5432_type {
	MAX5432,
	MAX5433,
	MAX5434,
	MAX5435,
};

struct max5432_cfg {
	int kohms;
};

static const struct max5432_cfg max5432_cfg[] = {
	[MAX5432] = { .kohms =  50, },
	[MAX5433] = { .kohms = 100, },
	[MAX5434] = { .kohms =  50, },
	[MAX5435] = { .kohms = 100, },
};

struct max5432_data {
	struct i2c_client *client;
	const struct max5432_cfg *cfg;
};

#define MAX5432_CHANNEL(ch) {					\
	.type = IIO_RESISTANCE,					\
	.indexed = 1,						\
	.output = 1,						\
	.channel = (ch),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
}

static const struct iio_chan_spec max5432_channels[] = {
	MAX5432_CHANNEL(0),
};

static int max5432_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct max5432_data *data = iio_priv(indio_dev);
	int ret;
	u8 result[1];

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = i2c_master_recv(data->client, result, 1);
		if (ret < 0)
			return ret;

		*val = result[0];
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 1000 * data->cfg->kohms;
		*val2 = MAX5432_MAX_POS;
		return IIO_VAL_FRACTIONAL;
	}

	return -EINVAL;
}

static int max5432_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct max5432_data *data = iio_priv(indio_dev);

	if (val2 != 0)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val > MAX5432_MAX_POS || val < 0)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	return i2c_smbus_write_byte_data(data->client, MAX5432_VREG, val);
//	return i2c_smbus_write_byte(data->client, MAX5432_COPY_V_TO_NV);
}

static const struct iio_info max5432_info = {
	.read_raw = max5432_read_raw,
	.write_raw = max5432_write_raw,
	.driver_module = THIS_MODULE,
};

static int max5432_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max5432_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	i2c_set_clientdata(client, indio_dev);

	data = iio_priv(indio_dev);
	data->client = client;
	data->cfg = &max5432_cfg[id->driver_data];

	indio_dev->dev.parent = dev;
	indio_dev->info = &max5432_info;
	indio_dev->channels = max5432_channels;
	indio_dev->num_channels = ARRAY_SIZE(max5432_channels);
	indio_dev->name = client->name;

	/* restore wiper regs from NV regs */
	ret = i2c_smbus_write_byte(client, MAX5432_COPY_NV_TO_V);
	if (ret < 0)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static int max5432_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);

	/* save V regs to NV regs */
	return i2c_smbus_write_byte(client, MAX5432_COPY_V_TO_NV);
}

#if defined(CONFIG_OF)
static const struct of_device_id max5432_dt_ids[] = {
	{ .compatible = "maxim,max5432", .data = &max5432_cfg[MAX5432] },
	{ .compatible = "maxim,max5433", .data = &max5432_cfg[MAX5433] },
	{ .compatible = "maxim,max5434", .data = &max5432_cfg[MAX5434] },
	{ .compatible = "maxim,max5435", .data = &max5432_cfg[MAX5435] },
	{}
};
MODULE_DEVICE_TABLE(of, max5432_dt_ids);
#endif /* CONFIG_OF */

static const struct i2c_device_id max5432_id[] = {
	{ "max5432", MAX5432 },
	{ "max5433", MAX5433 },
	{ "max5434", MAX5434 },
	{ "max5435", MAX5435 },
	{}
};
MODULE_DEVICE_TABLE(i2c, max5432_id);

static struct i2c_driver max5432_driver = {
	.driver = {
		.name	= "max5432",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(max5432_dt_ids),
	},
	.id_table	= max5432_id,
	.probe		= max5432_probe,
	.remove = max5432_remove,
};
module_i2c_driver(max5432_driver);

MODULE_AUTHOR("Tang Haifeng");
MODULE_DESCRIPTION("MAX5432 digital potentiometer");
MODULE_LICENSE("GPL v2");
