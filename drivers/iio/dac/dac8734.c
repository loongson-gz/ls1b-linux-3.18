/*
 * TI DAC8734 quad-channel
 * Digital to Analog Converters driver
 *
 * Copyright tanghaifeng. base on drivers/iio/dac/ad5764.c
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define DAC8734_NUM_CHANNELS 4

#define DAC8734_REG_DATA(x)			((0x04) | (x))
#define DAC8734_REG_FINE_GAIN(x)			((0x0c) | (x))
#define DAC8734_REG_OFFSET(x)			((0x08) | (x))

struct dac8734_chip_info {
	unsigned long int_vref;
	const struct iio_chan_spec *channels;
};

/**
 * struct dac8734_state - driver instance specific data
 * @spi:		spi_device
 * @chip_info:		chip info
 * @vref_reg:		vref supply regulators
 * @data:		spi transfer buffers
 */

struct dac8734_state {
	struct spi_device		*spi;
	const struct dac8734_chip_info	*chip_info;
	struct regulator_bulk_data	vref_reg[2];

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	union {
		__be32 d32;
		u8 d8[4];
	} data[2] ____cacheline_aligned;
};

enum dac8734_type {
	ID_DAC8734,
};

#define DAC8734_CHANNEL(_chan, _bits) {				\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.output = 1,						\
	.channel = (_chan),					\
	.address = (_chan),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
		BIT(IIO_CHAN_INFO_SCALE) |			\
		BIT(IIO_CHAN_INFO_CALIBSCALE) |			\
		BIT(IIO_CHAN_INFO_CALIBBIAS),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_OFFSET),	\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = (_bits),				\
		.storagebits = 16,				\
		.shift = 16 - (_bits),				\
	},							\
}

#define DECLARE_DAC8734_CHANNELS(_name, _bits) \
const struct iio_chan_spec _name##_channels[] = { \
	DAC8734_CHANNEL(0, (_bits)), \
	DAC8734_CHANNEL(1, (_bits)), \
	DAC8734_CHANNEL(2, (_bits)), \
	DAC8734_CHANNEL(3, (_bits)), \
};

static DECLARE_DAC8734_CHANNELS(dac8734, 16);

static const struct dac8734_chip_info dac8734_chip_infos[] = {
	[ID_DAC8734] = {
		.int_vref = 5000000,
		.channels = dac8734_channels,
	},
};

static int dac8734_write(struct iio_dev *indio_dev, unsigned int reg,
	unsigned int val)
{
	struct dac8734_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	st->data[0].d32 = cpu_to_be32((reg << 16) | val);

	ret = spi_write(st->spi, &st->data[0].d8[1], 3);
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int dac8734_read(struct iio_dev *indio_dev, unsigned int reg,
	unsigned int *val)
{
	struct dac8734_state *st = iio_priv(indio_dev);
	int ret;
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->data[0].d8[1],
			.len = 3,
			.cs_change = 1,
		}, {
			.rx_buf = &st->data[1].d8[1],
			.len = 3,
		},
	};

	mutex_lock(&indio_dev->mlock);

	st->data[0].d32 = cpu_to_be32((1 << 23) | (reg << 16));

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret >= 0)
		*val = be32_to_cpu(st->data[1].d32) & 0xffff;

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int dac8734_chan_info_to_reg(struct iio_chan_spec const *chan, long info)
{
	switch (info) {
	case 0:
		return DAC8734_REG_DATA(chan->address);
	case IIO_CHAN_INFO_CALIBBIAS:
		return DAC8734_REG_OFFSET(chan->address);
	case IIO_CHAN_INFO_CALIBSCALE:
		return DAC8734_REG_FINE_GAIN(chan->address);
	default:
		break;
	}

	return 0;
}

static int dac8734_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long info)
{
	const int max_val = (1 << chan->scan_type.realbits);
	unsigned int reg;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (val >= max_val || val < 0)
			return -EINVAL;
		val <<= chan->scan_type.shift;
		break;
	case IIO_CHAN_INFO_CALIBBIAS:
		if (val >= 128 || val < -128)
			return -EINVAL;
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		if (val >= 32 || val < -32)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	reg = dac8734_chan_info_to_reg(chan, info);
	return dac8734_write(indio_dev, reg, (u16)val);
}

static int dac8734_get_channel_vref(struct dac8734_state *st,
	unsigned int channel)
{
	if (st->chip_info->int_vref)
		return st->chip_info->int_vref;
	else
		return regulator_get_voltage(st->vref_reg[channel / 2].consumer);
}

static int dac8734_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
	struct dac8734_state *st = iio_priv(indio_dev);
	unsigned int reg;
	int vref;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		reg = DAC8734_REG_DATA(chan->address);
		ret = dac8734_read(indio_dev, reg, val);
		if (ret < 0)
			return ret;
		*val >>= chan->scan_type.shift;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBBIAS:
		reg = DAC8734_REG_OFFSET(chan->address);
		ret = dac8734_read(indio_dev, reg, val);
		if (ret < 0)
			return ret;
		*val = sign_extend32(*val, 7);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		reg = DAC8734_REG_FINE_GAIN(chan->address);
		ret = dac8734_read(indio_dev, reg, val);
		if (ret < 0)
			return ret;
		*val = sign_extend32(*val, 8);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		/* vout = 4 * vref + ((dac_code / 65536) - 0.5) */
		vref = dac8734_get_channel_vref(st, chan->channel);
		if (vref < 0)
			return vref;

		*val = vref * 4 / 1000;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_OFFSET:
		*val = -(1 << chan->scan_type.realbits) / 2;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static const struct iio_info dac8734_info = {
	.read_raw = dac8734_read_raw,
	.write_raw = dac8734_write_raw,
	.driver_module = THIS_MODULE,
};

static int dac8734_probe(struct spi_device *spi)
{
	enum dac8734_type type = spi_get_device_id(spi)->driver_data;
	struct iio_dev *indio_dev;
	struct dac8734_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL) {
		dev_err(&spi->dev, "Failed to allocate iio device\n");
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;
	st->chip_info = &dac8734_chip_infos[type];

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &dac8734_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = DAC8734_NUM_CHANNELS;
	indio_dev->channels = st->chip_info->channels;

	if (st->chip_info->int_vref == 0) {
		st->vref_reg[0].supply = "vrefAB";
		st->vref_reg[1].supply = "vrefCD";

		ret = devm_regulator_bulk_get(&st->spi->dev,
			ARRAY_SIZE(st->vref_reg), st->vref_reg);
		if (ret) {
			dev_err(&spi->dev, "Failed to request vref regulators: %d\n",
				ret);
			return ret;
		}

		ret = regulator_bulk_enable(ARRAY_SIZE(st->vref_reg),
			st->vref_reg);
		if (ret) {
			dev_err(&spi->dev, "Failed to enable vref regulators: %d\n",
				ret);
			return ret;
		}
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to register iio device: %d\n", ret);
		goto error_disable_reg;
	}

	return 0;

error_disable_reg:
	if (st->chip_info->int_vref == 0)
		regulator_bulk_disable(ARRAY_SIZE(st->vref_reg), st->vref_reg);
	return ret;
}

static int dac8734_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct dac8734_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	if (st->chip_info->int_vref == 0)
		regulator_bulk_disable(ARRAY_SIZE(st->vref_reg), st->vref_reg);

	return 0;
}

static const struct spi_device_id dac8734_ids[] = {
	{ "dac8734", ID_DAC8734 },
	{ }
};
MODULE_DEVICE_TABLE(spi, dac8734_ids);

static struct spi_driver dac8734_driver = {
	.driver = {
		.name = "dac8734",
		.owner = THIS_MODULE,
	},
	.probe = dac8734_probe,
	.remove = dac8734_remove,
	.id_table = dac8734_ids,
};
module_spi_driver(dac8734_driver);

MODULE_AUTHOR("tanghaifeng");
MODULE_DESCRIPTION("TI DAC8734 DAC");
MODULE_LICENSE("GPL v2");
