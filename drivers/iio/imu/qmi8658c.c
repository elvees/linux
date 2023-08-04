// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2016, Intel Corporation.
 * Copyright (c) 2019, Martin Kelly.
 * Copyright 2023 RnD Center "ELVEES", JSC
 *
 * This driver is based on Bosch BMI160 driver.
 *
 * TODO: Add support of:
 *	* other operating modes, except Accel+Gyro and Power Down;
 *	* CTRL9 commands and Wake on Motion;
 *	* reading timestamps;
 *	* reading temperature from internal sensor;
 *	* FIFO buffer;
 *	* device self test.
 */

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/of_irq.h>
#include <linux/bitfield.h>

#include <linux/iio/iio.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>

#define QMI8658C_REG_WHO_AM_I 0x00
#define QMI8658C_WHO_AM_I_VAL 0x05

#define QMI8658C_REG_CTRL_1 0x02
#define QMI8658C_SIM BIT(7)
#define QMI8658C_ADDR_AUTO_INC BIT(6)
#define QMI8658C_DATA_ENDIAN BIT(5)
#define QMI8658C_SENS_DISABLE BIT(0)

#define QMI8658C_REG_CTRL_2 0x03
#define QMI8658C_ACCEL_SCALE_2G 0x00
#define QMI8658C_ACCEL_SCALE_4G 0x01
#define QMI8658C_ACCEL_SCALE_8G 0x02
#define QMI8658C_ACCEL_SCALE_16G 0x03

#define QMI8658C_REG_CTRL_3 0x04
#define QMI8658C_GYRO_SCALE_16DPS 0x00
#define QMI8658C_GYRO_SCALE_32DPS 0x01
#define QMI8658C_GYRO_SCALE_64DPS 0x02
#define QMI8658C_GYRO_SCALE_128DPS 0x03
#define QMI8658C_GYRO_SCALE_256DPS 0x04
#define QMI8658C_GYRO_SCALE_512DPS 0x05
#define QMI8658C_GYRO_SCALE_1024DPS 0x06
#define QMI8658C_GYRO_SCALE_2048DPS 0x07

#define QMI8658C_REG_CTRL_ODR GENMASK(3, 0)
#define QMI8658C_REG_CTRL_SCALE GENMASK(6, 4)

#define QMI8658C_REG_CTRL_4 0x05
#define QMI8658C_MAG_SETTINGS_ODR_MASK GENMASK(2, 0)
#define QMI8658C_MAG_SETTINGS_EXT_DEV GENMASK(6, 3)

#define QMI8658C_REG_CTRL_5 0x06
#define QMI8658C_GYRO_LPF_MODE_MASK GENMASK(6, 5)
#define QMI8658C_GYRO_LPF_EN BIT(4)
#define QMI8658C_ACCEL_LPF_MODE_MASK GENMASK(2, 1)
#define QMI8658C_ACCEL_LPF_EN BIT(0)
#define QMI8658C_LPF_MODE_2_62 0x00
#define QMI8658C_LPF_MODE_3_59 0x01
#define QMI8658C_LPF_MODE_5_32 0x02
#define QMI8658C_LPF_MODE_14_0 0x03

#define QMI8658C_REG_CTRL_6 0x07

#define QMI8658C_REG_CTRL_7 0x08
#define QMI8658C_SYNC_SMPL BIT(7)
#define QMI8658C_SYS_HS BIT(6)
#define QMI8658C_GYRO_MODE BIT(4)
#define QMI8658C_S_EN BIT(3)
#define QMI8658C_MAG_EN BIT(2)
#define QMI8658C_GYRO_EN BIT(1)
#define QMI8658C_ACCEL_EN BIT(0)

#define QMI8658C_REG_CTRL_8 0x09

#define QMI8658C_REG_CTRL_9 0x0a

#define QMI8658C_REG_CAL1_L 0x0b
#define QMI8658C_REG_CAL1_H 0x0c
#define QMI8658C_REG_CAL2_L 0x0d
#define QMI8658C_REG_CAL2_H 0x0e
#define QMI8658C_REG_CAL3_L 0x0f
#define QMI8658C_REG_CAL3_H 0x10
#define QMI8658C_REG_CAL4_L 0x11
#define QMI8658C_REG_CAL4_H 0x12

#define QMI8658C_REG_STATUSINT 0x2d
#define QMI8658C_CTRL9_CMD_DONE BIT(7)
#define QMI8658C_LOCKED BIT(1)
#define QMI8658C_AVAIL BIT(0)

#define QMI8658C_REG_STATUS0 0x2e
#define QMI8658C_SDA BIT(3)
#define QMI8658C_GDA BIT(1)
#define QMI8658C_ADA BIT(0)

#define QMI8658C_REG_STATUS1 0x2f
#define QMI8658C_SIGNIFICANT_MOTION BIT(7)
#define QMI8658C_NO_MOTION BIT(6)
#define QMI8658C_ANY_MOTION BIT(5)
#define QMI8658C_PEDOMETER BIT(4)
#define QMI8658C_TAP BIT(1)

#define QMI8658C_REG_DATA_TIMESTAMP_L 0x30
#define QMI8658C_REG_DATA_TIMESTAMP_M 0x31
#define QMI8658C_REG_DATA_TIMESTAMP_H 0x32

#define QMI8658C_REG_DATA_TEMP_L 0x33
#define QMI8658C_REG_DATA_TEMP_H 0x34

/* (1/256) degrees per LSB from datasheet */
#define QMI8658C_TEMP_MICRO_UNIT 3906

#define QMI8658C_REG_DATA_ACCEL_X_L 0x35
#define QMI8658C_REG_DATA_ACCEL_X_H 0x36
#define QMI8658C_REG_DATA_ACCEL_Y_L 0x37
#define QMI8658C_REG_DATA_ACCEL_Y_H 0x38
#define QMI8658C_REG_DATA_ACCEL_Z_L 0x39
#define QMI8658C_REG_DATA_ACCEL_Z_H 0x3a

#define QMI8658C_REG_DATA_GYRO_X_L 0x3b
#define QMI8658C_REG_DATA_GYRO_X_H 0x3c
#define QMI8658C_REG_DATA_GYRO_Y_L 0x3d
#define QMI8658C_REG_DATA_GYRO_Y_H 0x3e
#define QMI8658C_REG_DATA_GYRO_Z_L 0x3f
#define QMI8658C_REG_DATA_GYRO_Z_H 0x40

#define QMI8658C_REG_DATA_MAG_X_L 0x41
#define QMI8658C_REG_DATA_MAG_X_H 0x42
#define QMI8658C_REG_DATA_MAG_Y_L 0x43
#define QMI8658C_REG_DATA_MAG_Y_H 0x44
#define QMI8658C_REG_DATA_MAG_Z_L 0x45
#define QMI8658C_REG_DATA_MAG_Z_H 0x46

#define QMI8658C_REG_RESET 0x60
#define QMI8658C_CMD_SOFTRESET 0xff

#define QMI8658C_SOFTRESET_MSLEEP 150
#define QMI8658C_ACCEL_TURN_ON_MSLEEP 3
#define QMI8658C_GYRO_TURN_ON_MSLEEP 60

#define QMI8658C_ACCEL_CHANNEL_POS GENMASK(2, 0)
#define QMI8658C_GYRO_CHANNEL_POS GENMASK(5, 3)

#define QMI8658C_ACCEL_LAST_NORM_ODR 0x08
#define QMI8658C_ACCEL_FIRST_LOW_POW_ODR 0x0c

#define QMI8658C_CHANNEL(_type, _axis, _index)                                 \
	{                                                                      \
		.type = _type, .modified = 1, .channel2 = IIO_MOD_##_axis,     \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                  \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |         \
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),      \
		.scan_index = _index,                                          \
		.scan_type = {                                                 \
			.sign = 's',                                           \
			.realbits = 16,                                        \
			.storagebits = 16,                                     \
			.endianness = IIO_LE,                                  \
		},                                                             \
		.ext_info = qmi8658c_ext_info, \
	}

/* scan indexes follow DATA register order */
enum qmi8658c_scan_axis {
	QMI8658C_SCAN_ACCEL_X = 0,
	QMI8658C_SCAN_ACCEL_Y,
	QMI8658C_SCAN_ACCEL_Z,
	QMI8658C_SCAN_GYRO_X,
	QMI8658C_SCAN_GYRO_Y,
	QMI8658C_SCAN_GYRO_Z,
};

enum qmi8658c_sensor_type {
	QMI8658C_UNKNOWN_TYPE = -1,
	QMI8658C_ACCEL,
	QMI8658C_GYRO,
	QMI8658C_MAG,
};

const struct regmap_config qmi8658c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

struct qmi8658c_regs {
	u8 data; /* LSB byte register for X-axis */
	u8 config;
	u8 config_odr_mask;
	u8 config_scale_mask;
};

static struct qmi8658c_regs qmi8658c_regs[] = {
	[QMI8658C_ACCEL] = {
		.data	= QMI8658C_REG_DATA_ACCEL_X_L,
		.config	= QMI8658C_REG_CTRL_2,
		.config_odr_mask = QMI8658C_REG_CTRL_ODR,
		.config_scale_mask = QMI8658C_REG_CTRL_SCALE,
	},
	[QMI8658C_GYRO] = {
		.data	= QMI8658C_REG_DATA_GYRO_X_L,
		.config	= QMI8658C_REG_CTRL_3,
		.config_odr_mask = QMI8658C_REG_CTRL_ODR,
		.config_scale_mask = QMI8658C_REG_CTRL_SCALE,
	},
};

struct qmi8658c_scale {
	u8 bits;
	int uscale;
};

static const struct qmi8658c_scale qmi8658c_accel_scale[] = {
	{ QMI8658C_ACCEL_SCALE_2G, 61 },
	{ QMI8658C_ACCEL_SCALE_4G, 122 },
	{ QMI8658C_ACCEL_SCALE_8G, 244 },
	{ QMI8658C_ACCEL_SCALE_16G, 488 },
};

static const struct qmi8658c_scale qmi8658c_gyro_scale[] = {
	{ QMI8658C_GYRO_SCALE_16DPS, 488 },
	{ QMI8658C_GYRO_SCALE_32DPS, 976 },
	{ QMI8658C_GYRO_SCALE_64DPS, 1953 },
	{ QMI8658C_GYRO_SCALE_128DPS, 3906 },
	{ QMI8658C_GYRO_SCALE_256DPS, 7812 },
	{ QMI8658C_GYRO_SCALE_512DPS, 15625 },
	{ QMI8658C_GYRO_SCALE_1024DPS, 31250 },
	{ QMI8658C_GYRO_SCALE_2048DPS, 62500 },
};

struct qmi8658c_scale_item {
	const struct qmi8658c_scale *tbl;
	int num;
};

static const struct  qmi8658c_scale_item qmi8658c_scale_table[] = {
	[QMI8658C_ACCEL] = {
		.tbl	= qmi8658c_accel_scale,
		.num	= ARRAY_SIZE(qmi8658c_accel_scale),
	},
	[QMI8658C_GYRO] = {
		.tbl	= qmi8658c_gyro_scale,
		.num	= ARRAY_SIZE(qmi8658c_gyro_scale),
	},
};

struct qmi8658c_odr {
	u8 bits;
	int odr;
	int uodr;
};

static const struct qmi8658c_odr qmi8658c_accel_odr[] = {
	{ 0x00, 8000, 0 },
	{ 0x01, 4000, 0 },
	{ 0x02, 2000, 0 },
	{ 0x03, 1000, 0 },
	{ 0x04, 500, 0 },
	{ 0x05, 250, 0 },
	{ 0x06, 125, 0 },
	{ 0x07, 62, 500000 },
	{ 0x08, 31, 250000 },
	{ 0x0c, 128, 0 },
	{ 0x0d, 21, 0 },
	{ 0x0e, 11, 0 },
	{ 0x0f, 3, 0 },
};

static const struct qmi8658c_odr qmi8658c_gyro_odr[] = {
	{ 0x00, 8000, 0 },
	{ 0x01, 4000, 0 },
	{ 0x02, 2000, 0 },
	{ 0x03, 1000, 0 },
	{ 0x04, 500, 0 },
	{ 0x05, 250, 0 },
	{ 0x06, 125, 0 },
	{ 0x07, 62, 500000 },
	{ 0x08, 31, 250000 },
};

struct qmi8658c_odr_item {
	const struct qmi8658c_odr *tbl;
	int num;
};

static const struct  qmi8658c_odr_item qmi8658c_odr_table[] = {
	[QMI8658C_ACCEL] = {
		.tbl	= qmi8658c_accel_odr,
		.num	= ARRAY_SIZE(qmi8658c_accel_odr),
	},
	[QMI8658C_GYRO] = {
		.tbl	= qmi8658c_gyro_odr,
		.num	= ARRAY_SIZE(qmi8658c_gyro_odr),
	},
};

enum qmi8658c_operating_modes {
	POWER_ON_DEFAULT = 0,
	LOW_POWER,
	POWER_DOWN,
	NORMAL_ACCEL_ONLY,
	LOW_POWER_ACCEL_ONLY,
	SNOOZE_GYRO,
	GYRO_ONLY,
	MAG_ONLY,
	ACCEL_MAG,
	ACCEL_GYRO,
	ACCEL_GYRO_MAG,
	ACCEL_SNOOZE_GYRO,
	ACCEL_MAG_SNOOZE_GYRO,
	WAKE_ON_MOTION,
};

struct qmi8658c_data {
	struct i2c_client *client;
	struct regmap *regmap;
	struct iio_trigger *trig;

	struct iio_mount_matrix orientation;

	bool low_power;

	/*
	 * Ensure natural alignment for timestamp if present.
	 * Max length needed: 2 bytes * 3 channels * 2 sensors +
	 * + 4 bytes padding + 8 byte ts.
	 * If fewer channels are enabled, less space may be needed, as
	 * long as the timestamp is still aligned to 8 bytes.
	 */
	__le16 buf[12] __aligned(8);
};

static const struct iio_mount_matrix *
qmi8658c_get_mount_matrix(const struct iio_dev *indio_dev,
			  const struct iio_chan_spec *chan)
{
	struct qmi8658c_data *data = iio_priv(indio_dev);

	return &data->orientation;
}

static const struct iio_chan_spec_ext_info qmi8658c_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, qmi8658c_get_mount_matrix),
	{}
};

static const struct iio_chan_spec qmi8658c_channels[] = {
	QMI8658C_CHANNEL(IIO_ACCEL, X, QMI8658C_SCAN_ACCEL_X),
	QMI8658C_CHANNEL(IIO_ACCEL, Y, QMI8658C_SCAN_ACCEL_Y),
	QMI8658C_CHANNEL(IIO_ACCEL, Z, QMI8658C_SCAN_ACCEL_Z),
	QMI8658C_CHANNEL(IIO_ANGL_VEL, X, QMI8658C_SCAN_GYRO_X),
	QMI8658C_CHANNEL(IIO_ANGL_VEL, Y, QMI8658C_SCAN_GYRO_Y),
	QMI8658C_CHANNEL(IIO_ANGL_VEL, Z, QMI8658C_SCAN_GYRO_Z),
};

static enum qmi8658c_sensor_type
qmi8658c_get_sensor_type(enum iio_chan_type iio_type)
{
	switch (iio_type) {
	case IIO_ACCEL:
		return QMI8658C_ACCEL;
	case IIO_ANGL_VEL:
		return QMI8658C_GYRO;
	default:
		return QMI8658C_UNKNOWN_TYPE;
	}
}

static int qmi8658c_set_scale(struct qmi8658c_data *data,
			      enum qmi8658c_sensor_type t, int uscale)
{
	int i;

	if (!uscale)
		return -EINVAL;

	if (t == QMI8658C_UNKNOWN_TYPE)
		return -EINVAL;

	for (i = 0; i < qmi8658c_scale_table[t].num; i++)
		if (qmi8658c_scale_table[t].tbl[i].uscale == uscale)
			break;

	if (i == qmi8658c_scale_table[t].num)
		return -EINVAL;

	return regmap_update_bits(
		data->regmap, qmi8658c_regs[t].config,
		qmi8658c_regs[t].config_scale_mask,
		FIELD_PREP(QMI8658C_REG_CTRL_SCALE,
			   qmi8658c_scale_table[t].tbl[i].bits));
}

static int qmi8658c_get_scale(struct qmi8658c_data *data,
			      enum qmi8658c_sensor_type t, int *uscale)
{
	int i, ret, val;

	if (t == QMI8658C_UNKNOWN_TYPE)
		return -EINVAL;

	ret = regmap_read(data->regmap, qmi8658c_regs[t].config, &val);
	if (ret)
		return ret;

	val = FIELD_GET(QMI8658C_REG_CTRL_SCALE, val);

	for (i = 0; i < qmi8658c_scale_table[t].num; i++)
		if (qmi8658c_scale_table[t].tbl[i].bits == val) {
			*uscale = qmi8658c_scale_table[t].tbl[i].uscale;
			return 0;
		}

	return -EINVAL;
}

static int qmi8658c_set_odr(struct qmi8658c_data *data,
			    enum qmi8658c_sensor_type t, int odr, int uodr)
{
	int i;

	if (t == QMI8658C_UNKNOWN_TYPE)
		return -EINVAL;

	for (i = 0; i < qmi8658c_odr_table[t].num; i++)
		if (qmi8658c_odr_table[t].tbl[i].odr == odr &&
		    qmi8658c_odr_table[t].tbl[i].uodr == uodr)
			break;

	if (i == qmi8658c_odr_table[t].num)
		return -EINVAL;

	if (!data->low_power && (qmi8658c_odr_table[t].tbl[i].bits >
				 QMI8658C_ACCEL_LAST_NORM_ODR)) {
		dev_err(&data->client->dev,
			"ODR is not suit for non-low power mode\n");
		return -EINVAL;
	}

	if (data->low_power && (qmi8658c_odr_table[t].tbl[i].bits <
				QMI8658C_ACCEL_FIRST_LOW_POW_ODR)) {
		dev_err(&data->client->dev,
			"ODR is not suit for low power mode\n");
		return -EINVAL;
	}

	return regmap_update_bits(data->regmap, qmi8658c_regs[t].config,
				  qmi8658c_regs[t].config_odr_mask,
				  qmi8658c_odr_table[t].tbl[i].bits);
}

static int qmi8658c_get_odr(struct qmi8658c_data *data,
			    enum qmi8658c_sensor_type t, int *odr, int *uodr)
{
	int i, ret, val;

	if (t == QMI8658C_UNKNOWN_TYPE)
		return -EINVAL;

	ret = regmap_read(data->regmap, qmi8658c_regs[t].config, &val);
	if (ret)
		return ret;

	val &= qmi8658c_regs[t].config_odr_mask;

	for (i = 0; i < qmi8658c_odr_table[t].num; i++)
		if (val == qmi8658c_odr_table[t].tbl[i].bits)
			break;

	if (i == qmi8658c_odr_table[t].num)
		return -EINVAL;

	*odr = qmi8658c_odr_table[t].tbl[i].odr;
	*uodr = qmi8658c_odr_table[t].tbl[i].uodr;

	return 0;
}

static int qmi8658c_get_data(struct qmi8658c_data *data,
			     enum qmi8658c_sensor_type t, int axis, int *val)
{
	int reg;
	int ret;
	__le16 sample;

	if (t == QMI8658C_UNKNOWN_TYPE)
		return -EINVAL;

	reg = qmi8658c_regs[t].data + (axis - IIO_MOD_X) * sizeof(sample);

	ret = regmap_bulk_read(data->regmap, reg, &sample, sizeof(sample));
	if (ret)
		return ret;

	*val = sign_extend32(le16_to_cpu(sample), 15);

	return 0;
}

static bool qmi8658c_data_avail(int scan_mask, u8 status0)
{
	bool accel_req = (scan_mask & QMI8658C_ACCEL_CHANNEL_POS) > 0;
	bool gyro_req = (scan_mask & QMI8658C_GYRO_CHANNEL_POS) > 0;

	bool accel_avail = (status0 & QMI8658C_ADA) && accel_req;
	bool gyro_avail = ((status0 & QMI8658C_GDA) >> 1) && gyro_req;

	return accel_avail || gyro_avail;
}

static irqreturn_t qmi8658c_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct qmi8658c_data *data = iio_priv(indio_dev);
	int i, ret = 0, j = 0, base = QMI8658C_REG_DATA_ACCEL_X_L;
	u8 status0;
	__le16 sample;

	ret = regmap_bulk_read(data->regmap, QMI8658C_REG_STATUS0, &status0,
			       sizeof(status0));
	if (ret)
		goto done;

	if (!qmi8658c_data_avail(*indio_dev->active_scan_mask, status0))
		goto done;

	for_each_set_bit(i, indio_dev->active_scan_mask,
			  indio_dev->masklength) {
		ret = regmap_bulk_read(data->regmap,
				       (base + i * sizeof(sample)), &sample,
				       sizeof(sample));
		if (ret)
			goto done;
		data->buf[j++] = sample;
	}
	iio_push_to_buffers_with_timestamp(indio_dev, data->buf, pf->timestamp);
done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int qmi8658c_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int *val,
			     int *val2, long mask)
{
	int ret;
	struct qmi8658c_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_ANGL_VEL:
		case IIO_ACCEL:
			ret = qmi8658c_get_data(
				data, qmi8658c_get_sensor_type(chan->type),
				chan->channel2, val);
			return ret ? ret : IIO_VAL_INT;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
		case IIO_ACCEL:
			*val = 0;
			ret = qmi8658c_get_scale(
				data, qmi8658c_get_sensor_type(chan->type),
				val2);

			return ret ? ret : IIO_VAL_INT_PLUS_MICRO;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = qmi8658c_get_odr(
			data, qmi8658c_get_sensor_type(chan->type), val, val2);

		return ret ? ret : IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}

	return 0;
}

static int qmi8658c_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, int val,
			      int val2, long mask)
{
	struct qmi8658c_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		/* Scale is always below 1, so pass only it's micro part in val2 */
		return qmi8658c_set_scale(
			data, qmi8658c_get_sensor_type(chan->type), val2);
		break;

	case IIO_CHAN_INFO_SAMP_FREQ:
		return qmi8658c_set_odr(
			data, qmi8658c_get_sensor_type(chan->type), val, val2);

	default:
		return -EINVAL;
	}

	return 0;
}

/* Sampling frequency in Hz */
static IIO_CONST_ATTR(in_accel_sampling_frequency_available,
		      "3 11 21 128 31.25 62.5 125 250 500 1000 2000 4000 8000");
static IIO_CONST_ATTR(in_anglvel_sampling_frequency_available,
		      "31.25 62.5 125 250 500 1000 2000 4000 8000");

/* Scale is 1/Sensitivity, which is in LSB/g or LSB/dps (degrees per second),
 * according to datasheet.
 */
static IIO_CONST_ATTR(in_accel_scale_available,
		      "0.000488 0.000244 0.000122 0.000061");
static IIO_CONST_ATTR(
	in_anglvel_scale_available,
	"0.0625 0.03125 0.015625 0.007812 0.003906 0.001953 0.000976 0.000488");

static struct attribute *qmi8658c_attrs[] = {
	&iio_const_attr_in_accel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_anglvel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_accel_scale_available.dev_attr.attr,
	&iio_const_attr_in_anglvel_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group qmi8658c_attrs_group = {
	.attrs = qmi8658c_attrs,
};

static const struct iio_info qmi8658c_info = {
	.read_raw = qmi8658c_read_raw,
	.write_raw = qmi8658c_write_raw,
	.attrs = &qmi8658c_attrs_group,
};

/* Enables or disables internal 2 MHz oscillator for sensors */
int qmi8658c_clocks_ctrl(struct qmi8658c_data *data, bool enable)
{
	int ret;

	ret = regmap_update_bits(data->regmap, QMI8658C_REG_CTRL_1,
				 QMI8658C_SENS_DISABLE, !enable);
	if (ret)
		return ret;

	/* Turn On Time from Power Down mode */
	if (enable)
		msleep(QMI8658C_SOFTRESET_MSLEEP);

	return ret;
}

int qmi8658c_get_power_state(struct qmi8658c_data *data, bool *power_is_down)
{
	int ret, val;

	*power_is_down = false;
	ret = regmap_read(data->regmap, QMI8658C_REG_CTRL_1, &val);
	if (ret)
		return ret;
	*power_is_down = (val & QMI8658C_SENS_DISABLE) > 0;
	return 0;
}

static int qmi8658c_set_mode(struct qmi8658c_data *data,
			     enum qmi8658c_operating_modes new_mode)
{
	int ret, odr, uodr;
	bool power_is_down;

	ret = qmi8658c_get_power_state(data, &power_is_down);
	if (ret)
		return ret;

	if (power_is_down && (new_mode != POWER_DOWN)) {
		ret = qmi8658c_clocks_ctrl(data, true);
		if (ret)
			return ret;
	}

	data->low_power = false;
	/* TODO: support other modes */
	switch (new_mode) {
	case POWER_ON_DEFAULT:
		break;

	case LOW_POWER:
		data->low_power = true;
		break;

	case POWER_DOWN:
		ret = qmi8658c_clocks_ctrl(data, false);
		if (ret)
			return ret;

		ret = regmap_update_bits(data->regmap, QMI8658C_REG_CTRL_7,
					 QMI8658C_ACCEL_EN | QMI8658C_GYRO_EN,
					 (unsigned int)((~QMI8658C_ACCEL_EN) &
							(~QMI8658C_GYRO_EN)));
		if (ret)
			return ret;

		break;

	case NORMAL_ACCEL_ONLY:
		break;

	case LOW_POWER_ACCEL_ONLY:
		break;

	case SNOOZE_GYRO:
		break;

	case GYRO_ONLY:
		break;

	case MAG_ONLY:
		break;

	case ACCEL_MAG:
		break;

	case ACCEL_GYRO:
		ret = regmap_update_bits(
			data->regmap, QMI8658C_REG_CTRL_7,
			QMI8658C_GYRO_MODE | QMI8658C_ACCEL_EN |
				QMI8658C_GYRO_EN | QMI8658C_MAG_EN,
			(unsigned int)(((~QMI8658C_GYRO_MODE) &
					(~QMI8658C_MAG_EN)) |
				       QMI8658C_ACCEL_EN | QMI8658C_GYRO_EN));
		if (ret)
			return ret;

		ret = qmi8658c_get_odr(data,
				       qmi8658c_get_sensor_type(IIO_ANGL_VEL),
				       &odr, &uodr);
		if (ret)
			return ret;

		msleep(QMI8658C_GYRO_TURN_ON_MSLEEP + (3000 / odr));

		break;

	case ACCEL_GYRO_MAG:
		break;

	case ACCEL_SNOOZE_GYRO:
		break;

	case ACCEL_MAG_SNOOZE_GYRO:
		break;

	case WAKE_ON_MOTION:
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

int qmi8658c_sync_sample_ctrl(struct iio_dev *indio_dev, bool enable)
{
	struct qmi8658c_data *data = iio_priv(indio_dev);
	int ret = 0;
	unsigned int val;

	ret = regmap_read(data->regmap, QMI8658C_REG_CTRL_7, &val);
	if (ret)
		return ret;

	val &= ~QMI8658C_SYNC_SMPL;
	if (enable)
		val |= QMI8658C_SYNC_SMPL;

	ret = regmap_write(data->regmap, QMI8658C_REG_CTRL_7, val);
	if (ret)
		return ret;

	return ret;
}

static int qmi8658c_data_rdy_trigger_set_state(struct iio_trigger *trig,
					       bool enable)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);

	return qmi8658c_sync_sample_ctrl(indio_dev, enable);
}

static const struct iio_trigger_ops qmi8658c_trigger_ops = {
	.set_trigger_state = &qmi8658c_data_rdy_trigger_set_state,
};

int qmi8658c_probe_trigger(struct iio_dev *indio_dev, int irq, u32 irq_type)
{
	struct qmi8658c_data *data = iio_priv(indio_dev);
	int ret;

	data->trig = devm_iio_trigger_alloc(&indio_dev->dev, "%s-dev%d",
					    indio_dev->name, indio_dev->id);

	if (data->trig == NULL)
		return -ENOMEM;

	ret = devm_request_irq(&indio_dev->dev, irq,
			       &iio_trigger_generic_data_rdy_poll, irq_type,
			       "qmi8658c", data->trig);
	if (ret)
		return ret;

	data->trig->dev.parent = regmap_get_device(data->regmap);
	data->trig->ops = &qmi8658c_trigger_ops;

	iio_trigger_set_drvdata(data->trig, indio_dev);

	ret = devm_iio_trigger_register(&indio_dev->dev, data->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(data->trig);

	return 0;
}

static int qmi8658c_setup_irq(struct iio_dev *indio_dev, int irq)
{
	struct irq_data *desc;
	u32 irq_type;

	desc = irq_get_irq_data(irq);
	if (!desc) {
		dev_err(&indio_dev->dev, "Could not find IRQ %d\n", irq);
		return -EINVAL;
	}
	irq_type = irqd_get_trigger_type(desc);

	return qmi8658c_probe_trigger(indio_dev, irq, irq_type);
}

static int qmi8658c_chip_init(struct qmi8658c_data *data)
{
	int ret;
	unsigned int val;
	struct device *dev = regmap_get_device(data->regmap);

	ret = regmap_write(data->regmap, QMI8658C_REG_RESET,
			   QMI8658C_CMD_SOFTRESET);
	if (ret)
		return ret;

	msleep(QMI8658C_SOFTRESET_MSLEEP);

	ret = regmap_read(data->regmap, QMI8658C_REG_WHO_AM_I, &val);
	if (ret) {
		dev_err(dev, "Error reading Who Am I\n");
		return ret;
	}
	if (val != QMI8658C_WHO_AM_I_VAL) {
		dev_err(dev, "Wrong Who Am I, got %x expected %x\n", val,
			QMI8658C_WHO_AM_I_VAL);
		return -ENODEV;
	}

	ret = qmi8658c_set_mode(data, ACCEL_GYRO);
	if (ret)
		goto power_down;

	ret = regmap_update_bits(data->regmap, QMI8658C_REG_CTRL_1,
				 QMI8658C_ADDR_AUTO_INC,
				 (unsigned int)QMI8658C_ADDR_AUTO_INC);
	if (ret)
		goto power_down;

	return 0;

power_down:
	qmi8658c_set_mode(data, POWER_DOWN);
	return ret;
}

static void qmi8658c_chip_uninit(void *data)
{
	struct qmi8658c_data *qmi_data = data;
	struct device *dev = regmap_get_device(qmi_data->regmap);
	int ret;

	regmap_update_bits(qmi_data->regmap, QMI8658C_REG_CTRL_1,
			   QMI8658C_ADDR_AUTO_INC,
			   (unsigned int)(~QMI8658C_ADDR_AUTO_INC));

	ret = qmi8658c_set_mode(qmi_data, POWER_DOWN);
	if (ret)
		dev_err(dev, "Failed to set Power Down mode: %d\n", ret);
}

static int qmi8658c_i2c_probe(struct i2c_client *client)
{
	struct regmap *regmap;
	struct iio_dev *indio_dev;
	struct qmi8658c_data *data;
	int irq;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_i2c(client, &qmi8658c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Failed to register i2c regmap: %pe\n",
			regmap);
		return PTR_ERR(regmap);
	}

	data = iio_priv(indio_dev);

	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	data->regmap = regmap;

	ret = iio_read_mount_matrix(&client->dev, "mount-matrix",
				    &data->orientation);
	if (ret)
		return ret;

	ret = qmi8658c_chip_init(data);
	if (ret)
		return ret;

	indio_dev->channels = qmi8658c_channels;
	indio_dev->num_channels = ARRAY_SIZE(qmi8658c_channels);
	indio_dev->name = "qmi8658c";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &qmi8658c_info;

	ret = devm_add_action_or_reset(&client->dev, qmi8658c_chip_uninit,
				       data);
	if (ret)
		return ret;

	ret = devm_iio_triggered_buffer_setup(&client->dev, indio_dev,
					      iio_pollfunc_store_time,
					      qmi8658c_trigger_handler, NULL);
	if (ret)
		return ret;

	/* Use INT2 to monitor data enable */
	irq = of_irq_get_byname(client->dev.of_node, "INT2");
	if (irq > 0) {
		ret = qmi8658c_setup_irq(indio_dev, irq);
		if (ret)
			dev_err(&client->dev, "Failed to setup IRQ %d\n", irq);
	} else {
		dev_info(&client->dev, "Not setting up IRQ trigger\n");
	}

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret < 0) {
		dev_err(&client->dev,
			"Failed to register QMI8658c IIO device\n");
		return ret;
	}

	return 0;
}

static const struct i2c_device_id qmi8658c_i2c_id[] = {
	{ "qmi8658c" },
	{}
};
MODULE_DEVICE_TABLE(i2c, qmi8658c_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id qmi8658c_of_match[] = {
	{ .compatible = "qst,qmi8658c" },
	{},
};
MODULE_DEVICE_TABLE(of, qmi8658c_of_match);
#endif

static struct i2c_driver qmi8658c_i2c_driver = {
	.driver = {
		.name = "qmi8658c_i2c",
		.of_match_table = of_match_ptr(qmi8658c_of_match),
	},
	.probe_new = qmi8658c_i2c_probe,
	.id_table = qmi8658c_i2c_id,
};

module_i2c_driver(qmi8658c_i2c_driver);

MODULE_DESCRIPTION("QMI8658C I2C driver");
MODULE_LICENSE("GPL v2");
