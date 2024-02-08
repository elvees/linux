// SPDX-License-Identifier: GPL-2.0
//
// Support for Vishay VCNL36821S proximity sensor on i2c bus.
// Based on Vishay VCNL4000 driver code.


#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/regmap.h>

#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#define VCNL36821S_DEV_ID	0x26

#define VCNL36821S_PS_CONF1		0x00
#define VCNL36821S_PS_CONF2		0x03
#define VCNL36821S_PS_CONF3		0x04
#define VCNL36821S_PS_THDL		0x05
#define VCNL36821S_PS_THDH		0x06
#define VCNL36821S_PS_CANC		0x07
#define VCNL36821S_PS_AC		0x08
#define VCNL36821S_PS_DATA		0xF8
#define VCNL36821S_INT_FLAG		0xF9
#define VCNL36821S_ID			0xFA
#define VCNL36821S_PS_AC_DATA		0xFB

/*  PS_CONF1_LOW */
#define VCNL36821S_PS_ON	BIT(1)
#define VCNL36821S_PS_INIT	BIT(7)

#define VCNL36821S_CONF1_RESERVED	BIT(9)

/*  PS_CONF2_LOW */
#define VCNL36821S_PS_ST	BIT(0)
#define VCNL36821S_PS_INT	GENMASK(3, 2)
#define VCNL36821S_PS_PERS	GENMASK(5, 4)
#define VCNL36821S_PS_PERIOD	GENMASK(7, 6)

#define VCNL36821S_PS_INT_ENABLE	0xC
#define VCNL36821S_PS_INT_DISABLE	0

/*  PS_CONF2_HIGH */
#define VCNL36821S_PS_HG	BIT(10)
#define VCNL36821S_PS_ITB	BIT(11)
#define VCNL36821S_PS_MPS	GENMASK(13, 12)
#define VCNL36821S_PS_IT	GENMASK(15, 14)

#define VCNL36821S_PS_ITB_COEFF	25  /* ITB = 25us or ITB = 50us */

#define VCNL36821S_PS_ITB_DEFAULT	0x1  /* 50 us */
#define VCNL36821S_PS_IT_DEFAULT	0x3  /* 8T */

/* PS_CONF3_LOW */
#define VCNL36821S_PS_SP_INT	BIT(2)
#define VCNL36821S_PS_FORCENUM	BIT(4)
#define VCNL36821S_PS_FOR_TRIG	BIT(5)
#define VCNL36821S_PS_AF	BIT(6)

/* PS_CONF4_HIGH */
#define VCNL36821S_LED_I_MASK	GENMASK(11, 8)
#define VCNL36821S_PS_SC_MASK	GENMASK(15, 13)

/* LED current selection setting */
#define VCNL36821S_50mA		0x8
#define VCNL36821S_66mA		0x9
#define VCNL36821S_82mA		0xA
#define VCNL36821S_98mA		0xB
#define VCNL36821S_114mA	0xC
#define VCNL36821S_130mA	0xD
#define VCNL36821S_144mA	0xE
#define VCNL36821S_156mA	0xF

/* INT_Flag */
#define VCNL36821S_PS_IF_AWAY	BIT(8)
#define VCNL36821S_PS_IF_CLOSE	BIT(9)
#define VCNL36821S_PS_SPFLAG	BIT(12)
#define VCNL36821S_PS_ACFLAG	BIT(13)

#define VCNL36821S_PS_THDL_DEFAULT	0
#define VCNL36821S_PS_THDH_DEFAULT	0xFFF

/**
 * struct vcnl36821s_data - vcnl36821s specific data.
 * @regmap:		device register map.
 * @dev:		vcnl36821s device.
 * @rev:		revision id.
 * @itb_val:		proximity integration time bank.
 * @it_val:		proximity integration time.
 * @persistence:	number of consecutive measurements above / below threshold.
 * @thresh_low:		low threshold value.
 * @thresh_high:	high threshold value.
 * @drdy_trigger0:	data ready trigger.
 */
struct vcnl36821s_data {
	struct i2c_client *client;
	struct regmap *regmap;
	struct device *dev;
	u8 rev;
	unsigned int itb_val;
	unsigned int it_val;
	unsigned int persistence;
	unsigned int thresh_low;
	unsigned int thresh_high;
	struct iio_trigger *drdy_trigger0;
};

static inline bool vcnl36821s_is_triggered(struct vcnl36821s_data *data)
{
	int ret;
	int reg, raw_data;

	ret = regmap_read(data->regmap, VCNL36821S_INT_FLAG, &reg);
	if (ret < 0)
		return false;
	ret = regmap_read(data->regmap, VCNL36821S_PS_DATA, &raw_data);

	return !!(reg &
		(VCNL36821S_PS_IF_AWAY | VCNL36821S_PS_IF_CLOSE));
}

static irqreturn_t vcnl36821s_drdy_irq_thread(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct vcnl36821s_data *data = iio_priv(indio_dev);

	if (vcnl36821s_is_triggered(data)) {
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY,
						    0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_EITHER),
				iio_get_time_ns(indio_dev));
		iio_trigger_poll_chained(data->drdy_trigger0);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

/* Triggered buffer */
static irqreturn_t vcnl36821s_trigger_consumer_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct vcnl36821s_data *data = iio_priv(indio_dev);
	/* 1x16-bit + naturally aligned ts */
	u16 buffer[8] __aligned(8) = {0};
	int ret;

	ret = regmap_read(data->regmap, VCNL36821S_PS_DATA, (int *)buffer);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"Failed to read trigger consumer.\n");
		goto fail_read;
	}
	iio_push_to_buffers_with_timestamp(indio_dev, buffer,
					   iio_get_time_ns(indio_dev));

fail_read:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int vcnl36821s_ps_drdy_set_state(struct iio_trigger *trigger,
					bool enable_drdy)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trigger);
	struct vcnl36821s_data *data = iio_priv(indio_dev);
	int val = enable_drdy ? VCNL36821S_PS_INT_ENABLE :
				VCNL36821S_PS_INT_DISABLE;

	return regmap_update_bits(data->regmap, VCNL36821S_PS_CONF2,
				  VCNL36821S_PS_INT, val);
}

static const struct iio_trigger_ops vcnl36821s_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
	.set_trigger_state = vcnl36821s_ps_drdy_set_state,
};

static int vcnl36821s_set_power_state(struct vcnl36821s_data *data,
				      bool state)
{
	u8 state_bit = FIELD_PREP(VCNL36821S_PS_ON, state);

	return regmap_update_bits(data->regmap,
				  VCNL36821S_PS_CONF1,
				  VCNL36821S_PS_ON,
				  state_bit);
}

static int vcnl36821s_reset(struct vcnl36821s_data *data)
{
	regmap_write(data->regmap, VCNL36821S_PS_CONF1, 0x1);
	regmap_write(data->regmap, VCNL36821S_PS_CONF2, 0x1);
	regmap_write(data->regmap, VCNL36821S_PS_CONF3, 0);
	regmap_write(data->regmap, VCNL36821S_PS_THDL, 0);
	regmap_write(data->regmap, VCNL36821S_PS_THDH, 0);
	regmap_write(data->regmap, VCNL36821S_PS_CANC, 0);
	regmap_write(data->regmap, VCNL36821S_PS_AC, 0);
	return 0;
}

static int vcnl36821s_set_sc_current(struct vcnl36821s_data *data)
{
	int ret;
	u32 led_i, val;
	bool ps_sc;

	ps_sc = device_property_read_bool(data->dev,
					  "vishay,sunlight-cancellation");
	if (ps_sc) {
		ret = regmap_set_bits(data->regmap,
				      VCNL36821S_PS_CONF3,
				      VCNL36821S_PS_SC_MASK);
		if (ret) {
			dev_err(data->dev,
				"Failed to set sunlight cancellation: %d\n",
				ret);
			return ret;
		}
	}

	ret = device_property_read_u32(data->dev,
				       "vishay,led-current",
				       &led_i);
	if (ret)
		return 0;

	switch (led_i) {
	case 50:
		val = VCNL36821S_50mA;
		break;
	case 66:
		val = VCNL36821S_66mA;
		break;
	case 82:
		val = VCNL36821S_82mA;
		break;
	case 98:
		val = VCNL36821S_98mA;
		break;
	case 114:
		val = VCNL36821S_114mA;
		break;
	case 130:
		val = VCNL36821S_130mA;
		break;
	case 144:
		val = VCNL36821S_144mA;
		break;
	case 156:
		val = VCNL36821S_156mA;
		break;
	default:
		return -EINVAL;
	}

	led_i = FIELD_PREP(VCNL36821S_LED_I_MASK, val);
	ret = regmap_update_bits(data->regmap, VCNL36821S_PS_CONF3,
				 VCNL36821S_LED_I_MASK, led_i);
	if (ret)
		dev_err(data->dev,
			"Failed to set led current: %d\n", ret);

	return ret;
}

static int vcnl36821s_ps_start(struct vcnl36821s_data *data)
{
	return regmap_clear_bits(data->regmap, VCNL36821S_PS_CONF2,
				 VCNL36821S_PS_ST);
}

static int vcnl36821s_init(struct vcnl36821s_data *data)
{
	int itb, it, ret;
	u32 reg;

	ret = regmap_read(data->regmap, VCNL36821S_ID, &reg);
	if (ret) {
		dev_err(data->dev,
			"Failed to read product revision: %d\n", ret);
		return ret;
	}

	if (reg != VCNL36821S_DEV_ID) {
		dev_err(data->dev,
			"Product id (%x) did not match vcnl36821s (%x)\n",
			reg,
			VCNL36821S_DEV_ID);
		return -ENODEV;
	}

	data->rev = reg;

	vcnl36821s_reset(data);

	ret = vcnl36821s_set_power_state(data, 1);
	if (ret < 0)
		return ret;

	ret = vcnl36821s_set_sc_current(data);
	if (ret) {
		dev_err(data->dev,
			"Failed to set sunlight cancellation/current: %d\n",
			ret);
		return ret;
	}

	ret = regmap_set_bits(data->regmap,
			      VCNL36821S_PS_CONF1,
			      VCNL36821S_CONF1_RESERVED |
			      VCNL36821S_PS_ON |
			      VCNL36821S_PS_INIT);

	/* set default integration time bank - 50 us */
	itb = FIELD_PREP(VCNL36821S_PS_ITB,
			 VCNL36821S_PS_ITB_DEFAULT);
	ret = regmap_update_bits(data->regmap, VCNL36821S_PS_CONF2,
				 VCNL36821S_PS_ITB, itb);
	if (ret) {
		dev_err(data->dev,
			"Failed to set default ITB, %d\n",
			ret);
		return ret;
	}
	data->itb_val = VCNL36821S_PS_ITB_DEFAULT;

	/* set default integration time - 8T */
	it = FIELD_PREP(VCNL36821S_PS_IT,
			VCNL36821S_PS_IT_DEFAULT);
	ret = regmap_update_bits(data->regmap, VCNL36821S_PS_CONF2,
				 VCNL36821S_PS_IT, it);
	if (ret) {
		dev_err(data->dev,
			"Failed to set default IT, %d\n",
			ret);
		return ret;
	}
	data->it_val = VCNL36821S_PS_IT_DEFAULT;

	/* set default HIGH threshold */
	ret = regmap_write(data->regmap, VCNL36821S_PS_THDH,
			   VCNL36821S_PS_THDH_DEFAULT);
	if (ret) {
		dev_err(data->dev,
			"Failed to set default THDH, %d\n",
			ret);
		return ret;
	}
	data->thresh_high = VCNL36821S_PS_THDH_DEFAULT;

	/* set default LOW threshold */
	ret = regmap_write(data->regmap, VCNL36821S_PS_THDL,
			   VCNL36821S_PS_THDL_DEFAULT);
	if (ret) {
		dev_err(data->dev,
			"Failed to set default THDL, %d\n",
			ret);
		return ret;
	}
	data->thresh_low = VCNL36821S_PS_THDL_DEFAULT;

	ret = vcnl36821s_ps_start(data);
	if (ret) {
		dev_err(data->dev,
			"Failed to start sensor: %d\n",
			ret);
		return ret;
	}

	ret = regmap_read(data->regmap, VCNL36821S_PS_DATA,
			  &reg);

	regmap_update_bits(data->regmap, VCNL36821S_PS_CONF2,
			   VCNL36821S_PS_INT,
			   VCNL36821S_PS_INT_ENABLE);

	return ret;
};

static int vcnl36821s_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask)
{
	struct vcnl36821s_data *data = iio_priv(indio_dev);
	int ret;
	u32 raw_data;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (!ret) {
			ret = regmap_read(data->regmap,
					  VCNL36821S_PS_DATA,
					  &raw_data);

			iio_device_release_direct_mode(indio_dev);
			if (!ret) {
				*val = raw_data;
				ret = IIO_VAL_INT;
			}
		}
		return ret;
	case IIO_CHAN_INFO_INT_TIME:
		*val = (data->itb_val + 1) * VCNL36821S_PS_ITB_COEFF;
		if (data->it_val)
			*val *= BIT(data->it_val);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int vcnl36821s_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	int itb, it, it_val, ret;
	struct vcnl36821s_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_INT_TIME:
		if (val <= 0 || val > 400)
			return -EINVAL;

		if (val == VCNL36821S_PS_ITB_COEFF) {
			ret = regmap_clear_bits(data->regmap,
						 VCNL36821S_PS_CONF2,
						 VCNL36821S_PS_ITB |
						 VCNL36821S_PS_IT);
			if (!ret) {
				data->itb_val = 0;
				data->it_val = 0;
			}
		} else {
			itb = FIELD_PREP(VCNL36821S_PS_ITB, 1);
			ret = regmap_update_bits(data->regmap,
						 VCNL36821S_PS_CONF2,
						 VCNL36821S_PS_ITB,
						 itb);
			if (ret)
				return ret;
			data->itb_val = 1;

			it_val = ffs(val / (VCNL36821S_PS_ITB_COEFF * 2)) - 1;
			it = FIELD_PREP(VCNL36821S_PS_IT,
					it_val);
			ret = regmap_update_bits(data->regmap,
						 VCNL36821S_PS_CONF2,
						 VCNL36821S_PS_IT,
						 it);
			if (!ret)
				data->it_val = it_val;
		}

		return ret;
	default:
		return -EINVAL;
	}
}

static int vcnl36821s_read_thresh(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  enum iio_event_type type,
				  enum iio_event_direction dir,
				  enum iio_event_info info,
				  int *val, int *val2)
{
	struct vcnl36821s_data *data = iio_priv(indio_dev);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		switch (dir) {
		case IIO_EV_DIR_RISING:
			*val = data->thresh_high;
			return IIO_VAL_INT;
		case IIO_EV_DIR_FALLING:
			*val = data->thresh_low;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
		break;
	case IIO_EV_INFO_PERIOD:
		*val = data->persistence;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}

}

static int vcnl36821s_write_thresh(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info,
				   int val, int val2)
{
	struct vcnl36821s_data *data = iio_priv(indio_dev);
	int pers, ret;

	switch (info) {
	case IIO_EV_INFO_VALUE:
		/* 16 bit threshold range 0 - 4095 */
		if (val < 0 || val > 4095)
			return -EINVAL;

		if (dir == IIO_EV_DIR_RISING) {
			if (val < data->thresh_low)
				return -EINVAL;
			ret = regmap_write(data->regmap,
					   VCNL36821S_PS_THDH,
					   val);
			if (ret)
				return ret;
			data->thresh_high = val;
		} else {
			if (val > data->thresh_high)
				return -EINVAL;
			ret = regmap_write(data->regmap,
					   VCNL36821S_PS_THDL,
					   val);
			if (ret)
				return ret;
			data->thresh_low = val;
		}
		return ret;
	case IIO_EV_INFO_PERIOD:
		/* allow only 1-4 as persistence value */
		if ((val + 1) < 1 || (val + 1) > 4)
			return -EINVAL;

		pers = FIELD_PREP(VCNL36821S_PS_PERS, val);
		ret = regmap_update_bits(data->regmap,
					 VCNL36821S_PS_CONF2,
					 VCNL36821S_PS_PERS,
					 pers);
		if (!ret)
			data->persistence = val;
		return ret;
	default:
		return -EINVAL;
	}
}

static IIO_CONST_ATTR_INT_TIME_AVAIL("50 100 200 400");

static struct attribute *vcnl36821s_attributes[] = {
	&iio_const_attr_integration_time_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group vcnl36821s_attribute_group = {
	.attrs = vcnl36821s_attributes,
};

static const struct iio_info vcnl36821s_info = {
	.read_raw		= vcnl36821s_read_raw,
	.write_raw		= vcnl36821s_write_raw,
	.read_event_value	= vcnl36821s_read_thresh,
	.write_event_value	= vcnl36821s_write_thresh,
	.attrs			= &vcnl36821s_attribute_group,
};

static const struct iio_event_spec vcnl36821s_event_spec[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_PERIOD),
	},
};

static const struct iio_buffer_setup_ops iio_triggered_buffer_setup_ops = {
	.validate_scan_mask = &iio_validate_scan_mask_onehot,
};

static const struct iio_chan_spec vcnl36821s_channels[] = {
	{
		.type = IIO_PROXIMITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_INT_TIME),
		.event_spec = vcnl36821s_event_spec,
		.num_event_specs = ARRAY_SIZE(vcnl36821s_event_spec),
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
};

static const struct regmap_config vcnl36821s_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 16,
	.max_register	= VCNL36821S_PS_AC_DATA,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int vcnl36821s_probe_trigger(struct iio_dev *indio_dev)
{
	int ret;
	struct vcnl36821s_data *data = iio_priv(indio_dev);

	data->drdy_trigger0 = devm_iio_trigger_alloc(
			indio_dev->dev.parent,
			"%s-dev%d", indio_dev->name, indio_dev->id);
	if (!data->drdy_trigger0)
		return -ENOMEM;

	data->drdy_trigger0->dev.parent = indio_dev->dev.parent;
	data->drdy_trigger0->ops = &vcnl36821s_trigger_ops;
	iio_trigger_set_drvdata(data->drdy_trigger0, indio_dev);
	ret = devm_iio_trigger_register(indio_dev->dev.parent,
					data->drdy_trigger0);
	if (ret) {
		dev_err(&data->client->dev, "Failed to register iio trigger\n");
		return ret;
	}

	/* Trigger setup */
	ret = devm_iio_triggered_buffer_setup(indio_dev->dev.parent, indio_dev,
					      NULL, vcnl36821s_trigger_consumer_handler,
					      &iio_triggered_buffer_setup_ops);
	if (ret < 0) {
		dev_err(&data->client->dev, "Failed to set up iio-triggered buffer\n");
		return ret;
	}

	/* IRQ to trigger mapping */
	ret = devm_request_threaded_irq(&data->client->dev, data->client->irq,
					NULL, vcnl36821s_drdy_irq_thread,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"vcnl36821s_event", indio_dev);
	if (ret < 0)
		dev_err(&data->client->dev, "Failed to request irq %d for trigger0\n",
				data->client->irq);
	return ret;
}

static int vcnl36821s_probe(struct i2c_client *client)
{
	struct vcnl36821s_data *data;
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	int ret;

	regmap = devm_regmap_init_i2c(client,
				      &vcnl36821s_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Failed to initialize regmap\n");
		return PTR_ERR(regmap);
	}

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	data->regmap = regmap;
	data->dev = &client->dev;

	ret = vcnl36821s_init(data);
	if (ret)
		return ret;

	indio_dev->info = &vcnl36821s_info;
	indio_dev->channels = vcnl36821s_channels;
	indio_dev->num_channels = ARRAY_SIZE(vcnl36821s_channels);
	indio_dev->name = "vcnl36821s";
	indio_dev->modes = INDIO_DIRECT_MODE;

	if (client->irq > 0) {
		ret = vcnl36821s_probe_trigger(indio_dev);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to initialize vcnl36821s trigger\n");
			return ret;
		}
	}

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct of_device_id vcnl36821s_of_match[] = {
	{
		.compatible = "vishay,vcnl36821s",
	},
	{}
};
MODULE_DEVICE_TABLE(of, vcnl36821s_of_match);

static struct i2c_driver vcnl36821s_driver = {
	.driver = {
		.name   = "vcnl36821s",
		.of_match_table = vcnl36821s_of_match,
	},
	.probe_new  = vcnl36821s_probe,
};
module_i2c_driver(vcnl36821s_driver);

MODULE_DESCRIPTION("Vishay VCNL36821S proximity sensor driver");
MODULE_LICENSE("GPL");
