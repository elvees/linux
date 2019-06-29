/*
 * Copyright 2018 RnD Center "ELVEES", JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

/*
 * Driver support only parallel video to MIPI CSI2 direction.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>

#include <media/soc_camera.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-image-sizes.h>

#define TC358748_MAX_WIDTH 1920
#define TC358748_MAX_HEIGHT 1080
#define TC358748_DEFAULT_WIDTH 1280
#define TC358748_DEFAULT_HEIGHT 720

#define REG_CHIPID		0x0
#define REG_SYSCTL		0x2
#define REG_CONFCTL		0x4
#define REG_FIFOCTL		0x6
#define REG_DATAFMT		0x8
#define REG_PLLCTL0		0x16
#define REG_PLLCTL1		0x18
#define REG_CLKCTL		0x20
#define REG_WORDCNT		0x22
#define REG_PPI_STARTCNTRL	0x204
#define REG_HSTXVREGEN		0x234
#define REG_CSI_CONFW		0x500
#define REG_CSI_START		0x518

#define SYSCTL_RESET		BIT(0)
#define SYSCTL_SLEEP		BIT(1)

#define CONFCTL_DATALINE(x)	((x) & 0x3)
#define CONFCTL_AUTO		BIT(2)
#define CONFCTL_PCLKP		BIT(3)
#define CONFCTL_HVALIDP		BIT(4)
#define CONFCTL_VVALIDP		BIT(5)
#define CONFCTL_PPEN		BIT(6)
#define CONFCTL_PDATAF(x)	(((x) & 0x3) << 8)
#define CONFCTL_BT656EN		BIT(12)
#define CONFCTL_INTE2N		BIT(13)
#define CONFCTL_TRIEN		BIT(15)

#define DATAFMT_UDTEN		BIT(0)
#define DATAFMT_PDFORMAT(x)	(((x) & 0xF) << 4)
#define PDFORMAT_RAW8		0
#define PDFORMAT_RAW10		0x1
#define PDFORMAT_RAW12		0x2
#define PDFORMAT_RGB888		0x3
#define PDFORMAT_RGB666		0x4
#define PDFORMAT_RGB565		0x5
#define PDFORMAT_YUV422_8bit	0x6
#define PDFORMAT_RAW14		0x8
#define PDFORMAT_YUV422_10bit	0x9
#define PDFORMAT_YUV444		0xa

#define PLLCTL0_FBD(x)		((x) & 0xFF)
#define PLLCTL0_PRD(x)		(((x) & 0xF) << 12)

#define PLLCTL1_PLLEN		BIT(0)
#define PLLCTL1_RESETB		BIT(1)
#define PLLCTL1_CKEN		BIT(4)
#define PLLCTL1_BYPCKEN		BIT(5)
#define PLLCTL1_LFBREN		BIT(6)
#define PLLCTL1_FRS(x)		(((x) & 0x3) << 10)
#define PLLCTL1_LBWS(x)		(((x) & 0x3) << 8)

#define CLKCTL_SCLK(x)		((x) & 0x3)
#define CLKCTL_MCLK(x)		(((x) & 0x3) << 2)
#define CLKCTL_PPICLK(x)	(((x) & 0x3) << 4)

#define CSI_CONFW_DATA(x)	((x) & 0xFFFF)
#define CSI_CONFW_ADDR(x)	(((x) & 0x1F) << 24)
#define CSI_CONFW_MODE(x)	(((x) & 0x7) << 29)


struct tc358748_color_format {
	u32 code;
	enum v4l2_colorspace colorspace;
};

struct tc358748_priv {
	struct v4l2_subdev subdev;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *exp;
	struct v4l2_ctrl *exp_abs;
	struct v4l2_rect crop_rect;
	const struct tc358748_color_format *cfmt;
	u32 pll_fbd;
	u32 pll_prd;
	u32 pll_frs;
	bool vvalid_invert;
	bool hvalid_invert;
	bool enabled;
};

/*
 * supported color format list
 */
static const struct tc358748_color_format tc358748_cfmts[] = {
	{
		.code		= MEDIA_BUS_FMT_RGB888_1X24,
		.colorspace	= V4L2_COLORSPACE_SRGB,
	},
};

static int reg_read(struct i2c_client *client, u16 reg, u32 *val)
{
	int ret;
	__be16 data16;
	__be32 data32;

	data16 = cpu_to_be16(reg);
	ret = i2c_master_send(client, (char *)&data16, 2);
	if (ret < 2) {
		dev_err(&client->dev, "%s: i2c read error, reg: 0x%x\n",
			__func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	if (reg < 0x100) {
		ret = i2c_master_recv(client, (char *)&data16, 2);
		*val = be16_to_cpu(data16);
	} else {
		ret = i2c_master_recv(client, (char *)&data32, 4);
		*val = __swahw32(be32_to_cpu(data32));
	}

	if (ret < 1) {
		dev_err(&client->dev, "%s: i2c read error, reg: 0x%x\n",
			__func__, reg);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static int reg_write(struct i2c_client *client, u16 reg, u32 val)
{
	int ret;
	int size;
	__be16 data16;
	char data[6];

	data16 = cpu_to_be16(reg);
	memcpy(data, &data16, sizeof(data16));
	size = sizeof(data16);
	if (reg < 0x100) {
		data16 = cpu_to_be16(val);
		memcpy(data + size, &data16, sizeof(data16));
		size += sizeof(data16);
	} else {
		data16 = cpu_to_be16(val & 0xffff);
		memcpy(data + size, &data16, sizeof(data16));
		size += sizeof(data16);
		data16 = cpu_to_be16(val >> 16);
		memcpy(data + size, &data16, sizeof(data16));
		size += sizeof(data16);
	}
	ret = i2c_master_send(client, data, size);
	if (ret < size) {
		dev_err(&client->dev, "%s: i2c write error, reg: 0x%x\n",
			__func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int tc358748_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= 0;
	a->bounds.top			= 0;
	a->bounds.width			= TC358748_MAX_WIDTH;
	a->bounds.height		= TC358748_MAX_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int tc358748_g_mbus_config(struct v4l2_subdev *sd,
				  struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_4_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	return 0;
}

/* Find a data format by a pixel code in an array */
static const struct tc358748_color_format *tc358748_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tc358748_cfmts); i++)
		if (tc358748_cfmts[i].code == code)
			return &tc358748_cfmts[i];

	return NULL;
}

static int tc358748_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(tc358748_cfmts))
		return -EINVAL;

	code->code = tc358748_cfmts[code->index].code;
	return 0;
}

static struct tc358748_priv *to_tc358748(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client),
			    struct tc358748_priv, subdev);
}

static int tc358748_get_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tc358748_priv *priv = to_tc358748(client);
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct tc358748_color_format *fmt =
			tc358748_find_datafmt(mf->code);

	mf->width = priv->crop_rect.width;
	mf->height = priv->crop_rect.height;

	if (!fmt) {
		mf->code = tc358748_cfmts[0].code;
		mf->colorspace = tc358748_cfmts[0].colorspace;
	}

	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int tc358748_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *format)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tc358748_priv *priv = to_tc358748(client);
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct tc358748_color_format *fmt =
			tc358748_find_datafmt(mf->code);

	if (mf->width > TC358748_MAX_WIDTH ||
	    mf->height > TC358748_MAX_HEIGHT) {
		priv->crop_rect.width = TC358748_MAX_WIDTH;
		priv->crop_rect.height = TC358748_MAX_HEIGHT;
		mf->width = priv->crop_rect.width;
		mf->height = priv->crop_rect.height;
	}
	priv->crop_rect.width = mf->width;
	priv->crop_rect.height = mf->height;
	if (priv->enabled) {
		ret = reg_write(client, REG_WORDCNT, priv->crop_rect.width * 3);
		if (ret)
			return ret;
	}

	if (!fmt) {
		mf->code = tc358748_cfmts[0].code;
		mf->colorspace = tc358748_cfmts[0].colorspace;
	}

	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int tc358748_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static const struct v4l2_ctrl_ops tc358748_ctrl_ops = {
	.s_ctrl = tc358748_s_ctrl,
};

static struct v4l2_subdev_video_ops tc358748_subdev_video_ops = {
	.cropcap	= tc358748_cropcap,
	.g_mbus_config	= tc358748_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops tc358748_subdev_pad_ops = {
	.enum_mbus_code = tc358748_enum_mbus_code,
	.get_fmt	= tc358748_get_fmt,
	.set_fmt	= tc358748_set_fmt,
};

static int tc358748_set_pll(struct i2c_client *client,
			    struct tc358748_priv *priv)
{
	u16 pllctl0 = PLLCTL0_FBD(priv->pll_fbd) | PLLCTL0_PRD(priv->pll_prd);
	u16 pllctl1 = PLLCTL1_LBWS(2) | PLLCTL1_FRS(priv->pll_frs);
	u16 clkctl = CLKCTL_SCLK(0) | CLKCTL_MCLK(0) | CLKCTL_PPICLK(0);
	int ret;

	ret = reg_write(client, REG_SYSCTL, SYSCTL_SLEEP);
	if (ret)
		return ret;

	ret = reg_write(client, REG_CLKCTL, clkctl);
	if (ret)
		return ret;

	ret = reg_write(client, REG_PLLCTL0, pllctl0);
	if (ret)
		return ret;

	ret = reg_write(client, REG_PLLCTL1, pllctl1);
	if (ret)
		return ret;

	udelay(10);
	pllctl1 |= PLLCTL1_RESETB | PLLCTL1_PLLEN | PLLCTL1_CKEN;
	ret = reg_write(client, REG_PLLCTL1, pllctl1);
	if (ret)
		return ret;

	return reg_write(client, REG_SYSCTL, 0);
}

static int tc358748_set_csi(struct i2c_client *client)
{
	int ret = reg_write(client, REG_CSI_START, 1);

	if (ret)
		return ret;

	/* Use 4 CSI lanes */
	ret = reg_write(client, REG_CSI_CONFW,
			CSI_CONFW_MODE(5) | CSI_CONFW_ADDR(3) |
			CSI_CONFW_DATA(0x6));
	if (ret)
		return ret;

	/* Continuous clock mode */
	ret = reg_write(client, REG_CSI_CONFW,
			CSI_CONFW_MODE(5) | CSI_CONFW_ADDR(3) |
			CSI_CONFW_DATA(0x20));
	if (ret)
		return ret;

	/* High speed */
	ret = reg_write(client, REG_CSI_CONFW,
			CSI_CONFW_MODE(5) | CSI_CONFW_ADDR(3) |
			CSI_CONFW_DATA(0x80));
	if (ret)
		return ret;

	/* Enable regulators */
	ret = reg_write(client, REG_HSTXVREGEN, 0x1f);
	if (ret)
		return ret;

	return reg_write(client, REG_PPI_STARTCNTRL, 1);
}

static int tc358748_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tc358748_priv *priv = to_tc358748(client);
	u32 reg;
	int ret;

	ret = reg_read(client, REG_SYSCTL, &reg);
	if (ret)
		return ret;

	if (!on) {
		priv->enabled = false;
		ret = reg_write(client, REG_CONFCTL,
				CONFCTL_TRIEN | CONFCTL_AUTO);
		if (ret)
			return ret;

		return reg_write(client, REG_SYSCTL, reg | SYSCTL_RESET);
	}

	ret = reg_write(client, REG_SYSCTL, reg & ~SYSCTL_RESET);
	if (ret)
		return ret;

	usleep_range(1000, 2000);
	ret = tc358748_set_pll(client, priv);
	if (ret)
		return ret;

	ret = tc358748_set_csi(client);
	if (ret)
		return ret;

	ret = reg_write(client, REG_DATAFMT, DATAFMT_PDFORMAT(PDFORMAT_RGB888));
	if (ret)
		return ret;

	ret = reg_write(client, REG_FIFOCTL, 0x10);
	if (ret)
		return ret;

	reg = CONFCTL_TRIEN | CONFCTL_AUTO | CONFCTL_PPEN;
	if (priv->vvalid_invert)
		reg |= CONFCTL_VVALIDP;

	if (priv->hvalid_invert)
		reg |= CONFCTL_HVALIDP;

	ret = reg_write(client, REG_CONFCTL, reg);
	if (ret)
		return ret;

	ret = reg_write(client, REG_WORDCNT, priv->crop_rect.width * 3);
	if (ret)
		return ret;

	priv->enabled = true;

	return 0;
}

static struct v4l2_subdev_core_ops tc358748_subdev_core_ops = {
	.s_power = tc358748_s_power,
};

static struct v4l2_subdev_ops tc358748_subdev_ops = {
	.core = &tc358748_subdev_core_ops,
	.video = &tc358748_subdev_video_ops,
	.pad = &tc358748_subdev_pad_ops,
};

static int tc358748_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct tc358748_priv *priv;
	u32 id;
	u32 pixclk;
	u32 extclk;
	int ret = reg_read(client, REG_CHIPID, &id);

	if (ret)
		return ret;

	if ((id >> 8) != 0x44) {
		dev_err(&client->dev, "Bad chip ID: %#x\n", id);
		return -EINVAL;
	}

	if (of_property_read_u32(client->dev.of_node, "clock-frequency",
				 &extclk)) {
		dev_err(&client->dev,
			"Not found clock-frequency property in device-tree\n");
		return -EINVAL;
	}
	if (extclk != 40000000) {
		dev_err(&client->dev,
			"Input clock frequency %d Hz is not supported\n",
			extclk);
		return -EINVAL;
	}

	if (of_property_read_u32(client->dev.of_node, "pixel-frequency",
				 &pixclk)) {
		dev_err(&client->dev,
			"Not found pixel-frequency property in device-tree\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	switch (pixclk) {
	case 72000000:
		priv->pll_fbd = 0x6b;
		priv->pll_prd = 0x4;
		priv->pll_frs = 0x1;
		break;
	case 126000000:
		priv->pll_fbd = 0xbc;
		priv->pll_prd = 0x4;
		priv->pll_frs = 0x1;
		break;
	default:
		dev_err(&client->dev, "Pixel frequency %d is not supported\n",
			pixclk);
		return -EINVAL;
	}

	priv->vvalid_invert = of_property_read_bool(client->dev.of_node,
						    "vvalid-invert");
	priv->hvalid_invert = of_property_read_bool(client->dev.of_node,
						    "hvalid-invert");

	priv->crop_rect.width = TC358748_DEFAULT_WIDTH;
	priv->crop_rect.height = TC358748_DEFAULT_HEIGHT;
	priv->cfmt = &tc358748_cfmts[0];

	v4l2_i2c_subdev_init(&priv->subdev, client, &tc358748_subdev_ops);

	/* Dummy controls are needed to deceive VINC driver */
	v4l2_ctrl_handler_init(&priv->hdl, 0);
	priv->gain = v4l2_ctrl_new_std(&priv->hdl, &tc358748_ctrl_ops,
		V4L2_CID_GAIN, 0, 95, 1, 32);
	priv->exp = v4l2_ctrl_new_std(&priv->hdl, &tc358748_ctrl_ops,
		V4L2_CID_EXPOSURE, 1, 17600, 1, 17280);
	priv->exp_abs = v4l2_ctrl_new_std(&priv->hdl, &tc358748_ctrl_ops,
		V4L2_CID_EXPOSURE_ABSOLUTE, 1, 332, 1, 326);
	priv->subdev.ctrl_handler = &priv->hdl;
	if (priv->hdl.error)
		return priv->hdl.error;
	v4l2_ctrl_handler_setup(&priv->hdl);

	v4l2_async_register_subdev(&priv->subdev);
	dev_info(&client->dev, "registered\n");

	return 0;
}

static int tc358748_remove(struct i2c_client *client)
{
	struct tc358748_priv *priv = to_tc358748(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);
	return 0;
}

static const struct of_device_id tc358748_of_match[] = {
	{ .compatible = "toshiba,tc358748", },
	{},
};
MODULE_DEVICE_TABLE(of, tc358748_of_match);

static const struct i2c_device_id tc358748_id[] = {
	{ "tc358748", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tc358748_id);

static struct i2c_driver tc358748_i2c_driver = {
	.driver = {
		.name = "tc358748",
		.of_match_table = of_match_ptr(tc358748_of_match),
	},
	.probe    = tc358748_probe,
	.remove   = tc358748_remove,
	.id_table = tc358748_id,
};

module_i2c_driver(tc358748_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for Toshiba TC358748");
MODULE_LICENSE("GPL v2");
