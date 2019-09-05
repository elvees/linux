// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019 RnD Center "ELVEES", JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
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

/* Registers list from
 * https://github.com/ultragtx/kernel-glass/blob/97940c951341f07c1868c2c0d10143d9f5bc02eb/drivers/video/omap2/displays/panel-tc358762.h
 */
#define PPI_STARTPPI		0x104
#define PPI_D0S_ATMR		0x144
#define PPI_D1S_ATMR		0x148
#define PPI_D0S_CLRSIPOCOUNT	0x164
#define PPI_D1S_CLRSIPOCOUNT	0x168
#define DSI_STARTDSI		0x204
#define DSI_LANEENABLE		0x210
#define LCDCTRL_PORT		0x420
#define VFUEN			0x434
#define SYSCTRL			0x464
#define SYSPLL3			0x470
#define SYSPMCTRL		0x47c

#define TC358762_MAX_WIDTH 1366U
#define TC358762_MAX_HEIGHT 768U
#define TC358762_DEFAULT_WIDTH 1280U
#define TC358762_DEFAULT_HEIGHT 720U

struct tc358762_color_format {
	u32 code;
	enum v4l2_colorspace colorspace;
};

struct tc358762_priv {
	struct v4l2_subdev subdev;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *exp;
	struct v4l2_ctrl *exp_abs;
	struct v4l2_rect frame_size;
	u32 pll_value;
};

static const struct tc358762_color_format tc358762_cfmts[] = {
	{
		.code		= MEDIA_BUS_FMT_RGB888_1X24,
		.colorspace	= V4L2_COLORSPACE_SRGB,
	},
};

static int reg_read(struct i2c_client *client, u16 reg, u32 *val)
{
	int ret;
	__be16 addr;
	__le32 data32;

	addr = cpu_to_be16(reg);
	ret = i2c_master_send(client, (char *)&addr, 2);
	if (ret < 2) {
		dev_err(&client->dev, "i2c write error, reg: 0x%x\n", reg);
		return ret < 0 ? ret : -EIO;
	}

	ret = i2c_master_recv(client, (char *)&data32, 4);
	if (ret < 4) {
		dev_err(&client->dev, "i2c read error, reg: 0x%x\n", reg);
		return ret < 0 ? ret : -EIO;
	}
	*val = le32_to_cpu(data32);

	return 0;
}

static int reg_write(struct i2c_client *client, u16 reg, u32 val)
{
	int ret;
	struct {
		__be16 addr;
		__le32 data32;
	} __packed buf;

	buf.addr = cpu_to_be16(reg);
	buf.data32 = cpu_to_le32(val);

	ret = i2c_master_send(client, (char *)&buf, sizeof(buf));
	if (ret < sizeof(buf)) {
		dev_err(&client->dev, "i2c write error, reg: 0x%x\n", reg);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int tc358762_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= 0;
	a->bounds.top			= 0;
	a->bounds.width			= TC358762_MAX_WIDTH;
	a->bounds.height		= TC358762_MAX_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int tc358762_g_mbus_config(struct v4l2_subdev *sd,
				  struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_PARALLEL;
	cfg->flags = V4L2_MBUS_MASTER;
	return 0;
}

static int tc358762_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(tc358762_cfmts))
		return -EINVAL;

	code->code = tc358762_cfmts[code->index].code;
	return 0;
}

static struct tc358762_priv *to_tc358762(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client),
			    struct tc358762_priv, subdev);
}

/* Find a data format by a pixel code in an array */
static const struct tc358762_color_format *tc358762_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tc358762_cfmts); i++)
		if (tc358762_cfmts[i].code == code)
			return &tc358762_cfmts[i];

	return NULL;
}

static int tc358762_get_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tc358762_priv *priv = to_tc358762(client);
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct tc358762_color_format *fmt =
			tc358762_find_datafmt(mf->code);

	mf->width = priv->frame_size.width;
	mf->height = priv->frame_size.height;
	if (!fmt) {
		mf->code = tc358762_cfmts[0].code;
		mf->colorspace = tc358762_cfmts[0].colorspace;
	}
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int tc358762_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tc358762_priv *priv = to_tc358762(client);
	struct v4l2_mbus_framefmt *mf = &format->format;

	priv->frame_size.width = clamp(mf->width, 0U, TC358762_MAX_WIDTH);
	priv->frame_size.height = clamp(mf->height, 0U, TC358762_MAX_HEIGHT);

	return tc358762_get_fmt(sd, cfg, format);
}

static int tc358762_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static const struct v4l2_ctrl_ops tc358762_ctrl_ops = {
	.s_ctrl = tc358762_s_ctrl,
};

static struct v4l2_subdev_video_ops tc358762_subdev_video_ops = {
	.cropcap	= tc358762_cropcap,
	.g_mbus_config	= tc358762_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops tc358762_subdev_pad_ops = {
	.enum_mbus_code = tc358762_enum_mbus_code,
	.get_fmt	= tc358762_get_fmt,
	.set_fmt	= tc358762_set_fmt,
};

static struct {
	u16 reg;
	u32 data;
} tc358762_init_seq[] = {
	{ SYSPMCTRL, 0 },               /* Disable sleep mode */
	{ 0, 0 },                       /* Delay */
	{ DSI_LANEENABLE, 0x7 },        /* Enable Clk, Lane0 and Lane1 */
	{ PPI_D0S_CLRSIPOCOUNT, 0x5 },
	{ PPI_D1S_CLRSIPOCOUNT, 0x5 },
	{ LCDCTRL_PORT, 0x150 },        /* DPI_ENABLE, RGB888 */
	{ SYSCTRL, 0x40f },
	{ PPI_STARTPPI, 0x1 },
	{ DSI_STARTDSI, 0x1 },
	{ VFUEN, 0x1 },
};

static int tc358762_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tc358762_priv *priv = to_tc358762(client);
	u32 reg;
	int ret;
	int i;

	/* The driver can enable TC358762 the first time only. The sequence for
	 * disabling it and reenabling is unknown, because full datasheet
	 * isn't available.
	 */
	ret = reg_read(client, VFUEN, &reg);
	if (ret)
		return ret;

	/* If already started */
	if (reg == 0x1)
		return 0;

	for (i = 0; i < ARRAY_SIZE(tc358762_init_seq); i++) {
		if (tc358762_init_seq[i].reg) {
			ret = reg_write(client, tc358762_init_seq[i].reg,
					tc358762_init_seq[i].data);
			if (ret) {
				dev_err(&client->dev,
					 "Filed to write to TC358762");
				return ret;
			}
		} else
			usleep_range(5000, 7000);
	}

	msleep(100);

	ret = reg_write(client, SYSPLL3, priv->pll_value);
	if (ret)
		return ret;

	return 0;
}

static struct v4l2_subdev_core_ops tc358762_subdev_core_ops = {
	.s_power = tc358762_s_power,
};

static struct v4l2_subdev_ops tc358762_subdev_ops = {
	.core = &tc358762_subdev_core_ops,
	.video = &tc358762_subdev_video_ops,
	.pad = &tc358762_subdev_pad_ops,
};

static int tc358762_probe(struct i2c_client *client,
			  const struct i2c_device_id *did)
{
	struct tc358762_priv *priv;
	u32 pixclk, extclk, pll_div1, pll_div2, pll_mul;

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

	if (pixclk != 16000000) {
		dev_err(&client->dev, "Pixel frequency %d is not supported\n",
			pixclk);
		return -EINVAL;
	}

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	pll_mul = 4;
	pll_div1 = extclk * pll_mul / pixclk;
	pll_div2 = 2;
	priv->pll_value = ((pll_mul - 1) << 28) | (pll_div1 << 16) |
			  (pll_div2 << 20);

	priv->frame_size.width = TC358762_DEFAULT_WIDTH;
	priv->frame_size.height = TC358762_DEFAULT_HEIGHT;

	v4l2_i2c_subdev_init(&priv->subdev, client, &tc358762_subdev_ops);

	/* Dummy controls are needed to deceive VINC driver */
	v4l2_ctrl_handler_init(&priv->hdl, 0);
	priv->gain = v4l2_ctrl_new_std(&priv->hdl, &tc358762_ctrl_ops,
		V4L2_CID_GAIN, 0, 95, 1, 32);
	priv->exp = v4l2_ctrl_new_std(&priv->hdl, &tc358762_ctrl_ops,
		V4L2_CID_EXPOSURE, 1, 17600, 1, 17280);
	priv->exp_abs = v4l2_ctrl_new_std(&priv->hdl, &tc358762_ctrl_ops,
		V4L2_CID_EXPOSURE_ABSOLUTE, 1, 332, 1, 326);
	priv->subdev.ctrl_handler = &priv->hdl;
	if (priv->hdl.error)
		return priv->hdl.error;
	v4l2_ctrl_handler_setup(&priv->hdl);

	v4l2_async_register_subdev(&priv->subdev);
	v4l2_info(&priv->subdev, "registered\n");

	return 0;
}

static int tc358762_remove(struct i2c_client *client)
{
	struct tc358762_priv *priv = to_tc358762(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);
	return 0;
}

static const struct of_device_id tc358762_of_match[] = {
	{ .compatible = "toshiba,tc358762", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, tc358762_of_match);

static const struct i2c_device_id tc358762_id[] = {
	{ "tc358762" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, tc358762_id);

static struct i2c_driver tc358762_i2c_driver = {
	.driver = {
		.name = "tc358762",
		.of_match_table = of_match_ptr(tc358762_of_match),
	},
	.probe    = tc358762_probe,
	.remove   = tc358762_remove,
	.id_table = tc358762_id,
};

module_i2c_driver(tc358762_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for Toshiba TC358762");
MODULE_LICENSE("GPL v2");
