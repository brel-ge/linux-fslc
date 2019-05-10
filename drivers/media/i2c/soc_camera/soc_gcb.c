/*
 * Driver for GCB CMOS Image Sensor from Omnivision
 *
 * Copyright (C) 2011, Bastian Hecht <hechtb@gmail.com>
 *
 * Based on Sony IMX074 Camera Driver
 * Copyright (C) 2010, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * Based on Omnivision OV7670 Camera Driver
 * Copyright (C) 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/module.h>
#include <linux/v4l2-mediabus.h>

#include <media/soc_camera.h>
#include <media/v4l2-clk.h>
#include <media/v4l2-subdev.h>


/*
 * About GCB resolution, cropping and binning:
 * This sensor supports it all, at least in the feature description.
 * Unfortunately, no combination of appropriate registers settings could make
 * the chip work the intended way. As it works with predefined register lists,
 * some undocumented registers are presumably changed there to achieve their
 * goals.
 * This driver currently only works for resolutions up to 720 lines with a
 * 1:1 scale. Hopefully these restrictions will be removed in the future.
 */
#define GCB_MAX_WIDTH	1920
#define GCB_MAX_HEIGHT	1080

/* default sizes */
#define GCB_DEFAULT_WIDTH	640
#define GCB_DEFAULT_HEIGHT	480

/* minimum extra blanking */
#define BLANKING_EXTRA_WIDTH		500
#define BLANKING_EXTRA_HEIGHT		20
#define BLANKING_MIN_HEIGHT		400

struct gcb_datafmt {
	u32	code;
	enum v4l2_colorspace		colorspace;
};

struct gcb {
	struct v4l2_subdev		subdev;
	const struct gcb_datafmt	*fmt;
	struct v4l2_rect                crop_rect;
	struct v4l2_clk			*clk;

	/* blanking information */
	int total_width;
	int total_height;
};

static const struct gcb_datafmt gcb_colour_fmts[] = {
	{MEDIA_BUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
};

static struct gcb *to_gcb(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct gcb, subdev);
}

/* Find a data format by a pixel code in an array */
static const struct gcb_datafmt
			*gcb_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(gcb_colour_fmts); i++)
		if (gcb_colour_fmts[i].code == code)
			return gcb_colour_fmts + i;

	return NULL;
}


static int gcb_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gcb *priv = to_gcb(client);
	const struct gcb_datafmt *fmt = gcb_find_datafmt(mf->code);

	if (format->pad)
		return -EINVAL;

	mf->width = priv->crop_rect.width;
	mf->height = priv->crop_rect.height;

	if (!fmt) {
		if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
			return -EINVAL;
		mf->code	= gcb_colour_fmts[0].code;
		mf->colorspace	= gcb_colour_fmts[0].colorspace;
	}

	mf->field	= V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		priv->fmt = fmt;
	else
		cfg->try_fmt = *mf;
	return 0;
}

static int gcb_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gcb *priv = to_gcb(client);

	const struct gcb_datafmt *fmt = priv->fmt;

	if (format->pad)
		return -EINVAL;

	mf->code	= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->width	= priv->crop_rect.width;
	mf->height	= priv->crop_rect.height;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int gcb_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(gcb_colour_fmts))
		return -EINVAL;

	code->code = gcb_colour_fmts[code->index].code;
	return 0;
}

static int gcb_set_selection(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_selection *sel)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gcb *priv = to_gcb(client);
	struct v4l2_rect rect = sel->r;
	int ret;

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE ||
	    sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	v4l_bound_align_image(&rect.width, 48, GCB_MAX_WIDTH, 1,
			      &rect.height, 32, GCB_MAX_HEIGHT, 1, 0);

	priv->crop_rect.width	= rect.width;
	priv->crop_rect.height	= rect.height;
	priv->total_width	= rect.width + BLANKING_EXTRA_WIDTH;
	priv->total_height	= max_t(int, rect.height +
							BLANKING_EXTRA_HEIGHT,
							BLANKING_MIN_HEIGHT);
	priv->crop_rect.width		= rect.width;
	priv->crop_rect.height		= rect.height;

	return ret;
}

static int gcb_get_selection(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_selection *sel)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gcb *priv = to_gcb(client);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = GCB_MAX_WIDTH;
		sel->r.height = GCB_MAX_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP:
		sel->r = priv->crop_rect;
		return 0;
	default:
		return -EINVAL;
	}
}

static int gcb_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->flags = V4L2_MBUS_CSI2_2_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
					V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	return 0;
}

static int gcb_s_power(struct v4l2_subdev *sd, int on)
{
	int ret = 0;
	return ret;
}

static const struct v4l2_subdev_video_ops gcb_subdev_video_ops = {
	.g_mbus_config	= gcb_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops gcb_subdev_pad_ops = {
	.enum_mbus_code = gcb_enum_mbus_code,
	.get_selection	= gcb_get_selection,
	.set_selection	= gcb_set_selection,
	.get_fmt	= gcb_get_fmt,
	.set_fmt	= gcb_set_fmt,
};

static const struct v4l2_subdev_core_ops gcb_subdev_core_ops = {
	.s_power	= gcb_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= gcb_get_register,
	.s_register	= gcb_set_register,
#endif
};

static const struct v4l2_subdev_ops gcb_subdev_ops = {
	.core	= &gcb_subdev_core_ops,
	.video	= &gcb_subdev_video_ops,
	.pad	= &gcb_subdev_pad_ops,
};

static int gcb_video_probe(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	int ret;
	ret = gcb_s_power(subdev, 1);
	if (ret < 0)
		return ret;

	dev_info(&client->dev, "Probe\n");

	ret = 0;
	return ret;
}

static int gcb_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct gcb *priv;
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	int ret;

	if (!ssdd) {
		dev_err(&client->dev, "GCB: missing platform data!\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct gcb), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&priv->subdev, client, &gcb_subdev_ops);

	priv->fmt		= &gcb_colour_fmts[0];

	priv->crop_rect.width	= GCB_DEFAULT_WIDTH;
	priv->crop_rect.height	= GCB_DEFAULT_HEIGHT;
	priv->crop_rect.left	= (GCB_MAX_WIDTH - GCB_DEFAULT_WIDTH) / 2;
	priv->crop_rect.top	= (GCB_MAX_HEIGHT - GCB_DEFAULT_HEIGHT) / 2;
	priv->total_width = GCB_DEFAULT_WIDTH + BLANKING_EXTRA_WIDTH;
	priv->total_height = BLANKING_MIN_HEIGHT;

	priv->clk = v4l2_clk_get(&client->dev, "mclk");
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);

	ret = gcb_video_probe(client);
	if (ret < 0)
		v4l2_clk_put(priv->clk);

	return ret;
}

static int gcb_remove(struct i2c_client *client)
{
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	struct gcb *priv = to_gcb(client);

	v4l2_clk_put(priv->clk);
	if (ssdd->free_bus)
		ssdd->free_bus(ssdd);

	return 0;
}

static const struct i2c_device_id gcb_id[] = {
	{ "gcb", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, gcb_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id gcb_of_match[] = {
	{ .compatible = "gemmi,soc_gcb" },
	{ },
};
MODULE_DEVICE_TABLE(of, gcb_of_match);
#endif

static struct i2c_driver gcb_i2c_driver = {
	.driver = {
		.name = "gcb",
		.of_match_table = of_match_ptr(gcb_of_match),
	},
	.probe		= gcb_probe,
	.remove		= gcb_remove,
	.id_table	= gcb_id,
};

module_i2c_driver(gcb_i2c_driver);

MODULE_DESCRIPTION("Omnivision GCB Camera driver");
MODULE_AUTHOR("Bastian Hecht <hechtb@gmail.com>");
MODULE_LICENSE("GPL v2");
