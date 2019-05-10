/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2017 Mentor Graphics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* min/typical/max system clock (xclk) frequencies */
#define GCB_XCLK_MIN  6000000
#define GCB_XCLK_MAX 54000000

#define GCB_FRAME_RATE 30
#define GCB_CLK_VGA         24000000 /* Desired clock rate */
#define GCB_CLK_C_CAMII     24000000 /* Desired clock rate */
#define GCB_CLK_NTSC        27000000 /* Desired clock rate */
#define GCB_CLK_SQUARE_400  18000000 /* Desired clock rate */
#define GCB_CLK_MIN         20000000 /* 12000000  */
#define GCB_CLK_MAX 2700000
#define GCB_MAX_WIDTH 640  /* Max width for this camera */
#define GCB_MAX_HEIGHT 480 /* Max height for this camera */

enum gcb_mode_id {
	GCB_MODE_VGA = 0,
	GCB_MODE_NTSC,
	GCB_MODE_SQUARE_400,
	GCB_MODE_C_CAMII,
	GCB_MODE_FPD_DEV,
	GCB_NUM_MODES,
};

enum gcb_frame_rate {
	GCB_30_FPS,
	GCB_60_FPS,
	GCB_NUM_FRAMERATES,
};

struct gcb_pixfmt {
	u32 code;
	u32 colorspace;
};

static const struct gcb_pixfmt gcb_formats[] = {
	{ MEDIA_BUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_SRGB, },
};


static const int gcb_framerates[] = {
	[GCB_30_FPS] = 30,
	[GCB_60_FPS] = 60,
};

struct gcb_mode_info {
	enum gcb_mode_id id;
	u32 width;
	u32 height;
	u32 framerate;
	u32 clock_curr;
	u32 nobt_hs_inv;
	u32 nobt_vs_inv;
};

struct gcb_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exposure;
	};
	struct {
		struct v4l2_ctrl *auto_wb;
		struct v4l2_ctrl *blue_balance;
		struct v4l2_ctrl *red_balance;
	};
	struct {
		struct v4l2_ctrl *auto_gain;
		struct v4l2_ctrl *gain;
	};
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *light_freq;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *hue;
	struct v4l2_ctrl *test_pattern;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
};

struct gcb_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */

	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;

	/* lock to protect all members below */
	struct mutex lock;

	struct v4l2_mbus_framefmt fmt;
	bool pending_fmt_change;

	const struct gcb_mode_info *current_mode;
	const struct gcb_mode_info *last_mode;
	enum gcb_frame_rate current_fr;
	struct v4l2_fract frame_interval;

	struct gcb_ctrls ctrls;

	u32 prev_sysclk, prev_hts;

	bool pending_mode_change;
	bool streaming;
};

static inline struct gcb_dev *to_gcb_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gcb_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct gcb_dev,
			     ctrls.handler)->sd;
}


/* power-on sensor init reg table */
static const struct gcb_mode_info gcb_mode_init_data = {
	.id = GCB_MODE_VGA,
	.width = 640,
	.height = 480,
	.framerate = 30,
	.clock_curr = GCB_CLK_VGA,
	.nobt_hs_inv = 0,
	.nobt_vs_inv = 1,
};

static const struct gcb_mode_info
gcb_mode_data[GCB_NUM_MODES] = {
			{
			.id = GCB_MODE_VGA,
			.width = 640,
			.height = 480,
			.framerate = 30,
			.clock_curr = GCB_CLK_VGA,
			.nobt_hs_inv = 0,
			.nobt_vs_inv = 1
		},
		{
			.id = GCB_MODE_NTSC,
			.width = 720,
			.height = 260,
			.framerate = 30,
			.clock_curr = GCB_CLK_NTSC,
			.nobt_hs_inv = 1,
			.nobt_vs_inv = 0
		},
		{
			.id = GCB_MODE_SQUARE_400,
			.width = 400,
			.height = 400,
			.framerate = 30,
			.clock_curr = GCB_CLK_SQUARE_400,
			.nobt_hs_inv = 0,
			.nobt_vs_inv = 1
		},
		{
			.id = GCB_MODE_C_CAMII,
			.width = 1280,
			.height = 968,
			.framerate = 30,
			.clock_curr = GCB_CLK_C_CAMII,
			.nobt_hs_inv = 0,
			.nobt_vs_inv = 1
		},
		{
			.id = GCB_MODE_FPD_DEV,
			.width = 640,
			.height = 720,
			.framerate = 30,
			.clock_curr = GCB_CLK_C_CAMII,
			.nobt_hs_inv = 0,
			.nobt_vs_inv = 1
		},
};


/*
 * This is supposed to be ranging from 1 to 8, but the value is always
 * set to 3 in the vendor kernels.
 */
#define GCB_PLL_PREDIV	3

#define GCB_PLL_MULT_MIN	4
#define GCB_PLL_MULT_MAX	252

/*
 * This is supposed to be ranging from 1 to 16, but the value is
 * always set to either 1 or 2 in the vendor kernels.
 */
#define GCB_SYSDIV_MIN	1
#define GCB_SYSDIV_MAX	16

/*
 * Hardcode these values for scaler and non-scaler modes.
 * FIXME: to be re-calcualted for 1 data lanes setups
 */
#define GCB_MIPI_DIV_PCLK	2
#define GCB_MIPI_DIV_SCLK	1

/*
 * This is supposed to be ranging from 1 to 2, but the value is always
 * set to 2 in the vendor kernels.
 */
#define GCB_PLL_ROOT_DIV			2
#define GCB_PLL_CTRL3_PLL_ROOT_DIV_2		BIT(4)

/*
 * We only supports 8-bit formats at the moment
 */
#define GCB_BIT_DIV				2
#define GCB_PLL_CTRL0_MIPI_MODE_8BIT		0x08

/*
 * This is supposed to be ranging from 1 to 8, but the value is always
 * set to 2 in the vendor kernels.
 */
#define GCB_SCLK_ROOT_DIV	2

/*
 * This is hardcoded so that the consistency is maintained between SCLK and
 * SCLK 2x.
 */
#define GCB_SCLK2X_ROOT_DIV (GCB_SCLK_ROOT_DIV / 2)

/*
 * This is supposed to be ranging from 1 to 8, but the value is always
 * set to 1 in the vendor kernels.
 */
#define GCB_PCLK_ROOT_DIV			1
#define GCB_PLL_SYS_ROOT_DIVIDER_BYPASS	0x00



static const struct gcb_mode_info *
gcb_find_mode(struct gcb_dev *sensor, enum gcb_frame_rate fr,
		 int width, int height, bool nearest)
{
	const struct gcb_mode_info *mode;

	mode = v4l2_find_nearest_size(gcb_mode_data,
				      ARRAY_SIZE(gcb_mode_data),
				      width, height,
				      width, height);

	if (!mode ||
	    (!nearest && (mode->width != width || mode->height != height)))
		return NULL;

	if (fr == GCB_60_FPS)
		return NULL;

	return mode;
}


static int gcb_set_framefmt(struct gcb_dev *sensor,
			       struct v4l2_mbus_framefmt *format);



/* --------------- Subdev Operations --------------- */

static int gcb_s_power(struct v4l2_subdev *sd, int on)
{
	int ret = 0;
	struct gcb_dev *sensor = to_gcb_dev(sd);
	struct device *dev = &sensor->i2c_client->dev;
	dev_info(dev, __func__);
	return ret;
}

static int gcb_try_frame_interval(struct gcb_dev *sensor,
				     struct v4l2_fract *fi,
				     u32 width, u32 height)
{
	const struct gcb_mode_info *mode;
	enum gcb_frame_rate rate = GCB_30_FPS;
	int minfps, maxfps, best_fps, fps;
	int i;

	minfps = gcb_framerates[GCB_30_FPS];
	maxfps = gcb_framerates[GCB_60_FPS];

	if (fi->numerator == 0) {
		fi->denominator = maxfps;
		fi->numerator = 1;
		rate = GCB_60_FPS;
		goto find_mode;
	}

	fps = clamp_val(DIV_ROUND_CLOSEST(fi->denominator, fi->numerator),
			minfps, maxfps);

	best_fps = minfps;
	for (i = 0; i < ARRAY_SIZE(gcb_framerates); i++) {
		int curr_fps = gcb_framerates[i];

		if (abs(curr_fps - fps) < abs(best_fps - fps)) {
			best_fps = curr_fps;
			rate = i;
		}
	}

	fi->numerator = 1;
	fi->denominator = best_fps;

find_mode:
	mode = gcb_find_mode(sensor, rate, width, height, false);
	return mode ? rate : -EINVAL;
}

static int gcb_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct gcb_dev *sensor = to_gcb_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg,
						 format->pad);
	else
		fmt = &sensor->fmt;

	format->format = *fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}

static int gcb_try_fmt_internal(struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt,
				   enum gcb_frame_rate fr,
				   const struct gcb_mode_info **new_mode)
{
	struct gcb_dev *sensor = to_gcb_dev(sd);
	const struct gcb_mode_info *mode;
	int i;

	mode = gcb_find_mode(sensor, fr, fmt->width, fmt->height, true);
	if (!mode)
		return -EINVAL;
	fmt->width = mode->width;
	fmt->height = mode->height;

	if (new_mode)
		*new_mode = mode;

	for (i = 0; i < ARRAY_SIZE(gcb_formats); i++)
		if (gcb_formats[i].code == fmt->code)
			break;
	if (i >= ARRAY_SIZE(gcb_formats))
		i = 0;

	fmt->code = gcb_formats[i].code;
	fmt->colorspace = gcb_formats[i].colorspace;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);

	return 0;
}

static int gcb_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct gcb_dev *sensor = to_gcb_dev(sd);
	const struct gcb_mode_info *new_mode;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	ret = gcb_try_fmt_internal(sd, mbus_fmt,
				      sensor->current_fr, &new_mode);
	if (ret)
		goto out;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(sd, cfg, 0);
	else
		fmt = &sensor->fmt;

	*fmt = *mbus_fmt;

	if (new_mode != sensor->current_mode) {
		sensor->current_mode = new_mode;
		sensor->pending_mode_change = true;
	}
	if (mbus_fmt->code != sensor->fmt.code)
		sensor->pending_fmt_change = true;

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int gcb_set_framefmt(struct gcb_dev *sensor,
			       struct v4l2_mbus_framefmt *format)
{
	int ret = 0;
	return ret;
}

/*
 * Sensor Controls.
 */

static int gcb_set_ctrl_hue(struct gcb_dev *sensor, int value)
{
	int ret = 0;
	return ret;
}

static int gcb_set_ctrl_contrast(struct gcb_dev *sensor, int value)
{
	int ret = 0;
	return ret;
}

static int gcb_set_ctrl_saturation(struct gcb_dev *sensor, int value)
{
	int ret = 0;
	return ret;
}

static int gcb_set_ctrl_white_balance(struct gcb_dev *sensor, int awb)
{
	int ret = 0;
	return ret;
}

static int gcb_set_ctrl_exposure(struct gcb_dev *sensor,
				    enum v4l2_exposure_auto_type auto_exposure)
{
	int ret = 0;
	return ret;
}

static int gcb_set_ctrl_gain(struct gcb_dev *sensor, bool auto_gain)
{
	int ret = 0;
	return ret;
}

static int gcb_set_ctrl_test_pattern(struct gcb_dev *sensor, int value)
{
	int ret = 0;
	return ret;
}

static int gcb_set_ctrl_light_freq(struct gcb_dev *sensor, int value)
{
	int ret = 0;
	return ret;
}

static int gcb_set_ctrl_hflip(struct gcb_dev *sensor, int value)
{
	int ret = 0;
	return ret;
}

static int gcb_set_ctrl_vflip(struct gcb_dev *sensor, int value)
{
	int ret = 0;
	return ret;
}

static int gcb_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	return ret;
}

static int gcb_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct gcb_dev *sensor = to_gcb_dev(sd);
	int ret;

	/* v4l2_ctrl_lock() locks our own mutex */


	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		ret = gcb_set_ctrl_gain(sensor, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = gcb_set_ctrl_exposure(sensor, ctrl->val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = gcb_set_ctrl_white_balance(sensor, ctrl->val);
		break;
	case V4L2_CID_HUE:
		ret = gcb_set_ctrl_hue(sensor, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		ret = gcb_set_ctrl_contrast(sensor, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		ret = gcb_set_ctrl_saturation(sensor, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = gcb_set_ctrl_test_pattern(sensor, ctrl->val);
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		ret = gcb_set_ctrl_light_freq(sensor, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = gcb_set_ctrl_hflip(sensor, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = gcb_set_ctrl_vflip(sensor, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static const struct v4l2_ctrl_ops gcb_ctrl_ops = {
	.g_volatile_ctrl = gcb_g_volatile_ctrl,
	.s_ctrl = gcb_s_ctrl,
};

static const char * const test_pattern_menu[] = {
	"Disabled",
	"Color bars",
};

static int gcb_init_controls(struct gcb_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &gcb_ctrl_ops;
	struct gcb_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int ret;

	v4l2_ctrl_handler_init(hdl, 32);

	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	/* Auto/manual white balance */
	ctrls->auto_wb = v4l2_ctrl_new_std(hdl, ops,
					   V4L2_CID_AUTO_WHITE_BALANCE,
					   0, 1, 1, 1);
	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BLUE_BALANCE,
						0, 4095, 1, 0);
	ctrls->red_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE,
					       0, 4095, 1, 0);
	/* Auto/manual exposure */
	ctrls->auto_exp = v4l2_ctrl_new_std_menu(hdl, ops,
						 V4L2_CID_EXPOSURE_AUTO,
						 V4L2_EXPOSURE_MANUAL, 0,
						 V4L2_EXPOSURE_AUTO);
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					    0, 65535, 1, 0);
	/* Auto/manual gain */
	ctrls->auto_gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_AUTOGAIN,
					     0, 1, 1, 1);
	ctrls->gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAIN,
					0, 1023, 1, 0);

	ctrls->saturation = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SATURATION,
					      0, 255, 1, 64);
	ctrls->hue = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HUE,
				       0, 359, 1, 0);
	ctrls->contrast = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_CONTRAST,
					    0, 255, 1, 0);
	ctrls->test_pattern =
		v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(test_pattern_menu) - 1,
					     0, 0, test_pattern_menu);
	ctrls->hflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP,
					 0, 1, 1, 0);
	ctrls->vflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP,
					 0, 1, 1, 0);

	ctrls->light_freq =
		v4l2_ctrl_new_std_menu(hdl, ops,
				       V4L2_CID_POWER_LINE_FREQUENCY,
				       V4L2_CID_POWER_LINE_FREQUENCY_AUTO, 0,
				       V4L2_CID_POWER_LINE_FREQUENCY_50HZ);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	ctrls->gain->flags |= V4L2_CTRL_FLAG_VOLATILE;
	ctrls->exposure->flags |= V4L2_CTRL_FLAG_VOLATILE;

	v4l2_ctrl_auto_cluster(3, &ctrls->auto_wb, 0, false);
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_gain, 0, true);
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_exp, 1, true);

	sensor->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int gcb_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->pad != 0)
		return -EINVAL;
	if (fse->index >= GCB_NUM_MODES)
		return -EINVAL;

	fse->min_width =
		gcb_mode_data[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height =
		gcb_mode_data[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int gcb_enum_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	struct gcb_dev *sensor = to_gcb_dev(sd);
	struct v4l2_fract tpf;
	int ret;

	if (fie->pad != 0)
		return -EINVAL;
	if (fie->index >= GCB_NUM_FRAMERATES)
		return -EINVAL;

	tpf.numerator = 1;
	tpf.denominator = gcb_framerates[fie->index];

	ret = gcb_try_frame_interval(sensor, &tpf,
					fie->width, fie->height);
	if (ret < 0)
		return -EINVAL;

	fie->interval = tpf;
	return 0;
}

static int gcb_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct gcb_dev *sensor = to_gcb_dev(sd);

	mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int gcb_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct gcb_dev *sensor = to_gcb_dev(sd);
	const struct gcb_mode_info *mode;
	int frame_rate, ret = 0;

	if (fi->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	mode = sensor->current_mode;

	frame_rate = gcb_try_frame_interval(sensor, &fi->interval,
					       mode->width, mode->height);
	if (frame_rate < 0) {
		/* Always return a valid frame interval value */
		fi->interval = sensor->frame_interval;
		goto out;
	}

	mode = gcb_find_mode(sensor, frame_rate, mode->width,
				mode->height, true);
	if (!mode) {
		ret = -EINVAL;
		goto out;
	}

	if (mode != sensor->current_mode ||
	    frame_rate != sensor->current_fr) {
		sensor->current_fr = frame_rate;
		sensor->frame_interval = fi->interval;
		sensor->current_mode = mode;
		sensor->pending_mode_change = true;
	}
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int gcb_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0)
		return -EINVAL;
	if (code->index >= ARRAY_SIZE(gcb_formats))
		return -EINVAL;

	code->code = gcb_formats[code->index].code;
	return 0;
}

static int gcb_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct gcb_dev *sensor = to_gcb_dev(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);

	if (sensor->streaming == !enable) {
		if (enable && sensor->pending_mode_change) {
			sensor->pending_mode_change = false;
		}

		if (enable && sensor->pending_fmt_change) {
			ret = gcb_set_framefmt(sensor, &sensor->fmt);
			if (ret)
				goto out;
			sensor->pending_fmt_change = false;
		}

		if (!ret)
			sensor->streaming = enable;
	}
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops gcb_core_ops = {
	.s_power = gcb_s_power,
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops gcb_video_ops = {
	.g_frame_interval = gcb_g_frame_interval,
	.s_frame_interval = gcb_s_frame_interval,
	.s_stream = gcb_s_stream,
};

static const struct v4l2_subdev_pad_ops gcb_pad_ops = {
	.enum_mbus_code = gcb_enum_mbus_code,
	.get_fmt = gcb_get_fmt,
	.set_fmt = gcb_set_fmt,
	.enum_frame_size = gcb_enum_frame_size,
	.enum_frame_interval = gcb_enum_frame_interval,
};

static const struct v4l2_subdev_ops gcb_subdev_ops = {
	.core = &gcb_core_ops,
	.video = &gcb_video_ops,
	.pad = &gcb_pad_ops,
};


static int gcb_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct gcb_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	int ret;
	dev_info(dev, "Probe started!");

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;

	/*
	 * default init sequence initialize sensor to
	 * YUV422 UYVY VGA@30fps
	 */
	fmt = &sensor->fmt;
	fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = 640;
	fmt->height = 480;
	fmt->field = V4L2_FIELD_NONE;
	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = gcb_framerates[GCB_30_FPS];
	sensor->current_fr = GCB_30_FPS;
	sensor->current_mode =
		&gcb_mode_data[GCB_MODE_VGA];
	sensor->last_mode = sensor->current_mode;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev),
						  NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &sensor->ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "Could not parse endpoint\n");
		return ret;
	}

	v4l2_i2c_subdev_init(&sensor->sd, client, &gcb_subdev_ops);
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	mutex_init(&sensor->lock);

	ret = gcb_init_controls(sensor);
	if (ret)
		goto entity_cleanup;

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret)
		goto free_ctrls;

	dev_info(dev, "Probe finished successfully!");
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	return ret;
}

static int gcb_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gcb_dev *sensor = to_gcb_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);

	return 0;
}

static const struct i2c_device_id gcb_id[] = {
	{"gcb", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, gcb_id);

static const struct of_device_id gcb_dt_ids[] = {
	{ .compatible = "gemmi,gcb" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gcb_dt_ids);

static struct i2c_driver gcb_i2c_driver = {
	.driver = {
		.name  = "gcb",
		.of_match_table	= gcb_dt_ids,
	},
	.id_table = gcb_id,
	.probe    = gcb_probe,
	.remove   = gcb_remove,
};

module_i2c_driver(gcb_i2c_driver);

MODULE_DESCRIPTION("GCB Camera Subdev Driver");
MODULE_LICENSE("GPL");
