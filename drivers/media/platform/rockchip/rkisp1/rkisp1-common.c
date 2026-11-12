// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Rockchip ISP1 Driver - Common definitions
 *
 * Copyright (C) 2019 Collabora, Ltd.
 */

#include <media/mipi-csi2.h>
#include <media/v4l2-rect.h>

#include "rkisp1-common.h"

unsigned int rkisp1_map_reg(const struct rkisp1_device *rkisp1,
			    unsigned int addr)
{
	if (!rkisp1_is_pico(rkisp1))
		return addr;

	/*
	 * VI block through IRCL. Pico has no register at the Rockchip
	 * VI_DPCL offset; a read there aborts the bus.
	 */
	if (addr < RKISP1_CIF_VI_DPCL)
		return addr;
	if (addr == RKISP1_CIF_VI_DPCL)
		return RKISP1_REG_NONE;
	if (addr >= RKISP1_CIF_ISP_CTRL && addr <= RKISP1_CIF_ISP_ACQ_V_SIZE)
		return addr;
	if (addr == RKISP1_CIF_ISP_ACQ_NR_FRAMES)
		return RKISP1_REG_NONE;

	/* Resizers keep the V10 bases. */
	if (addr >= RKISP1_CIF_MRSZ_BASE && addr < RKISP1_CIF_MRSZ_BASE + 0x80)
		return addr;
	if (addr >= RKISP1_CIF_SRSZ_BASE && addr < RKISP1_CIF_SRSZ_BASE + 0x80)
		return addr;

	/*
	 * MI moves from 0x1400 to 0x0e00. Pico has no self-path block
	 * after the main-path offset registers, none between the MP
	 * shadow block and the MI IRQ block, and none between
	 * MP_CR_BASE_AD_INIT2 and Y_LLENGTH (that gap includes
	 * MI_XTD_FORMAT_CTRL).
	 */
	if (addr >= RKISP1_CIF_MI_BASE && addr < RKISP1_CIF_MI_BASE + 0x180) {
		if (addr >= RKISP1_CIF_MI_SP_Y_BASE_AD_INIT &&
		    addr <= RKISP1_CIF_MI_SP_CR_OFFS_CNT_START)
			return RKISP1_REG_NONE;
		if (addr > RKISP1_CIF_MI_MP_CR_OFFS_CNT_SHD &&
		    addr < RKISP1_CIF_MI_IMSC)
			return RKISP1_REG_NONE;
		if (addr >= RKISP1_CIF_MI_SP_Y_BASE_AD_INIT2 &&
		    addr < RKISP1_CIF_MI_MP_Y_LLENGTH)
			return RKISP1_REG_NONE;
		if (addr == RKISP1_CIF_MI_MP_Y_SLICE_OFFSET ||
		    addr == RKISP1_CIF_MI_MP_C_SLICE_OFFSET)
			return RKISP1_REG_NONE;
		return RKISP1_PICO_MI_BASE + (addr - RKISP1_CIF_MI_BASE);
	}

	/* ISP IRQ block */
	if (addr >= RKISP1_CIF_ISP_IMSC && addr <= RKISP1_CIF_ISP_ISR)
		return RKISP1_PICO_ISP_IMSC + (addr - RKISP1_CIF_ISP_IMSC);
	if (addr == RKISP1_CIF_ISP_ERR)
		return RKISP1_PICO_ISP_ERR;
	if (addr == RKISP1_CIF_ISP_ERR_CLR)
		return RKISP1_PICO_ISP_ERR + 4;

	/* Output window (Pico has no IS block). */
	if (addr >= RKISP1_CIF_ISP_OUT_H_OFFS && addr <= RKISP1_CIF_ISP_OUT_V_SIZE)
		return RKISP1_PICO_ISP_OUT_H_OFFS +
		       (addr - RKISP1_CIF_ISP_OUT_H_OFFS);
	if (addr >= RKISP1_CIF_ISP_OUT_H_OFFS_SHD &&
	    addr <= RKISP1_CIF_ISP_OUT_V_SIZE_SHD)
		return RKISP1_PICO_ISP_OUT_H_OFFS_SHD +
		       (addr - RKISP1_CIF_ISP_OUT_H_OFFS_SHD);

	if (addr == RKISP1_CIF_ISP_DEMOSAIC)
		return RKISP1_PICO_ISP_DEMOSAIC;

	if (addr >= RKISP1_CIF_ISP_CC_COEFF_0 && addr <= RKISP1_CIF_ISP_CC_COEFF_8)
		return RKISP1_PICO_ISP_CC_COEFF_0 +
		       (addr - RKISP1_CIF_ISP_CC_COEFF_0);
	if (addr >= RKISP1_CIF_ISP_CT_COEFF_0 && addr <= RKISP1_CIF_ISP_CT_COEFF_8)
		return RKISP1_PICO_ISP_CT_COEFF_0 +
		       (addr - RKISP1_CIF_ISP_CT_COEFF_0);
	if (addr >= RKISP1_CIF_ISP_CT_OFFSET_R &&
	    addr <= RKISP1_CIF_ISP_CT_OFFSET_B)
		return RKISP1_PICO_ISP_CT_OFFSET_R +
		       (addr - RKISP1_CIF_ISP_CT_OFFSET_R);

	if (addr >= RKISP1_CIF_ISP_GAMMA_OUT_MODE_V10 &&
	    addr <= RKISP1_CIF_ISP_GAMMA_OUT_Y_16_V10)
		return RKISP1_PICO_ISP_GAMMA_OUT_MODE +
		       (addr - RKISP1_CIF_ISP_GAMMA_OUT_MODE_V10);

	if (addr >= RKISP1_CIF_ISP_AWB_PROP_V10 &&
	    addr <= RKISP1_CIF_ISP_AWB_THRESH_V10)
		return RKISP1_PICO_ISP_AWB_PROP +
		       (addr - RKISP1_CIF_ISP_AWB_PROP_V10);
	if (addr == RKISP1_CIF_ISP_AWB_GAIN_G_V10)
		return RKISP1_PICO_ISP_DGAIN_G;
	if (addr == RKISP1_CIF_ISP_AWB_GAIN_RB_V10)
		return RKISP1_PICO_ISP_DGAIN_RB;
	if (addr == RKISP1_CIF_ISP_AWB_WHITE_CNT_V10)
		return RKISP1_PICO_ISP_AWB_WHITE_CNT;
	if (addr == RKISP1_CIF_ISP_AWB_MEAN_V10)
		return RKISP1_PICO_ISP_AWB_MEAN;

	if (addr == RKISP1_CIF_ISP_BLS_CTRL)
		return RKISP1_PICO_ISP_BLS_CTRL;
	if (addr >= RKISP1_CIF_ISP_BLS_A_FIXED && addr <= RKISP1_CIF_ISP_BLS_D_FIXED)
		return RKISP1_PICO_ISP_BLS_A_FIXED +
		       (addr - RKISP1_CIF_ISP_BLS_A_FIXED);

	if (addr == RKISP1_CIF_ISP_FILT_MODE)
		return RKISP1_PICO_ISP_FILT_MODE;
	if (addr >= RKISP1_CIF_ISP_FILT_THRESH_BL0 &&
	    addr <= RKISP1_CIF_ISP_FILT_FAC_BL1)
		return RKISP1_PICO_ISP_FILT_THRESH_BL0 +
		       (addr - RKISP1_CIF_ISP_FILT_THRESH_BL0);

	if (addr >= RKISP1_CIF_ISP_EXP_CTRL &&
	    addr <= RKISP1_CIF_ISP_EXP_MEAN_44_V10)
		return RKISP1_PICO_ISP_EXP_CTRL + (addr - RKISP1_CIF_ISP_EXP_CTRL);

	if (addr >= RKISP1_CIF_ISP_CAC_CTRL && addr <= RKISP1_CIF_ISP_Y_NORM)
		return RKISP1_PICO_ISP_CAC_CTRL + (addr - RKISP1_CIF_ISP_CAC_CTRL);

	/* LSC, hist, AF, DPF, dual-crop, IS, MIPI CSI: not on Pico. */
	return RKISP1_REG_NONE;
}

static const struct rkisp1_mbus_info rkisp1_formats[] = {
	{
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8,
		.pixel_enc	= V4L2_PIXEL_ENC_YUV,
		.direction	= RKISP1_ISP_SD_SRC,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB10_1X10,
		.pixel_enc	= V4L2_PIXEL_ENC_BAYER,
		.mipi_dt	= MIPI_CSI2_DT_RAW10,
		.bayer_pat	= RKISP1_RAW_RGGB,
		.bus_width	= 10,
		.direction	= RKISP1_ISP_SD_SINK | RKISP1_ISP_SD_SRC,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.pixel_enc	= V4L2_PIXEL_ENC_BAYER,
		.mipi_dt	= MIPI_CSI2_DT_RAW10,
		.bayer_pat	= RKISP1_RAW_BGGR,
		.bus_width	= 10,
		.direction	= RKISP1_ISP_SD_SINK | RKISP1_ISP_SD_SRC,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG10_1X10,
		.pixel_enc	= V4L2_PIXEL_ENC_BAYER,
		.mipi_dt	= MIPI_CSI2_DT_RAW10,
		.bayer_pat	= RKISP1_RAW_GBRG,
		.bus_width	= 10,
		.direction	= RKISP1_ISP_SD_SINK | RKISP1_ISP_SD_SRC,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG10_1X10,
		.pixel_enc	= V4L2_PIXEL_ENC_BAYER,
		.mipi_dt	= MIPI_CSI2_DT_RAW10,
		.bayer_pat	= RKISP1_RAW_GRBG,
		.bus_width	= 10,
		.direction	= RKISP1_ISP_SD_SINK | RKISP1_ISP_SD_SRC,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB12_1X12,
		.pixel_enc	= V4L2_PIXEL_ENC_BAYER,
		.mipi_dt	= MIPI_CSI2_DT_RAW12,
		.bayer_pat	= RKISP1_RAW_RGGB,
		.bus_width	= 12,
		.direction	= RKISP1_ISP_SD_SINK | RKISP1_ISP_SD_SRC,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR12_1X12,
		.pixel_enc	= V4L2_PIXEL_ENC_BAYER,
		.mipi_dt	= MIPI_CSI2_DT_RAW12,
		.bayer_pat	= RKISP1_RAW_BGGR,
		.bus_width	= 12,
		.direction	= RKISP1_ISP_SD_SINK | RKISP1_ISP_SD_SRC,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG12_1X12,
		.pixel_enc	= V4L2_PIXEL_ENC_BAYER,
		.mipi_dt	= MIPI_CSI2_DT_RAW12,
		.bayer_pat	= RKISP1_RAW_GBRG,
		.bus_width	= 12,
		.direction	= RKISP1_ISP_SD_SINK | RKISP1_ISP_SD_SRC,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG12_1X12,
		.pixel_enc	= V4L2_PIXEL_ENC_BAYER,
		.mipi_dt	= MIPI_CSI2_DT_RAW12,
		.bayer_pat	= RKISP1_RAW_GRBG,
		.bus_width	= 12,
		.direction	= RKISP1_ISP_SD_SINK | RKISP1_ISP_SD_SRC,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB8_1X8,
		.pixel_enc	= V4L2_PIXEL_ENC_BAYER,
		.mipi_dt	= MIPI_CSI2_DT_RAW8,
		.bayer_pat	= RKISP1_RAW_RGGB,
		.bus_width	= 8,
		.direction	= RKISP1_ISP_SD_SINK | RKISP1_ISP_SD_SRC,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR8_1X8,
		.pixel_enc	= V4L2_PIXEL_ENC_BAYER,
		.mipi_dt	= MIPI_CSI2_DT_RAW8,
		.bayer_pat	= RKISP1_RAW_BGGR,
		.bus_width	= 8,
		.direction	= RKISP1_ISP_SD_SINK | RKISP1_ISP_SD_SRC,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG8_1X8,
		.pixel_enc	= V4L2_PIXEL_ENC_BAYER,
		.mipi_dt	= MIPI_CSI2_DT_RAW8,
		.bayer_pat	= RKISP1_RAW_GBRG,
		.bus_width	= 8,
		.direction	= RKISP1_ISP_SD_SINK | RKISP1_ISP_SD_SRC,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG8_1X8,
		.pixel_enc	= V4L2_PIXEL_ENC_BAYER,
		.mipi_dt	= MIPI_CSI2_DT_RAW8,
		.bayer_pat	= RKISP1_RAW_GRBG,
		.bus_width	= 8,
		.direction	= RKISP1_ISP_SD_SINK | RKISP1_ISP_SD_SRC,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_1X16,
		.pixel_enc	= V4L2_PIXEL_ENC_YUV,
		.mipi_dt	= MIPI_CSI2_DT_YUV422_8B,
		.yuv_seq	= RKISP1_CIF_ISP_ACQ_PROP_YCBYCR,
		.bus_width	= 16,
		.direction	= RKISP1_ISP_SD_SINK,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YVYU8_1X16,
		.pixel_enc	= V4L2_PIXEL_ENC_YUV,
		.mipi_dt	= MIPI_CSI2_DT_YUV422_8B,
		.yuv_seq	= RKISP1_CIF_ISP_ACQ_PROP_YCRYCB,
		.bus_width	= 16,
		.direction	= RKISP1_ISP_SD_SINK,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_1X16,
		.pixel_enc	= V4L2_PIXEL_ENC_YUV,
		.mipi_dt	= MIPI_CSI2_DT_YUV422_8B,
		.yuv_seq	= RKISP1_CIF_ISP_ACQ_PROP_CBYCRY,
		.bus_width	= 16,
		.direction	= RKISP1_ISP_SD_SINK,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_VYUY8_1X16,
		.pixel_enc	= V4L2_PIXEL_ENC_YUV,
		.mipi_dt	= MIPI_CSI2_DT_YUV422_8B,
		.yuv_seq	= RKISP1_CIF_ISP_ACQ_PROP_CRYCBY,
		.bus_width	= 16,
		.direction	= RKISP1_ISP_SD_SINK,
	},
};

const struct rkisp1_mbus_info *rkisp1_mbus_info_get_by_index(unsigned int index)
{
	if (index >= ARRAY_SIZE(rkisp1_formats))
		return NULL;

	return &rkisp1_formats[index];
}

const struct rkisp1_mbus_info *rkisp1_mbus_info_get_by_code(u32 mbus_code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rkisp1_formats); i++) {
		const struct rkisp1_mbus_info *fmt = &rkisp1_formats[i];

		if (fmt->mbus_code == mbus_code)
			return fmt;
	}

	return NULL;
}

static const struct v4l2_rect rkisp1_sd_min_crop = {
	.width = RKISP1_ISP_MIN_WIDTH,
	.height = RKISP1_ISP_MIN_HEIGHT,
	.top = 0,
	.left = 0,
};

void rkisp1_sd_adjust_crop_rect(struct v4l2_rect *crop,
				const struct v4l2_rect *bounds)
{
	v4l2_rect_set_min_size(crop, &rkisp1_sd_min_crop);
	v4l2_rect_map_inside(crop, bounds);
}

void rkisp1_sd_adjust_crop(struct v4l2_rect *crop,
			   const struct v4l2_mbus_framefmt *bounds)
{
	struct v4l2_rect crop_bounds = {
		.left = 0,
		.top = 0,
		.width = bounds->width,
		.height = bounds->height,
	};

	rkisp1_sd_adjust_crop_rect(crop, &crop_bounds);
}

void rkisp1_bls_swap_regs(enum rkisp1_fmt_raw_pat_type pattern,
			  const u32 input[4], u32 output[4])
{
	static const unsigned int swap[4][4] = {
		[RKISP1_RAW_RGGB] = { 0, 1, 2, 3 },
		[RKISP1_RAW_GRBG] = { 1, 0, 3, 2 },
		[RKISP1_RAW_GBRG] = { 2, 3, 0, 1 },
		[RKISP1_RAW_BGGR] = { 3, 2, 1, 0 },
	};

	for (unsigned int i = 0; i < 4; ++i)
		output[i] = input[swap[pattern][i]];
}
