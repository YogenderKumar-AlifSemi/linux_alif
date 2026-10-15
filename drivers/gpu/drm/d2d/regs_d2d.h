/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 * Author: Yogender Kumar Arya <yogender.kumar@alifsemi.com>
 */

#ifndef _ALIF_D2D_REGS_H
#define _ALIF_D2D_REGS_H

#include <linux/bitops.h>

#define	D2D_MAKE_MASK(high, low)	((u32)GENMASK(high, low))

#define D2_CONTROL					0
#define  D2C_LIM1ENABLE					BIT(0)
#define  D2C_LIM2ENABLE					BIT(1)
#define  D2C_LIM3ENABLE					BIT(2)
#define  D2C_LIM4ENABLE					BIT(3)
#define  D2C_LIM5ENABLE					BIT(4)
#define  D2C_LIM6ENABLE					BIT(5)
#define  D2C_QUAD1ENABLE				BIT(6)
#define  D2C_QUAD2ENABLE				BIT(7)
#define  D2C_QUAD3ENABLE				BIT(8)
#define  D2C_LIM1THRESHOLD				BIT(9)
#define  D2C_LIM2THRESHOLD				BIT(10)
#define  D2C_LIM3THRESHOLD				BIT(11)
#define  D2C_LIM4THRESHOLD				BIT(12)
#define  D2C_LIM5THRESHOLD				BIT(13)
#define  D2C_LIM6THRESHOLD				BIT(14)
#define  D2C_BAND1ENABLE				BIT(15)
#define  D2C_BAND2ENABLE				BIT(16)
#define  D2C_UNION12					BIT(17)
#define  D2C_UNION34					BIT(18)
#define  D2C_UNION56					BIT(19)
#define  D2C_UNIONAB					BIT(20)
#define  D2C_UNIONCD					BIT(21)
#define  D2C_SPANABORT					BIT(22)
#define  D2C_SPANSTORE					BIT(23)
#define  D2C_LIMITERPRECISION				BIT(24)

#define D2_CONTROL2					1
#define  D2C_PATTERNENABLE				BIT(0)
#define  D2C_TEXTUREENABLE				BIT(1)
#define  D2C_PATTERNSOURCEL5				BIT(2)
#define  USE_ACB					BIT(3)
#define  D2C_READFORMAT3				BIT(4)
#define  D2C_READFORMAT4				BIT(5)
#define  D2C_BSFA					BIT(6)
#define  D2C_BDFA					BIT(7)
#define  D2C_WRITEFORMAT3				BIT(8)
#define  D2C_BSF					BIT(9)
#define  D2C_BDF					BIT(10)
#define  D2C_BSI					BIT(11)
#define  D2C_BDI					BIT(12)
#define  D2C_BC2					BIT(13)
#define  D2C_TEXTURECLAMPX				BIT(14)
#define  D2C_TEXTURECLAMPY				BIT(15)
#define  D2C_TEXTUREFILTERX				BIT(16)
#define  D2C_TEXTUREFILTERY				BIT(17)
#define  D2C_READFORMAT1				BIT(18)
#define  D2C_READFORMAT2				BIT(19)
#define  D2C_WRITEFORMAT1				BIT(20)
#define  D2C_WRITEFORMAT2				BIT(21)
#define  D2C_WRITEALPHA1				BIT(22)
#define  D2C_WRITEALPHA2				BIT(23)
#define  D2C_RLE_ENABLE					BIT(24)
#define  D2C_CLUT_ENABLE				BIT(25)
#define  D2C_COLKEY_ENABLE				BIT(26)
#define  D2C_CLUTFORMAT1				BIT(27)
#define  D2C_BSIA					BIT(28)
#define  D2C_BDIA					BIT(29)
#define  D2C_RLEFORMAT1					BIT(30)
#define  D2C_RLEFORMAT2					BIT(31)

#define D2_L1START					4
#define D2_L2START					5
#define D2_L3START					6
#define D2_L4START					7
#define D2_L5START					8
#define D2_L6START					9
#define D2_L1XADD					10
#define D2_L2XADD					11
#define D2_L3XADD					12
#define D2_L4XADD					13
#define D2_L5XADD					14
#define D2_L6XADD					15
#define D2_L1YADD					16
#define D2_L2YADD					17
#define D2_L3YADD					18
#define D2_L4YADD					19
#define D2_L5YADD					20
#define D2_L6YADD					21
#define D2_L1BAND					22
#define D2_L2BAND					23

#define D2_COLOR1					25
#define  D2_COLOR1_ALPHA_MASK				D2D_MAKE_MASK(31, 24)
#define  D2_COLOR1_ALPHA_SHIFT				24
#define  D2_COLOR1_RED_MASK				D2D_MAKE_MASK(23, 16)
#define  D2_COLOR1_RED_SHIFT				16
#define  D2_COLOR1_GREEN_MASK				D2D_MAKE_MASK(15, 8)
#define  D2_COLOR1_GREEN_SHIFT				8
#define  D2_COLOR1_BLUE_MASK				D2D_MAKE_MASK(7, 0)
#define  D2_COLOR1_BLUE_SHIFT				0

#define D2_COLOR2					26
#define  D2_COLOR2_ALPHA_MASK				D2D_MAKE_MASK(31, 24)
#define  D2_COLOR2_ALPHA_SHIFT				24
#define  D2_COLOR2_RED_MASK				D2D_MAKE_MASK(23, 16)
#define  D2_COLOR2_RED_SHIFT				16
#define  D2_COLOR2_GREEN_MASK				D2D_MAKE_MASK(15, 8)
#define  D2_COLOR2_GREEN_SHIFT				8
#define  D2_COLOR2_BLUE_MASK				D2D_MAKE_MASK(7, 0)
#define  D2_COLOR2_BLUE_SHIFT				0

#define D2_PATTERN					29
#define D2_SIZE						30
#define D2_PITCH					31
#define D2_ORIGIN					32
#define D2_LUSTART					36
#define D2_LUXADD					37
#define D2_LUYADD					38
#define D2_LVSTARTI					39
#define D2_LVSTARTF					40
#define D2_LVXADDI					41
#define D2_LVYADDI					42
#define D2_LVYXADDF					43
#define D2_TEXPITCH					45

#define D2_TEXMASK					46
#define  D2_TEXMASK_TEXUMASK_MASK			D2D_MAKE_MASK(31, 11)
#define  D2_TEXMASK_TEXUMASK_SHIFT			11
#define  D2_TEXMASK_TEXVMASK_MASK			D2D_MAKE_MASK(10, 0)
#define  D2_TEXMASK_TEXVMASK_SHIFT			0

#define D2_TEXORIGIN					47

#define D2_IRQCTL					48
#define  D2IRQCTL_ENABLE_FINISH_ENUM			BIT(0)
#define  D2IRQCTL_ENABLE_FINISH_DLIST			BIT(1)
#define  D2IRQCTL_CLR_FINISH_ENUM			BIT(2)
#define  D2IRQCTL_CLR_FINISH_DLIST			BIT(3)
#define  D2IRQCTL_ENABLE_BUS_ERROR			BIT(4)
#define  D2IRQCTL_CLR_BUS_ERROR				BIT(5)

#define D2_CACHECTL					49
#define  D2C_CACHECTL_ENABLE_FB				BIT(0)
#define  D2C_CACHECTL_FLUSH_FB				BIT(1)
#define  D2C_CACHECTL_ENABLE_TX				BIT(2)
#define  D2C_CACHECTL_FLUSH_TX				BIT(3)

#define D2_DLISTSTART					50
/* Packed dlist special indices (TRM 4.4.2.3). */
#define  D2_DLIST_PAD					0x80
#define  D2_DLIST_SPECIAL				0xff
#define  D2_DLIST_END					BIT(0)
#define  D2_DLIST_FLUSH					BIT(1)
#define  D2_DLIST_WAIT_FB				BIT(2)
#define D2_PERFCOUNT1					51
#define D2_PERFCOUNT2					52

#define D2_PERFTRIGGER					53
#define  D2_PERFTRIGGER_1_MASK				D2D_MAKE_MASK(15, 0)
#define  D2_PERFTRIGGER_1_SHIFT				0
#define  D2_PERFTRIGGER_2_MASK				D2D_MAKE_MASK(31, 16)
#define  D2_PERFTRIGGER_2_SHIFT				16
/* Types of Perf triggers present. */
enum{
	D2PC_NONE            = 0,
	D2PC_DAVECYCLES      = 1,
	D2PC_FBREADS         = 2,
	D2PC_FBWRITES        = 3,
	D2PC_TXREADS         = 4,
	D2PC_INVPIXELS       = 5,
	D2PC_INVPIXELS_MISS  = 6,
	D2PC_DLRCYCLES       = 7,
	D2PC_FBREADHITS      = 8,
	D2PC_FBREADMISSES    = 9,
	D2PC_FBWRITEHITS     = 10,
	D2PC_FBWRITEMISSES   = 11,
	D2PC_TEXREADHITS     = 12,
	D2PC_TEXREADMISSES   = 13,
	D2_PC_DLRBURSTREADS  = 17,
	D2_PC_DLRWORDSREAD   = 18,
	D2PC_RLEREWINDS      = 20,
	D2_PC_TEXBURSTREADS  = 21,
	D2_PC_TEXWORDSREAD   = 22,
	D2_PC_FBBURSTREADS   = 23,
	D2_PC_FBWORDSREAD    = 24,
	D2_PC_FBBURSTWRITES  = 25,
	D2_PC_FBWORDSWRITTEN = 26,
	D2PC_CLKCYCLES       = 31,
};

#define D2_TEXCLUT					54
#define  D2_TEXCLUT_CLUT_INDEX_MASK			D2D_MAKE_MASK(31, 24)
#define  D2_TEXCLUT_CLUT_INDEX_SHIFT			24
#define  D2_TEXCLUT_CLUT_ENTRY_MASK			D2D_MAKE_MASK(23, 0)
#define  D2_TEXCLUT_CLUT_ENTRY_SHIFT			0

#define D2_TEXCLUT_ADDR					55
#define  D2_TEXCLUT_ADDR_CLUT_ADDR_MASK			D2D_MAKE_MASK(7, 0)
#define  D2_TEXCLUT_ADDR_CLUT_ADDR_SHIFT		0

#define D2_TEXCLUT_DATA					56

#define D2_TEXCLUT_OFFSET				57
#define  D2_TEXCLUT_OFFSET_CLUT_OFFSET_MASK		D2D_MAKE_MASK(7, 0)
#define  D2_TEXCLUT_OFFSET_CLUT_OFFSET_SHIFT		0

#define D2_COLKEY					58
#define  D2_COLKEY_COLORKEY_MASK			D2D_MAKE_MASK(23, 0)
#define  D2_COLKEY_COLORKEY_SHIFT			0

/* Read only status control registers  */
#define D2_STATUS					0
#define  D2C_BUSY_ENUM					BIT(0)
#define  D2C_BUSY_WRITE					BIT(1)
#define  D2C_CACHE_DIRTY				BIT(2)
#define  D2C_DLISTACTIVE				BIT(3)
#define  D2C_IRQ_ENUM					BIT(4)
#define  D2C_IRQ_DLIST					BIT(5)
#define  D2C_IRQ_BUS_ERROR				BIT(6)
#define  D2C_IRQ_BUS_ERROR_SRC_MASK			D2D_MAKE_MASK(10, 8)
#define  D2C_IRQ_BUS_ERROR_SRC_SHIFT			8
/* Types of Bus errors. */
enum{
	D2C_IRQ_BUS_ERROR_SRC_MFB = 1,
	D2C_IRQ_BUS_ERROR_SRC_MTX = 2,
	D2C_IRQ_BUS_ERROR_SRC_MDL = 4,
};

#define D2_HWREVISION					1
#define  D2_HWREVISION_HWREVISION_MASK			D2D_MAKE_MASK(7, 0)
#define  D2_HWREVISION_HWREVISION_SHIFT			0
#define  D2_HWREVISION_HWBRANCH_MASK			D2D_MAKE_MASK(11, 8)
#define  D2_HWREVISION_HWBRANCH_SHIFT			8
#define  D2_HWREVISION_HWTYPE_MASK			D2D_MAKE_MASK(15, 12)
#define  D2_HWREVISION_HWTYPE_SHIFT			12
#define  D2_HWREVISION_HWTYPE_STANDARD			0       // D/AVE2DT-S
#define  D2_HWREVISION_HWTYPE_LIGHT			1       // D/AVE2DT-L
#define  D2_HWREVISION_FEATURES_MASK			D2D_MAKE_MASK(28, 16)
#define  D2_HWREVISION_FEATURES_SHIFT			16
#define  D2_HWREVISION_FEATURES_SWDAVE			BIT(16)
#define  D2_HWREVISION_FEATURES_DLR			BIT(17)
#define  D2_HWREVISION_FEATURES_FB_CACHE		BIT(18)
#define  D2_HWREVISION_FEATURES_TX_CACHE		BIT(19)
#define  D2_HWREVISION_FEATURES_PERFCOUNT		BIT(20)
#define  D2_HWREVISION_FEATURES_TEXCLUT			BIT(21)
#define  D2_HWREVISION_FEATURES_FBPREFETCH		BIT(22)
#define  D2_HWREVISION_FEATURES_RLEUNIT			BIT(23)
#define  D2_HWREVISION_FEATURES_TEXCLUT256		BIT(24)
#define  D2_HWREVISION_FEATURES_COLORKEY		BIT(25)
#define  D2_HWREVISION_FEATURES_HILIMITERPRECISION	BIT(26)
#define  D2_HWREVISION_FEATURES_ALPHACHANNELBLENDING	BIT(27)
#define  D2_HWREVISION_FEATURES_BURSTSPLITTING		BIT(28)

#define D2_MAXREGISTER	D2_COLKEY
#endif /* _ALIF_D2D_REGS_H */
