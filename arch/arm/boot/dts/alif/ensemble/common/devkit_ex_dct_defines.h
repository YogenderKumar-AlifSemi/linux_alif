/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2025 Alif Semiconductor.
 * Author: Jagadeesh Krishnanjanappa <jagadeesh@alifsemi.com>
 */

#ifndef __DEVKIT_DCT_H__
#define __DEVKIT_DCT_H__

/* HWSEM defines */
#define HWSEM0_STATUS "disabled"
#define HWSEM1_STATUS "disabled"
#define HWSEM2_STATUS "disabled"
#define HWSEM3_STATUS "disabled"
#define HWSEM4_STATUS "disabled"
#define HWSEM5_STATUS "disabled"
#define HWSEM6_STATUS "disabled"
#define HWSEM7_STATUS "disabled"
#define HWSEM8_STATUS "disabled"
#define HWSEM9_STATUS "disabled"
#define HWSEM10_STATUS "disabled"
#define HWSEM11_STATUS "disabled"
#define HWSEM12_STATUS "disabled"
#define HWSEM13_STATUS "disabled"
#define HWSEM14_STATUS "disabled"
#define HWSEM15_STATUS "disabled"

/* MHU defines */
#define M55_HP_MHU0_TX_STATUS "okay"
#define M55_HP_MHU1_TX_STATUS "okay"
#define M55_HP_MHU0_RX_STATUS "okay"
#define M55_HP_MHU1_RX_STATUS "okay"
#define M55_HE_MHU0_TX_STATUS "okay"
#define M55_HE_MHU1_TX_STATUS "okay"
#define M55_HE_MHU0_RX_STATUS "okay"
#define M55_HE_MHU1_RX_STATUS "okay"
#define SE_MHU0_TX_STATUS "okay"
#define SE_MHU1_TX_STATUS "okay"
#define SE_MHU0_RX_STATUS "okay"
#define SE_MHU1_RX_STATUS "okay"

/* Ethernet defines */
#define ETH_STATUS "disabled"

/* SDMMC defines */
#define SDHCI_STATUS "disabled"

/* SPI defines */
#define SPI0_STATUS "disabled"
#define SPI1_STATUS "disabled"
#define SPI2_STATUS "disabled"
#define SPI3_STATUS "okay"

/* UART defines */
#define UART0_STATUS "disabled"
#define UART1_STATUS "disabled"
#define UART2_STATUS "okay"
#define UART3_STATUS "disabled"
#define UART4_STATUS "disabled"
#define UART5_STATUS "disabled"
#define UART6_STATUS "disabled"
#define UART7_STATUS "disabled"

/* USB defines */
#define HSUSB_STATUS "disabled"

/* UTIMER defines */
#define UTIMER0_STATUS "disabled"

/* DSI defines */
#define DSI_STATUS "disabled"

/* DPI defines */
#define DPI_STATUS "disabled"

/*
 * Display interface for the CDC200 output path.
 * Yocto/DCT must select exactly one:
 *   0 = MIPI DSI (serial_panel.dtsi)
 *   1 = parallel RGB / DPI (parallel_panel.dtsi)
 *
 * Do not include both fragments in one DTB: they share CDC port@0 and
 * the second overlay overwrites remote-endpoint (dtc graph_endpoint
 * warning) and can apply RGB pinctrl on a DSI build.
 */
#define CDC200_PARALLEL 0

/* CSI defines */
#define CSI_STATUS "disabled"

/* CPI defines */
#define CPI_STATUS "disabled"

/* ADC defines */
#define ADC120_STATUS "disabled"
#define ADC121_STATUS "disabled"
#define ADC122_STATUS "disabled"
#define ADC24_STATUS  "disabled"

/* CMP defines */
#define CMP0_STATUS "disabled"
#define CMP1_STATUS "disabled"
#define CMP2_STATUS "disabled"
#define CMP3_STATUS "disabled"

/* DAC defines */
#define DAC120_STATUS "disabled"
#define DAC121_STATUS "disabled"

/* I2C Defines */
#define I2C0_STATUS "disabled"
#define I2C1_STATUS "disabled"
#define I2C2_STATUS "disabled"
#define I2C3_STATUS "disabled"

/* I3C Defines */
#define I3C0_STATUS "disabled"

/* I2S Defines */
#define I2S0_STATUS "disabled"
#define I2S1_STATUS "disabled"
#define I2S2_STATUS "disabled"
#define I2S3_STATUS "disabled"

/* CDC200 Defines */
#define CDC200_STATUS "disabled"

/* D2D Defines */
#define DAVE_2D_STATUS "disabled"

/* PDM Defines */
#define PDM_STATUS "disabled"

/* CPU Defines */
#define CPU1_STATUS "okay"

/* CPUFreq Defines */
#define CPUFREQ_STATUS "disabled"

/* Memory Defines */
#define MEM_STITCH_STATUS "okay"
#define MEM_HYPER_STATUS  "disabled"
#define MEM_HYP_STITCH_STATUS "disabled"

/* Ethosu NPU */
#define ETHOSU_NPU_STATUS "disabled"
#endif /* __DEVKIT_DCT_H__ */
