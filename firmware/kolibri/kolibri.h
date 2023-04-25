/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef KOLIBRI_H_
#define KOLIBRI_H_

#define MUSLI_SPI_RX_PIN   12    // CSPI_SO
#define MUSLI_SPI_CSN_PIN  13
#define MUSLI_SPI_SCK_PIN  14
#define MUSLI_SPI_TX_PIN   11    // CSPI_SI

#define MUSLI_CMD_READY 0x00
#define MUSLI_CMD_INIT 0x01

#define MUSLI_CMD_GPIO_SET_DIR 0x10
#define MUSLI_CMD_GPIO_DISABLE_PULLS 0x11
#define MUSLI_CMD_GPIO_PULL_UP 0x12
#define MUSLI_CMD_GPIO_PULL_DOWN 0x13

#define MUSLI_CMD_GPIO_GET 0x20
#define MUSLI_CMD_GPIO_PUT 0x21

#define MUSLI_CMD_SPI_READ 0x80
#define MUSLI_CMD_SPI_WRITE 0x81
#define MUSLI_CMD_CFG_PIO_SPI 0x8f

#define MUSLI_CMD_REBOOT 0xf0

// pins

#define ICE40_CDONE 10
#define ICE40_CRESET 15

#define KOLIBRI_CLKOUT 23

#define KOLIBRI_TX 0
#define KOLIBRI_RX 1
#define KOLIBRI_CTS 2
#define KOLIBRI_RTS 3

#define SPI_MODE_HW 1
#define SPI_MODE_PIO 2

#endif
