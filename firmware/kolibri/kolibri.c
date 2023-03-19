/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2023 Lone Dynamics Corporation <info@lonedynamics.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Kolibri Firmware (work-in-progress)
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <strings.h>
#include <string.h>

// Pico
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"

#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"

#include "tusb.h"

#include "pio_spi.h"
#include "spi_master_tx.pio.h"
#include "kolibri.h"

uint8_t spi_mode = SPI_MODE_HW;

void core1_main(void);

void init_ldprog(void);
void init_ldprog_done(void);
void init_gpio(void);
void init_pio_spi(void);
void init_kolibri(void);
void init_kolibri_postconf(void);
void ice40_reset(void);
void pio_spi_cfg(uint8_t pin_sck, uint8_t pin_mosi, uint8_t pin_miso);

void cdc_task(void);
void musli_task(void);
void on_uart_rx(void);

uint8_t spi_pio_offset;

bool ldprog_busy = false;
bool kolibri_fpga_configured = false;

pio_spi_inst_t spi_pio = {
	.pio = pio0,
	.sm = 1
};

#define SPI_MASTER_TX_PIO pio0
#define SPI_MASTER_TX_SM 2

void pio_spi_master_tx_write8_blocking(PIO pio, uint sm, const uint8_t *src, size_t len);
static void process_kbd_report(hid_keyboard_report_t const *report);
void spi_send_scancode(uint8_t code);

void cdc_printf(const char *fmt, ...);

void cdc_printf(const char *fmt, ...) {
	char buf[256];
	va_list args;
	va_start(args, fmt);
	vsprintf(buf, fmt, args);
	tud_cdc_write_str(buf);
	tud_cdc_write_flush();
	va_end(args);
}

void musli_task() {

	uint8_t buf[64];

	if (!tud_vendor_available()) return;

	uint32_t len = tud_vendor_read(buf, 64);

	/*
	char tmp[256];
	sprintf(tmp, "RX %d bytes from host\r\n", len);
	tud_cdc_write_str(tmp);
	sprintf(tmp, "[%x %x %x %x]\r\n", buf[0], buf[1], buf[2], buf[3]);
	tud_cdc_write_str(tmp);
	tud_cdc_write_flush();
	*/

	if (buf[0] == MUSLI_CMD_INIT) {
		cdc_printf("init %d\r\n", buf[1]);
		if (buf[1] == 0x00) init_ldprog();
		if (buf[1] == 0x01) init_gpio();
		if (buf[1] == 0x02) init_pio_spi();
		if (buf[1] == 0x03) init_ldprog_done();
	}

	if (buf[0] == MUSLI_CMD_GPIO_SET_DIR) {
		gpio_set_dir(buf[1], buf[2]);
	}

	if (buf[0] == MUSLI_CMD_GPIO_DISABLE_PULLS) {
		gpio_disable_pulls(buf[1]);
	}

	if (buf[0] == MUSLI_CMD_GPIO_PULL_UP) {
		gpio_pull_up(buf[1]);
	}

	if (buf[0] == MUSLI_CMD_GPIO_PULL_DOWN) {
		gpio_pull_down(buf[1]);
	}

	if (buf[0] == MUSLI_CMD_GPIO_GET) {
		uint8_t lbuf[64];
		bzero(lbuf, 64);
		uint8_t val = gpio_get(buf[1]);
		lbuf[0] = val;
		tud_vendor_write(lbuf, 64);
		tud_vendor_flush();
	}

	if (buf[0] == MUSLI_CMD_GPIO_PUT) {
		gpio_put(buf[1], buf[2]);
	}

	if (buf[0] == MUSLI_CMD_SPI_READ) {
		uint8_t lbuf[64];
		bzero(lbuf, 64);
		// cdc_printf("reading %d bytes from spi [mode: %d] ...\r\n", buf[1], spi_mode);
		if (spi_mode == SPI_MODE_HW)
			spi_read_blocking(spi1, 0, lbuf, buf[1]);
		else if (spi_mode == SPI_MODE_PIO)
			pio_spi_read8_blocking(&spi_pio, lbuf, buf[1]);
		tud_vendor_write(lbuf, 64);
		tud_vendor_flush();
	}

	if (buf[0] == MUSLI_CMD_SPI_WRITE) {
		// cdc_printf("writing %d bytes to spi [mode: %d] ...\r\n", buf[1], spi_mode);
		if (spi_mode == SPI_MODE_HW)
			spi_write_blocking(spi1, buf+4, buf[1]);
		else if (spi_mode == SPI_MODE_PIO)
			pio_spi_write8_blocking(&spi_pio, buf+4, buf[1]);
	}

	if (buf[0] == MUSLI_CMD_CFG_PIO_SPI) {
		pio_spi_cfg(buf[1], buf[2], buf[3]);
	}

	if (buf[0] == MUSLI_CMD_REBOOT) {
		cdc_printf("rebooting ...\r\n", buf[1]);
		watchdog_reboot(0, 0, 0);
	}

}

void init_kolibri(void) {

	spi_mode = SPI_MODE_HW;
	ldprog_busy = false;

	cdc_printf("init_kolibri\r\n");

	// these have external pull-ups
	gpio_init(ICE40_CDONE);
	gpio_disable_pulls(ICE40_CDONE);

	gpio_init(ICE40_CRESET);
	gpio_disable_pulls(ICE40_CRESET);

	ice40_reset();

}

void init_kolibri_postconf(void) {

	kolibri_fpga_configured = true;

	cdc_printf("fpga configured.\r\n");

	// release CSPI bus
   gpio_init(MUSLI_SPI_CSN_PIN);
   gpio_init(MUSLI_SPI_TX_PIN);
   gpio_init(MUSLI_SPI_RX_PIN);
   gpio_init(MUSLI_SPI_SCK_PIN);
   gpio_disable_pulls(MUSLI_SPI_CSN_PIN);
   gpio_disable_pulls(MUSLI_SPI_TX_PIN);
   gpio_disable_pulls(MUSLI_SPI_RX_PIN);
   gpio_disable_pulls(MUSLI_SPI_SCK_PIN);

}

void ice40_reset(void) {

	// release CSPI bus
   gpio_init(MUSLI_SPI_CSN_PIN);
   gpio_init(MUSLI_SPI_TX_PIN);
   gpio_init(MUSLI_SPI_RX_PIN);
   gpio_init(MUSLI_SPI_SCK_PIN);
   gpio_disable_pulls(MUSLI_SPI_CSN_PIN);
   gpio_disable_pulls(MUSLI_SPI_TX_PIN);
   gpio_disable_pulls(MUSLI_SPI_RX_PIN);
   gpio_disable_pulls(MUSLI_SPI_SCK_PIN);

	// pull CRESET low
	gpio_init(ICE40_CRESET);
	gpio_disable_pulls(ICE40_CRESET);
	gpio_set_dir(ICE40_CRESET, 1);
	gpio_put(ICE40_CRESET, 0);

	sleep_ms(100);

	// release CRESET
	gpio_init(ICE40_CRESET);
	gpio_disable_pulls(ICE40_CRESET);

	// ICE40 should now load from flash and raise CDONE ...

}

void init_ldprog(void) {

	spi_mode = SPI_MODE_HW;
	ldprog_busy = true;

	cdc_printf("init_ldprog\r\n");

   gpio_init(MUSLI_SPI_CSN_PIN);
   gpio_disable_pulls(MUSLI_SPI_CSN_PIN);
   gpio_init(MUSLI_SPI_RX_PIN);
   gpio_disable_pulls(MUSLI_SPI_RX_PIN);
   gpio_init(MUSLI_SPI_TX_PIN);
   gpio_disable_pulls(MUSLI_SPI_TX_PIN);
   gpio_init(MUSLI_SPI_SCK_PIN);
   gpio_disable_pulls(MUSLI_SPI_SCK_PIN);

	gpio_set_function(MUSLI_SPI_RX_PIN, GPIO_FUNC_SPI);
	gpio_set_function(MUSLI_SPI_SCK_PIN, GPIO_FUNC_SPI);
	gpio_set_function(MUSLI_SPI_TX_PIN, GPIO_FUNC_SPI);
	spi_init(spi1, 1000 * 1000);

	gpio_init(ICE40_CDONE);
	gpio_init(ICE40_CRESET);
	gpio_disable_pulls(ICE40_CDONE);
	gpio_disable_pulls(ICE40_CRESET);

	kolibri_fpga_configured = false;

}

void init_ldprog_done(void) {
	cdc_printf("init_ldprog_done\r\n");
	ldprog_busy = false;
}

void init_gpio(void) {
	cdc_printf("init_gpio\r\n");
	spi_deinit(spi1);
	//uart_deinit(uart0);
	for (int i = 0; i <= 3; i++) {
		gpio_init(i);
		gpio_disable_pulls(i);
	}
	for (int i = 8; i <= 11; i++) {
		gpio_init(i);
		gpio_disable_pulls(i);
	}
}

void init_pio_spi(void) {

	spi_mode = SPI_MODE_PIO;

	cdc_printf("init_pio_spi\r\n");

	for (int i = 8; i <= 11; i++) {
		gpio_init(i);
		gpio_disable_pulls(i);
	}

}

void pio_spi_cfg(uint8_t pin_sck, uint8_t pin_mosi, uint8_t pin_miso) {

	cdc_printf("pio_spi sck %d mosi %d miso %d\r\n", pin_sck, pin_mosi, pin_miso);

  	pio_spi_init(spi_pio.pio, spi_pio.sm, spi_pio_offset,
		8,       // 8 bits per SPI frame
		31.25f,  // 1 MHz @ 125 clk_sys
		false,   // CPHA = 0
		false,   // CPOL = 0
		pin_sck,
		pin_mosi,
		pin_miso
	);
}

void on_uart_rx() {
	while (uart_is_readable(uart0)) {
		uint8_t ch = uart_getc(uart0);
		tud_cdc_write_char(ch);
		tud_cdc_write_flush();
	}
}

int main(void) {

	// set the sys clock to 120mhz
	set_sys_clock_khz(120000, true);

	// init tinyusb
	tud_init(BOARD_TUD_RHPORT);

	// init debug UART
	uart_init(uart0, 115200);
	gpio_set_function(KOLIBRI_TX, GPIO_FUNC_UART);
	gpio_set_function(KOLIBRI_RX, GPIO_FUNC_UART);
	gpio_set_function(KOLIBRI_CTS, GPIO_FUNC_UART);
	gpio_set_function(KOLIBRI_RTS, GPIO_FUNC_UART);

	uart_set_hw_flow(uart0, true, true);
	uart_set_format(uart0, 8, 1, UART_PARITY_NONE);

	irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
	irq_set_enabled(UART0_IRQ, true);
	uart_set_irq_enables(uart0, true, false);

	// enable clock output
	clock_gpio_init(KOLIBRI_CLKOUT, CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_VALUE_CLK_USB, 1);

	cdc_printf("initializing fpga ...\r\n");
	init_kolibri();

//   printf("starting second core ...\n");
//   multicore_reset_core1();
//   multicore_launch_core1(core1_main);

	// spi master PIO for programming flash
	spi_pio_offset = pio_add_program(spi_pio.pio, &spi_cpha0_program);

	int kfc_prev = 0;

	while (1) {

		tight_loop_contents();

		tud_task();
		cdc_task();
		musli_task();

		int cdone = gpio_get(ICE40_CDONE);
		if (cdone && cdone != kfc_prev && !kolibri_fpga_configured && !ldprog_busy) {
			init_kolibri_postconf();
		}
		kfc_prev = cdone;


//		printf("hello!\n");
//		sleep_ms(100);

	}

	return 0;

}

void core1_main(void) {

	sleep_ms(10);

	while (true) {
	}

}

void cdc_task(void)
{
	if (tud_cdc_connected()) {

		if (tud_cdc_available()) {

			uint8_t buf[64];
			uint32_t count = tud_cdc_read(buf, sizeof(buf));

			for (uint32_t i = 0; i < count; i++) {
				uart_putc(uart0, buf[i]);
			}

			tud_cdc_write_flush();

		}

	}

}

void __time_critical_func(pio_spi_master_tx_write8_blocking)(PIO pio, uint sm, const uint8_t *src, size_t len) {
    size_t tx_remain = len;
    // Do 8 bit accesses on FIFO, so that write data is byte-replicated. This
    // gets us the left-justification for free (for MSB-first shift-out)
    io_rw_8 *txfifo = (io_rw_8 *) &pio->txf[sm];
    while (tx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(pio, sm)) {
            *txfifo = *src++;
            --tx_remain;
        }
    }
}
