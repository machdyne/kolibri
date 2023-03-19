# Kolibri FPGA Dongle

Kolibri is a USB FPGA dongle designed by Lone Dynamics Corporation.

![Kolibri FPGA Dongle](https://github.com/machdyne/kolibri/blob/a79fe49601e184ec470370f372922697ff4b6bc8/kolibri.png)

This repo contains schematics, example firmware, gateware and documentation.

Find more information on the [Kolibri product page](https://machdyne.com/product/kolibri-fpga-dongle/).

## Blinky 

Building the blinky example requires [Yosys](https://github.com/YosysHQ/yosys), [nextpnr-ice40](https://github.com/YosysHQ/nextpnr) and [IceStorm](https://github.com/YosysHQ/icestorm).

Assuming they are installed, you can simply type `make` to build the gateware, which will be written to output/blinky.bin. You can then connect the device to your computer and use the latest version of [ldprog](https://github.com/machdyne/ldprog) to write the gateware to the device.

## Programming

The RP2040 firmware, FPGA SRAM and flash can be programmed over the USB connector.

Configure the FPGA SRAM:

```
$ ldprog -Ks blinky.bin
```

Program the flash:

```
$ ldprog -Kf blinky.bin
```

## Firmware

Kolibri ships with RP2040 [firmware](firmware) based on the [Müsli](https://github.com/machdyne/musli) firmware which allows it to communicate with [ldprog](https://github.com/machdyne/ldprog). The firmware also provides a USB CDC bridge to a UART on the FPGA (default: 115200 8N1).

The firmware is responsible for initializing the system, [configuring and outputting the system clock](https://raspberrypi.github.io/pico-sdk-doxygen/group__hardware__clocks.html#details), and either configuring the FPGA or telling the FPGA to configure itself from flash.

The system clock (CLK\_RP) is 48MHz by default.

The firmware can be updated by holding down the BOOT button, connecting the device to your computer, and then dragging and dropping a new UF2 file to the device filesystem.

The firmware can be built from source or you can use the latest `kolibri.uf2` binary from the firmware directory.

### Default RP2040 to FPGA IO mapping

| Signal | RP2040 | FPGA |
| ------ | ------ | ---- |
| RP\_GPIO0 | UART0 TX | UART RX |
| RP\_GPIO1 | UART0 RX | UART TX |
| RP\_GPIO2 | UART0 CTS | UART RTS |
| RP\_GPIO3 | UART0 RTS | UART CTS |
| RP\_GPIO4 | SPI0 RX | SPI TX |
| RP\_GPIO5 | SPI0 CS | SPI CS |
| RP\_GPIO6 | SPI0 SCK | SPI SCK |
| RP\_GPIO7 | SPI0 TX | SPI RX |

Note: SPI isn't currently used.

## SOC

[Zucker](https://github.com/machdyne/zucker) is an experimental RISC-V SOC that supports Kolibri.
