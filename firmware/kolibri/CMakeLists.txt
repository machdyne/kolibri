cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(kolibri)

add_executable(kolibri
        kolibri.c
        )

target_sources(kolibri PUBLIC
	${CMAKE_CURRENT_LIST_DIR}/kolibri.c
	${CMAKE_CURRENT_LIST_DIR}/pio_spi.c
	${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
	)

target_include_directories(kolibri PUBLIC
   ${CMAKE_CURRENT_LIST_DIR}
   )

pico_sdk_init()

pico_generate_pio_header(kolibri ${CMAKE_CURRENT_LIST_DIR}/spi.pio)
pico_generate_pio_header(kolibri ${CMAKE_CURRENT_LIST_DIR}/spi_master_tx.pio)

target_link_libraries(kolibri PRIVATE pico_stdlib hardware_resets hardware_uart hardware_irq hardware_spi hardware_pio pico_multicore hardware_dma tinyusb_device)

pico_add_extra_outputs(kolibri)
