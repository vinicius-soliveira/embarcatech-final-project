add_executable (final_project main.c ../inc/ssd1306/ssd1306_i2c.c)

# Add the standard include files to the build
target_include_directories(final_project PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(final_project
	pico_stdlib
        hardware_i2c
        hardware_dma
        hardware_timer
        hardware_adc
        hardware_pio
        hardware_clocks
        )

# Generate PIO header
pico_generate_pio_header(final_project ${CMAKE_CURRENT_LIST_DIR}/../inc/ws2818b.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(final_project 0)
pico_enable_stdio_usb(final_project  1) 

pico_add_extra_outputs(final_project)

