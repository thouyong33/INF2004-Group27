add_executable(test_ir_dout
        test_ir_main.c
        ir_sensor.c
        )

target_link_libraries(test_ir_dout
        pico_stdlib
        hardware_gpio
        )

pico_enable_stdio_usb(test_ir_dout 1)
pico_enable_stdio_uart(test_ir_dout 0)

pico_add_extra_outputs(test_ir_dout)
