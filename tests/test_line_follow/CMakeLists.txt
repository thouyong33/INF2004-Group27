add_executable(test_line_follow
        test_line_follow_main.c
        ir_sensor.c
        motor.c
        )

target_link_libraries(test_line_follow
        pico_stdlib
        hardware_gpio
        )

pico_enable_stdio_usb(test_line_follow 1)
pico_enable_stdio_uart(test_line_follow 0)

pico_add_extra_outputs(test_line_follow)
