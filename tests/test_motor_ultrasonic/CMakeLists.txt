# Motor and Ultrasonic Integration Test
add_executable(test_motor_ultrasonic
        main.c
        gpio.c
        motor.c
        ultrasonic.c
        )

# Link required libraries
target_link_libraries(test_motor_ultrasonic
        pico_stdlib
        hardware_gpio
        )

# Enable USB output, disable UART output
pico_enable_stdio_usb(test_motor_ultrasonic 1)
pico_enable_stdio_uart(test_motor_ultrasonic 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(test_motor_ultrasonic)