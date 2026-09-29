/*!
 * @file gpio.c
 *
 * @brief GPIO control implementation for Raspberry Pi Pico W
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2024 Your Name. All rights reserved.
 */

#include "hardware/gpio.h"
#include "gpio.h"

/*!
 * @brief Initialize a GPIO pin as input or output
 *
 * @param[in] pin        GPIO pin number
 * @param[in] b_output   True for output, false for input
 */
void
gpio_init_pin (uint8_t pin, bool b_output)
{
    gpio_init(pin);
    
    if (b_output)
    {
        gpio_set_dir(pin, GPIO_OUT);
    }
    else
    {
        gpio_set_dir(pin, GPIO_IN);
    }
}

/*!
 * @brief Set GPIO pin to logic high (3.3V)
 *
 * @param[in] pin    GPIO pin number
 */
void
gpio_set_high (uint8_t pin)
{
    gpio_put(pin, true);
}

/*!
 * @brief Set GPIO pin to logic low (0V)
 *
 * @param[in] pin    GPIO pin number
 */
void
gpio_set_low (uint8_t pin)
{
    gpio_put(pin, false);
}

/*!
 * @brief Read current state of GPIO pin
 *
 * @param[in] pin    GPIO pin number
 * @return           True if high, false if low
 */
bool
gpio_read (uint8_t pin)
{
    return gpio_get(pin);
}

/*** end of file ***/
