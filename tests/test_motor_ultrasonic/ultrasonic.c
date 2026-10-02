/*!
 * @file ultrasonic.c
 *
 * @brief HC-SR04 ultrasonic sensor implementation
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2024 Your Name. All rights reserved.
 */

#include "pico/stdlib.h"
#include "ultrasonic.h"
#include "gpio.h"

/*!
 * @brief Initialize ultrasonic sensor GPIO pins
 */
void
ultrasonic_init (void)
{
    gpio_init_pin(ULTRASONIC_TRIG_PIN, true);   // Trigger as output
    gpio_init_pin(ULTRASONIC_ECHO_PIN, false);  // Echo as input
    
    // Ensure trigger starts low
    gpio_set_low(ULTRASONIC_TRIG_PIN);
}

/*!
 * @brief Measure distance using HC-SR04 sensor
 *
 * @return Distance in centimeters (0 if measurement failed)
 *
 * @par
 * NOTE: Speed of sound assumed to be 343 m/s at 20°C
 * Distance formula: distance_cm = (pulse_duration_us / 58)
 */
uint16_t
ultrasonic_get_distance_cm (void)
{
    uint32_t pulse_start_us = 0u;
    uint32_t pulse_end_us   = 0u;
    uint32_t pulse_width_us = 0u;
    uint32_t timeout_start  = 0u;
    uint16_t distance_cm    = 0u;
    bool     b_timeout      = false;
    
    // Ensure trigger is low
    gpio_set_low(ULTRASONIC_TRIG_PIN);
    sleep_us(2u);
    
    // Send 10us trigger pulse
    gpio_set_high(ULTRASONIC_TRIG_PIN);
    sleep_us(10u);
    gpio_set_low(ULTRASONIC_TRIG_PIN);
    
    // Wait for echo pin to go HIGH (start of pulse)
    timeout_start = time_us_32();
    while (false == gpio_read(ULTRASONIC_ECHO_PIN))
    {
        if ((time_us_32() - timeout_start) > ULTRASONIC_TIMEOUT_US)
        {
            b_timeout = true;
            break;
        }
    }
    
    if (false == b_timeout)
    {
        pulse_start_us = time_us_32();
        
        // Wait for echo pin to go LOW (end of pulse)
        timeout_start = time_us_32();
        while (true == gpio_read(ULTRASONIC_ECHO_PIN))
        {
            if ((time_us_32() - timeout_start) > ULTRASONIC_TIMEOUT_US)
            {
                b_timeout = true;
                break;
            }
        }
    }
    
    if (false == b_timeout)
    {
        pulse_end_us = time_us_32();
        pulse_width_us = pulse_end_us - pulse_start_us;
        
        // Calculate distance: pulse_width / 58 = distance in cm
        distance_cm = (uint16_t)(pulse_width_us / 58u);
    }
    
    return distance_cm;
}

/*** end of file ***/
