/** @file ir_sensor.c
 *
 * @brief Digital IR sensor interface implementation for center and right sensors.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2024. All rights reserved.
 */

#include "pico/stdlib.h"
#include "ir_sensor.h"

/*!
 * @brief Initialize the GPIO pins used by the IR sensors.
 */
void
ir_sensor_init (void)
{
    gpio_init(IR_CENTER_DOUT_PIN);
    gpio_set_dir(IR_CENTER_DOUT_PIN, GPIO_IN);

    gpio_init(IR_RIGHT_DOUT_PIN);
    gpio_set_dir(IR_RIGHT_DOUT_PIN, GPIO_IN);
}

/*!
 * @brief Read the digital output of the center IR sensor.
 *
 * @return True if black line is detected, false otherwise.
 */
bool
ir_sensor_center_read (void)
{
    return (gpio_get(IR_CENTER_DOUT_PIN));
}

/*!
 * @brief Read the digital output of the right IR sensor.
 *
 * @return True if black line is detected, false otherwise.
 */
bool
ir_sensor_right_read (void)
{
    return (gpio_get(IR_RIGHT_DOUT_PIN));
}

/*!
 * @brief Interpret the combined state of the center and right IR sensors.
 *
 * @return Current interpreted line state.
 */
ir_state_t
ir_sensor_get_state (void)
{
    bool       b_center_state = false;
    bool       b_right_state  = false;
    ir_state_t state          = IR_STATE_NO_LINE;

    b_center_state = ir_sensor_center_read();
    b_right_state  = ir_sensor_right_read();

    if ((false == b_center_state) && (false == b_right_state))
    {
        state = IR_STATE_NO_LINE;
    }
    else if ((true == b_center_state) && (false == b_right_state))
    {
        state = IR_STATE_CENTER_ON_LINE;
    }
    else if ((false == b_center_state) && (true == b_right_state))
    {
        state = IR_STATE_RIGHT_ON_LINE;
    }
    else
    {
        state = IR_STATE_BOTH_ON_LINE;
    }

    return (state);
}

/*** end of file ***/
