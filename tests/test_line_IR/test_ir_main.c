/** @file test_ir_main.c
 *
 * @brief Interpreted digital IR sensor test for center and right sensors.
 *
 * @par
 * This test reads the DOUT signals from the center and right IR sensors,
 * prints the raw logic levels, and prints the interpreted line state.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2024. All rights reserved.
 */

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "ir_sensor.h"

#define IR_TEST_SAMPLE_DELAY_MSEC    (200u)

/*!
 * @brief Convert IR state enum to printable string.
 *
 * @param[in] state   Interpreted IR state.
 *
 * @return Pointer to constant string describing the state.
 */
static char const *
ir_test_state_to_str (ir_state_t state)
{
    char const * p_state_str = "UNKNOWN";

    switch (state)
    {
        case IR_STATE_NO_LINE:
            p_state_str = "NO_LINE";
        break;

        case IR_STATE_CENTER_ON_LINE:
            p_state_str = "CENTER_ON_LINE";
        break;

        case IR_STATE_RIGHT_ON_LINE:
            p_state_str = "RIGHT_ON_LINE";
        break;

        case IR_STATE_BOTH_ON_LINE:
            p_state_str = "BOTH_ON_LINE";
        break;

        default:
            p_state_str = "INVALID_STATE";
        break;
    }

    return (p_state_str);
}

/*!
 * @brief Main entry point for the interpreted IR sensor test.
 *
 * @return This function should not return.
 */
int
main (void)
{
    bool       b_center_state = false;
    bool       b_right_state  = false;
    ir_state_t state          = IR_STATE_NO_LINE;

    stdio_init_all();
    ir_sensor_init();

    sleep_ms(2000u);

    printf("\r\n");
    printf("========================================\r\n");
    printf("INTERPRETED IR SENSOR TEST\r\n");
    printf("========================================\r\n");
    printf("Center IR DOUT -> GPIO4\r\n");
    printf("Right  IR DOUT -> GPIO0\r\n");
    printf("Logic mapping  -> 1 = black, 0 = white\r\n");
    printf("Sample period  -> %u ms\r\n", IR_TEST_SAMPLE_DELAY_MSEC);
    printf("========================================\r\n\r\n");

    for (;;)
    {
        b_center_state = ir_sensor_center_read();
        b_right_state  = ir_sensor_right_read();
        state          = ir_sensor_get_state();

        printf("Raw: CENTER=%u RIGHT=%u | State: %s\r\n",
               (unsigned int) b_center_state,
               (unsigned int) b_right_state,
               ir_test_state_to_str(state));

        sleep_ms(IR_TEST_SAMPLE_DELAY_MSEC);
    }

    return (0);
}

/*** end of file ***/
