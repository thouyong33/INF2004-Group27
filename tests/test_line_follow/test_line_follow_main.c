/** @file test_line_follow_main.c
 *
 * @brief Limited line-following test using center and right IR sensors.
 *
 * @par
 * This is a basic line-following test using only two sensors:
 * center and right. It is not a full line-following implementation.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2024. All rights reserved.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "ir_sensor.h"
#include "motor.h"

#define LINE_TEST_LOOP_DELAY_MSEC    (50u)

/*!
 * @brief Convert IR state enum to readable string.
 *
 * @param[in] state   Interpreted sensor state.
 *
 * @return Pointer to constant state string.
 */
static char const *
line_test_state_to_str (ir_state_t state)
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
 * @brief Execute motor action based on current IR sensor state.
 *
 * @param[in] state   Interpreted sensor state.
 */
static void
line_test_apply_action (ir_state_t state)
{
    switch (state)
    {
        case IR_STATE_CENTER_ON_LINE:
            motor_forward();
        break;

        case IR_STATE_RIGHT_ON_LINE:
            motor_turn_right();
        break;

        case IR_STATE_NO_LINE:
            motor_stop_all();
        break;

        case IR_STATE_BOTH_ON_LINE:
            motor_stop_all();
        break;

        default:
            motor_stop_all();
        break;
    }
}

/*!
 * @brief Main entry point for limited line-following test.
 *
 * @return This function should not return.
 */
int
main (void)
{
    ir_state_t state = IR_STATE_NO_LINE;

    stdio_init_all();
    ir_sensor_init();
    motor_init();

    sleep_ms(2000u);

    printf("\r\n");
    printf("========================================\r\n");
    printf("LIMITED LINE FOLLOW TEST\r\n");
    printf("========================================\r\n");
    printf("Sensors used:\r\n");
    printf("  Center IR -> GPIO4\r\n");
    printf("  Right  IR -> GPIO0\r\n");
    printf("Logic:\r\n");
    printf("  1 = black line detected\r\n");
    printf("  0 = white surface\r\n");
    printf("Actions:\r\n");
    printf("  CENTER_ON_LINE -> forward\r\n");
    printf("  RIGHT_ON_LINE  -> turn right\r\n");
    printf("  NO_LINE        -> stop\r\n");
    printf("  BOTH_ON_LINE   -> stop\r\n");
    printf("========================================\r\n\r\n");

    for (;;)
    {
        state = ir_sensor_get_state();

        line_test_apply_action(state);

        printf("State: %s\r\n", line_test_state_to_str(state));

        sleep_ms(LINE_TEST_LOOP_DELAY_MSEC);
    }

    return (0);
}

/*** end of file ***/
