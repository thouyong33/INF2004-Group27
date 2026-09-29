/** @file ir_sensor.h
 *
 * @brief Digital IR sensor interface for center and right line sensors.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2024. All rights reserved.
 */

#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#define IR_CENTER_DOUT_PIN     (4u)
#define IR_RIGHT_DOUT_PIN      (0u)

typedef enum
{
    IR_STATE_NO_LINE = 0,
    IR_STATE_CENTER_ON_LINE,
    IR_STATE_RIGHT_ON_LINE,
    IR_STATE_BOTH_ON_LINE

} ir_state_t;

void       ir_sensor_init(void);
bool       ir_sensor_center_read(void);
bool       ir_sensor_right_read(void);
ir_state_t ir_sensor_get_state(void);

#endif /* IR_SENSOR_H */

/*** end of file ***/
