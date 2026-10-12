/**
 * @file ir_sensor.h
 * @brief Buddy 3: the three IR reflectance modules, digital outputs only.
 *
 * Layout (photo 2026-10-12): centre (GP4) and right (GP26) sit on the
 * front bracket ~10 mm apart, centred on the car, both inside the 18 mm
 * line when on track. Left (GP6) hangs out beside the left wheel and is
 * the only barcode reader; it also sees the left arm of a cross junction.
 *
 * Each module's comparator output reads 1 over black (no reflection) and
 * 0 over white, as found with the earlier Pico SDK test; the bring-up test
 * confirms this. The AOUT pins are not wired (no spare ADC).
 */

#ifndef BUDDY3_IR_SENSOR_H
#define BUDDY3_IR_SENSOR_H

#include <stdint.h>

#define IR_BIT_LEFT     (1u << 0)
#define IR_BIT_CENTRE   (1u << 1)
#define IR_BIT_RIGHT    (1u << 2)

/** @brief Set GP6, GP4 and GP26 as SIO inputs (GP26 boots with its input off). */
void ir_init(void);

/** @brief All three sensors at once: IR_BIT_* set where the sensor sees black. */
uint32_t ir_read(void);

#endif /* BUDDY3_IR_SENSOR_H */
