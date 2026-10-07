/**
 * @file imu.h
 * @brief Buddy 4: GY-511 (LSM303DLHC) accelerometer + magnetometer on I2C1.
 *
 * Polled register-level I2C1 on GP2 (SDA) / GP3 (SCL). No interrupts, no
 * kernel sample driver (that driver's unit 0 claims the motor pins).
 */

#ifndef BUDDY4_IMU_H
#define BUDDY4_IMU_H

#include <stdint.h>
#include <tk/tkernel.h>

#define IMU_ACCEL_ADDR   0x19u   /**< LSM303DLHC linear acceleration */
#define IMU_MAG_ADDR     0x1Eu   /**< LSM303DLHC magnetic field */

/**
 * @brief Raw sensor sample.
 * Accel in mg (high-resolution, +/-2 g, 1 mg/LSB).
 * Mag in raw counts (+/-1.3 gauss: 1100 LSB/gauss X/Y, 980 LSB/gauss Z).
 */
typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t mx;
    int16_t my;
    int16_t mz;
} imu_raw_t;

/** @brief Reset I2C1, route GP2/GP3 to it and set 100 kHz. */
void i2c1_init(void);

/**
 * @brief Probe one 7-bit address with a 1-byte read.
 * @return E_OK if the device acknowledged, E_IO if not.
 */
ER i2c1_probe(uint8_t addr);

/** @brief Write one register. */
ER i2c1_write_reg(uint8_t addr, uint8_t reg, uint8_t value);

/** @brief Read n consecutive bytes starting at reg (n <= 16). */
ER i2c1_read_regs(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint32_t n);

/** @brief Configure accel (100 Hz, HR, +/-2 g) and mag (75 Hz, +/-1.3 G, continuous). */
ER imu_init(void);

/** @brief Read one accel + mag sample. */
ER imu_read_raw(imu_raw_t *p_raw);

#endif /* BUDDY4_IMU_H */
