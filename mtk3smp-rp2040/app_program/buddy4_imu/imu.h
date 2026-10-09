/**
 * @file imu.h
 * @brief Buddy 4: LSM303D accelerometer + magnetometer on I2C1.
 *
 * The board is sold as a GY-511 (LSM303DLHC), but the part fitted answers
 * WHO_AM_I = 0x49 at 0x1D: an LSM303D, with accel and mag on one address.
 * (Identified 2026-10-07 by bus scan; nothing answers at 0x19/0x1E.)
 *
 * Polled register-level I2C1 on GP2 (SDA) / GP3 (SCL). No interrupts, no
 * kernel sample driver (that driver's unit 0 claims the motor pins).
 */

#ifndef BUDDY4_IMU_H
#define BUDDY4_IMU_H

#include <stdint.h>
#include <tk/tkernel.h>

#define IMU_ADDR             0x1Du   /**< LSM303D, SA0 high */
#define IMU_WHO_AM_I_VALUE   0x49u

/**
 * @brief Raw sensor sample.
 * Accel in mg (+/-2 g, 0.061 mg/LSB).
 * Mag in raw counts (+/-2 gauss, 0.080 mgauss/LSB).
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

/** @brief Read the LSM303D WHO_AM_I register (expect 0x49). */
ER imu_who_am_i(uint8_t *p_id);

/**
 * @brief Check WHO_AM_I, then configure accel (100 Hz, +/-2 g) and mag
 * (50 Hz high resolution, +/-2 gauss, continuous).
 * @return E_NOEXS if the chip is not an LSM303D.
 */
ER imu_init(void);

/** @brief Read one accel + mag sample. */
ER imu_read_raw(imu_raw_t *p_raw);

/**
 * @brief Read one sample with the accelerometer calibrated (6-face test).
 * Car frame: X forward, Y right, Z down; at rest upright a = (0, 0, -1000) mg.
 * Mag is still raw (calibrated in step 4).
 */
ER imu_read_cal(imu_raw_t *p_sample);

#endif /* BUDDY4_IMU_H */
