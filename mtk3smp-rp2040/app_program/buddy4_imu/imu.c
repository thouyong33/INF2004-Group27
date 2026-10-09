/**
 * @file imu.c
 * @brief Buddy 4: polled I2C1 master and LSM303D access.
 *
 * The RP2040 I2C block is a Synopsys DW_apb_i2c. This driver polls its
 * status registers; timeouts use the kernel clock (1 ms tick), because the
 * RP2040 hardware microsecond TIMER is held in reset by this port.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <sys/sysdef.h>

#include "../common/pin_defs.h"
#include "imu.h"

/*----------------------------------------------------------------------------
 * I2C1 registers (RP2040 datasheet 4.3.17)
 *---------------------------------------------------------------------------*/
#define I2C1_REG(off)            (0x40048000u + (off))
#define IC_CON                   I2C1_REG(0x00u)
#define IC_TAR                   I2C1_REG(0x04u)
#define IC_DATA_CMD              I2C1_REG(0x10u)
#define IC_FS_SCL_HCNT           I2C1_REG(0x1Cu)
#define IC_FS_SCL_LCNT           I2C1_REG(0x20u)
#define IC_RAW_INTR_STAT         I2C1_REG(0x34u)
#define IC_RX_TL                 I2C1_REG(0x38u)
#define IC_TX_TL                 I2C1_REG(0x3Cu)
#define IC_CLR_TX_ABRT           I2C1_REG(0x54u)
#define IC_CLR_STOP_DET          I2C1_REG(0x60u)
#define IC_ENABLE                I2C1_REG(0x6Cu)
#define IC_RXFLR                 I2C1_REG(0x78u)
#define IC_SDA_HOLD              I2C1_REG(0x7Cu)
#define IC_TX_ABRT_SOURCE        I2C1_REG(0x80u)
#define IC_ENABLE_STATUS         I2C1_REG(0x9Cu)
#define IC_FS_SPKLEN             I2C1_REG(0xA0u)

#define IC_CON_MASTER_MODE       (1u << 0)
#define IC_CON_SPEED_FAST        (2u << 1)
#define IC_CON_RESTART_EN        (1u << 5)
#define IC_CON_SLAVE_DISABLE     (1u << 6)
#define IC_CON_TX_EMPTY_CTRL     (1u << 8)

#define IC_DATA_CMD_READ         (1u << 8)
#define IC_DATA_CMD_STOP         (1u << 9)
#define IC_DATA_CMD_RESTART      (1u << 10)

#define IC_RAW_TX_ABRT           (1u << 6)
#define IC_RAW_STOP_DET          (1u << 9)

/* 100 kHz from clk_sys (SYSCLK MHz), same arithmetic as the Pico SDK */
#define I2C_BAUD_HZ              100000u
#define I2C_CLK_HZ               (SYSCLK * 1000000u)
#define I2C_PERIOD               ((I2C_CLK_HZ + (I2C_BAUD_HZ / 2u)) / I2C_BAUD_HZ)
#define I2C_LCNT                 ((I2C_PERIOD * 3u) / 5u)
#define I2C_HCNT                 (I2C_PERIOD - I2C_LCNT)
#define I2C_SPKLEN               ((I2C_LCNT < 16u) ? 1u : (I2C_LCNT / 16u))
#define I2C_SDA_HOLD_CNT         (((I2C_CLK_HZ * 3u) / 10000000u) + 1u)

#define I2C_TIMEOUT_MS           10u
#define I2C_MAX_READ             16u

/*----------------------------------------------------------------------------
 * LSM303D registers (accel and mag share one address)
 *---------------------------------------------------------------------------*/
#define LSM_WHO_AM_I             0x0Fu
#define LSM_OUT_X_L_M            0x08u   /* mag X, Y, Z little-endian */
#define LSM_CTRL1                0x20u
#define LSM_CTRL2                0x21u
#define LSM_CTRL5                0x24u
#define LSM_CTRL6                0x25u
#define LSM_CTRL7                0x26u
#define LSM_OUT_X_L_A            0x28u   /* accel X, Y, Z little-endian */
#define LSM_AUTO_INC             0x80u   /* set MSB of reg for burst reads */

#define LSM_A_100HZ_XYZ          0x67u   /* AODR 100 Hz, X/Y/Z on */
#define LSM_A_2G                 0x00u   /* +/-2 g, 773 Hz anti-alias */
#define LSM_M_HIRES_50HZ         0x70u   /* M_RES high, M_ODR 50 Hz, temp off */
#define LSM_M_2GAUSS             0x00u   /* +/-2 gauss */
#define LSM_M_CONTINUOUS         0x00u   /* MD = 00 */

/* +/-2 g: 0.061 mg/LSB */
#define LSM_A_UG_PER_LSB         61L

/*----------------------------------------------------------------------------
 * I2C1
 *---------------------------------------------------------------------------*/
static uint32_t
now_ms(void)
{
    SYSTIM tim;

    tk_get_tim(&tim);
    return tim.lo;
}

/* Clear a pending abort; return E_IO if one was pending. */
static ER
i2c1_check_abort(void)
{
    if (0u != (in_w(IC_RAW_INTR_STAT) & IC_RAW_TX_ABRT))
    {
        (void)in_w(IC_TX_ABRT_SOURCE);
        (void)in_w(IC_CLR_TX_ABRT);
        return E_IO;
    }

    return E_OK;
}

/* Wait for STOP to finish the transfer, then clear it. */
static ER
i2c1_wait_stop(uint32_t start_ms)
{
    while (0u == (in_w(IC_RAW_INTR_STAT) & IC_RAW_STOP_DET))
    {
        if ((now_ms() - start_ms) > I2C_TIMEOUT_MS)
        {
            return E_TMOUT;
        }
    }

    (void)in_w(IC_CLR_STOP_DET);
    return E_OK;
}

static void
i2c1_set_target(uint8_t addr)
{
    out_w(IC_ENABLE, 0u);
    while (0u != (in_w(IC_ENABLE_STATUS) & 1u))
    {
        ;
    }
    out_w(IC_TAR, (UW)addr);
    out_w(IC_ENABLE, 1u);
}

void
i2c1_init(void)
{
    /* Release I2C1 from reset */
    set_w(RESETS_RESET, RESETS_RESET_I2C1);
    clr_w(RESETS_RESET, RESETS_RESET_I2C1);
    while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_I2C1))
    {
        ;
    }

    out_w(IC_ENABLE, 0u);
    out_w(IC_CON, IC_CON_MASTER_MODE | IC_CON_SPEED_FAST | IC_CON_RESTART_EN |
                  IC_CON_SLAVE_DISABLE | IC_CON_TX_EMPTY_CTRL);
    out_w(IC_RX_TL, 0u);
    out_w(IC_TX_TL, 0u);
    out_w(IC_FS_SCL_HCNT, I2C_HCNT);
    out_w(IC_FS_SCL_LCNT, I2C_LCNT);
    out_w(IC_FS_SPKLEN, I2C_SPKLEN);
    out_w(IC_SDA_HOLD, I2C_SDA_HOLD_CNT);

    /* GP2 = I2C1 SDA, GP3 = I2C1 SCL; pull-ups on (the GY-511 has its own) */
    out_w(GPIO(I2C_SDA), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
    out_w(GPIO(I2C_SCL), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
    out_w(GPIO_CTRL(I2C_SDA), GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO_CTRL(I2C_SCL), GPIO_CTRL_FUNCSEL_I2C);

    out_w(IC_ENABLE, 1u);
}

ER
i2c1_probe(uint8_t addr)
{
    uint32_t start;
    ER err;

    i2c1_set_target(addr);
    (void)i2c1_check_abort();
    (void)in_w(IC_CLR_STOP_DET);

    start = now_ms();
    out_w(IC_DATA_CMD, IC_DATA_CMD_READ | IC_DATA_CMD_STOP);

    err = i2c1_wait_stop(start);
    if (E_OK != i2c1_check_abort())
    {
        return E_IO;                /* address not acknowledged */
    }

    /* Drain the byte that was read */
    while (0u != in_w(IC_RXFLR))
    {
        (void)in_w(IC_DATA_CMD);
    }

    return err;
}

ER
i2c1_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint32_t start;
    ER err;

    i2c1_set_target(addr);
    (void)i2c1_check_abort();
    (void)in_w(IC_CLR_STOP_DET);

    start = now_ms();
    out_w(IC_DATA_CMD, (UW)reg);
    out_w(IC_DATA_CMD, (UW)value | IC_DATA_CMD_STOP);

    err = i2c1_wait_stop(start);
    if (E_OK != i2c1_check_abort())
    {
        return E_IO;
    }

    return err;
}

ER
i2c1_read_regs(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint32_t n)
{
    uint32_t start;
    uint32_t i;
    UW cmd;

    if ((NULL == p_buf) || (0u == n) || (n > I2C_MAX_READ))
    {
        return E_PAR;
    }

    i2c1_set_target(addr);
    (void)i2c1_check_abort();
    (void)in_w(IC_CLR_STOP_DET);

    start = now_ms();
    out_w(IC_DATA_CMD, (UW)reg);

    for (i = 0u; i < n; i++)
    {
        cmd = IC_DATA_CMD_READ;
        if (0u == i)
        {
            cmd |= IC_DATA_CMD_RESTART;
        }
        if ((n - 1u) == i)
        {
            cmd |= IC_DATA_CMD_STOP;
        }
        out_w(IC_DATA_CMD, cmd);
    }

    for (i = 0u; i < n; i++)
    {
        while (0u == in_w(IC_RXFLR))
        {
            if (E_OK != i2c1_check_abort())
            {
                (void)i2c1_wait_stop(start);
                return E_IO;
            }
            if ((now_ms() - start) > I2C_TIMEOUT_MS)
            {
                return E_TMOUT;
            }
        }
        p_buf[i] = (uint8_t)(in_w(IC_DATA_CMD) & 0xFFu);
    }

    return i2c1_wait_stop(start);
}

/*----------------------------------------------------------------------------
 * LSM303D
 *---------------------------------------------------------------------------*/
ER
imu_who_am_i(uint8_t *p_id)
{
    return i2c1_read_regs(IMU_ADDR, LSM_WHO_AM_I, p_id, 1u);
}

ER
imu_init(void)
{
    ER err;
    uint8_t id;

    err = imu_who_am_i(&id);
    if ((E_OK == err) && (IMU_WHO_AM_I_VALUE != id))
    {
        err = E_NOEXS;
    }

    if (E_OK == err)
    {
        err = i2c1_write_reg(IMU_ADDR, LSM_CTRL1, LSM_A_100HZ_XYZ);
    }
    if (E_OK == err)
    {
        err = i2c1_write_reg(IMU_ADDR, LSM_CTRL2, LSM_A_2G);
    }
    if (E_OK == err)
    {
        err = i2c1_write_reg(IMU_ADDR, LSM_CTRL5, LSM_M_HIRES_50HZ);
    }
    if (E_OK == err)
    {
        err = i2c1_write_reg(IMU_ADDR, LSM_CTRL6, LSM_M_2GAUSS);
    }
    if (E_OK == err)
    {
        err = i2c1_write_reg(IMU_ADDR, LSM_CTRL7, LSM_M_CONTINUOUS);
    }

    return err;
}

static int16_t
le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[1] << 8 | p[0]);
}

ER
imu_read_raw(imu_raw_t *p_raw)
{
    uint8_t a[6];
    uint8_t m[6];
    ER err;

    if (NULL == p_raw)
    {
        return E_PAR;
    }

    err = i2c1_read_regs(IMU_ADDR, LSM_OUT_X_L_A | LSM_AUTO_INC, a, 6u);
    if (E_OK != err)
    {
        return err;
    }

    err = i2c1_read_regs(IMU_ADDR, LSM_OUT_X_L_M | LSM_AUTO_INC, m, 6u);
    if (E_OK != err)
    {
        return err;
    }

    p_raw->ax = (int16_t)(((int32_t)le16(&a[0]) * LSM_A_UG_PER_LSB) / 1000L);
    p_raw->ay = (int16_t)(((int32_t)le16(&a[2]) * LSM_A_UG_PER_LSB) / 1000L);
    p_raw->az = (int16_t)(((int32_t)le16(&a[4]) * LSM_A_UG_PER_LSB) / 1000L);

    p_raw->mx = le16(&m[0]);
    p_raw->my = le16(&m[2]);
    p_raw->mz = le16(&m[4]);

    return E_OK;
}

/*----------------------------------------------------------------------------
 * Accelerometer calibration (6-face test, 2026-10-09)
 *
 * Faces (raw mg):  X +1074 / -951,  Y +1039 / -952,  Z +1125 / -897.
 * The error was mostly offset, not scale: Z's +114 mg offset is why the
 * car read ~900 mg at rest. corrected = (raw - offset) * scale / 1000.
 *
 * Axes, from which face read +1 g: X = forward (nose up -> +X),
 * Y = right (left side down -> +Y), Z = down (upright -> -Z). The sensor
 * frame is already the car frame (forward-right-down), so no remapping.
 *---------------------------------------------------------------------------*/
#define ACC_OFF_X_MG             61
#define ACC_OFF_Y_MG             43
#define ACC_OFF_Z_MG             114
#define ACC_SCALE_X_X1000        988L
#define ACC_SCALE_Y_X1000        1005L
#define ACC_SCALE_Z_X1000        989L

/*----------------------------------------------------------------------------
 * Magnetometer calibration (step 4, 2026-10-09)
 *
 * Car turned through 360 deg flat: by hand with motors off, centre
 * (-905, -4475), radius (7710, 6975); spinning under PI with motors on,
 * centre (-775, -4230), radius (7777, 6942). The motors move the centre
 * only ~2 deg of heading, so the average of both is used. Hard iron is the
 * centre offset; soft iron is the X/Y radius ratio (Y stretched by 1.113
 * to make the circle round). Z is only centred, from the floor run (the
 * table read -1186, the floor -434: surroundings, not the car).
 *---------------------------------------------------------------------------*/
#define MAG_OFF_X                (-840)
#define MAG_OFF_Y                (-4352)
#define MAG_OFF_Z                (-434)
#define MAG_SCALE_Y_X1000        1113L

static int16_t
acc_correct(int16_t raw, int16_t offset, int32_t scale_x1000)
{
    return (int16_t)((((int32_t)raw - offset) * scale_x1000) / 1000L);
}

ER
imu_read_cal(imu_raw_t *p_sample)
{
    ER err;

    err = imu_read_raw(p_sample);
    if (E_OK != err)
    {
        return err;
    }

    p_sample->ax = acc_correct(p_sample->ax, ACC_OFF_X_MG, ACC_SCALE_X_X1000);
    p_sample->ay = acc_correct(p_sample->ay, ACC_OFF_Y_MG, ACC_SCALE_Y_X1000);
    p_sample->az = acc_correct(p_sample->az, ACC_OFF_Z_MG, ACC_SCALE_Z_X1000);

    p_sample->mx = acc_correct(p_sample->mx, MAG_OFF_X, 1000L);
    p_sample->my = acc_correct(p_sample->my, MAG_OFF_Y, MAG_SCALE_Y_X1000);
    p_sample->mz = acc_correct(p_sample->mz, MAG_OFF_Z, 1000L);

    return E_OK;
}

/*----------------------------------------------------------------------------
 * Attitude and tilt-compensated heading (no libm: soft-float helpers)
 *---------------------------------------------------------------------------*/
#define F_PI                     3.14159265f
#define F_RAD_TO_CDEG            (18000.0f / F_PI)

static float
f_abs(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* Newton iterations from a rough guess; plenty for these magnitudes */
static float
f_sqrt(float x)
{
    float r;
    int32_t i;

    if (x <= 0.0f)
    {
        return 0.0f;
    }

    r = (x > 1.0f) ? (x * 0.5f) : 1.0f;
    for (i = 0; i < 20; i++)
    {
        r = 0.5f * (r + (x / r));
    }
    return r;
}

/* atan2 with max error ~0.25 deg: atan(z) ~= z*pi/4 + 0.273*z*(1 - |z|) */
static float
f_atan2(float y, float x)
{
    float z;
    float a;

    if ((0.0f == x) && (0.0f == y))
    {
        return 0.0f;
    }

    if (f_abs(x) >= f_abs(y))
    {
        z = y / x;
        a = (z * (F_PI / 4.0f)) + (0.273f * z * (1.0f - f_abs(z)));
        if (x < 0.0f)
        {
            a += (y >= 0.0f) ? F_PI : -F_PI;
        }
    }
    else
    {
        z = x / y;
        a = (z * (F_PI / 4.0f)) + (0.273f * z * (1.0f - f_abs(z)));
        a = ((y > 0.0f) ? (F_PI / 2.0f) : (-F_PI / 2.0f)) - a;
    }

    return a;
}

/*
 * The accelerometer reads the reaction to gravity, so the gravity vector
 * in the car frame (forward-right-down) is g = -a. Level: g = (0, 0, +1).
 *   roll  = atan2(gy, gz)               right side down is positive
 *   pitch = atan2(-gx, sqrt(gy^2+gz^2)) nose up is positive
 * The mag vector is rotated back to the horizontal plane, then
 *   heading = atan2(-Yh, Xh)            0 = magnetic north, clockwise +
 */
ER
imu_read_attitude(imu_attitude_t *p_att, imu_raw_t *p_sample)
{
    imu_raw_t s;
    ER err;
    float gx, gy, gz;
    float g_yz, g_all;
    float sin_r, cos_r, sin_p, cos_p;
    float xh, yh;
    float heading;

    if (NULL == p_att)
    {
        return E_PAR;
    }

    err = imu_read_cal(&s);
    if (E_OK != err)
    {
        return err;
    }

    gx = -(float)s.ax;
    gy = -(float)s.ay;
    gz = -(float)s.az;

    g_yz = f_sqrt((gy * gy) + (gz * gz));
    g_all = f_sqrt((gx * gx) + (g_yz * g_yz));
    if ((g_yz <= 0.0f) || (g_all <= 0.0f))
    {
        return E_SYS;
    }

    sin_r = gy / g_yz;
    cos_r = gz / g_yz;
    sin_p = -gx / g_all;
    cos_p = g_yz / g_all;

    xh = ((float)s.mx * cos_p) + ((float)s.my * sin_r * sin_p)
       + ((float)s.mz * cos_r * sin_p);
    yh = ((float)s.my * cos_r) - ((float)s.mz * sin_r);

    heading = f_atan2(-yh, xh) * F_RAD_TO_CDEG;
    if (heading < 0.0f)
    {
        heading += 36000.0f;
    }
    if (heading >= 36000.0f)
    {
        heading = 0.0f;             /* -0.0 wrapped up to exactly 360 */
    }

    p_att->roll_cdeg = (int16_t)(f_atan2(gy, gz) * F_RAD_TO_CDEG);
    p_att->pitch_cdeg = (int16_t)(f_atan2(-gx, g_yz) * F_RAD_TO_CDEG);
    p_att->heading_cdeg = (uint16_t)heading;

    if (NULL != p_sample)
    {
        *p_sample = s;
    }

    return E_OK;
}
