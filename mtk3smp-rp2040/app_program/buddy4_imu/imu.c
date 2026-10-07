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
