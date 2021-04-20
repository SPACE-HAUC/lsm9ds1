/**
 * @file lsm9ds1.c
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief Function definitions for LSM9DS1 Magnetometer I2C driver.
 * @version 0.1
 * @date 2020-03-19
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include "i2cbus/i2cbus.h"
#include "lsm9ds1.h"

#define eprintf(str, ...) \
    fprintf(stderr, "%s, %d: " str "\n", __func__, __LINE__, ##__VA_ARGS__); \
    fflush(stderr)

int lsm9ds1_init(lsm9ds1 *dev, uint8_t bus, uint8_t xl_addr, uint8_t mag_addr, uint8_t ctx)
{
    int accel_stat = 1;
    if (dev == NULL)
    {
        eprintf("Pointer to device struct is NULL!");
        return -1;
    }
    if (dev->accel_dev == NULL || dev->mag_dev == NULL)
    {
        eprintf("Pointer to magnetometer and/or accelerometer device is NULL");
        return -1;
    }
    if (i2cbus_open(dev->accel_dev, bus, xl_addr) < 0)
    {
        perror("i2cbus, accelerometer");
        return -1;
    }
    if (i2cbus_open(dev->mag_dev, bus, mag_addr) < 0)
    {
        perror("i2cbus, magnetometer");
        return -1;
    }
    // First things first, set context
    dev->accel_dev->ctx = ctx;
    dev->mag_dev->ctx = ctx;
    // Verify mag identity
    uint8_t buf[2] = {MAG_WHO_AM_I, 0};
    if (i2cbus_xfer(dev->mag_dev, buf, 1, buf + 1, 1, 0) < 0)
    {
        eprintf("Could not get magnetic field descriptor");
        return -1;
    }
    if (buf[1] != MAG_IDENT)
    {
        eprintf("Identity did not match");
        return -1;
    }
    // disable accel+gyro
    char obuf[2];
    obuf[0] = LSM9DS1_CTRL_REG1_G;
    obuf[1] = 0x00;
    accel_stat = i2cbus_write(dev->accel_dev, obuf, 2);
    obuf[0] = LSM9DS1_CTRL_REG5_XL;
    accel_stat = i2cbus_write(dev->accel_dev, obuf, 2);
    obuf[0] = LSM9DS1_CTRL_REG6_XL;
    accel_stat = i2cbus_write(dev->accel_dev, obuf, 2);
    if (accel_stat < 0)
    {
        eprintf("Error configuring accelerometer + gyro");
    }
    // also configure magnetometer for SPACE HAUC use I2C_SLAVE
    MAG_DATA_RATE drate;
    drate.data_rate = 0b101;
    drate.fast_odr = 0;
    drate.operative_mode = 0b11;
    drate.temp_comp = 1;
    MAG_RESET rst;
    rst.full_scale = 0b00;
    rst.reboot = 0;
    rst.soft_rst = 0;
    MAG_DATA_READ dread;
    dread.bdu = 0;
    dread.fast_read = 0;
    int mag_stat = lsm9ds1_config_mag(dev, drate, rst, dread);

    return mag_stat;
}

int lsm9ds1_config_mag(lsm9ds1 *dev, MAG_DATA_RATE datarate, MAG_RESET rst, MAG_DATA_READ dread)
{
    int stat = 1;
    uint8_t buf[2];
    buf[0] = MAG_CTRL_REG1_M;
    buf[1] = datarate.data;
    if (i2cbus_write(dev->mag_dev, buf, 2) != 2)
    {
        eprintf("Data rate config failed.");
        stat = 0;
    }
    buf[0] = MAG_CTRL_REG2_M;
    buf[1] = rst.data;
    if (i2cbus_write(dev->mag_dev, buf, 2) != 2)
    {
        eprintf("Reset config failed.");
        stat = 0;
    }
    buf[0] = MAG_CTRL_REG3_M;
    buf[1] = 0x00;
    if (i2cbus_write(dev->mag_dev, buf, 2) != 2)
    {
        eprintf("Reg3 config failed.");
        stat = 0;
    }
    buf[0] = MAG_CTRL_REG4_M;
    buf[1] = MAG_CTRL_REG4_DATA;
    if (i2cbus_write(dev->mag_dev, buf, 2) != 2)
    {
        eprintf("Reg4 config failed.");
        stat = 0;
    }
    buf[0] = MAG_CTRL_REG5_M;
    buf[1] = dread.data;
    if (i2cbus_write(dev->mag_dev, buf, 2) != 2)
    {
        eprintf("Data read config failed.");
        stat = 0;
    }
    return stat;
}

int lsm9ds1_reset_mag(lsm9ds1 *dev)
{
    static MAG_RESET rst = {.reboot = 1};
    static uint8_t buf[2] = {MAG_CTRL_REG2_M, 0x0};
    buf[1] = rst.data;
    if (i2cbus_write(dev->mag_dev, buf, 2) != 2)
    {
        eprintf("Reset failed.");
        return -1;
    }
    return 1;
}

int lsm9ds1_read_mag(lsm9ds1 *dev, short *B)
{
    // printf("In read_mag %d\n", __LINE__);
    uint8_t buf[2] ={MAG_OUT_X_L - 1, };
    for (int i = 0; i < 3; i++)
    {
        B[i] = 0; // initialize with 0
        // printf("In read_mag %d\n", __LINE__);
        buf[0]++; // select register
        if (i2cbus_xfer(dev->mag_dev, buf, 1, buf + 1, 1, 0) < 0)
        {
            eprintf("Error reading magnetometer data from register 0x%02x", buf[0]);
            return -1;
        }
        B[i] |= buf[1];
        buf[0]++; // select the next register
        if (i2cbus_xfer(dev->mag_dev, buf, 1, buf + 1, 1, 0) < 0)
        {
            eprintf("Error reading magnetometer data from register 0x%02x", buf[0]);
            return -1;
        }
        B[i] |= 0xff00 & ((short)buf[1] << 8);
    }
    return 1;
}

int lsm9ds1_offset_mag(lsm9ds1 *dev, short *offset)
{
    uint8_t buf[2] = {MAG_OFFSET_X_REG_L_M - 1, };
    for (int i = 0; i < 3; i++)
    {
        buf[0]++; // insert the command into buffer
        buf[1] = (uint8_t)offset[i];
        if (i2cbus_write(dev->mag_dev, buf, 2) != 2)
        {
            eprintf("Mag offset failed, register 0x%02x", buf[0]);
            return -1;
        }
        buf[0]++; // insert the next reg address
        buf[1] = (uint8_t)(offset[i] >> 8);
        if (i2cbus_write(dev->mag_dev, buf, 2) != 2)
        {
            eprintf("Mag offset failed, register 0x%02x", buf[0]);
            return -1;
        }
    }
    return 1;
}

void lsm9ds1_destroy(lsm9ds1 *dev)
{
    i2cbus_close(dev->accel_dev);
    i2cbus_close(dev->mag_dev);
}

#ifdef UNIT_TEST
#include <stdio.h>
#include <signal.h>

volatile sig_atomic_t done = 0;

void sighandler(int sig)
{
    done = 1;
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("Invocation: ./%s <Bus ID>\n\n", argv[0]);
        return 0;
    }
    int id = atoi(argv[0]);
    int xl_addr = LSM9DS1_XL_ADDR;
    int mag_addr = LSM9DS1_MAG_ADDR;
    lsm9ds1 *dev = (lsm9ds1 *)malloc(sizeof(lsm9ds1));
    if (lsm9ds1_init(dev, id, xl_addr, mag_addr, 0x0) < 0)
    {
        goto end;
    }
    signal(SIGINT, &sighandler);
    while (!done)
    {
        short B[3] = {0x0, 0x0, 0x0};
        if (lsm9ds1_read_mag(dev, B) < 0)
            printf("Error reading B\n");
        int num_char = printf("Bx = %.2e By = %.2e Bz = %.2e\r", B[0] / 6.842, B[1] / 6.842, B[2] / 6.842);
        fflush(stdout);
        usleep(100000);
        printf("\r");
        while(num_char--)
            printf(" ");
        printf("\r");
    }
    lsm9ds1_destroy(dev);
end:
    free(dev);
    return 0;
}
#endif
