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
#include <i2cbus/i2cbus.h>
#include "lsm9ds1.h"
/**
 * @brief Takes the pointer to the device struct, XL address and M address,
 * returns 1 on success, negative numbers on failure.
 * 
 * @param dev Pointer to lsm9ds1
 * @param xl_addr Accelerometer address on I2C Bus (default 0x6b)
 * @param mag_addr magnetometer address on I2C Bus (default 0x1e)
 * @return Returns 1 on success, -1 on failure
 */
int lsm9ds1_init(lsm9ds1 *dev, uint8_t id, uint8_t xl_addr, uint8_t mag_addr, uint8_t ctx)
{
    int accel_stat = 1;
    dev->accel_dev = (i2cbus *)malloc(sizeof(i2cbus));
    dev->mag_dev = (i2cbus *)malloc(sizeof(i2cbus));
    if (dev->accel_dev == NULL || dev->mag_dev == NULL)
    {
        perror("memory");
        return -1;
    }
    if (i2cbus_open(dev->accel_dev, id, xl_addr) < 0)
    {
        perror("i2cbus, accelerometer");
        return -1;
    }
    if (i2cbus_open(dev->accel_dev, id, xl_addr) < 0)
    {
        perror("i2cbus, magnetometer");
        return -1;
    }
    // First things first, set context
    dev->accel_dev->ctx = ctx;
    dev->mag_dev->ctx = ctx;
    // Verify mag identity
    uint8_t buf = MAG_WHO_AM_I;
    if (write(dev->mag_dev->fd, &buf, 1) < 1)
    {
        perror("LSM9DS1_INIT: Could not write to magnetic field descriptor.");
        return -1;
    }
    if (read(dev->mag_dev->fd, &buf, 1) < 1)
    {
        perror("LSM9DS1_INIT: Could not write to magnetic field descriptor.");
        return -1;
    }
    if (buf != MAG_IDENT)
    {
        perror("LSM9DS1_INIT: Identity did not match");
        return -1;
    }
    // disable accel+gyro
    char obuf[2];
    obuf[0] = LSM9DS1_CTRL_REG1_G;
    obuf[1] = 0x00;
    accel_stat = write(dev->accel_dev->fd, &obuf, 2);
    obuf[0] = LSM9DS1_CTRL_REG5_XL;
    accel_stat = write(dev->accel_dev->fd, &obuf, 2);
    obuf[0] = LSM9DS1_CTRL_REG6_XL;
    accel_stat = write(dev->accel_dev->fd, &obuf, 2);
    if (accel_stat < 2)
        perror("[LSM9DS1] Error configuring accelerometer + gyro");
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
/**
 * @brief Configure the data rate, reset vector and data granularity.
 * 
 * @param dev Pointer to lsm9ds1
 * @param datarate
 * @param rst 
 * @param dread 
 * @return Returns 1 on success, -1 on failure 
 */
int lsm9ds1_config_mag(lsm9ds1 *dev, MAG_DATA_RATE datarate, MAG_RESET rst, MAG_DATA_READ dread)
{
    int stat = 1;
    uint8_t buf[2];
    buf[0] = MAG_CTRL_REG1_M;
    buf[1] = *((char *)&datarate);
    if (write(dev->mag_dev->fd, &buf, 2) < 2)
    {
        perror("Data rate config failed.");
        stat = 0;
    }
    buf[0] = MAG_CTRL_REG2_M;
    buf[1] = *((char *)&rst);
    if (write(dev->mag_dev->fd, &buf, 2) < 2)
    {
        perror("Reset config failed.");
        stat = 0;
    }
    buf[0] = MAG_CTRL_REG3_M;
    buf[1] = 0x00;
    if (write(dev->mag_dev->fd, &buf, 2) < 2)
    {
        perror("Reg3 config failed.");
        stat = 0;
    }
    buf[0] = MAG_CTRL_REG4_M;
    buf[1] = MAG_CTRL_REG4_DATA;
    if (write(dev->mag_dev->fd, &buf, 2) < 2)
    {
        perror("Reg4 config failed.");
        stat = 0;
    }
    buf[0] = MAG_CTRL_REG5_M;
    buf[1] = *((char *)&dread);
    if (write(dev->mag_dev->fd, &buf, 2) < 2)
    {
        perror("Data read config failed.");
        stat = 0;
    }
    return stat;
}
/**
 * @brief Reset the magnetometer memory.
 * 
 * @param dev Pointer to lsm9ds1
 * @return Returns 1 on success, -1 on failure 
 */
int lsm9ds1_reset_mag(lsm9ds1 *dev)
{
    uint8_t buf[2];
    buf[0] = MAG_CTRL_REG2_M;
    buf[1] = 0x00;
    MAG_RESET rst = *((MAG_RESET *)&buf[1]);
    rst.reboot = 1;
    buf[1] = *((uint8_t *)&rst);
    if (write(dev->mag_dev->fd, &buf, 2) < 2)
    {
        perror("Reset failed.");
        return -1;
    }
    return 1;
}
/**
 * @brief Store the magnetic field readings in the array of shorts, order: X
 * Y Z
 * 
 * @param dev Pointer to lsm9ds1
 * @param B Pointer to an array of short of length 3 where magnetometer reading is stored
 * @return Returns 1 on success, -1 on failure 
 */
int lsm9ds1_read_mag(lsm9ds1 *dev, short *B)
{
    // printf("In read_mag %d\n", __LINE__);
    uint8_t buf, reg = MAG_OUT_X_L - 1;
    for (int i = 0; i < 3; i++)
    {
        B[i] = 0; // initialize with 0
        // printf("In read_mag %d\n", __LINE__);
        buf = ++reg; // insert the command into buffer
        // printf("Buf: 0x%x\n", buf);
        if (i2cbus_xfer(dev->mag_dev, &buf, 1, &buf, 1, 0) < 0)
        {
            fprintf(stderr, "%s: Error reading magnetometer data, ", __func__);
            perror("error");
            return -1;
        }
        B[i] |= buf;
        buf = ++reg; // select the next register
        if (i2cbus_xfer(dev->mag_dev, &buf, 1, &buf, 1, 0) < 0)
        {
            fprintf(stderr, "%s: Error reading magnetometer data, ", __func__);
            perror("error");
            return -1;
        }
        B[i] |= 0xff00 & ((short)buf << 8);
        // printf("In read_mag %d\n", __LINE__);
    }
    return 1;
}
/**
 * @brief Set the mag field offsets using the array, order: X Y Z
 * 
 * @param dev Pointer to lsm9ds1
 * @param offset Pointer to an array of shorts of length 3 where magnetometer offset is stored
 * @return Returns 1 on success, -1 on failure 
 */
int lsm9ds1_offset_mag(lsm9ds1 *dev, short *offset)
{
    uint8_t buf[2], reg = MAG_OFFSET_X_REG_L_M - 1;
    for (int i = 0; i < 3; i++)
    {
        buf[0] = ++reg; // insert the command into buffer
        buf[1] = (uint8_t)offset[i];
        if (write(dev->mag_dev->fd, &buf, 2) < 2)
        {
            perror("offset_mag failed");
            return -1;
        }
        buf[0] = ++reg; // insert the next reg address
        buf[1] = (uint8_t)(offset[i] >> 8);
        if (write(dev->mag_dev->fd, &buf, 2) < 2)
        {
            perror("offset_mag failed");
            return -1;
        }
    }
    return 1;
}
/**
 * @brief Closes the file descriptors for the mag and accel and frees the allocated memory.
 * 
 * @param dev Pointer to lsm9ds1
 */
void lsm9ds1_destroy(lsm9ds1 *dev)
{
    i2cbus_close(dev->accel_dev);
    i2cbus_close(dev->mag_dev);
    free(dev->accel_dev);
    free(dev->mag_dev);
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