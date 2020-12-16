/**
 * @file lsm9ds1.h
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief Function prototypes and data structures for LSM9DS1 Magnetometer I2C driver.
 * @version 0.1
 * @date 2020-03-19
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef LSM9DS1_H
#define LSM9DS1_H

#include <stdint.h>

#define LSM9DS1_XL_ADDR 0x6b  ///< Accelerometer address
#define LSM9DS1_MAG_ADDR 0x1e ///< Magnetometer address

/**
 * @brief  * Accelerometer and Gyro registers
 * 
 * NOTE: Few registers are used ONLY TO
 * power down the accelerometer and the gyroscope.
 * 
 */
#define LSM9DS1_CTRL_REG1_G 0x10 // Gyro control register
#define LSM9DS1_GYRO_PD 0x00     ///< Content of the gyro control register for power down
/*
 * Refer to documentation
 */
#define LSM9DS1_CTRL_REG5_XL 0x1f ///< Acceleration control register
#define LSM9DS1_XL_PD 0x00        ///< Disable outputs

/**
 * @brief  * ODR_XL[7:5]: Output data rate and power mode, 0 0 0 for power down.
 * FS_XL[4:3]: Full scale selection.
 * BW_SCAL_ODR[2:2]: Bandwidth selection, 0 -- default, 1 -- bandwidth from BW_XL.
 * BW_XL[1:0]: Custom bandwidth.
 * 
 */
#define LSM9DS1_CTRL_REG6_XL 0x20

/**
 * @brief Magnetometer registers
 * 
 */
typedef enum
{
    MAG_OFFSET_X_REG_L_M = 0x05, ///< Magnetometer X axis offset LOW byte
    MAG_OFFSET_X_REG_H_M,        ///< Magnetometer X axis offset HIGH byte
    MAG_OFFSET_Y_REG_L_M,        ///< Magnetometer Y axis offset LOW byte
    MAG_OFFSET_Y_REG_H_M,        ///< Magnetometer Y axis offset HIGH byte
    MAG_OFFSET_Z_REG_L_M,        ///< Magnetometer Z axis offset LOW byte
    MAG_OFFSET_Z_REG_H_M,        ///< Magnetometer Z axis offset HIGH byte
} MAG_OFFSET_REGISTERS;

#define MAG_CTRL_REG1_M 0x20 ///< Magnetometer control register 1 address
/**
 * @brief Configuration for magnetometer data rate.
 * 
 */
typedef struct __attribute__((packed))
{
    uint8_t self_test : 1; ///< Self test enable. Default: 0. (0: disabled, 1: enabled)
    uint8_t fast_odr : 1;  ///< Enables data rates faster than 80 Hz. Default: 0 (0: disabled, 1: enabled)
    /**
     * @brief Sets data rate from the sensor when fast_odr is disabled.
     * 
     * Set Data Rate in Hz.
     * 000: 0.625 Hz
     * 001: 1.25 Hz
     * 010: 2.5 Hz
     * 011: 5 Hz
     * 100: 10 Hz (Default)
     * 101: 20 Hz (SPACE HAUC setting)
     * 110: 40 Hz
     * 111: 80 Hz
     */
    uint8_t data_rate : 3;
    /**
    * @brief X and Y axes operative mode selection. Default value: 00
    * 
    * Operative mode for X and Y axes.
    * 00: LP mode (Default)
    * 01: Medium perf
    * 10: High perf
    * 11: Ultra-high perf
    */
    uint8_t operative_mode : 2;
    /**
     * @brief Temperature compensation enable.
     * 
     * Default value: 0
     * 
     * 0: Temperature compensation disabled
     * 1: Temperature compensation enabled
     */
    uint8_t temp_comp : 1;
} MAG_DATA_RATE;

#define MAG_CTRL_REG2_M 0x21 ///< Magnetometer control register 2 address
/**
 * @brief Reset or configure scale of Magnetometer
 */
typedef struct __attribute__((packed))
{
    uint8_t reserved : 2;  ///< Reserved, must be 0.
    uint8_t soft_rst : 1;  ///< Configuration registers and user register reset function. (0: default value; 1: reset operation)
    uint8_t reboot : 1;    ///< Reboot memory content. Default value: 0 (0: normal mode; 1: reboot memory content)
    uint8_t reserved2 : 1; ///< Reserved, must be 0.
                           /**
    * @brief Full-scale configuration. Default value: 00
    * 00: +/- 4 Gauss
    * 01: +/- 8 Gauss
    * 10: +/- 12 Gauss
    * 11: +/- 16 Gauss
    */
    uint8_t full_scale : 2;
    uint8_t reserved3 : 1; ///< Reserved, must be 0.
} MAG_RESET;

#define MAG_CTRL_REG3_M 0x22 ///< Magnetometer control register 3 address, write 0x0 to this.

#define MAG_CTRL_REG4_M 0x23    ///< Magnetometer control register 4 address
#define MAG_CTRL_REG4_DATA 0x0c ///< Magnetometer control register 4: [11][0 0], ultra high Z performance + little endian register data selection

#define MAG_CTRL_REG5_M 0x24 ///< Magnetometer control register 5 address
/**
 * @brief Configures data updating method of the magnetometer.
 */
typedef struct __attribute__((packed))
{
    uint8_t reserved : 6; ///< Reserved, must be 0.
    /**
     * @brief Block data update for magnetic data.
     * 0: Continuous update,
     * 1: Output registers not updated until MSB and LSB has been read
     * 
     */
    uint8_t bdu : 1;
    /**
     * @brief FAST_READ allows reading the high part of DATA OUT only in order to increase
     * reading efficiency. Default: 0
     * 0: FAST_READ disabled, 1: Enabled
     * 
     */
    uint8_t fast_read : 1;
} MAG_DATA_READ;
/**
 * @brief Magnetometer measurement register addresses
 * 
 */
typedef enum
{
    MAG_OUT_X_L = 0x28, ///< Magnetometer X axis measurement LOW byte
    MAG_OUT_X_H,        ///< Magnetometer X axis measurement HIGH byte
    MAG_OUT_Y_L,        ///< Magnetometer Y axis measurement LOW byte
    MAG_OUT_Y_H,        ///< Magnetometer Y axis measurement HIGH byte
    MAG_OUT_Z_L,        ///< Magnetometer Z axis measurement LOW byte
    MAG_OUT_Z_H         ///< Magnetometer Z axis measurement HIGH byte
} MAG_OUT_DATA;

#include <i2cbus/i2cbus.h>
/**
 * @brief LSM9DS1 Device Struct
 * 
 */
typedef struct
{
    i2cbus *accel_dev;
    i2cbus *mag_dev;
} lsm9ds1;

#define MAG_WHO_AM_I 0x0f    ///< Address of magnetometer ID register
#define MAG_IDENT 0b00111101 ///< Magnetometer ID

int lsm9ds1_init(lsm9ds1 *, uint8_t, uint8_t, uint8_t, uint8_t);
int lsm9ds1_config_mag(lsm9ds1 *, MAG_DATA_RATE, MAG_RESET, MAG_DATA_READ);
int lsm9ds1_reset_mag(lsm9ds1 *);
int lsm9ds1_read_mag(lsm9ds1 *, short *);
int lsm9ds1_offset_mag(lsm9ds1 *, short *);
void lsm9ds1_destroy(lsm9ds1 *);

#endif // LSM9DS1_H