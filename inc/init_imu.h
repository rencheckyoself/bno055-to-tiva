/// \file bmx_imu.h
/// \brief Functions to interact with the IMU. These functions initialize and allow
/// for easy interaction with the BNO055 IMU.

#ifndef BMX_IMU_HG
#define BMX_IMU_HG

#include "bno055.h"

/// \brief Configure I2C0. Sets the proper pins and other tiva settings to initialize i2c communication.
///
void ConfigureI2C();

/// \brief used read a value through i2c bus.
/// \param dev_address: the BNO055 address being used. This should be set in the BNO055_t struct.
/// \param reg_address: the registart to be read. Each function in the BNO055 driver lib has this preset.
/// \param *arr_data: an address to the array to store the data in. Each function in the BNO055 driver lib has this preset.
/// \param count: the number of bytes to read. Each function in the BNO055 driver lib has this preset.
/// \return comres: the result of the communication, (Per BNO055 documentation: 0 if success. -1 if failure)
s8 _imu_i2c_read(u8 dev_address, u8 reg_address, u8 *arr_data, u8 count);

/// \brief used to write a value through i2c bus.
/// \param dev_address: the BNO055 address being used. This should be set in the BNO055_t struct.
/// \param reg_address: the registart to be read. Each function in the BNO055 driver lib has this preset.
/// \param *var_data: an address to the variable to store the data in. Each function in the BNO055 driver lib has this preset.
/// \param count: the number of bytes to read. Each function in the BNO055 driver lib has this preset.
/// \return comres: the result of the communication, (Per BNO055 documentation: 0 if success. -1 if failure)
s8 _imu_i2c_write(u8 dev_address, u8 reg_address, u8 *var_data, u8 count);

/// \brief initiate a blocking delay
/// \param ms: the duration (in milliseconds) to delay for
void _ms_delay(u32 ms);

/// \brief used to set the initialization struct needed by the BNO055 driver
///
s8 init_imu(struct bno055_t *sensor);

#endif
