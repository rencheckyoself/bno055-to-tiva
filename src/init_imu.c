#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/qei.h"

#include "bno055.h"
#include "quaternion.h"
#include "init_imu.h"

static s8 error; // converted data to euler angles
static struct bno055_euler_float_t last_pos; // converted data to euler angles

void ConfigureI2C()
{
    /*
      See section 15 and 17 of the TivaWare™ Peripheral Driver Library for more usage info.
    */

    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    // Wait for peripheral to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {}

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

s8 _imu_i2c_read(u8 dev_address, u8 reg_address, u8 *arr_data, u8 count)
{
  s8 comres = 0;
  // This function is used to select the device to read from
  // false == write to slave
  I2CMasterSlaveAddrSet(I2C0_BASE, dev_address, false);

  // Set the I2C Bus to tell the device which first register is meant to be read
  I2CMasterDataPut(I2C0_BASE, reg_address);

  // send slave address, control bit, and register address byte to slave device
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

  //wait for MCU to finish transaction
  while(I2CMasterBusy(I2C0_BASE));

  // Read in 1 byte
  if(count == 1)
  {
    // specify that we are going to read from slave device
    // true == read from slave
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_address, true);

    //send slave address, control bit, and recieve the byte of data
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(I2CMasterBusy(I2C0_BASE));

    // write byte to data variable
    arr_data[0] = I2CMasterDataGet(I2C0_BASE);

    // Check for errors
    if (I2CMasterErr(I2C0_BASE)==I2C_MASTER_ERR_NONE)
    {
      comres = 0; // success
    }
    else
    {
      comres = -1; // error occured
    }
  }

  // Read in 2 bytes
  else if(count == 2)
  {
    // Get first byte
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_address, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C0_BASE));

    // put read data in data array
    arr_data[0] = I2CMasterDataGet(I2C0_BASE);

    // Check for errors
    if (I2CMasterErr(I2C0_BASE)==I2C_MASTER_ERR_NONE)
    {
      comres = 0; // success
    }
    else
    {
      comres = -1; // error occured
    }


    // Read final byte from slave
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C0_BASE));
    // put read data in data array
    arr_data[1] = I2CMasterDataGet(I2C0_BASE);

    // Check for errors
    if (I2CMasterErr(I2C0_BASE)==I2C_MASTER_ERR_NONE)
    {
      comres = 0; // success
    }
    else
    {
      comres = -1; // error occured
    }

  }

  // Read in more than 2 bytes
  else
  {
    u8 i;
    for(i = 0; i < count; i++)
    {
      if(i == 0)
      {
        // Start Communication
        I2CMasterSlaveAddrSet(I2C0_BASE, dev_address, true);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
      }
      else if(i == count -1)
      {
        // Read the last byte
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
      }
      else
      {
        // read middle byte
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
      }

      while(I2CMasterBusy(I2C0_BASE));

      // put read data in data array
      arr_data[i] = I2CMasterDataGet(I2C0_BASE);

      if (I2CMasterErr(I2C0_BASE)==I2C_MASTER_ERR_NONE)
      {
        comres = 0; // success
      }
      else
      {
        comres = -1; // error occured, end comms and exit
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        while(I2CMasterBusy(I2C0_BASE));
        i = count;
      }
    }
  }

  return comres;
}

s8 _imu_i2c_write(u8 dev_address, u8 reg_address, u8 *var_data, u8 count)
{
    s8 comres = 0;
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_address, false);

    //send the slave address, control bit, and registar address for where to write to
    I2CMasterDataPut(I2C0_BASE, reg_address);

    //Initiate send of data from the MCU
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait until MCU is done transferring.
    while(I2CMasterBusy(I2C0_BASE));

    // the BNO055 only ever writes 1 byte of info so if count != 1, throw an error
    if(count == 1)
    {

      // send the information to write
      I2CMasterDataPut(I2C0_BASE, *var_data);

      // Initiate send of data from the MCU
      I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

      // Wait until MCU is done transferring
      while(I2CMasterBusy(I2C0_BASE));

      if(I2CMasterErr(I2C0_BASE)==I2C_MASTER_ERR_NONE)
      {
        comres = 0; // Success
      }
      else
      {
        comres = -1; // Error
      }
    }
    else
    {
      comres = -1; // Error
    }

    return comres;
}

void ms_delay(u32 ms)
{
  SysCtlDelay(ms * 5334); // 16000000MHz/3000 ~= 5334 assembly commands per ms
}

void init_imu()
{
  static struct bno055_t sensor;

  // initialize setup struct and populate the required information
  sensor.bus_write = _imu_i2c_write;
  sensor.bus_read = _imu_i2c_read;
  sensor.delay_msec = ms_delay;
  sensor.dev_addr = BNO055_I2C_ADDR1;

  // bno055 builtin initialization function
  error = bno055_init(&sensor);
}

void set_imu_mode()
{
  error = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
}

struct bno055_euler_float_t get_abs_position()
{
  struct bno055_quaternion_t q; // the raw data from the sensor
  Quaternion qf; // cast the raw data from int into float

  error += bno055_read_quaternion_wxyz(&q); // read the sensor

  qf = bnoquat_to_float(&q); // convert raw data to a float

  scale_divide(&qf, QUATERNION_SCALING); // scale the raw data per the sensor data sheet

  last_pos = toEuler(&qf); // calculate the euler angles

  return last_pos;
}

Calibration calibrate_imu()
{
  u8 gyro_cal = 0, acc_cal = 0, mag_cal = 0, sys_cal = 0;

  error = bno055_get_gyro_calib_stat(&gyro_cal);
  error += bno055_get_mag_calib_stat(&mag_cal);
  error += bno055_get_accel_calib_stat(&acc_cal);
  error += bno055_get_sys_calib_stat(&sys_cal);

  Calibration output;
  output.gyro = gyro_cal;
  output.accl = acc_cal;
  output.magn = mag_cal;
  output.syst = sys_cal;

  return output;
}

s8 get_error()
{
  return error;
}
