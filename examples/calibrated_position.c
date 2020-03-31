//*****************************************************************************
//
// This file is a modified version of the 'hello_world' example provided by TI.
//
//*****************************************************************************

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "bno055.h"
#include "quaternion.h"
#include "init_imu.h"

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************
void printAngle(char* axis, float val);

int main(void)
{
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    FPULazyStackingEnable();

    // Set the clocking to run directly from the crystal.
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Initialize the UART.
    ConfigureUART();

    // Initialize the I2C
    ConfigureI2C();

    ms_delay(10);

    // Initialize the sensor
    init_imu();

    // Set the sensor operation mode, note in this mode the sensor should be calibrated first
    set_imu_mode(BNO055_OPERATION_MODE_NDOF);

    struct bno055_euler_float_t ea; // converted data to euler angles

    int calibrated = 0;
    Calibration cal;


    // Loop until the device is fully calibrated
    // Read 'Calibration' section of the data sheet for more info on how to calibrate
    while(calibrated == 0)
    {
      cal = calibrate_imu();

      UARTprintf("Calibration: ");
      UARTprintf("Gyro: %d\t", cal.gyro);
      UARTprintf("Acc: %d\t", cal.accl);
      UARTprintf("Mag: %d\t", cal.magn);
      UARTprintf("Sys: %d\n", cal.syst);

      if(cal.syst == 3)
      {
        calibrated = 1;
      }

      ms_delay(10);
    }

    while(1)
    {
      ea = get_abs_position(); // calculate the euler angles

      printAngle("x", ea.r);
      printAngle("y", ea.p);
      printAngle("z", ea.h);
      UARTprintf("\n");

      ms_delay(10);
    }
}



void printAngle(char* axis, float val)
{
  char msg_buf[20]; // message

  sprintf(msg_buf, "%s = %f\t", axis, val);
  UARTprintf("%s",msg_buf);
}
