//*****************************************************************************
//
// This file is a modified version of the 'hello_world' example provided by TI.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
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
    set_imu_mode();

    struct bno055_euler_float_t ea; // converted data to euler angles

    int calibrated = 0;
    Calibration cal;

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
