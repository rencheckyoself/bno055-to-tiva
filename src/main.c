//*****************************************************************************
//
// main.c - Simple hello world example.
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

//*****************************************************************************
//
//! A very simple ``hello world'' example.  It simply displays ``Hello World!''
//! on the UART and is a starting point for more complicated applications.
//!
//! UART0, connected to the Virtual Serial Port and running at
//! 115,200, 8-N-1, is used to display messages from this application.
//
//*****************************************************************************

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
void
ConfigureUART(void)
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
// Print "Hello World!" to the UART on the evaluation board.
//
//*****************************************************************************
int
main(void)
{
    //volatile uint32_t ui32Loop;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2 & PF3).
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    //
    // Initialize the UART.
    //
    ConfigureUART();

    //
    // Hello!
    //
    UARTprintf("Hello, world!\n");

    // do some unnecessary floating-point math to see if the Makefile works:
    float foo;
    double foo_int;
    double foo_frac;
    foo = sqrtf(2.05f*2.05f);
    foo_frac = modf((double) foo, &foo_int);
    UARTprintf("(int) foo = %d.\n",(int)foo);
    UARTprintf("(int) foo_int = %d.\n",((int) foo_int));
    UARTprintf("(int) foo_frac = %03d.\n",((int) (1000.0f*foo_frac)));

    char msg_buf[20];
    sprintf(msg_buf,"foo = %f.\n",foo);
    UARTprintf("%s",msg_buf);

    // reset our dummy floating point variable to 0:
    foo = 0.0f;
    while(1)
    {
        // increment our dummy floating point variable:
        foo++;

        // Turn on the LED.
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

        //
        // Delay for a bit.
        //
        SysCtlDelay(SysCtlClockGet() / 10 / 3);

        //
        // Turn off the BLUE LED.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);

        //
        // Delay for a bit.
        //
        SysCtlDelay(SysCtlClockGet() / 10 / 3);

        //
        // Hello!
        //
        sprintf(msg_buf,"foo = %5.3f.\n",sqrtf(foo));
        UARTprintf("%s",msg_buf);

    }
}

