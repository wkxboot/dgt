## Overview
-----------
The WDOG Example project is to demonstrate usage of the KSDK wdog driver. In this example,
quick test is first implemented to test the wdog. And then after 5 times of refreshing the
watchdog, a timeout reset is generated.

## Functional Description
-------------------------
This example show wdog refresh and reset fuction.

The example will initialize clock, pin mux configuration, initialize watch dog OSC and LED，then enable interrupt. After that,
configure the WWDT module to set watchdog feed time, warning time, window time. The example feeds WWDT only for the first 5 warning 
interrupts. After 5 times of refreshing WWDT, the WWDT module will generate a timeout reset.

## Toolchain Supported
---------------------
- IAR embedded Workbench 8.22.2
- Keil MDK 5.24a
- MCUXpresso10.2.0

## Hardware Requirements
------------------------
- Micro USB cable
- LPCXpresso824MAX board
- Personal Computer

## Board Settings
------------------------
No special settings are required.

## Run the Project
------------------------
Run the example by performing the following steps:

1. Connect a micro USB cable between the PC host and the CMSIS DAP port(J3 on the 
   LPCXpresso824MAX board).

2. Open a serial terminal in PC(for example PUTTY) with the following settings:
   - 9600 baud rate
   - 8 data bits
   - No parity
   - One stop bit
   - No flow control

3. Compile and download the program to the target board.
   More information about how to compile and program the project can refer to 

   [Getting Started with MCUXpresso SDK](../../../../../docs/Getting Started with MCUXpresso SDK.pdf).

4. Launch the debugger in your IDE to begin running the project.

5. Monitor the information on the debug console.

## Expected Result
------------------------
The log below shows example output of the WWDT driver example in the terminal window:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
--- Window mode refresh test start---
Watchdog reset occurred
~~~~~~~~~~~~~~~~~~~~~~

