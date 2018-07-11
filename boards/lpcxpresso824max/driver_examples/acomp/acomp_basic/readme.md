## Overview
-----------
The LPC_ACOMP Bsic Example shows the simplest way to use ACOMP driver and help user with a quick start.

In this example, user should indicate an input channel to capture a voltage signal (can be controlled by user) as the 
ACOMP's negative channel input. On the postive side, the internal voltage ladder is used to generate the fixed voltage about
half value of reference voltage.

When running the project, change the input voltage of user-defined channel, then the comparator's output would change
between logic one and zero when the user's voltage crosses the internal ladder's output. The endless loop in main() function
would detect the logic value of comparator's output, and change the LED. The LED would be turned on when the compare
output is logic one, or turned off when zero.

## Functional Description
-------------------------
In this example, ACOMP will compare the input voltage of user-defoned channel and internal voltage ladder's output, then ACOMP 
will use polling mode to check the result of compare and turn on/off LED.

1. Once the project starts, main routine will initialize clock, pin mux configuration, LED configuration
   and configure the ACOMP module to make it work.

2. Configure ACOMP negative and postive input channels and if use internal voltage ladder, enable and configure internal
   voltage ladder.

3. Use polling method to check whether the ACOMP output is logic one, if yes, turn on the LED, otherwise, turn off the LED. 

## Toolchain Supported
---------------------
- IAR embedded Workbench 8.11.3
- Keil MDK 5.23
- MCUXpresso10.1.0

## Hardware Requirements
------------------------
- Micro USB cable
- LPCXpresso824MAX board
- Personal Computer

## Board Settings
------------------------
J5-2 is ACOMP negative input pin, which can sample external voltage.

## Run the Project
------------------------
Run this example by performing the following steps:

1. Connect a micro USB cable between the PC host and the CMSIS DAP port(J3 on the 
   LPCXpresso824MAX board) for receiving debug information.

2. Open a serial terminal in PC(for example PUTTY) with the following settings:
   - 9600 baud rate
   - 8 data bits
   - No parity
   - One stop bit
   - No flow control

3. Compile and download the program to the target board.
   More information about how to compile and program the project can refer to 

   [Getting Started with MCUXpresso SDK](../../../../../docs/Getting Started with MCUXpresso SDK.pdf).

4. Start the slave board on another board first, then launch the debugger in your IDE to
   begin running this project.

5. Monitor the information on the debug console.

## Expected Result
------------------------
If the ACOMP input pin(J5-2) connects the GND, LED(D1) will be turned on.
If the ACOMP input pin(J5-2) connects the 3.3V, LED(D1) will be turned off.

