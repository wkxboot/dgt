## Overview
-----------
The lpc_adc_burst example shows how to use LPC ADC driver with the burst mode.

In this example, an outside input is used to created the input analog signal.
When user type in any key from the keyboard, the burst mode is enabled. Then the conversion 
sequence A would be started automatically, till the burst would be disabled in conversion completed ISR. 

## Functional Description
-------------------------
1.This example demonstrates how to configure the A sequences with burst mode, you can configure channel via 
  "DEMO_ADC_SAMPLE_CHANNEL_NUMBER".
  
2.Before configuration of the ADC begins, the ADC is put through a self-calibration cycle.  

3.Enable the Conversion-Complete or Sequence-Complete interrupt for sequences A.
  
4.After ADC channels are assigned to each of the sequences, if the user enters any key via terminal, burst mode will start.  
  
5.When the first conversion completes, the interrupt would be triggered. The ISR will stop the burst mode and print conversion result 
  to terminal.

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
J5-1 is ADC input pin, which can sample external voltage.

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




4. Monitor the information on the debug console.

## Expected Result
------------------------
Press any key and print user channel's result in serial terminal.

