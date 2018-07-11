## Overview
-----------
This example demonstrates configuration and use of the I2C master module in dma-driven
mode on communication with a I2C slave module calling the I2C functional APIs. This example
should work together with the `lpc_i2c_dma_b2b_slave` example.

**Transactional API and Functional API**

- Low level functional APIs provide common peripheral functionality, abstracting the 
  hardware peripheral register access into a set of state less basic functional operations.
  These APIs primarily focus on the control, configuration, and function of basic 
  peripheral operations. 

- Transactional APIs provide a quick method for customers to utilize higher-level 
  functionality of the peripherals. The transactional APIs utilize interrupts and perform 
  asynchronous operations without user intervention. Transactional APIs operate on high-
  level logic that requires data storage for internal operation context handling. However,
  the peripheral drivers do not allocate this memory space. Rather, the user passes in the
  memory to the driver for internal driver operation. 

- All driver examples calling the transactional API get the `transfer` in the project 
  naming. If the code size and performance are critical requirements, see the transactional
  API implementation and write custom code.

## Functional Description
-------------------------
In this example, I2C master will communicate with a slave module on another board, master 
will use dma mode and slave will use polling mode.

1. Once the project starts, main routine will initialize clock, pin mux configuration, and
   configure the I2C module to make it work in master DMA mode.
   
2. To make I2C master module run in dma mode, one dma channel should be configured for 
   sending and receiving.

3. Before starting the I2C master transmission, use DMA driver API to start transfer user 
   data, the I2C hardware will automatic read a byte after send a STOP, the I2C module may
   need to send a STOP when last byte was sent, so create a polling mode to send the last byte.

4. Checking if the data from slave is all right when transfer complete, and print the 
   information to the debug console on PC.


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
Connect pins of I2C master and slave(on another board) as below:

    Master - I2C0                  Slave - I2C0   
Pin Name   Board Location     Pin Name   Board Location
SCL        J1-1               SCL        J1-1
SDA        J1-2               SDA        J1-2
GND        J1-4               GND        J1-4

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

4. Start the slave board first, then launch the debugger in your IDE to begin running this project.

5. Monitor the information on the debug console.

## Expected Result
------------------------
The I2C master module will communicate with slave module in dma mode, slave module 
should be started first. Once the communication between master board and slave board is 
completed, the received data and information will be printed to the debug console on PC.
