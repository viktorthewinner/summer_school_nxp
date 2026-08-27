# led_blinky_peripheral

## Overview
The LED Blinky demo application provides a sanity check for the new SDK build environments and board bring up. The LED Blinky demo 
uses the systick interrupt to realize the function of timing delay. The example takes turns to shine the LED. The purpose of this 
demo is to provide a simple project for debugging and further development.
The code of this demo has been prepared and updated for use with the MCUXpresso Configuration Tools (Pins/Clocks/Peripherals).

## Supported Boards
- [FRDM-MCXA153](#board-specific-information-for-frdmmcxa153)

---

## Board-Specific Information for frdmmcxa153

### Hardware requirements
- Type-C USB cable
- FRDM-MCXA153 board
- Personal Computer

### Board settings
No special settings are required.

### Prepare the Demo
1.  Connect a Type-C USB cable between the host PC and the MCU-Link port(J15) on the target board.
2.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.
4.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

### Running the demo
When the demo runs successfully, you will find the LED is blinking.
