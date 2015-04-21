# Testing the CANode

The `Test.c` file included in this directory completely tests the hardware onboard the CANode board, both for the dsPIC33FJ128MC802 and dsPIC33EP256MC502 processors. Others may work, but have not been tested.

## Things tested:
 * All digital I/O shield connections. All pins are flipped high and low, when combined with the tester shield also in this repository, all LEDs should blink.
 * The amber and red status LEDs on the CANode should alternate blinking at a rather rapid pace, about 14Hz.
 * The input voltage sensor and temperature sensor are read.
 * The CAN peripheral outputs the raw ADC and converted values of the input voltage and temperature sensors, but the output values are multiplied by 100.
 * The UART also outputs the raw and converted sensor values for the input voltage and temperature sensors.

## Usage
The use of this testing code is for the CANode. It is recommended that the tester shield in this same repository also be used. It provides quick visual confirmation that all shield pins are working.

If running this code in debugging mode, note that the PGC and PGD pins on the shield headers will not be available as they'll be controlled by the PICkit and not driven by the PIC.

## Notes on using the tester shield
 * The reset button should work exactly as the onboard reset button does.
 * If running in debug mode, LED23 will be partially lit and LED12 will be off while running. They're only usable during regular operation.