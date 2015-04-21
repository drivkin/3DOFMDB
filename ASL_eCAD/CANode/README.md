# CAN Node

## Version
3.0

## Purpose

This is a small development platform built around the dsPIC33F-series of processors by Microchip as a replacement for the Explorer16 development board to be used in the field. With its PICtail extension boards installed, the Explorer16 board is quite large and has sketchy connections to these boards while under motion.

This replacement board was designed to be small, easy to manufacture on a board router, and be easily extensible via Shields ala Arduino platform (though with a different layout).

With the primary design goals of tolerating 24V input, having a CAN transceiver onboard, and being easily extensible and manufactorable, 

 * Tolerate 24V input voltage
 * Have CAN onboard
 * Include at least one status LED
 * Be extensible

## Features
 * 8-36V input
 * Small size (<50mm x <50mm)
 * Onboard CAN transceiver w/ optional grounding & 120ohm terminating resistor
 * 2 user LEDs: amber and red
 * 1% analog temperature sensor (provided by TC1047)
 * 1% analog input voltage sensor (voltage divider)
 * FTDI-cable header for debugging output via UART (TX & RX only)
 * PICkit-compatible ICSP header for programming & debugging
 * Reset button
 * .1" spacing between shield headers allowing perfboard prototyping for shields
 * Efficient switching regulator providing 1.5A at 5V with an LDO capable of 1A at 3.3V
 * Compatibility with all 28-pin PIC24H/dsPIC33E/dsPIC33F processors by Microchip
  * requires PGED at pin 14 and PGEC at pin 15

## Caveats/Notes

This board was designed for manufactoring that is capable of plated vias, so this board may not be possible to manufacture using a board router.

The CANode is a little under 50mm x 50mm, offering a very small amount of space for shields unless they want to exceed the board dimensions. This is one of the primary drawbacks of this board.

Additionally the PGC/PGD pin pairs have a 3.3kOhm resistor inline on those connections. This is to allow the programmer or debugger to operate while the shield is still connected. This does, however, render the internal weak pull-up resistors in the microcontroller useless for active-low, open-drain inputs. When the input is low, the pull-ups are about 90kOhm and the inline resistor is about 3.3kOhm creating a voltage divider that leave a high voltage on the pin.

If a 5V or 3.3V part is being decided between, using a 5V part will improve efficiency. The operating efficiency of the PTN78000 switching regulator increases as load increases, while the efficiency of the 3.3V LDO decreases as the load increases. Therefore it is suggested that parts be 5V where possible.

Programming and debugging is done through PGE3 on the dsPIC33F & PIC24H and through PGE2 on the dsPIC33E. The 3.3V from the programmer is connected to the 3.3V line of the board, so that the programmer can program the processor without it having a separate power source. This may not always work depending on the power draw of the specific processor, but has been shown to work with a dsPIC33FJ128MC802 and Microchip's PICkit3 programmer.

The CAN connector is a standard .1" header. It follows the pinout: 1 - CAN_HI, 2 - CAN_LO, 3 - GND. Pin 3 is unconnected by default with SJ2 providing a solder jumper to connect it to ground. Additionally SJ1 provides a solder-jumper connection for a 120Ohm terminating resistor across CAN_HI and CAN_LO, as they're a differential pair. Standard RC servo cables are a good choice for this connection, even having a black wire on the end for the GND pin facilitating orientation. A silkscreen label for GND is on the board to assist with orientation.

The UART connection is designed to connect to a TTL-level FTDI USB cable (TTL-232R-3V3) using the standard pinout (as seen on the cable side): 1 - GND, 4 - TX, 5 - RX. A silkscreen label for GND is on the board to assist with orientation.

## Designing shields

Shields will likely come in 2 primary designs, one small and one large. The small board will merely use the board area between the two headers. The larger design will extend upwards from the headers to the two top mounting holes, which can be used to support the shield via standoffs.

For the small board design, an orientation indicator should be added in the south-east corner. The indicator on the CANode board is at 1.67x0.25 and the shield's indicator should align with this. The headers have been oriented so that a power-ground short won't occur with the provided power pins, but the shield will likely not operate in a rotated position.

The large board may use the mounting holes at the bottom of the board, but it's recommended that it leaves the area between the mounting holes uncovered for usability. Besides this, there are a few other considerations to keep in mind. At the top the power connector hits the shield, so no components can be mounted above it. The capacitor next to it is also quite tall at 8.5mm and should not have components placed above it as well, though through-hole components should be able to fit (do at your own peril).


## Files
This project is an Eagle 6.x project relying on the ASL.lbr component library, which should be in the root directory.

./Board: Contains the Eagle project for the board.
./Code: Contains testcode in the form of a Simulink model (relies on both the ECAN_dspic project and Lubin's PIC blockset)
./Shields: Contains some general-use example shields for use with this board.

## Partslist

CANode board
  Qty Value                    Device               Partnames             Manufacturer ID                     Price (qty of 10, USD, Digikey)
  1   >=2A,>=40V               DIODE-SCHOTTKY240A   D1                    B240A-13-F                          0.38
  1   green                    LED1206              LED3                  LG N971-KN-1 (green)                0.09
  1   amber                    LED1206              LED2                  APT3216YC (amber)                   0.149
  1   red                      LED1206              LED1                  LH N974-KN-1 (red)                  0.09
  1   straight,.1"             M03LOCK              JP3                   68000-436HLF                        0.641
  1   right-angle,.1"          M06LOCK              JP2                   68016-436HLF                        1.015
  1   straight,.1"             M06LOCK              JP1                   68000-436HLF                        0.641
  1                            TAC_SWITCHPTH        S1                    B3F-1020                            0.393
  3   .1uF                     CAP1206              C7, C9, C10           C1206C104K5RACTU                    0.072
  1   1uF, >3.3V               CAP1206              C6                    CC1206KKX7R7BB105                   0.111
  1   2N7002P                  2N7002P              U6                    2N7002P,215                         0.127
  1   2k, 1%                   RESISTOR0805-RES     R9                    ERJ-6ENF2001V                       0.1
  4   4.7uF, >5V               CAP1206              C1, C2, C3, C5        UMK316AB7475KL-T                    0.19
  2   10k                      RESISTOR0805-RES     R5, R10               ERJ-6GEYJ103V                       0.1
  2   3.3k                     RESISTOR0805-RES     R11, R12              ERJ-6GEYJ332V                       0.1
  1   10uF, >3.3V              CAP1206              C8                    C3216Y5V1A106Z/0.85                 0.124
  2   21k, 1%                  RESISTOR0805-RES     R1, R7                ERJ-6ENF2102V                       0.1
  1   120                      RESISTOR0805-RES     R6                    ERJ-6GEYJ121V                       0.1
  1   100uF, >5V, <11mm height CAP_POL              C4                    10ZLG100MEFC6.3X7, UVR1E101MED      0.208,0.154
  3   330                      RESISTOR0805-RES     R2, R3, R8            ERJ-6GEYJ331V                       0.1
  1   470                      RESISTOR0805-RES     R4                    ERJ-6GEYJ471V                       0.1
  1   DSPIC33FJ128MC802-I/SO   DSPIC33FJ802MC-28    U2                    DSPIC33FJ128MC802-I/SO              5.26
  2   female                   M14DUALROW           J1, J3                PPTC072LFBN-RC                      0.959
  1   MCP1826T-3302E/DC        MCP1826              U3                    MCP1826T-3302E/DC                   0.68
  1   MCP2551-I/SN             MCP2551              U1                    MCP2551-I/SN                        1.02
  1   POW_CONNECTMALE-RA       POW_CONNECTMALE-RA   J2                    0026605020                          0.259
  1   PTN78000W                PTN78000THROUGH-HOLE U4                    PTN78000WAH                         13.5
  4   fm/fm, 1/4", 4-40, 3/8"  standoffs                                  Keystone Electronics 2202           0.402
  4   4-40, 1/4", philips      screws                                     PMS 440 0025 PH                     0.0298 (x100)
  1   TC1047                   TC1047               U5                    TC1047AVNBTR                        0.62

CANode Shield
  Qty Value                    Device               Partnames             Manufacturer ID
  2   male                     M14DUALROW                                 67996-114HLF                        0.703

Connectors
  Qty Value                    Device               Partnames             Manufacturer ID
  1   2-pos, female            POW_CONNECT                                09-48-3025                          0.425

Total cost: ~$32

## Board Assembly
There are two methods for populating the board, either by soldering individual components or by doing the entire top at once through a reflow process.

### Individual Soldering
If you are soldering by hand follow the following steps to populate the board. After every step check connections with a DMM.
 1. Add the power connector and reverse polarity diode.
 2. Add the 5V switching regulator and related components.
   * Power the board up and confirm stable 5V operation.
 3. Add the 3.3V regulator and related components.
   * Power the board up and confirm stable 3.3V operation.
 4. Solder on the reset button circuitry.
   * Use a DMM to check the lines are default high and pulled low when the button is pressed.
 5. Add the PIC processor and ICSP header.
   * Power it on and connect to a PC and attempt to program.
 6. Finish adding the rest of the onboard components doing all through-hole parts last.
 7. Run example code provided in this repository and confirm that it works as expected.

### Batch Soldering
 1. Use a skillet or reflow oven to solder down all SMDs on the top layer.
 2. Solder down the 5V switching regulator, output capacitor, and power connector.
 3. Test the board powering up with both 5V and 3.3V power.
 4. Solder down the reset circuitry and the ICSP header.
   * Confirm operation by attempting to program the PIC.
 5. Finish adding all through-hole parts on the top layer and confirm operation using provided test code.
