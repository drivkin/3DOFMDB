# Version 3.0
 * Added inline 3.3k resistors to the PGED/PGEC on the headers so that programming can still occur when a driving voltage is applied.
 * Changed the mounting holes so that there are 4, one in each corner.
 * Moved the shield headers to be parallel and opposing. Additionally they have been oriented so that inserting a shield rotated 180deg won't cause problems with power/ground pins.
 * Switched the surface-mount capacitor to a through-hole variant as they're a little more secure and easier to solder.
 * Improved groundplane to cover most of the surface.

# Version 2.0
 * Switched programming/debugging pins from PGE*2 to PGE*3 to support the new PIC24E and dsPIC33E.
 * Exposed the programming pins to the shield so that 14 GPIO pins are available.
 * Changed the available pins to include the programming/debugging pins PGE*3 so that all 4 PWM*H pins are exposed along with 2 PWM*L pins.

# Version 1.1 (Shield is electrically compatible with 1.0)
 * Corrected errors from version 1.0:
   * NPN transistor now used correctly as a low-side driver.
   * ICSP programming pins switched and now correct.
 * Separated LEDs for power from user-controlled LEDs.
 * Schematic includes LED colors.
 * Small routing efficiency improvements.
 * Added a GND label to the top plane so that it's shown when a board router is used.

# Version 1.0
 * Initial version
 * Includes several small errors including programming/debugging being broken. Unuseable w/o modifications.
