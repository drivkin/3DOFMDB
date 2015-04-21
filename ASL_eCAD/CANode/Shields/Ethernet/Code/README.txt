This project contains code for use with the Explorer 16 with a PIC24F128GA010 PIM installed. It
provides a serial2udp bridge. Serial is provided at 115200, 8n1. UDP is 

This code will eventually be modified to work with the CAN Node developed by the ASL.

 0. Install Microchip's TCPIP 5.x library.
 1. Create new MPLAB X project.
 2. Add all .c and .h files from this folder to the project.
 3. Add from the TCPIP stack: ARP.c, Delay.c, DHCPs.c, ENC28J60.c, Helpers.c, IP.c, Tick.c, UDP.c
 4. Modify the project to include headers from MICROCHIP_LIB_DIR/Microchip/Include
 5. Modify the project to include headers from the folder containing this project's header files.
 6. Compile
 7. ???
 8. Profit!