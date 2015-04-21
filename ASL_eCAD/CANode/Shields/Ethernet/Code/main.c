// Define this as the file containing main before including configuration files.
#define MAIN_FILE
#include "HardwareProfile.h"
#include "TCPIPConfig.h"

#define THIS_IS_STACK_APPLICATION
#include "TCPIP Stack/TCPIP.h"
#include "TCPIP Stack/StackTsk.h"
#include <GenericTypeDefs.h>
#include <stdio.h>

// Calculate the BRG register value necessary for 115200 baud with a 80MHz clock.
#define BAUD115200_BRG_REG 21

// Declare AppConfig structure and some other supporting stack variables. This is a global variable
// used by all of the TCPIP stack to pull common info.
APP_CONFIG AppConfig = {};

// Private helper functions.
static void InitAppConfig(void);
static void InitBoard(void);

// Declare a module-level variable for use as a temporary variable when processing packets. This
// will be filled with the info of the remote node after a UDP packet is received.
static NODE_INFO remoteNode;

// The local socket used for the UDP server for data connections.
static UDP_SOCKET localServerSocket;

// The remote socket for transmitting UDP packets to. It's set as soon as the first proper UDP data
// packet is received.
static UDP_SOCKET remoteServerSocket = INVALID_UDP_SOCKET;

int main(void)
{
    static DWORD t = 0;

	// Initialize hardware
	InitBoard();

    // Initialize stack-related hardware components that may be 
    // required by the UART configuration routines
    TickInit();

    // Initialize Stack and application related NV variables into AppConfig.
    InitAppConfig();

	// Seed the LFSRRand() function.
	LFSRSeedRand(GenerateRandomDWORD());

	// Initialize the MAC library.
    MACInit();

	// Initialize the ARP library.
    ARPInit();

	// Initialize UDP.
    UDPInit();

	// Open up a socket for our UDP server.
	localServerSocket = UDPOpenEx(NULL, UDP_OPEN_SERVER, UDP_SERVER_PORT, UDP_SERVER_PORT);
	if (localServerSocket == INVALID_UDP_SOCKET) {
		while (1);
	}

    // Now that all items are initialized, begin the co-operative
    // multitasking loop.  This infinite loop will continuously 
    // execute all stack-related tasks, as well as your own
    // application's functions.  Custom functions should be added
    // at the end of this loop.
    // Note that this is a "co-operative mult-tasking" mechanism
    // where every task performs its tasks (whether all in one shot
    // or part of it) and returns so that other tasks can do their
    // job.
    // If a task needs very long time to do its job, it must be broken
    // down into smaller pieces so that other tasks can have CPU time.
    while(1) {
        // Blink LED0 (right most one) every second.
        if(TickGet() - t >= TICK_SECOND/2ul)
        {
            t = TickGet();
            LED0_IO ^= 1;
        }

		// First prepare all UDP stuff.
		UDPTask();

		WORD dataCount;
		IP_ADDR tempLocalIP;
		BYTE cFrameType;
		BYTE cIPFrameType;

		// And then process as many incoming packets as we can. This loop ends as soon as a proper
		// packet IP or MAC packet is received.
		int cont = 1;
		while (cont) {
			// Before fetching new data, be sure all old UDP data is discarded.
			UDPDiscard();

			// Fetch a packet.
			if (!MACGetHeader(&remoteNode.MACAddr, &cFrameType)) {
				break;
			}

			// And now process the packet.
			switch(cFrameType) {
				// Process any ARP messages. These are used for determining a MAC-to-IP mapping.
				case MAC_ARP:
					ARPProcess();
					break;

				// Process any IP messages, specifically UDP.
				case MAC_IP:
					if (!IPGetHeader(&tempLocalIP, &remoteNode, &cIPFrameType, &dataCount)) {
						break;
					}

					if (cIPFrameType == IP_PROT_UDP) {
						// Stop processing packets if we came upon a UDP frame with application data
						// in it
						if (UDPProcess(&remoteNode, &tempLocalIP, dataCount)) {
							cont = 0;
						}
					}

					break;
			}
		}

		// Echo UDP packets back.
		int dataLen;
		if ((dataLen = UDPIsGetReady(localServerSocket))) {
			BYTE data[dataLen];
			UDPGetArray(data, dataLen);
			if (remoteServerSocket == INVALID_UDP_SOCKET) {
				// Warnings about remoteNode on the following line is fine. Really they just wrote
				// this function weird.
				remoteServerSocket = UDPOpenEx(&remoteNode, UDP_OPEN_NODE_INFO, 0, UDP_SERVER_PORT);
				if (remoteServerSocket == INVALID_UDP_SOCKET) {
					while (1);
				}
			}
			if (UDPIsPutReady(remoteServerSocket)) {
				DWORD dataQueued = UDPPutArray(data, dataLen);
				if (dataQueued < dataLen) {
					while (1);
				}
				UDPFlush();
			}
		}

        // Now run the DHCP server task. This needs to be run continuously as there's no way to tell
		// if any clients are connected to perform the lease once post-init. The default lease time
		// is also super short, 60s, so the server needs to be able to process new DHCP stuff.
        DHCPServerTask();
    }
}

static void InitBoard(void)
{    
    // Initialize LED0 as an output.
    LED0_TRIS = 0;

	// Set extra configuration stuff. This may depend both on the board used and the processor.
	// Defined in HardwareProfile.h.
	LowLevelInit();

	// Deassert all SPI chip select lines so there isn't any problem with initialization order.
    ENC_CS_IO = 1;
    ENC_CS_TRIS = 0;
}

static void InitAppConfig(void)
{
	AppConfig.MyMACAddr.v[0] = MAC_BYTE1;
	AppConfig.MyMACAddr.v[1] = MAC_BYTE2;
	AppConfig.MyMACAddr.v[2] = MAC_BYTE3;
	AppConfig.MyMACAddr.v[3] = MAC_BYTE4;
	AppConfig.MyMACAddr.v[4] = MAC_BYTE5;
	AppConfig.MyMACAddr.v[5] = MAC_BYTE6;
	AppConfig.MyIPAddr.Val = IP_ADDR_BYTE1 | IP_ADDR_BYTE2<<8ul | IP_ADDR_BYTE3<<16ul | IP_ADDR_BYTE4<<24ul;
	AppConfig.DefaultIPAddr.Val = AppConfig.MyIPAddr.Val;
	AppConfig.MyMask.Val = MASK_BYTE1 | MASK_BYTE2<<8ul | MASK_BYTE3<<16ul | MASK_BYTE4<<24ul;
	AppConfig.DefaultMask.Val = AppConfig.MyMask.Val;
	AppConfig.MyGateway.Val = GATE_BYTE1 | GATE_BYTE2<<8ul | GATE_BYTE3<<16ul | GATE_BYTE4<<24ul;
	AppConfig.PrimaryDNSServer.Val = PRIMARY_DNS_BYTE1 | PRIMARY_DNS_BYTE2<<8ul  | PRIMARY_DNS_BYTE3<<16ul  | PRIMARY_DNS_BYTE4<<24ul;
	AppConfig.SecondaryDNSServer.Val = SECONDARY_DNS_BYTE1 | SECONDARY_DNS_BYTE2<<8ul  | SECONDARY_DNS_BYTE3<<16ul  | SECONDARY_DNS_BYTE4<<24ul;

	// Load the default NetBIOS Host Name
	memcpypgm2ram(AppConfig.NetBIOSName, (const void*)HOSTNAME, 16);
	FormatNetBIOSName(AppConfig.NetBIOSName);
}
