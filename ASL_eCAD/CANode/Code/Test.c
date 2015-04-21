// Standard C libraries
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

// Standard Microchip libraries
#include <xc.h>
#include <pps.h>
#include <uart.h>
#include <adc.h>
#include <dma.h>
#include <ecan.h>

// Declare some data types for use with CAN messages.
typedef struct {
	unsigned ide:1; // 1 => extended data frame message
	unsigned srr:1; // 1 => extended data frame message
	unsigned sid:11;
	unsigned :3;

	unsigned eid17_6:12;
	unsigned :4;

	unsigned dlc:4; // Data length, 0=>0 bytes, 8 => 8 bytes
	unsigned rb0:1; // Must be 0
	unsigned :3;
	unsigned rb1:1; // Must be 0
	unsigned rtr:1; // 1 => remote transmission message
	unsigned eid5_0:6;

	unsigned data0:8;
	unsigned data1:8;
	unsigned data2:8;
	unsigned data3:8;
	unsigned data4:8;
	unsigned data5:8;
	unsigned data6:8;
	unsigned data7:8;
	unsigned :8;
	unsigned filhit:5; // Which filter caught this message
	unsigned :3;
} CanMessage;

// UART1 input reception buffer
uint16_t uart1InBuffer[80];
uint16_t *u1InBufferPtr = uart1InBuffer;

// ADC input struct. Provides enough space for 16 inputs (as req'd by the docs). Really only index
// position 1 (temperature sensor) and 5 (voltage sensor) will be populated.
#ifdef __dsPIC33FJ128MC802__
static volatile uint16_t adcDmaBuffer[16] __attribute__((space(dma)));
#elif __dsPIC33EP256MC502__
static volatile uint16_t adcDmaBuffer[16];
#endif

// Define the maximum value of the ADC input
#define ANmax 4095.0f

/// Declare some function prototypes
void Ecan1Init(void);
void Adc1Init(void);
void Uart1Init(void);

// Declare space for 4 CAN messages in DMA.
#ifdef __dsPIC33FJ128MC802__
static volatile uint16_t ecan1MsgBuf[4][8] __attribute__((space(dma)));
#elif __dsPIC33EP256MC502__
static volatile uint16_t ecan1MsgBuf[4][8];
#endif

// ECAN1 input reception buffer capable of holding 10 CAN messages
uint16_t ecan1InBuffer[10*8];
uint16_t *e1InBufferPtr = ecan1InBuffer;

// Set processor configuration settings
#ifdef __dsPIC33FJ128MC802__
	// Use internal RC to start; we then switch to PLL'd iRC.
	_FOSCSEL(FNOSC_FRC & IESO_OFF);
	// Clock Pragmas
	_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_NONE);
	// Disable watchdog timer
	_FWDT(FWDTEN_OFF);
	// Disable JTAG and specify port 3 for ICD pins.
	_FICD(JTAGEN_OFF & ICS_PGD3);
#elif __dsPIC33EP256MC502__
	// Use internal RC to start; we then switch to PLL'd iRC.
	_FOSCSEL(FNOSC_FRC & IESO_OFF);
	// Clock Pragmas
	_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_NONE);
	// Disable watchdog timer
	_FWDT(FWDTEN_OFF);
	// Disable JTAG and specify port 2 for ICD pins.
	_FICD(JTAGEN_OFF & ICS_PGD2);
#endif

// Keep track of the processor's operating frequency.
#define F_OSC 80000000L

int main()
{
	/// First step is to move over to the FRC w/ PLL clock from the default FRC clock.
	// Set the clock to 79.84MHz.
    PLLFBD = 63;            // M = 65
    CLKDIVbits.PLLPOST = 0; // N1 = 2
    CLKDIVbits.PLLPRE = 1;  // N2 = 3

	// Initiate Clock Switch to FRM oscillator with PLL.
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(OSCCON | 0x01);

	// Wait for Clock switch to occur.
	while (OSCCONbits.COSC != 1);

	// And finally wait for the PLL to lock.
    while (OSCCONbits.LOCK != 1);

	// Initialize ADCs for reading voltage and temperature sensors
	Adc1Init();

    // Initialize ECAN1 for input and output using DMA buffers 0 & 2
    Ecan1Init();

	// Initialize the UART for RX and TX @ 115.2kbaud.
	Uart1Init();
	
	// Open up both the onboard LEDs (A3=red, A4=amber)
	_TRISA3 = 0;
	_LATA3 = 0;
    _TRISA4 = 0;
	_LATA4 = 1;

	// Also open up all shield pins for digital output, used for testing shield connections.
	_TRISA0 = 0;
	_TRISA2 = 0;
	_TRISB0 = 0;
	_TRISB1 = 0;
	_TRISB2 = 0;
	_TRISB5 = 0;
	_TRISB6 = 0;
	_TRISB8 = 0;
	_TRISB9 = 0;
	_TRISB10 = 0;
	_TRISB12 = 0;
	_TRISB14 = 0;
	_TRISB15 = 0;
	
	// And configure the Peripheral Pin Select pins:
	PPSUnLock;

#ifdef __dsPIC33FJ128MC802__
	// To enable ECAN1 pins: TX on 7, RX on 4
	PPSOutput(OUT_FN_PPS_C1TX, OUT_PIN_PPS_RP7);
	PPSInput(IN_FN_PPS_C1RX, IN_PIN_PPS_RP4);

	// To enable UART1 pins: TX on 11, RX on 13
	PPSOutput(OUT_FN_PPS_U1TX, OUT_PIN_PPS_RP11);
	PPSInput(IN_FN_PPS_U1RX, IN_PIN_PPS_RP13);
#elif __dsPIC33EP256MC502__
	// To enable ECAN1 pins: TX on 39, RX on 36
	PPSOutput(OUT_FN_PPS_C1TX, OUT_PIN_PPS_RP39);
	PPSInput(IN_FN_PPS_C1RX, IN_PIN_PPS_RP36);

	// To enable UART1 pins: TX on 43, RX on 45
	PPSOutput(OUT_FN_PPS_U1TX, OUT_PIN_PPS_RP43);
	PPSInput(IN_FN_PPS_U1RX, IN_PIN_PPS_RPI45);
#endif

	PPSLock;

	// Keep a single instance of the CAN message to transmit around.
	CanMessage msg = {
		0,
		0,
		123,
		0,
		8, // 8 bytes of data in this message
		0,
		0,
		0,
		0,
		0,0,0,0,0,0,0,0, // No data yet.
		0
	};

	// Store some variables for lighting up all the LEDs on the shield.
	const int defaultLedValues[] = {0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0};
	int currentLed = 0;

    // We continually loop through processing CAN messages and also HIL messages.
	uint32_t timerCounter = 0;
    while (true) {

		// Echo all input data out.
		if (u1InBufferPtr > uart1InBuffer) {
			*u1InBufferPtr = '\0';
			putsUART1(uart1InBuffer);
			u1InBufferPtr = uart1InBuffer;
		}

		// Process any received CAN messages, echoing them back.
		if (e1InBufferPtr > ecan1InBuffer) {
			uint16_t *i = ecan1InBuffer;
			while (i < e1InBufferPtr) {
				while (C1TR01CONbits.TXREQ0); // Wait to make sure there are no messages pending being sent.
				memcpy((void*)ecan1MsgBuf[0], i, sizeof(CanMessage));
				C1TR01CONbits.TXREQ0 = 1; // And request that the message in buffer 0 be transmit.
				i += (sizeof(CanMessage) / 2);
			}
			e1InBufferPtr = ecan1InBuffer;
		}

		// Do some stuff at ~5Hz.
		if (++timerCounter > 100000) {

			// Blink both status and error LEDs.
			LATAbits.LATA3 ^= 1;
			LATAbits.LATA4 ^= 1;

			// Move the LED currently being blinked.
			// Store a new array of the values to write to the LEDs
			int newLedValues[13];
			{
				int i;
				for (i = 0; i < 13; ++i) {
					if (i == currentLed) {
						newLedValues[i] = defaultLedValues[i] ^ 1;
					} else {
						newLedValues[i] = defaultLedValues[i];
					}
				}
			}
			
			// Write all the new values to the LEDs
			_LATA0 = newLedValues[0];
			_LATA2 = newLedValues[1];
			_LATB0 = newLedValues[2];
			_LATB1 = newLedValues[3];
			_LATB2 = newLedValues[4];
			_LATB5 = newLedValues[5];
			_LATB6 = newLedValues[6];
			_LATB8 = newLedValues[7];
			_LATB9 = newLedValues[8];
			_LATB10 = newLedValues[9];
			_LATB12 = newLedValues[10];
			_LATB14 = newLedValues[11];
			_LATB15 = newLedValues[12];

			// And move on to the next LED
			if (++currentLed > 12) {
				currentLed = 0;
			}

			// Calculate reading of temp value in actual volts
			float tempSensed = 3.3f / ANmax * (float)adcDmaBuffer[1];
			// And convert to sensed temperature (Celsius)
			float tempActual = (tempSensed - 0.5) * 100;

			// Calculate reading of voltage value in actual volts
			float voltageSensed = 3.3 / ANmax * adcDmaBuffer[5];
			// And convert to what the sensed voltage is (Volts)
			float voltageActual = 23000.0 / 2000.0 * voltageSensed;

			// Transmit a CAN message containing the raw voltage of the analog sensors as well as
			// their converted values. Temperature is converted to centi-degrees Celsius and voltage
			// is in centivolts.
			// If there was an error in transmitting the last message, abort it so we can try again.
			// This allows for the CAN bus to be disconnected or causing errors and this test code
			// will still run successfully.
			if (C1TR01CONbits.TXERR0) {
				C1TR01CONbits.TXREQ0 = 0;
			}
			msg.data0 = ((uint16_t)(tempSensed*100)) & 0xFF;
			msg.data1 = (((uint16_t)(tempSensed*100)) >> 8) & 0xFF;
			msg.data2 = ((uint16_t)(tempActual*100)) & 0xFF;
			msg.data3 = (((uint16_t)(tempActual*100)) >> 8) & 0xFF;
			msg.data4 = ((uint16_t)(voltageSensed*100)) & 0xFF;
			msg.data5 = (((uint16_t)(voltageSensed*100)) >> 8) & 0xFF;
			msg.data6 = ((uint16_t)(voltageActual*100)) & 0xFF;
			msg.data7 = (((uint16_t)(voltageActual*100)) >> 8) & 0xFF;
			while (C1TR01CONbits.TXREQ0); // Wait to make sure there are no messages pending being sent.
			memcpy((void*)ecan1MsgBuf[0], &msg, sizeof(CanMessage));
			C1TR01CONbits.TXREQ0 = 1; // And request that the message in buffer 0 be transmit.
			
			// Format our data for UART transmission.
			char outStr[60];
			sprintf(outStr, "temp:%2.2fC (%1.3fV)\nvoltage:%2.2fV (%1.3fV)\n",
				(double)tempActual, (double)tempSensed, (double)voltageActual, (double)voltageSensed);

			// And transmit the same data over UART
			char *i = outStr;
			while (*i) {
				while (U1STAbits.UTXBF); // Wait until there's room in the hardware TX buffer
				U1TXREG = *i;
				++i;
			}

			// And reset the timer counter for the next run.u
			timerCounter = 0;
		}
    }
}

void Adc1Init(void)
{
	/// Initialize ADC for reading temperature and power rail voltage.
	// Use standard V_ref+/V_ref- voltage references.
	SetChanADC1(
		ADC_CH123_NEG_SAMPLEA_VREFN & ADC_CH123_NEG_SAMPLEB_VREFN & ADC_CH123_POS_SAMPLEA_0_1_2 & ADC_CH123_POS_SAMPLEB_0_1_2,
		ADC_CH0_POS_SAMPLEA_AN0 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN0 & ADC_CH0_NEG_SAMPLEB_VREFN
	);
	
	// Open AN1 (temperature) and AN5 (voltage) pins for 12-bit unsigned integer readings. Also note
	// that this will only store one sample per analog input.
#ifdef __dsPIC33FJ128MC802__
	OpenADC1(
		ADC_MODULE_ON & ADC_IDLE_CONTINUE & ADC_ADDMABM_SCATTR & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON & ADC_SIMULTANEOUS & ADC_SAMP_ON,
		ADC_VREF_AVDD_AVSS & ADC_SCAN_ON & ADC_SELECT_CHAN_0 & ADC_DMA_ADD_INC_2 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF,
		ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC & ADC_CONV_CLK_32Tcy,
		ADC_DMA_BUF_LOC_1,
		ENABLE_AN1_ANA & ENABLE_AN5_ANA,
		ENABLE_ALL_DIG_16_31,
		SCAN_NONE_16_31,
		SKIP_SCAN_AN0 & SKIP_SCAN_AN2 & SKIP_SCAN_AN3 & SKIP_SCAN_AN4 & SKIP_SCAN_AN6 & SKIP_SCAN_AN7 &
		SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 &
		SKIP_SCAN_AN14 & SKIP_SCAN_AN15
	);
#elif __dsPIC33EP256MC502__
	// @fixme The fillowing code works properly on the XC16 1.21 compiler, but I've filed a ticket
	// that will allow for the following:
	//  * ADC_SAMPLES_PER_INT_2 should be changed back to ADC_DMA_ADD_INC_2 to better describe what this constant is for.
	//  * ENABLE_AN3_ANA might need to be changed to AN5
	OpenADC1(
		ADC_MODULE_ON & ADC_IDLE_CONTINUE & ADC_ADDMABM_SCATTR & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_SSRC_AUTO & ADC_AUTO_SAMPLING_ON & ADC_SIMULTANEOUS & ADC_SAMP_ON,
		ADC_VREF_AVDD_AVSS & ADC_SCAN_ON & ADC_SELECT_CHAN_0 & ADC_SAMPLES_PER_INT_2 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF,
		ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC & ADC_CONV_CLK_32Tcy,
		ADC_DMA_EN & ADC_DMA_BUF_LOC_1,
		ENABLE_AN1_ANA, // AN1, which is RA1
		ENABLE_AN3_ANA, // AN5, which is RB3
		0, // Don't read any pins in portc
		0, // Don't read any pins in portd
		0, // Don't read any pins in porte
		0, // Don't read any pins in portf
		0, // Don't read any pins in portg
		0, // Don't read any pins in porth
		0, // Don't read any pins in porti
		0, // Don't read any pins in portj
		0, // Don't read any pins in portk
		SCAN_NONE_16_31,
		SKIP_SCAN_AN0 & SKIP_SCAN_AN2 & SKIP_SCAN_AN3 & SKIP_SCAN_AN4 & SKIP_SCAN_AN6 & SKIP_SCAN_AN7 &
		SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 &
		SKIP_SCAN_AN14 & SKIP_SCAN_AN15
	);
#endif

	// Open DMA1 for receiving ADC values
	OpenDMA1(DMA1_MODULE_ON & DMA1_SIZE_WORD & PERIPHERAL_TO_DMA1 & DMA1_INTERRUPT_BLOCK & DMA1_NORMAL & DMA1_PERIPHERAL_INDIRECT & DMA1_CONTINUOUS,
		  DMA1_AUTOMATIC,
#ifdef __dsPIC33FJ128MC802__
		  __builtin_dmaoffset(adcDmaBuffer),
#elif __dsPIC33EP256MC502__
		  (unsigned long int)adcDmaBuffer,
#endif
		  0ul,
		  (uint16_t)&ADC1BUF0,
		  1);
	DMA1REQbits.IRQSEL = 0x0D; // Attach this DMA to the ADC1 conversion done event
}

/**
 * Initialize ECAN1 with no filters at 250kbaud.
 */
void Ecan1Init(void)
{
    // Set ECAN1 into configuration mode
	C1CTRL1bits.REQOP = 4;
    while (C1CTRL1bits.OPMODE != 4);

    // Initialize the CAN node to 250kbaud, assuming a 80MHz clock.
	const uint16_t propagationSegmentLength = 5;
	const uint16_t phaseSegment1Length = 8;
	const uint16_t phaseSegment2Length = 6;
    const uint64_t f_baud = 250000L;
    const uint64_t f_tq = (propagationSegmentLength + phaseSegment1Length + phaseSegment2Length + 1) * f_baud;
	const uint64_t f_cy = F_OSC / 2;
    const uint8_t BRP = (uint8_t)(f_cy / (2L * f_tq));

    CAN1Initialize(CAN_SYNC_JUMP_WIDTH4 & CAN_BAUD_PRE_SCALE(BRP),
                   CAN_WAKEUP_BY_FILTER_DIS & CAN_PROPAGATIONTIME_SEG_TQ(propagationSegmentLength) & CAN_PHASE_SEG1_TQ(phaseSegment1Length) & CAN_PHASE_SEG2_TQ(phaseSegment2Length) & CAN_SEG2_FREE_PROG & CAN_SAMPLE3TIMES);

    // Use 4 buffers in DMA RAM (smallest value we can choose), other option is irrelevant.
    CAN1FIFOCon(CAN_DMA_BUF_SIZE_4 & CAN_FIFO_AREA_TRB0);

    // Setup message filters and masks.
    C1CTRL1bits.WIN = 1; // Allow configuration of masks and filters

	// Set Mask 0 to allow everything.
	CAN1SetMask(0, CAN_MASK_SID(0) & CAN_IGNORE_FILTER_TYPE, CAN_MASK_EID(0));

	// Set Filter 0 to use Mask 0.
	CAN1SetMaskSource(CAN_MASK_FILTER0_MASK0, CAN_MASK_FILTER8_NO_MASK);

	// Set Filter 0 to allow everything.
	CAN1SetFilter(0, CAN_FILTER_SID(0) & CAN_RX_EID_DIS, CAN_FILTER_EID(0));

	// Point filter 0 to our reception buffer (Buffer 1).
	CAN1SetBUFPNT1(CAN_FILTER0_RX_BUFFER1);

	// Enable Filter 0.
    CAN1EnableFilter(0);

    C1CTRL1bits.WIN = 0;

    // Return the modules to specified operating mode.
	CAN1SetOperationMode(CAN_IDLE_CON & CAN_MASTERCLK_FOSC & CAN_CAPTURE_DISABLE & CAN_REQ_OPERMODE_NOR & CAN_SFR_BUFFER_WIN,
			             CAN_DO_NOT_CMP_DATABYTES);

    // Enable interrupts for ECAN1
    ConfigIntCAN1(CAN_INVALID_MESSAGE_INT_DIS & CAN_WAKEUP_INT_DIS & CAN_ERR_INT_DIS & CAN_FIFO_INT_DIS & CAN_RXBUF_OVERFLOW_INT_DIS & CAN_RXBUF_INT_EN & CAN_TXBUF_INT_DIS,
                  CAN_INT_ENABLE & CAN_INT_PRI_7);

    // Configure buffer settings.
    // Specify details on the reception buffer (1) and the transmission buffer (0)
    CAN1SetTXRXMode(0, CAN_BUFFER0_IS_TX & CAN_ABORT_REQUEST_BUFFER0 & CAN_AUTOREMOTE_DISABLE_BUFFER0 & CAN_TX_HIGH_PRI_BUFFER0 &
			           CAN_BUFFER1_IS_RX & CAN_ABORT_REQUEST_BUFFER1 & CAN_AUTOREMOTE_DISABLE_BUFFER1 & CAN_TX_HIGH_PRI_BUFFER1);

	/// Set up necessary DMA channels for transmission and reception
	// ECAN1 transmission over DMA2
	OpenDMA2(DMA2_MODULE_ON & DMA2_SIZE_WORD & DMA2_TO_PERIPHERAL & DMA2_INTERRUPT_BLOCK & DMA2_NORMAL & DMA2_PERIPHERAL_INDIRECT & DMA2_CONTINUOUS,
		 DMA2_AUTOMATIC,
#ifdef __dsPIC33FJ128MC802__
		  __builtin_dmaoffset(ecan1MsgBuf),
#elif __dsPIC33EP256MC502__
		  (unsigned long int)ecan1MsgBuf,
#endif
		 0ul,
		 (uint16_t)&C1TXD,
		 7);
	DMA2REQbits.IRQSEL = 0x46; // Attach this DMA to the ECAN1 TX data sent event

	// ECAN1 reception over DMA0
	OpenDMA0(DMA0_MODULE_ON & DMA0_SIZE_WORD & PERIPHERAL_TO_DMA0 & DMA0_INTERRUPT_BLOCK & DMA0_NORMAL & DMA0_PERIPHERAL_INDIRECT & DMA0_CONTINUOUS,
		  DMA0_AUTOMATIC,
#ifdef __dsPIC33FJ128MC802__
		  __builtin_dmaoffset(ecan1MsgBuf),
#elif __dsPIC33EP256MC502__
		  (unsigned long int)ecan1MsgBuf,
#endif
		  0ul,
		  (uint16_t)&C1RXD,
		  7);
	DMA0REQbits.IRQSEL = 0x22; // Attach this DMA to the ECAN1 RX data ready event
}

void Uart1Init(void)
{
	OpenUART1(
		UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE & UART_MODE_FLOW & UART_UEN_00 & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_NO_PAR_8BIT & UART_UXRX_IDLE_ONE & UART_BRGH_SIXTEEN & UART_1STOPBIT,
		UART_INT_TX & UART_IrDA_POL_INV_ZERO & UART_SYNC_BREAK_DISABLED & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR,
		F_OSC/2/(16*115200) - 1
	);
    ConfigIntUART1(UART_RX_INT_EN & UART_RX_INT_PR2 &
                   UART_TX_INT_DIS & UART_TX_INT_PR6);
}

/**
 * UART1 reception interrupt. Just save all input characters to a buffer.
 */
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    IFS0bits.U1RXIF = 0;
	while (DataRdyUART1()) {
		*u1InBufferPtr++ = U1RXREG;
    }
}

/**
 * This is an interrupt handler for the ECAN1 peripheral.
 * It clears interrupt bits and pushes received message into
 * the circular buffer.
 */
void __attribute__((interrupt, no_auto_psv))_C1Interrupt(void)
{
    // Store any received CAN messages in a queue.
    if (C1INTFbits.RBIF) {
        // Obtain the buffer the message was stored into, checking that the value is valid to refer to a buffer
		int buffer;
        if (C1VECbits.ICODE < 32) {
            buffer = C1VECbits.ICODE;
        }

		// Copy the received message into our queue.
		memcpy(e1InBufferPtr++, (const void *)ecan1MsgBuf[buffer], sizeof(CanMessage));

        // Clear the buffer full status bit so more messages can be received.
        if (C1RXFUL1 & (1 << buffer)) {
            C1RXFUL1 &= ~(1 << buffer);
        }

        // Be sure to clear the interrupt flag.
        C1INTFbits.RBIF = 0;
    }

    // Clear the general ECAN1 interrupt flag.
    IFS2bits.C1IF = 0;

}