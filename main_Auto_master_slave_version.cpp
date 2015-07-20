#include "mbed.h"
#include "main.h"
#include "sx1276-inAir.h"
#include "Adafruit_SSD1306.h"
#include "myDebug.h"


//----- SHD3I Board -----
//SCLK=D13, MISO=D12, MOSI=D11
//DIO0=D2, DIO1=D8, DIO2=D4, DIO3=A4, DIO5=D3  DIO4=Not Used
//NSS(CS)=D7, Reset=A5
//----- NZ32-ST1L Board -----
//SCLK=PB3, MISO=PB4, MOSI=PB5
//DIO0=PB0, DIO1=PB1, DIO2=PC6, DIO3=PA10, DIO5=PC13  DIO4=Not Used
//CS=PC8, Reset=PA9
//----- Future modules using only DIO0 to DIO2
//To use with module that only use DIO0, 1 and 2, we have to change SX1276::StartCad()
//function to use DIO0 for CAD.


// Defines ////////////////////////////////////////////////////////////////////

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE   1

//OLED does not work on FRDM-KL25Z. It writes the initial message, but not messages after that. Seems like there is memory corruption
//somewhere. Maybe the 1024bytes defined by OLED library is too much??
//#define DISABLE_OLED

#ifndef __PACKED__
#define __PACKED__ __attribute__((packed))
#endif

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

#define RF_FREQUENCY								921000000	// Hz
#define TX_OUTPUT_POWER                             14        	// 14 dBm
#if USE_MODEM_LORA == 1

#define LORA_BANDWIDTH                              6       // 0: 7.8 kHz, 1: 10.4 kHz  2: 15.6kHz
															// 3: 20.8kHz  4: 31.25kHz  5: 41.7 kHz
															// 6: 62.5 kHz 7: 125 kHz   8: 250 kHz
															// 9: 500 kHz
#define LORA_SPREADING_FACTOR                       12     	// [SF7..SF12]
#define LORA_CODINGRATE                             1       // [1: 4/5,
															//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8        // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5        // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false	 // false = explicit mode, true = implicit mode(no header)
#define LORA_FHSS_ENABLED                           false
#define LORA_NB_SYMB_HOP                            4
#define LORA_IQ_INVERSION_ON                        false
#define LORA_CRC_ENABLED                            false

#elif USE_MODEM_FSK == 1

#define FSK_FDEV                                    25000     // Hz
#define FSK_DATARATE                                19200     // bps
#define FSK_BANDWIDTH                               50000     // Hz
#define FSK_AFC_BANDWIDTH                           83333     // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false
#define FSK_CRC_ENABLED                             true

#else
#error "Please define a modem in the compiler options."
#endif

#define RX_TIMEOUT_VALUE                                4500000 	//Timeout in us (0 = no timeout)
#define BUFFER_SIZE                                     32        	// Define the payload size here
#define NZ32_ST1L_REV1

// Calculate the corresponding acquisition measure for a given value in mV
#define MV(x) ((0xFFF*x)/3300)

// Local variables declarations ///////////////////////////////////////////////
#if( defined ( TARGET_KL25Z ) || defined ( TARGET_LPC11U6X ) )
DigitalOut led(LED2);
#else
DigitalOut led(LED1);
#endif

bool cadDoneTriggered = false;

//NZ32-ST1L control and monitoring inputs/outputs
#ifdef NZ32_ST1L_REV1
AnalogIn ainVSense(PB_12);
#else
AnalogIn ainVSense(PC_4);
#endif
AnalogIn ainVBatt(PC_5);

Serial pc(USBTX, USBRX);

typedef RadioState States_t;
volatile States_t State = LOWPOWER;

SX1276inAir9 Radio(OnTxDone, OnTxTimeout, OnRxDone, OnRxTimeout, OnRxError, NULL/*fhssChangeChannel*/, OnCadDone/*cadDone*/);

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

int16_t RssiValue = 0;
int8_t SnrValue = 0;


//Required if we enable "#define USE_FULL_ASSERT 1" line in "stm32l1xx_hal_conf.h" file. But, seems to not be working
void assert_failed(uint8_t* file, uint32_t line) {
	//mbed_assert_internal("myAssert", (const char *)file, line);
	//debug("\r\nASSRT_FAILED!!!!!");
	fprintf(stderr, "ASSRT_FAILED: file: %s, line %d \n", file, (int) line);
}

#if !defined(DISABLE_OLED)
// an I2C sub-class that provides a constructed default, for OLED
class I2CPreInit : public I2C
{
public:
    I2CPreInit(PinName sda, PinName scl) : I2C(sda, scl)
    {
        frequency(400000);
        //start();  //Does NOT work when this is defined!
    };
};
I2CPreInit gI2C(PB_9, PB_8);
//I2CPreInit gI2C(I2C_SDA, I2C_SCL);
// TEST TEST - Remove PA_8 later, just for testing!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
Adafruit_SSD1306_I2c gOled(gI2C, PA_8, 0x78, 64);
#endif

int main() {
	uint8_t i;
	int16_t waitTillPing = -1;
    bool isMaster = true;

    debug("\n\n\rSX1276 Ping Pong Demo Application\n\n\r");

    //gOled.setRotation(2);
    gOled.clearDisplay();
    gOled.setTextSize(2);
    gOled.setTextCursor(0,0);
    gOled.printf("Master!");
    gOled.display();


//    // TEST TEST
//    while(1) {
//    	led = !led;
//    	wait_ms(250);
//    }

	// verify the connection with the board
	while (Radio.Read(REG_VERSION) == 0x00) {
		debug("Radio could not be detected!\n\r", NULL);
		wait(1);
	}

	Radio.SetChannel(RF_FREQUENCY);

#if USE_MODEM_LORA == 1

	debug_if(LORA_FHSS_ENABLED, "\n\n\r             > LORA FHSS Mode < \n\n\r");
	debug_if(!LORA_FHSS_ENABLED, "\n\n\r             > LORA Mode < \n\n\r");

	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
			LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
			LORA_FIX_LENGTH_PAYLOAD_ON, LORA_CRC_ENABLED, LORA_FHSS_ENABLED,
			LORA_NB_SYMB_HOP, LORA_IQ_INVERSION_ON, 4000000);

	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
			LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,
			LORA_FIX_LENGTH_PAYLOAD_ON, 0, LORA_CRC_ENABLED, LORA_FHSS_ENABLED,
			LORA_NB_SYMB_HOP, LORA_IQ_INVERSION_ON, true);

#elif USE_MODEM_FSK == 1

	debug("\n\n\r              > FSK Mode < \n\n\r");
	Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
			FSK_DATARATE, 0,
			FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
			FSK_CRC_ENABLED, 0, 0, 0, 2000000 );

	Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
			0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
			0, FSK_FIX_LENGTH_PAYLOAD_ON, FSK_CRC_ENABLED,
			0, 0, false, true );

#else

#error "Please define a modem in the compiler options."

#endif

    debug_if (DEBUG_MESSAGE, "Starting Ping-Pong loop\r\n");

    led = 0;

    Radio.Rx( RX_TIMEOUT_VALUE );

    while(1)
    {
        switch( State ) {
        case RX:
            //debug_if (DEBUG_MESSAGE, "> RX RSSI = %d\n\r", RssiValue);
            waitTillPing = -1;  //Disable

            //We are master - Send "PING" and Receive "PONG"
            if (isMaster == true ) {
                if (BufferSize > 0) {
                    // RXed "PONG" - we(master) reply with "PING"
                    if (strncmp( (const char*)Buffer, (const char*)PongMsg, 4 ) == 0) {
                        led = !led;
                        debug("...Pong (RSSI=%d)\n\r", RssiValue);
                        // Send the next PING frame
                        strcpy( (char*)Buffer, (char*)PingMsg);
                        wait_ms( 10);
                        Radio.Send(Buffer, sizeof(PingMsg));    //Send PING message
                    }
                    // RXed "PING" - A master already exists then become a slave - reply with "PONG"
                    else if (strncmp( (const char*)Buffer, (const char*)PingMsg, 4 ) == 0) {
                        debug("...Ping\r\n");
                        led = !led;
                        isMaster = false;
                        // Send the next PONG frame
                        strcpy( (char*)Buffer, (char*)PongMsg);
                        wait_ms( 10);
                        Radio.Send(Buffer, sizeof(PongMsg));
                    }
                    // valid reception but neither a PING or a PONG message
                    else {
                        // Set device as master and start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            //We are slave - Send "PONG" and Receive "PING"
            else {
                if (BufferSize > 0)  {
                    //RXed "PING" - we(slave) reply with "PONG"
                    if (strncmp( (const char*)Buffer, (const char*)PingMsg, 4 ) == 0) {
                        led = !led;
                        debug("...Ping (RSSI=%d)\n\r", RssiValue);
                        strcpy( (char*)Buffer, (char*)PongMsg);
                        wait_ms( 10);
                        Radio.Send(Buffer, sizeof(PongMsg)); //Send PONG message
                    }
                    // valid reception but not a PING as expected
                    else {
                        // Set device as master and start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            State = LOWPOWER;
            break;
        case TX:
            //debug_if (DEBUG_MESSAGE, "> Tx Done\n\r");
            waitTillPing = -1;  //Disable

            if ( isMaster == true ) {
                debug("Ping...\r\n");
            }
            else {
                debug("Pong...\r\n");
            }
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
            if (waitTillPing == -1) {
                debug_if (DEBUG_MESSAGE, "> Rx Timeout\n\r");

				#if !defined(DISABLE_OLED)
				gOled.setTextSize(1);
				gOled.setTextCursor(0,20);  //First line was x2 text size = 16 pixels height!
				gOled.printf("ERR: RX Timeout!");
				gOled.display();
				#endif

                if (isMaster == true) {
                    // Send the next PING frame. Flash LED for 100ms.
                    led = !led;
                    wait_ms(100);
                    led = !led;

                    //Set random time from 10 to 3000ms to wait before sending "PING" message again. Random is required to
                    //prevent multiple nodes from sending same time, and not synching
                    //waitTillPing = (rand()%300);    //Each waitTillPing = 10ms, so 300 = 3000ms - !!!!!! rand() does NOT work !!!!
                    waitTillPing = (us_ticker_read()%300);    //Each waitTillPing = 10ms, so 300 = 3000ms
                    //debug_if (DEBUG_MESSAGE, "waitTillPing = %d\n\r", waitTillPing);
                }
                else {
                    Radio.Rx(RX_TIMEOUT_VALUE);
                    State = LOWPOWER;
                }
            }
            else {
                if (waitTillPing == 0) {
                    waitTillPing = -1;  //Disable
                    //debug_if (DEBUG_MESSAGE, "x\n\r");
                    //Send PING message
                    strcpy( (char*)Buffer, (char*)PingMsg);
                    Radio.Send(Buffer, sizeof(PingMsg));    //Send PING message
                    State = LOWPOWER;
                }
                else {
                    waitTillPing--;
                    wait_ms(10);
                }
            }
            break;
        case RX_ERROR:
            debug_if (DEBUG_MESSAGE, "> Rx Error\n\r");
            // We have received a Packet with a CRC error, send reply as if packet was correct
            if (isMaster == true) {
                // Send the next PING frame
                strcpy( (char*)Buffer, (char*)PingMsg);
                wait_ms( 10);
                Radio.Send(Buffer, sizeof(PingMsg));    //Send PING message
            }
            else {
                // Send the next PONG frame
                strcpy((char*)Buffer, (char*)PongMsg);
                wait_ms( 10);
                Radio.Send(Buffer, sizeof(PongMsg));    //Send PONG message
            }
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            debug_if (DEBUG_MESSAGE, "> Tx Timeout\n\r");
            waitTillPing = -1;  //Disable
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case LOWPOWER:
            break;
        default:
            State = LOWPOWER;
            break;
        }
    }
}

void OnTxDone(void) {
	Radio.Sleep();
	State = TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
	Radio.Sleep();
	BufferSize = size;
	memcpy(Buffer, payload, BufferSize);
	RssiValue = rssi;
	SnrValue = snr;
	State = RX;
}

void OnTxTimeout(void) {
	Radio.Sleep();
	State = TX_TIMEOUT;
}

void OnRxTimeout(void) {
	Radio.Sleep();
	State = RX_TIMEOUT;
}

void OnRxError(void) {
	Radio.Sleep();
	State = RX_ERROR;
}

/*
 * CAD Done callback prototype.
 * @param [IN] ChannelDetected    Channel Activity detected during the CAD
 */
void OnCadDone (bool channelActivityDetected) {
	cadDoneTriggered = true;
}


uint16_t getBattMV(void) {
	float fval;
	uint16_t meas;

//	//Measure Vbatt. It doesn't seem to make lots of a difference if we disable the DC/DC converter!
//	ctrVBatt = 0;
//	ctrVBatt.output();
//	wait_ms(150);
//	meas = ainVBatt.read_u16(); // Converts and read the analog input value
//	ctrVBatt.input();
//	//fval = ((float)meas / (float)0xffff) * 6600;	//6600 = 3300*2, because resistor divider = 2
//	fval = meas * ((float)6600.0 / (float)65535.0);

	return (uint16_t) fval;
}

uint16_t getSupplyMV(void) {
	float fval;
	uint16_t meas;

	//Following voltages were measured at input of ADC:
	//
	//----- Vusb=0V  &  5V supply=0V -----
	//Value is typically 4mV
	//
	//----- Vusb=5V  &  5V supply=0V -----
	//When only VUSB is supplied, the value is typically 1.95V
	//
	//----- Vusb=0V  &  5V supply=5V -----
	//When only VUSB is supplied, the value is typically 1.95V
	//
	//The reason this circuit does not work, is because of the reverse voltage of the diodes. It is given
	//as about 40 to 80uA. With a resistor value of 470K, this will put 5V on other side of diode.
	//Thus, no matter if 5V is at Vusb, Vsupply, or both, the read value will always be 1.95V.
	//To solve problem, resistors must be lowered to value where 80uA will no longer give more than 5V
	//voltage drop = 62k. Using two 47k resistors should work.

	//Measure Vusb/5V sense input
	meas = ainVSense.read_u16(); // Converts and read the analog input value
	fval = ((float) meas / (float) 0xffff) * 3300;

	return (uint16_t) fval;
}

