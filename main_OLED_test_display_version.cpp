#include "mbed.h"
#include "main.h"
#include "sx1276-inAir.h"
#include "myDebug.h"
#include "Adafruit_SSD1306.h"

//Configure to be Master or Slave
bool isMaster = 1;

#define NZ32_ST1L_REV1

//MODTRONIX
//This program is used to test the inAir modules. It requires two modules, one running
//as master, and the other as slave.
//Uses BW=500khz and SF=8 (12,500 bps), with short 500ms timeout.
//
//It uses a Modtronix NZ32-ST1L Board, with a inAir mounted in iMod port 2.
//
//=== Slave ===
//Compile program with "isMaster = false"
//
//=== Master ===
//Compile program with "isMaster = true"
//

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE   1

// DEFINES ////////////////////////////////////////////////////////////////////
/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

#define RF_FREQUENCY                                921300000 // 921.3 kHz
//#define RF_FREQUENCY                              433000000 // 433.3 kHz
#define TX_OUTPUT_POWER                             17        // 14 dBm
#if USE_MODEM_LORA == 1
#define LORA_BANDWIDTH                              LORA_BW_500000
#define LORA_SPREADING_FACTOR                       8		// SF7..SF12
#define LORA_CODINGRATE                             1       // 1=4/5, 2=4/6, 3=4/7, 4=4/8
#define LORA_PREAMBLE_LENGTH                        8       // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5       // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
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

#define RX_TIMEOUT_VALUE                                1000000   // in us
#define BUFFER_SIZE                                     32        // Define the payload size here
// Calculate the corresponding acquisition measure for a given value in mV
#define MV(x) ((0xFFF*x)/3300)

// GLOBAL VARIABLES ///////////////////////////////////////////////////////////
uint8_t     boardType;                  //The board type, a BOARD_XXX define
DigitalOut  led(LED1);
DigitalOut  enableFastCharge(PA_14);
DigitalIn   btnOK(PC_12, PullUp);
DigitalIn   btnStar(PC_1, PullUp);
DigitalIn   btnUp(PC_2, PullUp);
DigitalIn   btnDown(PC_10, PullUp);
typedef     RadioState States_t;
volatile    States_t State = LOWPOWER;

#define     SIZE_ARR_BATT_ADC	32	    //Must be factor of 2 value (4,8,16,32...)
uint8_t     arrBattAdcInit = 0;
uint8_t     putArrBattAdc = 0;
uint16_t    arrBattAdc[SIZE_ARR_BATT_ADC];

SX1276inAir* pRadio = NULL;

const uint8_t PingMsg[] = "pInG";
const uint8_t PongMsg[] = "pOnG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

int16_t RssiValue = 0.0;
int16_t RssiValueSlave = 0.0;
int8_t SnrValue = 0.0;
AppConfig appConfig;

//NZ32-ST1L control and monitoring inputs/outputs
#ifdef NZ32_ST1L_REV1
AnalogIn        ainVSense(PB_12);
#else
AnalogIn        ainVSense(PC_4);
#endif
AnalogIn        ainVBatt(PC_5);
DigitalInOut    ctrVBatt(PA_13);

// Class and Variables for OLED ///////////////////////////////////////////////
#if !defined(DISABLE_OLED)
// an I2C sub-class that provides a constructed default, for OLED
class I2CPreInit: public I2C {
public:
    I2CPreInit(PinName sda, PinName scl) :
            I2C(sda, scl) {
        frequency(400000);
        //start();  //Does NOT work when this is defined!
    }
    ;
};
I2CPreInit gI2C(PB_9, PB_8);
Adafruit_SSD1306_I2c gOled(gI2C, PA_8, 0x78, 64);
#endif

/** Main function */
int main() {
    uint16_t i;
    uint16_t delayedSendPing;
    uint16_t pongRxCnt = 0;
    bool radioInitilized = false;
    led = 0;
    delayedSendPing = 0;
    memset(arrBattAdc, 0, sizeof(arrBattAdc));

    //Configure default values
    memset(&appConfig, 0, sizeof(appConfig));
    appConfig.frequency = RF_FREQUENCY;
    appConfig.bw = LORA_BANDWIDTH;
    appConfig.sf = LORA_SPREADING_FACTOR;
    appConfig.power = TX_OUTPUT_POWER;
    appConfig.preambleLength = LORA_PREAMBLE_LENGTH;
    appConfig.symbolTimeout = LORA_SYMBOL_TIMEOUT;
    appConfig.numberSymHop = LORA_NB_SYMB_HOP;
    appConfig.conf.lora.codingRate = LORA_CODINGRATE;
    appConfig.conf.lora.fixLength = LORA_FIX_LENGTH_PAYLOAD_ON;
    appConfig.conf.lora.fshhEnable = LORA_FHSS_ENABLED;
    appConfig.conf.lora.iqInversionEnable = LORA_IQ_INVERSION_ON;
    appConfig.conf.lora.crcEnable = LORA_CRC_ENABLED;

    //Try and determine the board type by the defined channel and power
#if (RF_FREQUENCY < RF_MID_BAND_THRESH)
    boardType = BOARD_INAIR4; //Use BOARD_INAIR4, it is a 14dBm output board
#elif (TX_OUTPUT_POWER > 14)
    boardType = BOARD_INAIR9B; //Use BOARD_INAIR9B, it has TX connected to PA Boost
#else
            boardType = BOARD_INAIR9; //Use BOARD_INAIR9, it is a 14dBm output
#endif

    //Write Debug and OLED message
    if (isMaster == true) {
#if !defined(DISABLE_OLED)
        //gOled.setRotation(2);
        gOled.clearDisplay();
        gOled.setTextSize(2);
        gOled.setTextCursor(0, 0);
        if (boardType == BOARD_INAIR4)
            gOled.printf("inAir4");
        else if (boardType == BOARD_INAIR9)
            gOled.printf("inAir9");
        else
            gOled.printf("inAir9B");
        gOled.display();
        wait_ms(1000);
#endif
        debug_if(DEBUG_MESSAGE, "\n\n\rStarted test_inAir as Master\n\n\r");
    } else {
        debug_if(DEBUG_MESSAGE, "\n\n\rStarted test_inAir as Slave\n\n\r");
    }

    //Enable fast charging
    enableFastCharge = 0;   //Enable fast charging

    // TEST TEST For product photo!!!!!
#if(1)
    wait_ms(100);
    gOled.clearDisplay();
    gOled.setTextSize(2);

    //Line 1
    gOled.setTextCursor(0, 0);
    gOled.printf("inAir9 ");

    //Line 2
    gOled.setTextCursor(0, 16); //Second line(16), first column. First line is 0-13 (14x10 chars)
    gOled.printf("L=051");
    gOled.setTextCursor(68, 16); //Second line(16), column=68pixels. First line is 0-13 (14x10 chars)
    gOled.printf("R=053");

    //For 4 lines, use 33,41,49 and 57
    //For 3 lines, use 36,46,56
    //Line 3
    gOled.setTextSize(1);
    gOled.setTextCursor(0, 36);
    gOled.printf("F=915.6 BW=250 SF=12");

    gOled.setTextCursor(0, 46);
    gOled.printf("RXed OK=1020 Err=2200");

    gOled.setTextCursor(0, 56);
    gOled.printf("Mode: Running");

    gOled.fillRect(94, 1, 4, 2, 1); //x, y, w, h
    gOled.fillRect(92, 4, 8, 3, 1);
    gOled.fillRect(92, 8, 8, 3, 1);
    gOled.fillRect(92, 12, 8, 3, 1);
    gOled.setTextCursor(104, 5);
    gOled.printf("100%%");

    gOled.display();

    /*
    while (1) {
        led = !led;
        wait_ms(200);
    }
    */

     while(1){
     if (btnOK.read()==0) {
     led = 1;
     }
     else {
     led = 0;
     }

     {
     float fval = 0;
     uint32_t averBattAdc;
     uint16_t mvBatt;		//Battery mv value
     uint16_t percentBatt;	//Battery percentage

     //Assume voltage decreases linearly from 4.2 to 3.2V
     //Measure Vbatt. It doesn't seem to make lots of a difference if we disable the DC/DC converter!
     ctrVBatt = 0;
     ctrVBatt.output();
     wait_ms(150);
     arrBattAdc[(putArrBattAdc++) & (SIZE_ARR_BATT_ADC-1)] = ainVBatt.read_u16(); // Converts and read the analog input value
     //First time, fill whole array with read value (first element just got current value in code above)
     if (arrBattAdcInit == 0) {
     arrBattAdcInit = 1;
     for (i=1; i<SIZE_ARR_BATT_ADC; i++) {
     arrBattAdc[i] = arrBattAdc[0];
     }
     }
     averBattAdc = 0;
     for (i=0; i<SIZE_ARR_BATT_ADC; i++) {
     //averBattAdc += arrBattAdc[i];
     averBattAdc = averBattAdc + arrBattAdc[i];
     }
     averBattAdc = averBattAdc / SIZE_ARR_BATT_ADC;
     ctrVBatt.input();
     //fval = ((float)meas / (float)0xffff) * 6600;	//6600 = 3300*2, because resistor divider = 2
     fval = averBattAdc * ((float)6600.0 / (float)65535.0);
     mvBatt = (uint16_t)fval;
     if (mvBatt < 3200)
     percentBatt = 0;
     else {
     percentBatt = (mvBatt - 3200)/10;	//Convert to value from 0 to 1000, then divide by 10 to get 0-100 percentage
     }
     if (percentBatt > 100)
     percentBatt = 100;	//Not more than 100%

     gOled.setTextCursor(104, 5);
     gOled.printf("%d%%", percentBatt);
     if (percentBatt<100) gOled.putc(' ');	//Add 1 space after '%' if percentage is only 2 digits long
     if (percentBatt<10)  gOled.putc(' ');	//Add 2 spaces after '%' if percentage is only 1 digits long
     //gOled.printf("%d", mvBatt);
     //gOled.printf("%d", averBattAdc);
     gOled.display();
     wait_ms(500);
     }

     };

#endif

    while (1) {
        wait_ms(10);

        //Create and Initialise instance of SX1276inAir
        if (pRadio == NULL) {
            pRadio = new SX1276inAir(OnTxDone, OnTxTimeout, OnRxDone,
                    OnRxTimeout, OnRxError, NULL, NULL);
            pRadio->SetBoardType(boardType);
            radioInitilized = false;
            wait_ms(10);
        }

        //Wait for radio to be detected
        if (pRadio->Read(REG_VERSION) == 0x00) {
            debug_if(DEBUG_MESSAGE, "Radio could not be detected!\n\r", NULL);
#if !defined(DISABLE_OLED)
            gOled.setTextSize(2);
            gOled.setTextCursor(0, 48); //Last line(48), first column
            gOled.printf("No Radio! ");
            gOled.display();
            wait_ms(100);
#endif

            // Wait for radio to be detected
            while (pRadio->Read(REG_VERSION) == 0x00) {
                wait_ms(200);
            }

            //If radio was already initialised, delete it.
            if (radioInitilized == true) {
                delete pRadio;
                pRadio = NULL;
                wait_ms(100);
                continue;
            }
        }

        //Initialise radio
        if (radioInitilized == false) {
            radioInitilized = true;
            pRadio->SetChannel(RF_FREQUENCY);

#if USE_MODEM_LORA == 1
#if(DEBUG_MESSAGE == 1)
            debug_if(LORA_FHSS_ENABLED, "\r\n> LORA FHSS Mode\r\n");
            debug_if(!LORA_FHSS_ENABLED, "\r\n> LORA Mode\r\n");
#endif

            pRadio->SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                    LORA_IQ_INVERSION_ON, 2000000);

            pRadio->SetRxConfig(MODEM_LORA, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE, 0,
                    LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,
                    LORA_FIX_LENGTH_PAYLOAD_ON, 0, LORA_CRC_ENABLED,
                    LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP, LORA_IQ_INVERSION_ON,
                    true);
#elif USE_MODEM_FSK == 1
            debug_if (DEBUG_MESSAGE,"\r\n> FSK Mode\r\n");
            pRadio->SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                    FSK_DATARATE, 0,
                    FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                    FSK_CRC_ENABLED, 0, 0, 0, 2000000);

            pRadio->SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                    0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                    0, FSK_FIX_LENGTH_PAYLOAD_ON, FSK_CRC_ENABLED,
                    0, 0, false, true );

#else
#error "Please define a modem in the compiler options."
#endif

            //We are Master - Send "PING" and Receive "PONG"
            if (isMaster == true) {
                // Send the next PING frame
                strcpy((char*) Buffer, (char*) PingMsg);
                pRadio->Send(Buffer, sizeof(PingMsg)); //Send PING message
            }
            //We are Slave - Receive "PONG"
            else {
                pRadio->Rx(RX_TIMEOUT_VALUE);
            }
        }

        //Send delayed PING message
        if (delayedSendPing != 0) {
            delayedSendPing--;
            if (delayedSendPing == 0) {
                // Send the next PING frame
                strcpy((char*) Buffer, (char*) PingMsg);
                pRadio->Send(Buffer, sizeof(PingMsg)); //Send PING message
            }
        }

        switch (State) {
        case RX:
            //debug_if (DEBUG_MESSAGE, "> RX RSSI = %d\n\r", RssiValue);

            //We are Master - Send "PING" and Receive "PONG"
            if (isMaster == true) {
                if (BufferSize > 0) {
                    // RXed "PONGnn" - we(master) reply with "PING"
                    // The 2 bytes after "PONG" is the RSSI that that the slave received our PING at
                    if (strncmp((const char*) Buffer, (const char*) PongMsg, 4)
                            == 0) {
                        char c;
                        led = !led;
                        RssiValueSlave =
                                ((uint16_t) Buffer[sizeof(PongMsg)])
                                        + (((uint16_t) Buffer[sizeof(PongMsg)
                                                + 1]) << 8);
                        debug_if(DEBUG_MESSAGE, "...Pong - RSSI=%d, %d\n\r",
                                RssiValue, RssiValueSlave);

#if !defined(DISABLE_OLED)
                        //gOled.clearDisplay();
                        gOled.setTextSize(2);
                        gOled.setTextCursor(0, 16); //Second line(16), first column. First line is 0-13 (14x10 chars)
//                        switch(++pongRxCnt&0x03) {
//                            case 0: c='|'; break;
//                            case 1: c='/'; break;
//                            case 2: c='-'; break;
//                            case 3: c='\\'; break;
//                        }
//                        line[0]=c;
//                        line[1]=' ';
//                        gOled.printf("%c %03d, %03d\r\n", c, abs(RssiValue), abs(RssiValueSlave));
//                        gOled.setTextCursor(0,48);  //Last line(48), first column
//                        gOled.printf("          ");

                        if ((++pongRxCnt & 0x01) == 0) {
                            gOled.printf("L=%03d", abs(RssiValue));
                        } else {
                            gOled.printf("L-%03d", abs(RssiValue));
                        }
                        gOled.setTextCursor(68, 16); //Second line(16), column=68pixels. First line is 0-13 (14x10 chars)
                        if ((pongRxCnt & 0x01) == 0) {
                            gOled.printf("R=%03d", abs(RssiValueSlave));
                        } else {
                            gOled.printf("R-%03d", abs(RssiValueSlave));
                        }
                        gOled.display();
#endif

                        //Clear "RX Timeout" line
                        gOled.setTextCursor(0, 48); //Last line(48), first column
                        gOled.printf("          ");

//						gOled.setTextCursor(0,32);  //Third line(32), first column
//						gOled.setTextSize(1);
//						switch(++pongRxCnt&0x03) {
//							case 0: c='|'; break;
//							case 1: c='/'; break;
//							case 2: c='-'; break;
//							case 3: c='\\'; break;
//						}
//						gOled.printf("RSSI: L=%03d R=%03d %c\r\n", abs(RssiValue), abs(RssiValueSlave), c);
//						gOled.setTextCursor(0,48);  //Last line(48), first column

                        delayedSendPing = 100; //Send PING in 1000ms

                        // Send the next PING frame
//                        strcpy( (char*)Buffer, (char*)PingMsg);
//                        wait_ms( 10);
//                        pRadio->Send(Buffer, sizeof(PingMsg));    //Send PING message
                    }
                    // valid reception but neither a PING or a PONG message
                    else {
                        //Configure for receive again
                        pRadio->Rx(RX_TIMEOUT_VALUE);
                    }
                }
            }
            //We are Slave - Send "PONGnn" and Receive "PING"
            //We add the RSSI value of received message to "PONG" string
            else {
                if (BufferSize > 0) {
                    //RXed "PING" - we(slave) reply with "PONG"
                    if (strncmp((const char*) Buffer, (const char*) PingMsg, 4)
                            == 0) {
                        led = !led;
                        debug_if(DEBUG_MESSAGE, "...Ping (RSSI=%d)\n\r",
                                RssiValue);
                        strcpy((char*) Buffer, (char*) PongMsg);
                        Buffer[sizeof(PongMsg)] = (uint8_t) RssiValue;
                        Buffer[sizeof(PongMsg) + 1] = (uint8_t)(RssiValue >> 8);
                        wait_ms(10);
                        pRadio->Send(Buffer, sizeof(PongMsg) + 2); //Send PONG message + 2 byets RSSI value
                    }
                    // valid reception but not a PING as expected
                    else {
                        //Configure for receive again
                        pRadio->Rx(RX_TIMEOUT_VALUE);
                    }
                }
            }
            State = LOWPOWER;
            break;
        case TX:
            //debug_if (DEBUG_MESSAGE, "> Tx Done\n\r");

            if (isMaster == true) {
                //debug_if (DEBUG_MESSAGE,"Ping...\r\n");
            } else {
                debug_if(DEBUG_MESSAGE, "Pong...\r\n");
            }
            pRadio->Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
            debug_if(DEBUG_MESSAGE, "> Rx Timeout\n\r");
            //We are Master - Send the next PING frame. Flash LED for 100ms.
            if (isMaster == true) {
                led = !led;
#if !defined(DISABLE_OLED)
                gOled.setTextSize(2);
                gOled.setTextCursor(0, 48); //Last line(48), first column
                gOled.printf("Rx Timeout");
                gOled.display();
                //wait_ms(100);
#endif

                strcpy((char*) Buffer, (char*) PingMsg);
                pRadio->Send(Buffer, sizeof(PingMsg)); //Send PING message
                State = LOWPOWER;
            }
            //We are Slave - Configure for receive again
            else {
                pRadio->Rx(RX_TIMEOUT_VALUE);
                State = LOWPOWER;
            }
            break;
        case RX_ERROR:
            debug_if(DEBUG_MESSAGE, "> Rx Error\n\r");
            // We have received a Packet with a CRC error, send reply as if packet was correct
            if (isMaster == true) {
                // Send the next PING frame
                strcpy((char*) Buffer, (char*) PingMsg);
                wait_ms(10);
                pRadio->Send(Buffer, sizeof(PingMsg)); //Send PING message
            } else {
                // Send the next PONG frame
                strcpy((char*) Buffer, (char*) PongMsg);
                wait_ms(10);
                pRadio->Send(Buffer, sizeof(PongMsg)); //Send PONG message
            }
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            debug_if(DEBUG_MESSAGE, "> Tx Timeout\n\r");
            pRadio->Rx(RX_TIMEOUT_VALUE);
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
    pRadio->Sleep();
    State = TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    pRadio->Sleep();
    BufferSize = size;
    memcpy(Buffer, payload, BufferSize);
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
}

void OnTxTimeout(void) {
    pRadio->Sleep();
    State = TX_TIMEOUT;
}

void OnRxTimeout(void) {
    pRadio->Sleep();
    Buffer[BufferSize] = 0;
    State = RX_TIMEOUT;
}

void OnRxError(void) {
    pRadio->Sleep();
    State = RX_ERROR;
}

#if defined(NZ32_ST1L_REV1) || defined(NZ32_ST1L_REV2)
uint16_t getBattMV(void) {
    float fval = 0;

//	uint16_t meas;
//
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
#endif
