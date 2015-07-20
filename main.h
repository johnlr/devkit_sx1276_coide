/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    ( C )2014 Semtech

Description: Contains the callbacks for the IRQs and any application related details

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __MAIN_H__
#define __MAIN_H__

#define LORA_BW_7800	0	//7.8kHz
#define LORA_BW_10400	1	//10.4kHz
#define LORA_BW_15600	2	//15.6kHz
#define LORA_BW_20800	3	//20.8kHz
#define LORA_BW_31250	4	//31.25kHz
#define LORA_BW_41700	5	//41.7kHz
#define LORA_BW_62500	6	//62.5kHz
#define LORA_BW_125000	7	//125kHz
#define LORA_BW_250000	8	//250kHz
#define LORA_BW_500000	9	//500kHz

typedef struct {
	uint32_t	frequency;		//Frequench

	union {
		struct {
			uint8_t	codingRate:4;	//1=4/5, 2=4/6, 3=4/7,  4=4/8
			uint8_t crcEnable:1;
			uint8_t fixLength:1;	//0=variable, 1=fixed
			uint8_t fshhEnable:1;
			uint8_t iqInversionEnable:1;
		} lora;
		uint32_t 	Val;
	} conf;

	uint16_t		rxTimeout;		//RX Timeout in ms
	uint8_t			bw;				//bandwidth, a LORA_BW_XX define
									//0=7.8 1=10.4 2=15.6 3=20.8 4=31.25 5=41.7 6=62.5 7=125 8=250 9=500 kHz
	uint8_t			sf;				//Speading factor, a value from 7 to 12
	uint8_t			power;			//Output power in dBm, a value from 1 to 20
	uint8_t			preambleLength;	//Length of preamble, default = 5
	uint8_t			numberSymHop;
	uint8_t			symbolTimeout;

} AppConfig;


typedef struct {
    union {
        struct {
            uint8_t running         :1;
            uint8_t rxedPacket      :1;
            uint8_t updateDisplay   :1;

        } bit;
        uint32_t    Val;
    } flags;

    uint16_t    rxOK;           //Successful packets received
    uint16_t    rxErr;          //Error packets received
    uint16_t    rxCount;        //Count received packets

    int16_t     RssiValue;      //RSSI value of last reception, or 0xfff if receive timeout
    int16_t     RssiValueSlave; //RSSI value of slave
    int8_t      SnrValue;       //Signal to noise ratio
    uint8_t     boardType;      //The board type, a BOARD_XXX define
} AppData;


/*
 * Callback functions prototypes
 */
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * @brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * @brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * @brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * @brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

/*!
 * @brief Function executed on Radio Fhss Change Channel event
 */
void OnFhssChangeChannel( uint8_t channelIndex );

/*!
 * @brief Function executed on CAD Done event
 */
void OnCadDone( bool channelActivityDetected );

/*!
 * Get battery voltage in millivolts.
 */
uint16_t getBattMV( void );

/*!
 * Get Vusb or 5V supply voltage in millivolts.
 */
uint16_t getSupplyMV( void );


#endif // __MAIN_H__
