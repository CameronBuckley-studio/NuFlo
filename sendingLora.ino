//------ LORA Stuff
#include "LoRaWan_APP.h"
#include "Arduino.h"
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             21        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 85

//------- Pin Definitions
#define turbiditypin 2
#define temperaturepin 3
#define phpin 4


//------ Values we are probably gonna change
#define delayTime 2000


//------ Additional Variables
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;

bool lora_idle=true;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );

//----- Functions

void setup() {
// start talking to computer
  Serial.begin(115200);
//Defining what pin is our sensor plugged into
  pinMode(turbiditypin, INPUT);
  pinMode(temperaturepin, INPUT);
  pinMode(phpin, INPUT);

//Lora Stuff
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
	
    txNumber=0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 

}

void loop() {
  // Lora Stuff
	if(lora_idle == true)
	{
    delay(1000);
		sprintf(txpacket, "NUFLO1: Turbidity %d, Temperature %d, Ph %d, END OF MESSAGE %f", sensorValueTurb, sensorValueTemp, sensorValuePh);  //start a package
		Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));

		Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package out	
    lora_idle = false;
	}
  Radio.IrqProcess( );


  delay(delayTime);
}

//----- Lora Functions
void OnTxDone( void )
{
	Serial.println("TX done......");
	lora_idle = true;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
}