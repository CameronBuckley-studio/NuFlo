//------ LORA Stuff
#include "LoRaWan_APP.h"
#include "Arduino.h"

#define RF_FREQUENCY               915000000  // Hz
#define TX_OUTPUT_POWER            21         // dBm
#define LORA_BANDWIDTH             0          // [0: 125 kHz]
#define LORA_SPREADING_FACTOR      7          // [SF7..SF12]
#define LORA_CODINGRATE            1          // [1: 4/5]
#define LORA_PREAMBLE_LENGTH       8
#define LORA_SYMBOL_TIMEOUT        0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON       false
#define RX_TIMEOUT_VALUE           1000
#define BUFFER_SIZE                85

//------- Pin Definitions (use actual ESP32 ADC1 pins)
#define tdsPin    2  
#define temperaturePin  3   
#define phPin           4  

#define DELAY_TIME      2000  // ms

// Sensor readings
float   rawTds;
float   rawTemp;
float   rawPH;
float voltageTemp;
float temperatureC;
float phValue;

// LoRa variables
char  txPacket[BUFFER_SIZE];
bool  loraIdle = true;
unsigned long txCount = 0;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );

void setup() {
  Serial.begin(115200);

  pinMode(tdsPin,   INPUT);
  pinMode(temperaturePin, INPUT);
  pinMode(phPin,          INPUT);

  // LoRa init
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(
    MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
    true, 0, 0, LORA_IQ_INVERSION_ON, 3000
  );
}

void loop() {
  // --- 1) Read raw ADC ---
  rawTds       = analogRead(tdsPin);
  rawTemp      = analogRead(temperaturePin);
  rawPH        = analogRead(phPin);


  //NO TDS?
  //Temperature sensor seems wrong down here:
  voltageTemp   = rawTemp * (3.3 / 4095.0);
  temperatureC  = (voltageTemp - 0.5) * 100.0;

  float voltagePH   = rawPH * (5.0 / 1023.0);
  phValue           = (-5.76923 * voltagePH) + 21.48077;

  // --- 4) Send via LoRa when ready ---
  if (loraIdle) {
    delay(1000);
    int len = snprintf(txPacket, BUFFER_SIZE,
      "NUFLO1 | Turb:%d | T:%.1fC | pH:%.2f | #%lu",
      rawTurbidity,
      temperatureC,
      phValue,
      txCount
    );
    Serial.printf("\nSending packet \"%s\" (len=%d)\n", txPacket, len);
    Radio.Send((uint8_t*)txPacket, len);
    txCount++;
    loraIdle = false;
  }

  Radio.IrqProcess();
  delay(DELAY_TIME);
}

void OnTxDone(void) {
  Serial.println("TX done");
  loraIdle = true;
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.println("TX Timeout");
  loraIdle = true;
}
