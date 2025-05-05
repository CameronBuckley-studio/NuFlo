//------ LORA Stuff
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <math.h>

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

//------- Pin Definitions
#define tdsPin            2    // TDS sensor on ADC1_CH2 (GPIO2)
#define temperaturePin    3    // TMP36 on ADC1_CH3 (GPIO3)
#define phPin             4    // pH probe on ADC1_CH4 (GPIO4)

#define DELAY_TIME        2000 // ms between LoRa sends

// TDS‐filter settings
#define SCOUNT            30   // median filter sample count

// Sensor readings
float   voltageTemp;
float   temperatureC;
float   voltagePH;
float   phValue;
float   tdsValue;

// LoRa variables
char    txPacket[BUFFER_SIZE];
bool    loraIdle = true;
unsigned long txCount = 0;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );

// Median filter helper
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  memcpy(bTab, bArray, iFilterLen * sizeof(int));
  // simple bubble‐sort
  for (int i = 0; i < iFilterLen - 1; i++) {
    for (int j = 0; j < iFilterLen - i - 1; j++) {
      if (bTab[j] > bTab[j + 1]) {
        int tmp = bTab[j];
        bTab[j] = bTab[j + 1];
        bTab[j + 1] = tmp;
      }
    }
  }
  // return middle value
  if (iFilterLen & 1) {
    return bTab[(iFilterLen - 1) / 2];
  } else {
    return (bTab[iFilterLen/2] + bTab[iFilterLen/2 - 1]) / 2;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(tdsPin,        INPUT);
  pinMode(temperaturePin, INPUT);
  pinMode(phPin,           INPUT);

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
  // --- 1) Read raw ADC for temp and pH ---
  int rawTemp = analogRead(temperaturePin);
  int rawPH   = analogRead(phPin);

  // Convert TMP36 voltage to °C (ESP32 ADC is 12‑bit: 0–4095 over 3.3V)
  voltageTemp  = rawTemp * (3.3 / 4095.0);
  temperatureC = (voltageTemp - 0.5) * 100.0;

  // pH conversion (assuming 5V probe & 10‑bit scaling)
  voltagePH = rawPH * (5.0 / 1023.0);
  phValue   = (-5.76923 * voltagePH) + 21.48077;

  // --- 2) Compute TDS via median filter + temp compensation ---
  static int analogBuffer[SCOUNT];
  for (int i = 0; i < SCOUNT; i++) {
    analogBuffer[i] = analogRead(tdsPin);
  }
  float avgADC = getMedianNum(analogBuffer, SCOUNT);
  // Convert ADC → voltage (3.3V ref, 12‑bit)
  float avgVoltage = avgADC * (3.3 / 4095.0);
  // Temp compensation (probe datasheet formula)
  float compCoeff      = 1.0 + 0.02 * (temperatureC - 25.0);
  float compVoltage    = avgVoltage / compCoeff;
  // TDS calculation (ppm)
  tdsValue = (133.42 * pow(compVoltage, 3)
            - 255.86 * pow(compVoltage, 2)
            + 857.39 * compVoltage) * 0.5;

  // --- 3) Send via LoRa when ready ---
  if (loraIdle) {
    delay(1000);  // ensure radio is ready
    int len = snprintf(txPacket, BUFFER_SIZE,
      "NUFLO1 | TDS:%.0fppm | T:%.1fC | pH:%.2f | #%lu",
      tdsValue,
      temperatureC,
      phValue,
      txCount
    );
    Serial.printf("Sending packet \"%s\" (len=%d)\n", txPacket, len);
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
