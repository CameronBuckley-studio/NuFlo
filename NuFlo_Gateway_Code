//------ LORA Gateway Receiver Code
#include "LoRaWan_APP.h"
#include "Arduino.h"

#define RF_FREQUENCY               915000000  // Hz
#define LORA_BANDWIDTH             0          // [0: 125 kHz]
#define LORA_SPREADING_FACTOR      7          // [SF7..SF12]
#define LORA_CODINGRATE            1          // [1: 4/5]
#define LORA_PREAMBLE_LENGTH       8
#define LORA_SYMBOL_TIMEOUT        0          // symbols to wait for preamble
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON       false
#define RX_TIMEOUT_VALUE           3000       // ms to wait for a packet
#define BUFFER_SIZE                256        // max incoming packet size

static RadioEvents_t RadioEvents;
volatile bool    packetReceived = false;
uint8_t          rxBuffer[BUFFER_SIZE];
uint16_t         rxSize = 0;
int16_t          rxRssi = 0;
int8_t           rxSnr  = 0;

// Forward declarations
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting LoRa Gateway Receiver...");

  // LoRa init
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  RadioEvents.RxDone      = OnRxDone;
  RadioEvents.RxTimeout   = OnRxTimeout;
  RadioEvents.RxError     = OnRxError;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(
    MODEM_LORA,
    0,                      // not used for receive
    LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR,
    LORA_CODINGRATE,
    LORA_PREAMBLE_LENGTH,
    LORA_SYMBOL_TIMEOUT,
    LORA_FIX_LENGTH_PAYLOAD_ON,
    0,                      // no CRC on receive
    true,
    0,
    0,
    LORA_IQ_INVERSION_ON,
    RX_TIMEOUT_VALUE
  );

  // Start continuous reception
  Radio.Rx(RX_TIMEOUT_VALUE);
}

void loop() {
  // Process radio IRQs
  Radio.IrqProcess();

  if (packetReceived) {
    // Print raw data to Serial for computer
    Serial.print("Received packet: ");
    for (uint16_t i = 0; i < rxSize; i++) {
      Serial.write(rxBuffer[i]);
    }
    Serial.print("  | RSSI: "); Serial.print(rxRssi);
    Serial.print(" dBm  | SNR: ");  Serial.print(rxSnr);
    Serial.println(" dB");

    // Reset flag and re-arm reception
    packetReceived = false;
    Radio.Rx(RX_TIMEOUT_VALUE);
  }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  // Copy into buffer
  rxSize = min((uint16_t)BUFFER_SIZE, size);
  memcpy(rxBuffer, payload, rxSize);
  rxRssi = rssi;
  rxSnr  = snr;
  packetReceived = true;
}

void OnRxTimeout(void) {
  // Timeout: restart receiver
  Radio.Rx(RX_TIMEOUT_VALUE);
}

void OnRxError(void) {
  // Reception error: restart
  Radio.Rx(RX_TIMEOUT_VALUE);
}
