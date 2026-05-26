#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN     7
#define CSN_PIN    8
#define BUZZER_PIN A3
#define LED_PIN    A1

#define DATA 0
#define ACK  1
#define RTS  2
#define CTS  3

#define MAX_SIZE 32
#define TIMEOUT  500

uint64_t pipeAtuador = 0x5050505050LL;
uint8_t  origem  = 60;
uint8_t  gateway = 30;

RF24 radio(CE_PIN, CSN_PIN);
uint8_t buffer[MAX_SIZE];
int lastDist = 200;

// ── Buzzer passivo toggle manual ──────────────────────────────────────
void bip(int freq, int ms) {
  long periodo = 1000000L / freq / 2;
  long ciclos  = (long)ms * 1000L / (periodo * 2);
  for (long i = 0; i < ciclos; i++) {
    digitalWrite(BUZZER_PIN, HIGH); delayMicroseconds(periodo);
    digitalWrite(BUZZER_PIN, LOW);  delayMicroseconds(periodo);
  }
}

void atualizarFeedback(int dist) {
  static unsigned long lastBlink = 0;
  static unsigned long lastBeep  = 0;
  static bool ledState = false;
  unsigned long agora = millis();

  unsigned long intervaloLED, intervaloBuzz;
  int freq;

  if (dist > 200) {
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
    return;
  } else if (dist > 50) {
    intervaloLED  = 900;
    intervaloBuzz = 900;
    freq = 700;
  } else if (dist > 30) {
    intervaloLED  = 450;
    intervaloBuzz = 450;
    freq = 700;
  } else if (dist > 15) {
    intervaloLED  = 180;
    intervaloBuzz = 180;
    freq = 700;
  } else {
    intervaloLED  = 100;
    intervaloBuzz = 100;
    freq = 700;
  }

  // LED
  if (agora - lastBlink >= intervaloLED) {
    lastBlink = agora;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }

  // Buzzer com tone()
  if (agora - lastBeep >= intervaloBuzz) {
    lastBeep = agora;
    tone(BUZZER_PIN, freq, intervaloLED / 2);
  }
}

uint8_t checksum_f(uint8_t* msg, int size) {
  uint16_t sum = 0;
  for (int i = 0; i < size; i++) sum += msg[i];
  return (uint8_t)(sum & 0xFF);
}

int recebe(int type, int src) {
  radio.startListening();
  unsigned long inicio = millis();
  while (millis() - inicio < TIMEOUT) {
    if (radio.available()) {
      radio.read(buffer, MAX_SIZE);
      int tamanho = buffer[3];
      if (buffer[0] != src)    continue;
      if (buffer[1] != origem) continue;
      if (buffer[2] != type)   continue;
      if (tamanho > MAX_SIZE)  continue;
      if (buffer[tamanho-1] != checksum_f(buffer, tamanho-1)) continue;
      radio.flush_rx();
      return 0;
    }
  }
  return 1;
}

void setup() {
  Serial.begin(19200);

  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(LED_PIN,    OUTPUT); digitalWrite(LED_PIN,    LOW);

  if (!radio.begin()) { while (1) {} }

  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(76);
  radio.setDataRate(RF24_1MBPS);
  radio.setPayloadSize(MAX_SIZE);
  radio.setAutoAck(false);
  radio.setCRCLength(RF24_CRC_16);
  radio.openReadingPipe(1, pipeAtuador);
  radio.startListening();

  Serial.println(F("Atuador pronto."));
}

void loop() {
  if (recebe(DATA, gateway) == 0) {
    char distStr[8] = {0};
    int payloadLen = buffer[3] - 5;
    if (payloadLen > 0 && payloadLen < 8) {
      for (int i = 0; i < payloadLen; i++)
        distStr[i] = (char)buffer[i + 4];
      distStr[payloadLen] = '\0';
      lastDist = atoi(distStr);
    }
    // print de debug
    Serial.print(F("DIST recebida: "));
    Serial.println(lastDist);
  }

  atualizarFeedback(lastDist);
}