/* atuador */

#include <SPI.h>
#include "RF24.h"

#define CE_PIN 7
#define CSN_PIN 8
#define BUZZER_PIN A3
#define LED_PIN A1

#define DATA 0
#define ACK 1
#define RTS 2
#define CTS 3

#define MAX_SIZE 32
#define TIMEOUT 100
#define NETWORK_ID 0x68

const uint64_t PIPE_ADDR = 0x3030303030LL;

uint8_t origem = 60;
uint8_t gateway = 30;
uint8_t canalAtual = 12;

RF24 radio(CE_PIN, CSN_PIN);
uint8_t payload[MAX_SIZE];
uint8_t buffer[MAX_SIZE];
int lastDist = 201;

void atualizarFeedback(int dist) {
  static unsigned long lastBlink = 0;
  static unsigned long lastBeep = 0;
  static unsigned long intervaloAtual = 0;
  static bool ledState = false;
  unsigned long agora = millis();

  if (dist > 200) {
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
    return;
  }
  if (dist <= 15) {
    noTone(BUZZER_PIN);
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 3800);
    return;
  }

  unsigned long intervalo;
  int freq;

  if (dist > 30) {
    intervalo = 1000; freq = 350;
  }
  else if (dist > 15) {
    intervalo = 333; freq = 700;
  }
  else {
    intervalo = 150; freq = 1400;
  }

  if (intervalo != intervaloAtual) {
    intervaloAtual = intervalo;
    lastBlink = 0;
    lastBeep = 0;
    noTone(BUZZER_PIN);
  }

  if (agora - lastBlink >= intervalo) {
    lastBlink = agora;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }

  if (agora - lastBeep >= intervalo) {
    lastBeep = agora;
    noTone(BUZZER_PIN);
    tone(BUZZER_PIN, freq, intervalo / 2);
  }
}

uint8_t checksum_f(uint8_t* msg, int size) {
  uint16_t sum = 0;
  for (int i = 0; i < size; i++) sum += msg[i];
  return (uint8_t)(sum & 0xFF);
}

void envia(int dest, int tipo, uint8_t* mensagem, uint8_t dataLen) {
  radio.flush_tx();
  uint8_t total = dataLen + 6;
  payload[0] = origem;
  payload[1] = dest;
  payload[2] = tipo;
  payload[3] = total;
  payload[4] = NETWORK_ID;
  for (uint8_t i = 0; i < dataLen; i++) payload[i + 5] = mensagem[i];
  payload[total - 1] = checksum_f(payload, total - 1);

  unsigned long inicio = millis();
  while (millis() - inicio < TIMEOUT) {
    radio.startListening();
    delayMicroseconds(67);
    if (!radio.testCarrier()) {
      radio.stopListening();
      radio.write(&payload[0], MAX_SIZE);
      return;
    }
    radio.stopListening();
    delayMicroseconds(67);
  }
}

int recebe(int type, int src, unsigned int ms = TIMEOUT) {
  radio.startListening();
  unsigned long inicio = millis();
  while (millis() - inicio < ms) {
    if (radio.available()) {
      radio.read(buffer, MAX_SIZE);
      int tamanho = buffer[3];
      if (buffer[0] != src) continue;
      if (buffer[1] != origem) continue;
      if (buffer[2] != type) continue;
      if (buffer[4] != NETWORK_ID) continue;
      if (tamanho > MAX_SIZE) continue;
      if (buffer[tamanho-1] != checksum_f(buffer, tamanho-1)) continue;
      radio.flush_rx();
      return 0;
    }
  }
  return 1;
}

void escutar_ciclo_maca() {
  if (recebe(RTS, gateway, 25) != 0) return;
  envia(gateway, CTS, nullptr, 0);
  if (recebe(DATA, gateway) != 0) return;

  char distStr[8] = {0};
  int dataLen = buffer[3] - 6;
  if (dataLen > 0 && dataLen < 8) {
    for (int i = 0; i < dataLen; i++)
      distStr[i] = (char)buffer[i + 5];
    distStr[dataLen] = '\0';
    lastDist = atoi(distStr);
  }

  envia(gateway, ACK, nullptr, 0);

  Serial.print(F("DIST: "));
  Serial.println(lastDist);
}

void verificaComandoSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.startsWith("CH:")) {
    uint8_t ch = cmd.substring(3).toInt();
    if (ch <= 125) {
      canalAtual = ch;
      radio.setChannel(canalAtual);
      Serial.print(F("Canal: ")); Serial.println(canalAtual);
    }
  }
}

void setup() {
  Serial.begin(19200);

  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);

  if (!radio.begin()) { while (1) {} }
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(canalAtual);
  radio.setDataRate(RF24_1MBPS);
  radio.setPayloadSize(MAX_SIZE);
  radio.setAutoAck(false);
  radio.setCRCLength(RF24_CRC_DISABLED);
  radio.openReadingPipe(1, PIPE_ADDR);
  radio.openWritingPipe(PIPE_ADDR);
  radio.startListening();

  Serial.println(F("Atuador pronto"));
}

void loop() {
  verificaComandoSerial();
  escutar_ciclo_maca();
  atualizarFeedback(lastDist);
}
