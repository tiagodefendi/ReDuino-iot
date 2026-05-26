#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN  7
#define CSN_PIN 8

#define DATA 0
#define ACK  1
#define RTS  2
#define CTS  3

#define MAX_SIZE 32
#define TIMEOUT  500

uint64_t address[2]   = { 0x4040404040LL, 0x3030303030LL };
uint64_t pipeAtuador  = 0x5050505050LL;
uint8_t  origem  = 30;
uint8_t  sensor  = 47;

RF24 radio(CE_PIN, CSN_PIN);
uint8_t payload[MAX_SIZE];
uint8_t buffer[MAX_SIZE];

uint8_t checksum_f(uint8_t* msg, int size) {
  uint16_t sum = 0;
  for (int i = 0; i < size; i++) sum += msg[i];
  return (uint8_t)(sum & 0xFF);
}

void envia(int dest, int tipo, uint8_t* mensagem, uint8_t size) {
  radio.flush_tx();
  payload[0] = origem;
  payload[1] = dest;
  payload[2] = tipo;
  payload[3] = size + 1;
  for (int i = 0; i < size - 4; i++) payload[i + 4] = mensagem[i];
  payload[size] = checksum_f(payload, size);

  unsigned long inicio = millis();
  while (millis() - inicio < TIMEOUT) {
    radio.startListening();
    delayMicroseconds(130);
    if (!radio.testCarrier()) {
      radio.stopListening();
      radio.write(&payload[0], MAX_SIZE);
      return;
    }
    radio.stopListening();
    delayMicroseconds(270);
  }
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

void escutar_ciclo() {
  if (recebe(RTS, sensor) != 0) return;

  envia(sensor, CTS, (uint8_t*)"", 4);

  if (recebe(DATA, sensor) != 0) return;

  // extrai distancia
  char distStr[8] = {0};
  int payloadLen = buffer[3] - 5;
  if (payloadLen > 0 && payloadLen < 8) {
    for (int i = 0; i < payloadLen; i++)
      distStr[i] = (char)buffer[i + 4];
    distStr[payloadLen] = '\0';
  }

  // ACK ao sensor
  envia(sensor, ACK, (uint8_t*)"", 4);

  // envia ao Node.js — unico print permitido
  Serial.print(sensor);
  Serial.print(F(": "));
  Serial.println(distStr);

  // retransmite ao atuador
  radio.openWritingPipe(pipeAtuador);
  envia(60, DATA, (uint8_t*)distStr, strlen(distStr) + 4);
  radio.openWritingPipe(address[1]);
}

void setup() {
  Serial.begin(19200);
  while (!Serial) {}

  if (!radio.begin()) { while (1) {} }

  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(76);
  radio.setDataRate(RF24_1MBPS);
  radio.setPayloadSize(MAX_SIZE);
  radio.setAutoAck(false);
  radio.setCRCLength(RF24_CRC_16);
  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(1, address[0]);
}

void loop() {
  escutar_ciclo();
  delay(10);
}