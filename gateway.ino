/* gateway */

#include <SPI.h>
#include "RF24.h"

#define CE_PIN  7
#define CSN_PIN 8

#define DATA 0
#define ACK  1
#define RTS  2
#define CTS  3

#define MAX_SIZE    32
#define TIMEOUT     150
#define MAX_RETRIES 3
#define NETWORK_ID  0x68

const uint64_t PIPE_ADDR = 0x3030303030LL;

uint8_t origem     = 30;
uint8_t sensor     = 47;
uint8_t atuador    = 60;
uint8_t canalAtual = 12;

RF24 radio(CE_PIN, CSN_PIN);
uint8_t payload[MAX_SIZE];
uint8_t buffer[MAX_SIZE];

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

int recebe(int type, int src) {
  radio.startListening();
  unsigned long inicio = millis();
  while (millis() - inicio < TIMEOUT) {
    if (radio.available()) {
      radio.read(buffer, MAX_SIZE);
      int tamanho = buffer[3];
      if (buffer[0] != src)                                    continue;
      if (buffer[1] != origem)                                 continue;
      if (buffer[2] != type)                                   continue;
      if (buffer[4] != NETWORK_ID)                             continue;
      if (tamanho > MAX_SIZE)                                  continue;
      if (buffer[tamanho-1] != checksum_f(buffer, tamanho-1)) continue;
      radio.flush_rx();
      return 0;
    }
  }
  return 1;
}

void enviaAtuador(char* distStr) {
  uint8_t msgLen = strlen(distStr);
  for (int t = 0; t < MAX_RETRIES; t++) {
    envia(atuador, RTS, nullptr, 0);
    if (recebe(CTS, atuador) != 0) { delay(13*(t+1)); continue; }
    envia(atuador, DATA, (uint8_t*)distStr, msgLen);
    if (recebe(ACK, atuador) != 0) { delay(13*(t+1)); continue; }
    break;
  }
}

void escutar_ciclo() {
  if (recebe(RTS, sensor) != 0) return;
  envia(sensor, CTS, nullptr, 0);
  if (recebe(DATA, sensor) != 0) return;

  char distStr[8] = {0};
  int dataLen = buffer[3] - 6;
  if (dataLen > 0 && dataLen < 8) {
    for (int i = 0; i < dataLen; i++)
      distStr[i] = (char)buffer[i + 5];
    distStr[dataLen] = '\0';
  }

  envia(sensor, ACK, nullptr, 0);

  Serial.print(sensor);
  Serial.print(F(": "));
  Serial.println(distStr);

  enviaAtuador(distStr);
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
    }
  }
}

void setup() {
  Serial.begin(19200);
  while (!Serial) {}

  if (!radio.begin()) { while (1) {} }
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(canalAtual);
  radio.setDataRate(RF24_1MBPS);
  radio.setPayloadSize(MAX_SIZE);
  radio.setAutoAck(false);
  radio.setCRCLength(RF24_CRC_DISABLED);
  radio.openWritingPipe(PIPE_ADDR);
  radio.openReadingPipe(1, PIPE_ADDR);
}

void loop() {
  verificaComandoSerial();
  escutar_ciclo();
  delay(10);
}
