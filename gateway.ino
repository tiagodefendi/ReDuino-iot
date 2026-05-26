/* gateway */

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN  7
#define CSN_PIN 8

#define DATA 0
#define ACK  1
#define RTS  2
#define CTS  3

#define MAX_SIZE     32
#define TIMEOUT      500
#define MAX_RETRIES  3

uint64_t address[2]  = { 0x4040404040LL, 0x3030303030LL };
uint64_t pipeAtuador = 0x5050505050LL;
uint8_t  origem  = 30;
uint8_t  sensor  = 47;
uint8_t  atuador = 60;
uint8_t  canalAtual = 76;

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

// MACA completo gateway → atuador: RTS→CTS→DATA→ACK
void enviaAtuador(char* distStr) {
  uint8_t msg[8];
  uint8_t msgLen = strlen(distStr);
  for (uint8_t i = 0; i < msgLen; i++) msg[i] = (uint8_t)distStr[i];

  radio.openWritingPipe(pipeAtuador);

  for (int t = 0; t < MAX_RETRIES; t++) {
    // Etapa 1: RTS
    envia(atuador, RTS, (uint8_t*)"", 4);

    // Etapa 2: aguarda CTS
    if (recebe(CTS, atuador) != 0) { delay(50*(t+1)); continue; }

    // Etapa 3: DATA
    envia(atuador, DATA, msg, msgLen + 4);

    // Etapa 4: aguarda ACK
    if (recebe(ACK, atuador) != 0) { delay(50*(t+1)); continue; }

    break; // sucesso
  }

  radio.openWritingPipe(address[1]);
}

void escutar_ciclo() {
  // Etapa 1: aguarda RTS do sensor
  if (recebe(RTS, sensor) != 0) return;

  // Etapa 2: envia CTS
  envia(sensor, CTS, (uint8_t*)"", 4);

  // Etapa 3: aguarda DATA
  if (recebe(DATA, sensor) != 0) return;

  // extrai distancia
  char distStr[8] = {0};
  int payloadLen = buffer[3] - 5;
  if (payloadLen > 0 && payloadLen < 8) {
    for (int i = 0; i < payloadLen; i++)
      distStr[i] = (char)buffer[i + 4];
    distStr[payloadLen] = '\0';
  }

  // Etapa 4: envia ACK ao sensor
  envia(sensor, ACK, (uint8_t*)"", 4);

  // imprime pro Node.js
  Serial.print(sensor);
  Serial.print(F(": "));
  Serial.println(distStr);

  // retransmite pro atuador com MACA completo
  enviaAtuador(distStr);
}

// troca de canal via Serial: Node.js envia "CH:XX\n"
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
  radio.setCRCLength(RF24_CRC_16);
  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(1, address[0]);
}

void loop() {
  verificaComandoSerial();
  escutar_ciclo();
  delay(10);
}