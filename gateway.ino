/**
 * ReDuino — Gateway (Arduino Nano)
 * 
 * Hardware:
 *   - Arduino Nano
 *   - Módulo nRF24L01 (CE=7, CSN=8)
 *   - Conectado via USB ao computador (Node.js)
 * 
 * Função:
 *   Recebe pacotes do nó sensor (ID=47) via nRF24 e
 *   imprime na serial no formato "47: <distância>" para
 *   que o servidor Node.js processe e sirva ao frontend.
 * 
 *   Também recebe comandos da serial (enviados pelo Node.js)
 *   e encaminha ao nó atuador (se necessário no futuro).
 * 
 * Protocolo:
 *   O gateway atua como receptor passivo: escuta RTS,
 *   responde CTS, recebe DATA, responde ACK.
 *   Endereços invertidos em relação ao sensor.
 */

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

// -----------------------------------------------
// Pinos
// -----------------------------------------------
#define CE_PIN  7
#define CSN_PIN 8

// -----------------------------------------------
// Protocolo
// -----------------------------------------------
#define DATA 0
#define ACK  1
#define RTS  2
#define CTS  3

#define MAX_SIZE 32
#define TIMEOUT  500

// -----------------------------------------------
// Endereços e identidades
// -----------------------------------------------
// Gateway usa endereços invertidos para se comunicar
// com o sensor (sensor escreve em address[0], gateway lê em address[0])
uint64_t address[2] = { 0x4040404040LL, 0x3030303030LL };
uint8_t  origem  = 30; // ID deste gateway
uint8_t  sensor  = 47; // ID do nó sensor

RF24 radio(CE_PIN, CSN_PIN);

uint8_t payload[MAX_SIZE];
uint8_t buffer[MAX_SIZE];

// -----------------------------------------------
// Utilitários
// -----------------------------------------------
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

  for (int i = 0; i < size - 4; i++)
    payload[i + 4] = mensagem[i];

  uint8_t checksum = checksum_f(payload, size);
  payload[size] = checksum;

  unsigned long inicio = millis();
  while (millis() - inicio < TIMEOUT) {
    radio.startListening();
    delayMicroseconds(130);
    if (!radio.testCarrier()) {
      radio.stopListening();
      radio.write(&payload[0], size + 1);
      return;
    }
    delayMicroseconds(270);
  }
}

/**
 * Escuta por um pacote do tipo `type` vindo de `src`.
 * Retorna 0 em sucesso, 1 em timeout.
 */
int recebe(int type, int src) {
  radio.startListening();
  unsigned long inicio = millis();
  while (millis() - inicio < TIMEOUT) {
    if (radio.available()) {
      delay(10);
      radio.read(buffer, MAX_SIZE);
      int tamanho = buffer[3];

      if (buffer[0] != origem) continue; // destinado a mim?
      if (buffer[1] != src)    continue; // vem do nó esperado?
      if (buffer[2] != type)   continue; // tipo correto?
      if (tamanho > MAX_SIZE)  continue;

      uint8_t cs = checksum_f(buffer, tamanho - 1);
      if (buffer[tamanho - 1] != cs) continue;

      // Se é DATA, extrai e imprime na serial
      if (type == DATA) {
        char distStr[8] = {0};
        int payloadLen = tamanho - 5; // descontar header(4) + checksum(1)
        if (payloadLen > 0 && payloadLen < 8) {
          for (int i = 0; i < payloadLen; i++)
            distStr[i] = (char)buffer[i + 4];
          distStr[payloadLen] = '\0';

          // Formato reconhecido pelo Node.js: "47: 85"
          Serial.print(buffer[1]); // origem (47)
          Serial.print(F(": "));
          Serial.println(distStr); // distância
        }
      }

      radio.flush_rx();
      return 0;
    }
  }
  return 1;
}

/**
 * Ciclo completo de recepção: RTS → CTS → DATA → ACK
 */
void escutar_ciclo() {
  // 1. Espera RTS do sensor
  if (recebe(RTS, sensor) != 0) return;

  // 2. Responde CTS
  envia(sensor, CTS, "", 4);

  // 3. Espera DATA
  if (recebe(DATA, sensor) != 0) return;

  // 4. Responde ACK
  envia(sensor, ACK, "", 4);
}

// -----------------------------------------------
// Setup
// -----------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (!radio.begin()) {
    Serial.println(F("nRF24 não respondeu!"));
    while (1) {}
  }

  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(121);
  radio.setPayloadSize(MAX_SIZE);
  radio.setAutoAck(false);
  radio.setCRCLength(RF24_CRC_DISABLED);
  radio.setDataRate(RF24_250KBPS);

  // Gateway lê em address[0] (onde o sensor escreve)
  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(1, address[0]);

  printf_begin();
  radio.printPrettyDetails();

  Serial.println(F("=== ReDuino Gateway pronto ==="));
}

// -----------------------------------------------
// Loop
// -----------------------------------------------
void loop() {
  escutar_ciclo();
  delay(10);
}
