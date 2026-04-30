/**
 * ReDuino — Nó Sensor (Arduino Nano)
 * 
 * Hardware:
 *   - Arduino Nano
 *   - Módulo nRF24L01 (CE=7, CSN=8)
 *   - Sensor ultrassônico HC-SR04 (TRIG=3, ECHO=4)
 * 
 * Função:
 *   Mede a distância com o HC-SR04 e envia ao gateway via nRF24.
 *   Formato do payload: distância em cm como inteiro (2 bytes).
 * 
 * Protocolo (RTS → CTS → DATA → ACK):
 *   - Antes de enviar, faz RTS/CTS para checar o meio (CSMA-like).
 *   - Checksum de 1 byte no final de cada pacote.
 *   - Retransmite até MAX_RETRIES vezes em caso de falha.
 */

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

// -----------------------------------------------
// Pinos
// -----------------------------------------------
#define CE_PIN   7
#define CSN_PIN  8
#define TRIG_PIN 3
#define ECHO_PIN 4

// -----------------------------------------------
// Protocolo
// -----------------------------------------------
#define DATA 0
#define ACK  1
#define RTS  2
#define CTS  3

#define MAX_SIZE    32
#define TIMEOUT     500   // ms para esperar ACK/CTS
#define MAX_RETRIES 3     // tentativas de reenvio
#define SEND_INTERVAL 200 // ms entre leituras

// -----------------------------------------------
// Endereços e identidades
// -----------------------------------------------
uint64_t address[2] = { 0x4040404040LL, 0x3030303030LL };
uint8_t  origem  = 47;  // ID deste nó (sensor)
uint8_t  destino = 30;  // ID do gateway

RF24 radio(CE_PIN, CSN_PIN);

uint8_t payload[MAX_SIZE];
uint8_t buffer[MAX_SIZE];

// -----------------------------------------------
// Funções utilitárias
// -----------------------------------------------
uint8_t checksum_f(uint8_t* msg, int size) {
  uint16_t sum = 0;
  for (int i = 0; i < size; i++) sum += msg[i];
  return (uint8_t)(sum & 0xFF);
}

/**
 * Mede distância com HC-SR04.
 * Retorna distância em cm, ou -1 em caso de timeout.
 */
int medirDistancia() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duracao = pulseIn(ECHO_PIN, HIGH, 23200); // timeout ~400cm
  if (duracao == 0) return -1;

  int distancia = (int)(duracao * 0.034 / 2);
  return constrain(distancia, 0, 400);
}

// -----------------------------------------------
// Protocolo de envio (igual ao código de referência)
// -----------------------------------------------
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
  Serial.println(F("TimeOut! (canal ocupado)"));
}

int recebe(int type, int dest) {
  radio.startListening();
  unsigned long inicio = millis();
  while (millis() - inicio < TIMEOUT) {
    if (radio.available()) {
      delay(10);
      radio.read(buffer, MAX_SIZE);
      int tamanho = buffer[3];

      if (buffer[0] != dest)    continue;
      if (buffer[1] != origem)  continue;
      if (buffer[2] != type)    continue;
      if (tamanho > MAX_SIZE)   continue;

      uint8_t cs = checksum_f(buffer, tamanho - 1);
      if (buffer[tamanho - 1] != cs) continue;

      radio.flush_rx();
      return 0;
    }
  }
  return 1; // timeout
}

/**
 * Envia pacote de dados com protocolo RTS→CTS→DATA→ACK.
 * Retorna 0 em sucesso, >0 em erro.
 */
int envia_pacote(int dest, uint8_t* mensagem, uint8_t size) {
  if (size + 5 > MAX_SIZE) return 1;

  for (int tentativa = 0; tentativa < MAX_RETRIES; tentativa++) {
    envia(dest, RTS, "", 4);

    if (recebe(CTS, dest) != 0) {
      Serial.println(F("CTS não recebido, retentando..."));
      delay(50);
      continue;
    }

    envia(dest, DATA, mensagem, size + 4);

    if (recebe(ACK, dest) != 0) {
      Serial.println(F("ACK não recebido, retentando..."));
      delay(50);
      continue;
    }

    return 0; // sucesso
  }
  return 2; // falha após retentativas
}

// -----------------------------------------------
// Setup
// -----------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

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

  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(1, address[1]);

  printf_begin();
  radio.printPrettyDetails();

  Serial.println(F("=== ReDuino Sensor pronto ==="));
}

// -----------------------------------------------
// Loop
// -----------------------------------------------
void loop() {
  int distancia = medirDistancia();

  if (distancia < 0) {
    Serial.println(F("⚠️  Sensor sem resposta"));
    delay(SEND_INTERVAL);
    return;
  }

  // Monta payload: distância como string (ex: "085")
  // O gateway vai repassar como "47: 85" para o Node.js
  char msg[8];
  snprintf(msg, sizeof(msg), "%d", distancia);
  uint8_t msgBytes[8];
  uint8_t msgLen = strlen(msg);
  for (uint8_t i = 0; i < msgLen; i++) msgBytes[i] = msg[i];

  Serial.print(F("Enviando distância: "));
  Serial.print(distancia);
  Serial.println(F(" cm"));

  int resultado = envia_pacote(destino, msgBytes, msgLen);

  if (resultado == 0) {
    Serial.println(F("✓ Enviado com sucesso"));
  } else {
    Serial.print(F("✗ Falha no envio, código: "));
    Serial.println(resultado);
  }

  delay(SEND_INTERVAL);
}
