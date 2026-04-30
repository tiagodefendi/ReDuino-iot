/**
 * ReDuino — Nó Atuador (Arduino Nano)
 * 
 * Hardware:
 *   - Arduino Nano
 *   - Módulo nRF24L01 (CE=7, CSN=8)
 *   - Buzzer passivo (pino 5)
 *   - LED vermelho (pino 6)  — PERIGO / COLISÃO
 *   - LED amarelo (pino 9)   — ATENÇÃO
 *   - LED verde   (pino 10)  — SEGURO
 * 
 * Função:
 *   Recebe a distância do gateway via nRF24 e aciona
 *   buzzer + LEDs conforme a faixa de distância.
 * 
 *   Faixas (espelham o frontend):
 *     > 120 cm  → SEGURO   (LED verde,  1 beep/s)
 *     60-120 cm → ATENÇÃO  (LED amarelo, 3 beeps/s)
 *     25-60 cm  → PERIGO   (LED vermelho, 8 beeps/s)
 *     < 25 cm   → COLISÃO  (LED vermelho, beep contínuo)
 */

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

// -----------------------------------------------
// Pinos
// -----------------------------------------------
#define CE_PIN      7
#define CSN_PIN     8
#define BUZZER_PIN  5
#define LED_RED     6
#define LED_YELLOW  9
#define LED_GREEN   10

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
uint64_t address[2] = { 0x4040404040LL, 0x3030303030LL };
uint8_t  origem  = 30; // ID deste nó (atuador/gateway)
uint8_t  destino = 47; // ID do sensor

RF24 radio(CE_PIN, CSN_PIN);

uint8_t payload[MAX_SIZE];
uint8_t buffer[MAX_SIZE];

// -----------------------------------------------
// Estado
// -----------------------------------------------
int distanciaAtual = 200;
unsigned long ultimaRecepcao = 0;
#define TIMEOUT_SEM_DADO 2000 // ms sem receber → assume seguro

// -----------------------------------------------
// Funções utilitárias
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

int escutar() {
  radio.startListening();
  unsigned long inicio = millis();
  while (millis() - inicio < TIMEOUT) {
    if (radio.available()) {
      delay(10);
      radio.read(buffer, MAX_SIZE);
      int tamanho = buffer[3];

      if (buffer[0] != origem)   continue;
      if (buffer[1] != destino)  continue;
      if (tamanho > MAX_SIZE)    continue;

      uint8_t cs = checksum_f(buffer, tamanho - 1);
      if (buffer[tamanho - 1] != cs) continue;

      int tipo = buffer[2];

      if (tipo == RTS) {
        envia(destino, CTS, "", 4);
        // Agora espera o DATA
        return escutarData();
      }
    }
  }
  return 1;
}

int escutarData() {
  radio.startListening();
  unsigned long inicio = millis();
  while (millis() - inicio < TIMEOUT) {
    if (radio.available()) {
      delay(10);
      radio.read(buffer, MAX_SIZE);
      int tamanho = buffer[3];

      if (buffer[0] != origem)   continue;
      if (buffer[1] != destino)  continue;
      if (buffer[2] != DATA)     continue;
      if (tamanho > MAX_SIZE)    continue;

      uint8_t cs = checksum_f(buffer, tamanho - 1);
      if (buffer[tamanho - 1] != cs) continue;

      // Extrai payload (distância como string)
      char distStr[8] = {0};
      int payloadLen = tamanho - 5; // header(4) + checksum(1)
      if (payloadLen > 0 && payloadLen < 8) {
        for (int i = 0; i < payloadLen; i++)
          distStr[i] = (char)buffer[i + 4];
        distStr[payloadLen] = '\0';
        distanciaAtual = atoi(distStr);
        ultimaRecepcao = millis();
        Serial.print(F("📏 Distância recebida: "));
        Serial.println(distanciaAtual);
      }

      envia(destino, ACK, "", 4);
      radio.flush_rx();
      return 0;
    }
  }
  return 1;
}

// -----------------------------------------------
// Controle de LEDs e buzzer
// -----------------------------------------------
void desligarTudo() {
  digitalWrite(LED_RED,    LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN,  LOW);
  digitalWrite(BUZZER_PIN, LOW);
}

/**
 * Gera padrão de beep baseado na distância.
 * Chamado no loop — usa millis() para não bloquear.
 */
void atualizarAlarme(int dist) {
  static unsigned long ultimoBeep = 0;
  static bool buzzerOn = false;

  // Intervalo entre beeps (ms)
  unsigned long intervalo;
  int ledAtivo;

  if (dist > 120) {
    // SEGURO: 1 beep/s
    intervalo = 1000;
    ledAtivo = LED_GREEN;
  } else if (dist > 60) {
    // ATENÇÃO: 3 beeps/s ≈ 333ms
    intervalo = 333;
    ledAtivo = LED_YELLOW;
  } else if (dist > 25) {
    // PERIGO: 8 beeps/s ≈ 125ms
    intervalo = 125;
    ledAtivo = LED_RED;
  } else {
    // COLISÃO: buzzer e LED contínuos
    digitalWrite(LED_RED,    HIGH);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_GREEN,  LOW);
    digitalWrite(BUZZER_PIN, HIGH);
    return;
  }

  // Desliga LEDs que não são o ativo
  digitalWrite(LED_RED,    ledAtivo == LED_RED    ? HIGH : LOW);
  digitalWrite(LED_YELLOW, ledAtivo == LED_YELLOW ? HIGH : LOW);
  digitalWrite(LED_GREEN,  ledAtivo == LED_GREEN  ? HIGH : LOW);

  // Toggle do buzzer no intervalo
  unsigned long agora = millis();
  if (agora - ultimoBeep >= intervalo / 2) {
    ultimoBeep = agora;
    buzzerOn = !buzzerOn;
    digitalWrite(BUZZER_PIN, buzzerOn ? HIGH : LOW);
  }
}

// -----------------------------------------------
// Setup
// -----------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_RED,    OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN,  OUTPUT);
  desligarTudo();

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

  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(1, address[0]);

  printf_begin();
  radio.printPrettyDetails();

  Serial.println(F("=== ReDuino Atuador pronto ==="));
}

// -----------------------------------------------
// Loop
// -----------------------------------------------
void loop() {
  // Tenta receber um pacote do sensor
  escutar();

  // Se faz muito tempo sem receber, assume posição segura
  if (millis() - ultimaRecepcao > TIMEOUT_SEM_DADO && ultimaRecepcao != 0) {
    distanciaAtual = 200;
  }

  // Atualiza buzzer e LEDs
  atualizarAlarme(distanciaAtual);
}
