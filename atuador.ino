/* atuador - recebe mensagens do gateway com protocolo MACA e atualiza feedback visual e sonoro */

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

// ENDEREÇOS: 
uint64_t pipeAtuador       = 0x5050505050LL; // Onde o atuador escuta (Pipe 1)
uint64_t addressGatewayRX  = 0x4040404040LL; // Onde o Gateway escuta (Para onde o atuador vai responder)

uint8_t origem   = 60;
uint8_t gateway  = 30;
uint8_t canalAtual = 76;

RF24 radio(CE_PIN, CSN_PIN);
uint8_t payload[MAX_SIZE];
uint8_t buffer[MAX_SIZE];
int lastDist = 200;

// ── Funções de Alerta (Feedback) ──────────────────────────────────────
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
  } else if (dist > 45) {
    intervaloLED  = 1000; intervaloBuzz = 1000; freq = 700;
  } else if (dist > 30) {
    intervaloLED  = 333;  intervaloBuzz = 333;  freq = 700;
  } else if (dist > 15) {
    intervaloLED  = 150;  intervaloBuzz = 150;  freq = 700;
  } else {
    intervaloLED  = 100;  intervaloBuzz = 100;  freq = 700;
  }

  if (agora - lastBlink >= intervaloLED) {
    lastBlink = agora;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }

  if (agora - lastBeep >= intervaloBuzz) {
    lastBeep = agora;
    tone(BUZZER_PIN, freq, intervaloLED / 2);
  }
}

// ── Protocolo MACA e CSMA ─────────────────────────────────────────────
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
    if (!radio.testCarrier()) { // CSMA: Evita colisão se houver ruído/outra equipe transmitindo
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

// Executa o controle de fluxo MACA completo do lado do receptor
void escutar_ciclo_maca() {
  // Etapa 1: Aguarda pacientemente o RTS do Gateway
  if (recebe(RTS, gateway) != 0) return;

  // Etapa 2: Gateway quer falar conosco, responde CTS
  envia(gateway, CTS, (uint8_t*)"", 4);

  // Etapa 3: Aguarda o payload com o dado (DATA)
  if (recebe(DATA, gateway) != 0) return;

  // Se chegou aqui, recebemos o dado com sucesso. Extrai a string:
  char distStr[8] = {0};
  int payloadLen = buffer[3] - 5;
  if (payloadLen > 0 && payloadLen < 8) {
    for (int i = 0; i < payloadLen; i++)
      distStr[i] = (char)buffer[i + 4];
    distStr[payloadLen] = '\0';
    lastDist = atoi(distStr);
  }

  // Etapa 4: Envia ACK confirmando o recebimento de tudo para o Gateway finalizar o ciclo dele
  envia(gateway, ACK, (uint8_t*)"", 4);

  // Print de Debug Serial
  Serial.print(F("MACA OK -> DIST: "));
  Serial.println(lastDist);
}

// Garante que o Atuador mude de canal junto com o Gateway e o Sensor
void verificaComandoSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.startsWith("CH:")) {
    uint8_t ch = cmd.substring(3).toInt();
    if (ch <= 125) {
      canalAtual = ch;
      radio.setChannel(canalAtual);
      Serial.print(F("Atuador mudou para Canal: ")); Serial.println(canalAtual);
    }
  }
}

// ── Setup ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(19200);

  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(LED_PIN,    OUTPUT); digitalWrite(LED_PIN,    LOW);

  if (!radio.begin()) { while (1) {} }

  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(canalAtual);
  radio.setDataRate(RF24_1MBPS);
  radio.setPayloadSize(MAX_SIZE);
  radio.setAutoAck(false);
  radio.setCRCLength(RF24_CRC_16);
  
  // Configuração dos pipes bidirecionais
  radio.openReadingPipe(1, pipeAtuador);      // Escuta comandos direcionados a ele
  radio.openWritingPipe(addressGatewayRX);    // Escreve respostas voltadas ao Gateway
  radio.startListening();

  Serial.println(F("Atuador pronto"));
}

// ── Loop ──────────────────────────────────────────────────────────────
void loop() {
  verificaComandoSerial(); // Escuta o comando do back/monitor para mudar de canal
  escutar_ciclo_maca();    // Executa a máquina de estados RTS->CTS->DATA->ACK
  atualizarFeedback(lastDist);
}