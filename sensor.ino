/* sensor*/ 

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN   7
#define CSN_PIN  8
#define TRIG_PIN 2
#define ECHO_PIN 3

#define DATA 0
#define ACK  1
#define RTS  2
#define CTS  3

#define MAX_SIZE      32
#define TIMEOUT       500
#define MAX_RETRIES   3
#define SEND_INTERVAL 700

uint64_t address[2] = { 0x4040404040LL, 0x3030303030LL };
uint8_t  origem  = 47;
uint8_t  destino = 30;

RF24 radio(CE_PIN, CSN_PIN);
uint8_t payload[MAX_SIZE];
uint8_t buffer[MAX_SIZE];
uint8_t canalAtual = 76;

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
  Serial.println(F("CSMA timeout"));
}

int recebe(int type, int dest) {
  radio.startListening();
  unsigned long inicio = millis();
  while (millis() - inicio < TIMEOUT) {
    if (radio.available()) {
      radio.read(buffer, MAX_SIZE);
      int tamanho = buffer[3];
      if (buffer[0] != dest)   continue;
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

int envia_pacote(int dest, uint8_t* mensagem, uint8_t size) {
  if (size + 5 > MAX_SIZE) return 1;
  for (int t = 0; t < MAX_RETRIES; t++) {
    envia(dest, RTS, (uint8_t*)"", 4);
    if (recebe(CTS, dest) != 0) { delay(50*(t+1)); continue; }
    envia(dest, DATA, mensagem, size + 4);
    if (recebe(ACK, dest) != 0) { delay(50*(t+1)); continue; }
    return 0;
  }
  return 2;
}

int medirDistancia() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 38000);
  if (dur == 0) return -1;
  return constrain((int)(dur * 0.034 / 2), 0, 400);
}

// muda canal via Serial: envia "CH:XX" pelo monitor
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
  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  if (!radio.begin()) { while (1) {} }
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(canalAtual);
  radio.setDataRate(RF24_1MBPS);
  radio.setPayloadSize(MAX_SIZE);
  radio.setAutoAck(false);
  radio.setCRCLength(RF24_CRC_16);
  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(1, address[1]);
  Serial.println(F("Sensor pronto."));
}

void loop() {
  verificaComandoSerial();

  int dist = medirDistancia();
  if (dist < 0) { delay(SEND_INTERVAL); return; }

  Serial.print(F("Dist: ")); Serial.print(dist); Serial.print(F("cm -> "));

  char msg[8];
  snprintf(msg, sizeof(msg), "%d", dist);
  uint8_t msgBytes[8];
  uint8_t msgLen = strlen(msg);
  for (uint8_t i = 0; i < msgLen; i++) msgBytes[i] = (uint8_t)msg[i];

  int ret = envia_pacote(destino, msgBytes, msgLen);
  Serial.println(ret == 0 ? F("OK") : F("FALHOU"));

  delay(SEND_INTERVAL);
}