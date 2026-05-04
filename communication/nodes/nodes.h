#ifndef NODES_H
#define NODES_H

#include <Arduino.h>
#include <RF24.h>

// ── IDs dos nós ───────────────────────────────────────────────────────────────
#define ID_BACKEND  0x00
#define ID_GATEWAY  0x01
#define ID_SENSOR   0x02
#define ID_ALARM    0x03
#define ID_BCAST    0xFF

// ── Tipos de mensagem ─────────────────────────────────────────────────────────
#define MSG_RTS   0x01   // Request to Send
#define MSG_CTS   0x02   // Clear to Send
#define MSG_DATA  0x03   // Dado real
#define MSG_ACK   0x04   // Confirmação
#define MSG_NACK  0x05   // Falha
#define MSG_CMD   0x06   // Comando do backend para o nó

// ── Índices do pacote (5 bytes) ───────────────────────────────────────────────
#define PKT_ORIG  0
#define PKT_DEST  1
#define PKT_TYPE  2
#define PKT_SEQ   3
#define PKT_DATA  4
#define PKT_SIZE  5

// ── Configuração de rede ──────────────────────────────────────────────────────
#define RF_CHANNEL   37
#define RF_DATARATE  RF24_250KBPS
#define RF_PA_LEVEL  RF24_PA_MAX
static const uint64_t RF_ADDRESS = 0x3030303030LL;

// ── API pública ───────────────────────────────────────────────────────────────

// Inicializa o rádio com as configurações do protocolo.
// nodeId: ID deste nó (ID_SENSOR, ID_ALARM, etc.)
void nodesInit(RF24& radio, uint8_t nodeId);

// CSMA: retorna true se o canal está livre (sem portadora detectada).
bool csmaChannelFree(RF24& radio);

// Envia dado com protocolo MACA completo: RTS→CTS→DATA→ACK.
// dest: destino final (ID_BACKEND envia ao gateway que roteia pela serial).
// Retorna true se ACK recebido.
bool macaSend(RF24& radio, uint8_t origin, uint8_t dest, uint8_t seq, uint8_t data);

// Aguarda um pacote destinado a este nó (ou broadcast) por até timeoutMs.
// Preenche pkt[PKT_SIZE] e retorna true se recebeu.
bool macaReceive(RF24& radio, byte* pkt, uint16_t timeoutMs);

// Gateway: envia CMD direto a um nó sem handshake RTS/CTS (2 etapas: CMD→ACK).
// Usado pelo gateway para comandos do backend e roteamento nó→nó.
bool gatewaySendCmd(RF24& radio, uint8_t dest, uint8_t seq, uint8_t data);

#endif
