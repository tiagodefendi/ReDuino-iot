#include "nodes.h"

// ── Constantes internas ───────────────────────────────────────────────────────
#define CSMA_MAX_TRIES    8
#define CSMA_BACKOFF_MS   2
#define TIMEOUT_CTS       60    // ms
#define TIMEOUT_ACK       60    // ms
#define TIMEOUT_CMD_ACK   80    // ms
#define MACA_MAX_RETRIES  3

static uint8_t _nodeId = 0;

// ── Helpers internos ──────────────────────────────────────────────────────────

static void buildPkt(byte* pkt, uint8_t orig, uint8_t dest,
                     uint8_t type, uint8_t seq, uint8_t data) {
    pkt[PKT_ORIG] = orig;
    pkt[PKT_DEST] = dest;
    pkt[PKT_TYPE] = type;
    pkt[PKT_SEQ]  = seq;
    pkt[PKT_DATA] = data;
}

// Aguarda pacote do tipo esperado dentro do timeout. Filtra por destino.
static bool waitForType(RF24& radio, byte* pkt, uint8_t expectedType,
                        uint8_t expectedDest, uint16_t timeoutMs) {
    radio.startListening();
    unsigned long deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        if (radio.available()) {
            radio.read(pkt, PKT_SIZE);
            if (pkt[PKT_TYPE] == expectedType &&
                (pkt[PKT_DEST] == expectedDest || pkt[PKT_DEST] == ID_BCAST)) {
                return true;
            }
        }
    }
    return false;
}

static void radioWrite(RF24& radio, byte* pkt) {
    radio.stopListening();
    radio.write(pkt, PKT_SIZE);
}

// ── API pública ───────────────────────────────────────────────────────────────

void nodesInit(RF24& radio, uint8_t nodeId) {
    _nodeId = nodeId;
    radio.setPALevel(RF_PA_LEVEL);
    radio.setChannel(RF_CHANNEL);
    radio.setPayloadSize(PKT_SIZE);
    radio.setAutoAck(false);
    radio.setCRCLength(RF24_CRC_DISABLED);
    radio.setDataRate(RF_DATARATE);
    radio.openWritingPipe(RF_ADDRESS);
    radio.openReadingPipe(1, RF_ADDRESS);
    radio.startListening();
}

bool csmaChannelFree(RF24& radio) {
    radio.startListening();
    delayMicroseconds(200);
    bool busy = radio.testRPD();
    radio.stopListening();
    return !busy;
}

bool macaSend(RF24& radio, uint8_t origin, uint8_t dest, uint8_t seq, uint8_t data) {
    byte pkt[PKT_SIZE];

    for (uint8_t attempt = 0; attempt < MACA_MAX_RETRIES; attempt++) {

        // ── Etapa 0: CSMA — espera canal livre ────────────────────────────────
        for (uint8_t i = 0; i < CSMA_MAX_TRIES; i++) {
            if (csmaChannelFree(radio)) break;
            // backoff exponencial: janela dobra a cada tentativa
            delay(random(1 << i) * CSMA_BACKOFF_MS + 1);
            if (i == CSMA_MAX_TRIES - 1) return false;
        }

        // ── Etapa 1: RTS ──────────────────────────────────────────────────────
        // O campo DATA do RTS carrega o destino final para que o gateway
        // possa incluir essa informação no CTS (broadcast), fazendo outros
        // nós silenciarem durante a troca.
        buildPkt(pkt, origin, ID_GATEWAY, MSG_RTS, seq, dest);
        radioWrite(radio, pkt);

        // ── Etapa 2: aguarda CTS ──────────────────────────────────────────────
        if (!waitForType(radio, pkt, MSG_CTS, origin, TIMEOUT_CTS)) continue;

        // ── Etapa 3: DATA ─────────────────────────────────────────────────────
        buildPkt(pkt, origin, dest, MSG_DATA, seq, data);
        radioWrite(radio, pkt);

        // ── Etapa 4: aguarda ACK ──────────────────────────────────────────────
        if (!waitForType(radio, pkt, MSG_ACK, origin, TIMEOUT_ACK)) continue;

        radio.startListening();
        return true;
    }

    radio.startListening();
    return false;
}

bool macaReceive(RF24& radio, byte* pkt, uint16_t timeoutMs) {
    radio.startListening();
    unsigned long deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        if (radio.available()) {
            radio.read(pkt, PKT_SIZE);
            if (pkt[PKT_DEST] == _nodeId || pkt[PKT_DEST] == ID_BCAST) {
                return true;
            }
        }
    }
    return false;
}

bool gatewaySendCmd(RF24& radio, uint8_t dest, uint8_t seq, uint8_t data) {
    byte pkt[PKT_SIZE];

    // CSMA antes de transmitir mesmo para comandos do gateway
    for (uint8_t i = 0; i < CSMA_MAX_TRIES; i++) {
        if (csmaChannelFree(radio)) break;
        delay(random(1 << i) * CSMA_BACKOFF_MS + 1);
        if (i == CSMA_MAX_TRIES - 1) return false;
    }

    buildPkt(pkt, ID_GATEWAY, dest, MSG_CMD, seq, data);
    radioWrite(radio, pkt);

    return waitForType(radio, pkt, MSG_ACK, ID_GATEWAY, TIMEOUT_CMD_ACK);
}
