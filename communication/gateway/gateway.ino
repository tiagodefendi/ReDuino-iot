#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "nodes.h"

#define CE_PIN  7
#define CSN_PIN 8

RF24 radio(CE_PIN, CSN_PIN);

static uint8_t seqTx = 0;   // contador de sequência para transmissões do gateway

// ── Serial ────────────────────────────────────────────────────────────────────
// Formato recebido do backend: "D,V\n"   (destino, valor)
// Formato enviado ao backend:  "O,D,T,S,V\n"  (origem, destino, tipo, seq, dado)

static void serialForward(byte* pkt) {
    Serial.print(pkt[PKT_ORIG]); Serial.print(',');
    Serial.print(pkt[PKT_DEST]); Serial.print(',');
    Serial.print(pkt[PKT_TYPE]); Serial.print(',');
    Serial.print(pkt[PKT_SEQ]);  Serial.print(',');
    Serial.println(pkt[PKT_DATA]);
}

static void handleSerialCmd() {
    static char buf[16];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            buf[idx] = '\0';
            idx = 0;
            // Espera formato "D,V"
            char* sep = strchr(buf, ',');
            if (!sep) return;
            *sep = '\0';
            uint8_t dest = (uint8_t)atoi(buf);
            uint8_t val  = (uint8_t)atoi(sep + 1);
            gatewaySendCmd(radio, dest, seqTx++, val);
        } else if (idx < sizeof(buf) - 1) {
            buf[idx++] = c;
        }
    }
}

// ── Tratamento de pacotes RF recebidos ────────────────────────────────────────

static void handleRTS(byte* pkt) {
    // Responde CTS para o nó que pediu (PKT_ORIG).
    // O campo DATA do CTS repete o destino final para que outros nós
    // que ouvirem o CTS saibam que devem silenciar.
    byte cts[PKT_SIZE];
    cts[PKT_ORIG] = ID_GATEWAY;
    cts[PKT_DEST] = pkt[PKT_ORIG];
    cts[PKT_TYPE] = MSG_CTS;
    cts[PKT_SEQ]  = pkt[PKT_SEQ];
    cts[PKT_DATA] = pkt[PKT_DATA];   // destino final (informativo)

    radio.stopListening();
    radio.write(cts, PKT_SIZE);
    radio.startListening();
}

static void handleData(byte* pkt) {
    // Envia ACK de volta ao remetente
    byte ack[PKT_SIZE];
    ack[PKT_ORIG] = ID_GATEWAY;
    ack[PKT_DEST] = pkt[PKT_ORIG];
    ack[PKT_TYPE] = MSG_ACK;
    ack[PKT_SEQ]  = pkt[PKT_SEQ];
    ack[PKT_DATA] = 0;

    radio.stopListening();
    radio.write(ack, PKT_SIZE);
    radio.startListening();

    // Roteia o pacote
    if (pkt[PKT_DEST] == ID_BACKEND || pkt[PKT_DEST] == ID_GATEWAY) {
        // Entrega ao backend via serial
        serialForward(pkt);
    } else {
        // Retransmite para outro nó (roteamento nó→nó pelo gateway)
        gatewaySendCmd(radio, pkt[PKT_DEST], seqTx++, pkt[PKT_DATA]);
    }
}

static void handleAck(byte* pkt) {
    // ACK de um nó para um CMD enviado pelo gateway — não requer ação adicional
    (void)pkt;
}

// ── Setup ─────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    if (!radio.begin()) {
        Serial.println(F("radio hardware is not responding!!"));
        while (1) {}
    }

    nodesInit(radio, ID_GATEWAY);

    printf_begin();
    radio.printPrettyDetails();
}

// ── Loop ──────────────────────────────────────────────────────────────────────

void loop() {
    // 1. Comandos do backend via serial
    handleSerialCmd();

    // 2. Pacotes RF
    if (radio.available()) {
        byte pkt[PKT_SIZE];
        radio.read(pkt, PKT_SIZE);

        switch (pkt[PKT_TYPE]) {
            case MSG_RTS:  handleRTS(pkt);  break;
            case MSG_DATA: handleData(pkt); break;
            case MSG_ACK:  handleAck(pkt);  break;
            default: break;
        }
    }
}
