// DEPENDÊNCIA: copie nodes.h e nodes.cpp para esta pasta antes de compilar.

#include <SPI.h>
#include "RF24.h"
#include "nodes.h"

#define CE_PIN  7
#define CSN_PIN 8

RF24 radio(CE_PIN, CSN_PIN);

static uint8_t seq = 0;
static bool    ativo = true;   // ligado/desligado pelo backend

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    if (!radio.begin()) {
        Serial.println(F("radio nao encontrado"));
        while (1) {}
    }

    nodesInit(radio, ID_SENSOR);
    Serial.println(F("Sensor pronto"));
}

void loop() {
    // ── Recebe CMD do gateway ─────────────────────────────────────────────────
    byte pkt[PKT_SIZE];
    if (macaReceive(radio, pkt, 0)) {          // timeout 0 = não-bloqueante
        if (pkt[PKT_TYPE] == MSG_CMD) {
            ativo = (pkt[PKT_DATA] != 0);
            // Envia ACK manualmente (o gateway espera ACK após CMD)
            byte ack[PKT_SIZE] = { ID_SENSOR, ID_GATEWAY, MSG_ACK, pkt[PKT_SEQ], 0 };
            radio.stopListening();
            radio.write(ack, PKT_SIZE);
            radio.startListening();
            Serial.print(F("CMD recebido: ativo="));
            Serial.println(ativo);
        }
    }

    if (!ativo) {
        delay(100);
        return;
    }

    // ── Envia distância fake ao backend (a cada ~2 s) ─────────────────────────
    static unsigned long ultimo = 0;
    if (millis() - ultimo < 2000) return;
    ultimo = millis();

    uint8_t distancia = (uint8_t)random(10, 200);   // substitua pela leitura real

    bool ok = macaSend(radio, ID_SENSOR, ID_BACKEND, seq, distancia);
    Serial.print(F("Enviou distancia="));
    Serial.print(distancia);
    Serial.print(F("cm seq="));
    Serial.print(seq);
    Serial.print(F(" -> "));
    Serial.println(ok ? F("OK") : F("FALHOU"));
    seq++;

    // ── Envia distância também para o alarme ──────────────────────────────────
    ok = macaSend(radio, ID_SENSOR, ID_ALARM, seq, distancia);
    Serial.print(F("Enviou alarme seq="));
    Serial.print(seq);
    Serial.print(F(" -> "));
    Serial.println(ok ? F("OK") : F("FALHOU"));
    seq++;
}
