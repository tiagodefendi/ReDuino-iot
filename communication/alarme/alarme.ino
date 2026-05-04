// DEPENDÊNCIA: copie nodes.h e nodes.cpp para esta pasta antes de compilar.

#include <SPI.h>
#include "RF24.h"
#include "nodes.h"

#define CE_PIN  7
#define CSN_PIN 8

RF24 radio(CE_PIN, CSN_PIN);

static bool ativo = true;

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    if (!radio.begin()) {
        Serial.println(F("radio nao encontrado"));
        while (1) {}
    }

    nodesInit(radio, ID_ALARM);
    Serial.println(F("Alarme pronto"));
}

void loop() {
    byte pkt[PKT_SIZE];

    // macaReceive com timeout curto para não travar o loop
    if (!macaReceive(radio, pkt, 20)) return;

    switch (pkt[PKT_TYPE]) {

        case MSG_CMD: {
            // Comando do backend via gateway (liga/desliga)
            ativo = (pkt[PKT_DATA] != 0);

            // ACK manual para o gateway
            byte ack[PKT_SIZE] = { ID_ALARM, ID_GATEWAY, MSG_ACK, pkt[PKT_SEQ], 0 };
            radio.stopListening();
            radio.write(ack, PKT_SIZE);
            radio.startListening();

            Serial.print(F("CMD recebido: ativo="));
            Serial.println(ativo);
            break;
        }

        case MSG_DATA: {
            // Dado de distância enviado pelo sensor via gateway
            uint8_t distancia = pkt[PKT_DATA];
            Serial.print(F("Distancia recebida: "));
            Serial.print(distancia);
            Serial.println(F("cm"));

            // Lógica de alerta — substitua pelo controle real do LED/buzzer
            if (ativo && distancia < 30) {
                Serial.println(F("!! ALERTA DISTANCIA CRITICA !!"));
                // acionar LED / buzzer aqui
            }
            break;
        }

        default:
            break;
    }
}
