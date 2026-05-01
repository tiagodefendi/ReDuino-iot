/*
======================================================
Pisca-LED com manipulação direta de registradores AVR
======================================================
Alterna o estado de um LED conectado ao pino PC1
(pino analógico A1 no Arduino Nano) a cada 100ms,
usando registradores do ATmega328P diretamente em vez
das funções digitalWrite/pinMode do Arduino.
======================================================
*/

// Pino do LED: PC1 = bit 1 do PORTC (pino A1 no Arduino Nano)
#define LED PC1
// Verifica o Estado do LED
#define ESTADO_LED PORTC & (1<<LED)
// Liga o LED: seta o bit PC1 em PORTC (nível alto)
#define LIGAR_LED PORTC |= (1<<LED)
// Desliga o LED: limpa o bit PC1 em PORTC (nível baixo)
#define DESLIGAR_LED PORTC &= ~(1<<LED)

void setup() {
  // Configura PC1 como saída
  DDRC |= (1<<PC1);
}

void loop() {
  alternar_led();
  delay(100); // aguarda 100ms entre cada alternância
}

// Lê o estado atual do pino e inverte: se ligado desliga, se desligado liga
void alternar_led() {
  if (ESTADO_LED) { // se for um 1 ja esta ligado
    DESLIGAR_LED;
  }
  else {
    LIGAR_LED;
  }
}
