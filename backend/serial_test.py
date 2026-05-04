#!/usr/bin/env python3
"""
Simulador de backend para testar a comunicação serial com o gateway.

Uso:
    python serial_test.py                  # detecta porta automaticamente
    python serial_test.py /dev/ttyUSB0    # porta específica
    python serial_test.py COM3            # Windows

Dependência: pip install pyserial
"""

import sys
import serial
import serial.tools.list_ports
import threading
import time

BAUD = 115200

# ── Labels para exibição ───────────────────────────────────────────────────────
NODE_NAMES = {0: "Backend", 1: "Gateway", 2: "Sensor", 3: "Alarme", 255: "Broadcast"}
MSG_NAMES  = {1: "RTS", 2: "CTS", 3: "DATA", 4: "ACK", 5: "NACK", 6: "CMD"}

def node(n):
    return NODE_NAMES.get(int(n), f"Node{n}")

def msg_type(t):
    return MSG_NAMES.get(int(t), f"TYPE{t}")


# ── Detecção automática de porta ───────────────────────────────────────────────
def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    candidates = [
        p for p in ports
        if any(k in (p.description + p.manufacturer if p.manufacturer else p.description).lower()
               for k in ("arduino", "ch340", "cp210", "ftdi", "usb serial"))
    ]
    if candidates:
        return candidates[0].device
    if ports:
        print("Portas disponíveis:")
        for i, p in enumerate(ports):
            print(f"  [{i}] {p.device} — {p.description}")
        choice = input("Escolha o número da porta: ").strip()
        return ports[int(choice)].device
    return None


# ── Thread de leitura serial ───────────────────────────────────────────────────
def reader_thread(ser, stop_event):
    buffer = ""
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting).decode("ascii", errors="replace")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        parse_and_print(line)
            else:
                time.sleep(0.01)
        except serial.SerialException:
            print("\n[ERRO] Conexão serial perdida.")
            stop_event.set()


def parse_and_print(line):
    # Tenta interpretar como CSV do gateway: O,D,T,S,V
    parts = line.split(",")
    if len(parts) == 5:
        orig, dest, tipo, seq, dado = parts
        print(
            f"\n  [RX] {node(orig)} → {node(dest)} | "
            f"{msg_type(tipo)} seq={seq} dado={dado}"
        )
    else:
        # Linha de debug do Arduino (printPrettyDetails, etc.)
        print(f"\n  [DBG] {line}")

    print("CMD> ", end="", flush=True)


# ── Ajuda interativa ───────────────────────────────────────────────────────────
HELP = """
Comandos disponíveis:
  <dest>,<valor>   Envia CMD ao gateway  ex: 2,1  (liga sensor)
                                         ex: 3,0  (desliga alarme)
  nomes            Lista IDs dos nós
  sair / exit      Encerra o script
  ?                Mostra esta ajuda
"""

NODE_LIST = """
IDs dos nós:
  0 = Backend   1 = Gateway
  2 = Sensor    3 = Alarme
"""


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_arduino_port()
    if not port:
        print("Nenhuma porta serial encontrada. Informe a porta como argumento.")
        sys.exit(1)

    print(f"Conectando em {port} @ {BAUD} baud...")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"Erro ao abrir porta: {e}")
        sys.exit(1)

    time.sleep(2)   # aguarda Arduino resetar após conexão serial
    print(f"Conectado. Digite ? para ajuda.\n")

    stop_event = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    t.start()

    seq = 0
    try:
        while not stop_event.is_set():
            try:
                cmd = input("CMD> ").strip()
            except EOFError:
                break

            if not cmd:
                continue
            if cmd in ("sair", "exit", "q"):
                break
            if cmd == "?":
                print(HELP)
                continue
            if cmd == "nomes":
                print(NODE_LIST)
                continue

            # Valida formato D,V
            parts = cmd.split(",")
            if len(parts) == 2:
                try:
                    dest = int(parts[0])
                    val  = int(parts[1])
                    payload = f"{dest},{val}\n"
                    ser.write(payload.encode("ascii"))
                    print(f"  [TX] CMD → {node(dest)} valor={val}")
                except ValueError:
                    print("  Formato inválido. Use: <dest>,<valor>  ex: 2,1")
            else:
                print("  Formato inválido. Use: <dest>,<valor>  ex: 2,1")

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        ser.close()
        print("\nConexão encerrada.")


if __name__ == "__main__":
    main()
