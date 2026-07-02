"""
flash_sender.py
───────────────────────────────────────────────────────────────
Envía el contenido de un archivo .h generado por hex2bsl a un
MSP430 a través de UART (115200 baud, sin paridad, 8N1).

Protocolo:
  0. Byte de inicio     → 0x06 (señal de arranque)
  1. Tamaño del firmware  → 4 bytes big-endian (uint32_t)
  2. Bytes del firmware   → N bytes en bloques de 256
  3. Espera ACK (0x79) del MSP430 al finalizar

Uso:
  python flash_sender.py
  (el script pide puerto COM y archivo de forma interactiva)

Dependencias:
  pip install pyserial
"""

import os
import re
import sys
import struct
import time
import serial
import serial.tools.list_ports

# ─── Constantes ──────────────────────────────────────────────
BAUD_RATE        = 115200
PARITY           = serial.PARITY_NONE
BLOCK_SIZE       = 1024
ACK_BYTE         = 0x79
NACK_BYTE        = 0x1F
START_BYTE       = 0x06
TIMEOUT_S        = 10          # segundos esperando ACK por bloque
HEX2BSL_DIR      = "hex2bsl"  # carpeta con los .h

# Tiempo de espera entre bytes dentro de un bloque (segundos).
# A 9600 baud un byte tarda ~1.04ms en transmitirse.
# El MSP430 necesita leerlo con polling antes de que llegue el siguiente.
# Con 3ms de separación el eUSCI nunca se desborda (UCOE).
# Puedes reducirlo hasta ~1.5ms si quieres más velocidad.
INTER_BYTE_DELAY = 0.001

# Tiempo de espera tras abrir el puerto antes de enviar el primer byte.
# El Launchpad backchannel genera pulsos en la línea al conectarse,
# y el MSP430 necesita terminar su inicialización (GPIO/UART/SPI).
STARTUP_DELAY    = 2.0        # segundos — reducir si el arranque es más rápido

# ─── Colores ANSI (funcionan en Windows 10+, Linux, macOS) ───
class C:
    RESET  = "\033[0m"
    BOLD   = "\033[1m"
    CYAN   = "\033[96m"
    GREEN  = "\033[92m"
    YELLOW = "\033[93m"
    RED    = "\033[91m"
    GRAY   = "\033[90m"
    WHITE  = "\033[97m"

def banner():
    print(f"""
{C.CYAN}{C.BOLD}╔══════════════════════════════════════════╗
║        FIRMWARE UART SENDER  v1.0        ║
║      hex2bsl → UART → MSP430 FRAM        ║
╚══════════════════════════════════════════╝{C.RESET}
""")

# ─── Selección de puerto COM ──────────────────────────────────
def select_port() -> str:
    ports = serial.tools.list_ports.comports()
    if not ports:
        print(f"{C.RED}✗ No se encontraron puertos COM disponibles.{C.RESET}")
        sys.exit(1)

    print(f"{C.BOLD}Puertos COM disponibles:{C.RESET}")
    for i, p in enumerate(ports, 1):
        desc = p.description if p.description else "sin descripción"
        print(f"  {C.CYAN}[{i}]{C.RESET} {C.WHITE}{p.device}{C.RESET}  {C.GRAY}→ {desc}{C.RESET}")

    print()
    while True:
        try:
            choice = int(input(f"{C.YELLOW}Selecciona el número de puerto: {C.RESET}"))
            if 1 <= choice <= len(ports):
                return ports[choice - 1].device
        except ValueError:
            pass
        print(f"{C.RED}  Selección inválida. Ingresa un número entre 1 y {len(ports)}.{C.RESET}")

# ─── Selección de archivo .h ──────────────────────────────────
def select_firmware_file() -> str:
    if not os.path.isdir(HEX2BSL_DIR):
        print(f"{C.RED}✗ La carpeta '{HEX2BSL_DIR}' no existe en el directorio actual.{C.RESET}")
        print(f"{C.GRAY}  Crea la carpeta y coloca tus archivos .h dentro de ella.{C.RESET}")
        sys.exit(1)

    h_files = sorted([f for f in os.listdir(HEX2BSL_DIR) if f.endswith(".h")])
    if not h_files:
        print(f"{C.RED}✗ No se encontraron archivos .h en '{HEX2BSL_DIR}/'.{C.RESET}")
        sys.exit(1)

    print(f"\n{C.BOLD}Archivos de firmware disponibles en '{HEX2BSL_DIR}/':{C.RESET}")
    for i, f in enumerate(h_files, 1):
        full_path = os.path.join(HEX2BSL_DIR, f)
        size_kb = os.path.getsize(full_path) / 1024
        print(f"  {C.CYAN}[{i}]{C.RESET} {C.WHITE}{f}{C.RESET}  {C.GRAY}({size_kb:.1f} KB){C.RESET}")

    print()
    while True:
        try:
            choice = int(input(f"{C.YELLOW}Selecciona el número de archivo: {C.RESET}"))
            if 1 <= choice <= len(h_files):
                return os.path.join(HEX2BSL_DIR, h_files[choice - 1])
        except ValueError:
            pass
        print(f"{C.RED}  Selección inválida. Ingresa un número entre 1 y {len(h_files)}.{C.RESET}")

# ─── Parser del archivo .h ────────────────────────────────────
def parse_header_file(path: str) -> tuple[int, bytes]:
    """
    Extrae FIRMWARE_PART1_SIZE (o FIRMWARE_SIZE) y el arreglo
    firmware_part1[] de un archivo .h generado por hex2bsl.
    Retorna (size: int, data: bytes).
    """
    with open(path, "r") as f:
        content = f.read()

    # Buscar el #define de tamaño (acepta FIRMWARE_PART1_SIZE o FIRMWARE_SIZE)
    size_match = re.search(
        r"#define\s+FIRMWARE_PART1_SIZE\s+(\d+)|#define\s+FIRMWARE_SIZE\s+(\d+)",
        content
    )
    if not size_match:
        print(f"{C.RED}✗ No se encontró FIRMWARE_PART1_SIZE ni FIRMWARE_SIZE en el archivo.{C.RESET}")
        sys.exit(1)

    declared_size = int(size_match.group(1) or size_match.group(2))

    # Extraer todos los valores 0xHH del arreglo
    hex_values = re.findall(r"0x([0-9A-Fa-f]{2})", content)
    firmware_bytes = bytes(int(v, 16) for v in hex_values)

    if len(firmware_bytes) != declared_size:
        print(f"{C.YELLOW}⚠ Advertencia: el tamaño declarado ({declared_size}) "
              f"no coincide con los bytes extraídos ({len(firmware_bytes)}).{C.RESET}")
        print(f"{C.YELLOW}  Se usará el tamaño declarado en el #define.{C.RESET}")

    return declared_size, firmware_bytes[:declared_size]

# ─── Barra de progreso ────────────────────────────────────────
def progress_bar(sent: int, total: int, width: int = 40) -> str:
    pct   = sent / total
    filled = int(width * pct)
    bar   = "█" * filled + "░" * (width - filled)
    kb_sent  = sent / 1024
    kb_total = total / 1024
    return f"{C.CYAN}[{bar}]{C.RESET} {pct*100:5.1f}%  {kb_sent:.1f}/{kb_total:.1f} KB"

# ─── Envío de firmware ────────────────────────────────────────
def send_firmware(port: str, fw_size: int, fw_data: bytes):
    print(f"\n{C.BOLD}Abriendo {port} a {BAUD_RATE} baud...{C.RESET}")

    try:
        # Abrir sin activar DTR ni RTS — el Launchpad backchannel
        # genera pulsos espurios en TX cuando estos se activan,
        # que el MSP430 puede interpretar como bytes antes del 0x06.
        ser = serial.Serial()
        ser.port     = port
        ser.baudrate = BAUD_RATE
        ser.bytesize = serial.EIGHTBITS
        ser.parity   = PARITY
        ser.stopbits = serial.STOPBITS_ONE
        ser.timeout  = TIMEOUT_S
        ser.dtr      = False   # evita pulso DTR al abrir
        ser.rts      = False   # evita pulso RTS al abrir
        ser.open()
    except serial.SerialException as e:
        print(f"{C.RED}✗ No se pudo abrir el puerto: {e}{C.RESET}")
        sys.exit(1)

    print(f"{C.GREEN}✔ Puerto abierto correctamente.{C.RESET}")

    # Esperar a que la línea se estabilice y el MSP430 termine
    # su inicialización (GPIO, UART, SPI) antes de enviar el 0x06.
    print(f"  {C.GRAY}Esperando estabilización ({STARTUP_DELAY}s)...{C.RESET}", end="", flush=True)
    time.sleep(STARTUP_DELAY)
    ser.reset_input_buffer()   # descartar cualquier basura acumulada
    print(f"\r  {C.GREEN}✔ Línea estable, buffer limpio.         {C.RESET}")

    print(f"\n{C.BOLD}Enviando firmware...{C.RESET}")
    print(f"  Tamaño declarado : {fw_size} bytes")
    print(f"  Bytes a enviar   : {len(fw_data)} bytes\n")

    try:
        # ── 0. Enviar byte de inicio 0x06 ────────────────────
        ser.write(bytes([START_BYTE]))
        print(f"  {C.GRAY}Byte de inicio enviado: 0x{START_BYTE:02X}{C.RESET}")
        time.sleep(INTER_BYTE_DELAY)

        # ── 1. Enviar tamaño como uint32 big-endian (byte a byte) ──
        size_bytes = struct.pack(">I", fw_size)
        for b in size_bytes:
            ser.write(bytes([b]))
            time.sleep(INTER_BYTE_DELAY)
        print(f"  {C.GRAY}Tamaño enviado: {size_bytes.hex(' ').upper()}{C.RESET}")

        # ── 2. Enviar bloques con handshake ACK ──────────────
        #
        # Se envía BYTE A BYTE con un pequeño delay entre cada uno.
        # Esto evita el RX overflow (flag UCOE) en el eUSCI del MSP430,
        # que ocurre cuando el buffer de 1 byte se desborda porque el
        # firmware tarda más en leer que el PC en transmitir.
        #
        bytes_sent   = 0
        block_num    = 0
        total_blocks = (fw_size + BLOCK_SIZE - 1) // BLOCK_SIZE

        while bytes_sent < fw_size:
            block_num += 1
            chunk = fw_data[bytes_sent : bytes_sent + BLOCK_SIZE]

            # Enviar byte a byte con inter-byte delay
            for byte in chunk:
                ser.write(bytes([byte]))
                time.sleep(INTER_BYTE_DELAY)

            bytes_sent += len(chunk)

            # Esperar ACK del MSP430 para este bloque
            ser.timeout = TIMEOUT_S
            response = ser.read(1)

            if not response:
                print(f"\n{C.RED}✗ Timeout esperando ACK del bloque {block_num}/{total_blocks}.{C.RESET}")
                sys.exit(1)
            elif response[0] == NACK_BYTE:
                print(f"\n{C.RED}✗ NACK en bloque {block_num}/{total_blocks} — el MSP430 reportó error.{C.RESET}")
                sys.exit(1)
            elif response[0] != ACK_BYTE:
                print(f"\n{C.YELLOW}⚠ Respuesta inesperada en bloque {block_num}: 0x{response[0]:02X}{C.RESET}")
                sys.exit(1)

            # Actualizar barra de progreso
            print(f"\r  {progress_bar(bytes_sent, fw_size)}", end="", flush=True)

        print()  # salto de línea tras la barra

        # ── 3. Esperar ACK final ──────────────────────────────
        print(f"\n{C.BOLD}Esperando confirmación final del MSP430...{C.RESET}")
        ser.timeout = TIMEOUT_S
        response = ser.read(1)

        if not response:
            print(f"{C.RED}✗ Timeout: no se recibió ACK final del MSP430.{C.RESET}")
            sys.exit(1)
        elif response[0] == ACK_BYTE:
            print(f"{C.GREEN}{C.BOLD}✔ ACK final recibido — Firmware cargado exitosamente.{C.RESET}")
        elif response[0] == NACK_BYTE:
            print(f"{C.RED}✗ NACK final — el MSP430 reportó un error.{C.RESET}")
            sys.exit(1)
        else:
            print(f"{C.YELLOW}⚠ Respuesta inesperada: 0x{response[0]:02X}{C.RESET}")
            sys.exit(1)

    except serial.SerialException as e:
        print(f"\n{C.RED}✗ Error de comunicación: {e}{C.RESET}")
        sys.exit(1)
    finally:
        ser.close()
        print(f"{C.GRAY}Puerto {port} cerrado.{C.RESET}")

# ─── Main ─────────────────────────────────────────────────────
def main():
    # Habilitar colores ANSI en Windows
    if sys.platform == "win32":
        os.system("color")

    banner()

    # 1. Seleccionar puerto COM
    port = select_port()
    print(f"\n{C.GREEN}✔ Puerto seleccionado: {C.WHITE}{C.BOLD}{port}{C.RESET}")

    # 2. Seleccionar archivo de firmware
    fw_path = select_firmware_file()
    fw_name = os.path.basename(fw_path)
    print(f"\n{C.GREEN}✔ Archivo seleccionado: {C.WHITE}{C.BOLD}{fw_name}{C.RESET}")

    # 3. Parsear el archivo .h
    print(f"\n{C.BOLD}Analizando archivo...{C.RESET}")
    fw_size, fw_data = parse_header_file(fw_path)
    print(f"  {C.GREEN}✔{C.RESET} Tamaño del firmware : {C.WHITE}{fw_size}{C.RESET} bytes "
          f"({fw_size/1024:.1f} KB)")
    print(f"  {C.GREEN}✔{C.RESET} Bytes extraídos      : {C.WHITE}{len(fw_data)}{C.RESET}")

    # 4. Confirmar antes de enviar
    print(f"\n{C.YELLOW}¿Confirmar envío de '{fw_name}' por {port}? [s/N]: {C.RESET}", end="")
    confirm = input().strip().lower()
    if confirm not in ("s", "si", "sí", "y", "yes"):
        print(f"{C.GRAY}Operación cancelada.{C.RESET}")
        sys.exit(0)

    # 5. Enviar
    send_firmware(port, fw_size, fw_data)

if __name__ == "__main__":
    main()