#!/usr/bin/env python3
"""
msp430_bsl_programmer.py
────────────────────────
Programador BSL para MSP430FR5969 vía UART (backchannel).
No requiere manejo de pines TEST/RST; asume que el target ya
está en modo BSL antes de ejecutar este script.

Protocolo: UART BSL Frame (SLAU550)
  [0x80] [LEN_L] [LEN_H] [CMD] [DATA...] [CK_L] [CK_H]
  Checksum: CRC-16 CCITT-FALSE (poly=0x1021, init=0xFFFF)

Uso:
  python msp430_bsl_programmer.py                      # selección interactiva de puerto y archivo
  python msp430_bsl_programmer.py -p COM3 -f fw.txt    # modo directo
  python msp430_bsl_programmer.py --list-ports          # listar puertos disponibles
"""

import argparse
import glob
import os
import sys
import time
import struct
import serial
import serial.tools.list_ports

# ══════════════════════════════════════════════
#  CONSTANTES BSL
# ══════════════════════════════════════════════

BSL_HEADER          = 0x80

# Comandos (tabla 2-2, SLAU550)
CMD_RX_DATA_BLOCK   = 0x10   # Escribir bloque de memoria
CMD_RX_PASSWORD     = 0x11   # Enviar password
CMD_ERASE_SEGMENT   = 0x12   # Borrar segmento
CMD_LOCK_UNLOCK_INFO= 0x13   # Lock/Unlock INFO
CMD_RESERVED        = 0x14
CMD_RX_DATA_BLOCK_FAST = 0x1B
CMD_GET_ID          = 0x19
CMD_TX_DATA_BLOCK   = 0x18
CMD_TX_BSL_VERSION  = 0x19
CMD_TX_BUFFER_SIZE  = 0x1A
CMD_CHANGE_BAUD     = 0x52
CMD_CRC_CHECK       = 0x16
CMD_LOAD_PC         = 0x17

# Respuestas ACK del BSL
BSL_ACK             = 0x00
BSL_HEADER_INCORRECT= 0x51
BSL_CHECKSUM_INCORRECT = 0x52
BSL_PACKET_SIZE_ZERO= 0x53
BSL_PACKET_SIZE_TOO_BIG = 0x54
BSL_UNKNOWN_ERROR   = 0x55
BSL_UNKNOWN_BAUD    = 0x56

ACK_MESSAGES = {
    0x00: "ACK OK",
    0x51: "Error: Header incorrecto",
    0x52: "Error: Checksum incorrecto",
    0x53: "Error: Tamaño de paquete cero",
    0x54: "Error: Paquete demasiado grande",
    0x55: "Error: Desconocido",
    0x56: "Error: Baudrate desconocido",
}

# Tamaño máximo de datos por comando RX_DATA_BLOCK (SLAU550, tabla 2-3)
MAX_BLOCK_SIZE = 128

# Password por defecto (flash virgen = 0xFF x 32)
DEFAULT_PASSWORD = bytes([0xFF] * 32)

# ══════════════════════════════════════════════
#  CONSTANTES MISION BOSS
# ══════════════════════════════════════════════

PC_SUBSYSTEM_ID = 0x01 # ID del subsistema "Laptop/PC"
STOP_BYTE = 0x0A

# Comandos personalizados para interacción con Mission Boss (no es del BSL)
CMD_MB_SET_BYPASS_UART_MODE = 0xA0
CMD_MB_REPROGRAM_FRAM = 0xDC
CMD_MD_REPROGRAM_STM32 = 0xDE

# Respuestas ACK del Mission Boss
MB_ACK = 0x79

# ══════════════════════════════════════════════
#  CRC-16 CCITT-FALSE
# ══════════════════════════════════════════════

def crc16_ccitt_false(data: bytes) -> int:
    """Calcula CRC-16 CCITT-FALSE (poly=0x1021, init=0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

# ══════════════════════════════════════════════
#  PARSER TI-TXT
# ══════════════════════════════════════════════

def parse_ti_txt(filepath: str) -> list[tuple[int, bytes]]:
    """
    Lee un archivo TI-TXT (.txt) y devuelve una lista de
    (address, data_bytes) ordenada por dirección.

    Formato TI-TXT:
      @AAAA          → dirección de inicio (hex)
      HH HH HH ...   → bytes en hex separados por espacios
      q              → fin de archivo
    """
    blocks = []
    current_addr = None
    current_data = bytearray()

    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            if line.startswith("@"):
                # Guardar bloque anterior si existe
                if current_addr is not None and current_data:
                    blocks.append((current_addr, bytes(current_data)))
                # Nueva dirección base
                current_addr = int(line[1:], 16)
                current_data = bytearray()

            elif line.lower() == "q":
                # Fin de archivo
                if current_addr is not None and current_data:
                    blocks.append((current_addr, bytes(current_data)))
                break

            else:
                # Línea de datos hex
                hex_values = line.split()
                for hv in hex_values:
                    current_data.append(int(hv, 16))

    # Por si el archivo termina sin 'q'
    if current_addr is not None and current_data:
        blocks.append((current_addr, bytes(current_data)))

    return blocks

# ══════════════════════════════════════════════
#  CLASE PRINCIPAL BSL
# ══════════════════════════════════════════════

class MSP430_BSL:
    """Programador BSL para MSP430 vía puerto serie."""

    def __init__(self, port: str, baudrate: int = 9600, timeout: float = 5.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    def open(self):
        """Abre la conexión UART."""
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout
        )
        print(f"[UART] Puerto {self.port} abierto a {self.baudrate} baud, paridad EVEN")

    def close(self):
        """Cierra la conexión UART."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[UART] Puerto cerrado")

    # ─────────────────────────────────────────
    #  FRAME BSL
    # ─────────────────────────────────────────

    def _build_frame(self, cmd: int, data: bytes) -> bytes:
        """
        Construye el frame BSL:
          [0x80] [LEN_L] [LEN_H] [CMD] [DATA...] [CK_L] [CK_H]
        LEN = 1 (CMD) + len(DATA)
        CRC cubre CMD + DATA
        """
        payload = bytes([cmd]) + data
        length = len(payload)
        crc = crc16_ccitt_false(payload)

        frame = bytes([
            BSL_HEADER,
            length & 0xFF,
            (length >> 8) & 0xFF,
        ]) + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

        return frame

    def _send_frame(self, cmd: int, data: bytes, expect_response: bool = True) -> bytes | None:
        """Envía un frame BSL y opcionalmente espera respuesta."""
        frame = self._build_frame(cmd, data)
        self.ser.write(frame)
        self.ser.flush()
        time.sleep(0.4)  # pequeño delay tras envío

        if expect_response:
            return self._read_response()
        return None

    def _read_response(self) -> bytes:
        """Lee la respuesta del BSL (ACK byte o frame completo)."""
        response = self.ser.read(256)
        return response

    def _check_ack(self, response: bytes, cmd_name: str) -> bool:
        """Verifica el byte ACK en la respuesta."""
        if not response:
            print(f"  [{cmd_name}] ✗ Sin respuesta (timeout)")
            return False

        ack = response[0]
        msg = ACK_MESSAGES.get(ack, f"Respuesta desconocida: 0x{ack:02X}")

        if ack == BSL_ACK:
            print(f"  [{cmd_name}] ✓ {msg}")
            return True
        else:
            print(f"  [{cmd_name}] ✗ {msg} (raw: {response.hex()})")
            return False

    # ─────────────────────────────────────────
    #  COMANDOS BSL PÚBLICOS
    # ─────────────────────────────────────────

    def send_password(self, password: bytes = DEFAULT_PASSWORD) -> bool:
        """Envía la password de acceso BSL (CMD 0x11)."""
        print("[BSL] Enviando password...")
        if len(password) != 32:
            raise ValueError("La password debe ser exactamente 32 bytes")
        resp = self._send_frame(CMD_RX_PASSWORD, password)
        return self._check_ack(resp, "RX_PASSWORD")

    def mass_erase(self) -> bool:
        """
        Fuerza mass-erase enviando password incorrecta (todos 0x00).
        El BSL realiza mass-erase automáticamente ante password errónea.
        """
        print("[BSL] Enviando password incorrecta → Mass Erase...")
        wrong_password = bytes([0x00] * 32)
        resp = self._send_frame(CMD_RX_PASSWORD, wrong_password)
        # Mass-erase puede no devolver ACK limpio; se espera igual
        time.sleep(0.1)
        return True  # el BSL continúa después del erase

    def write_block(self, address: int, data: bytes) -> bool:
        """Escribe un bloque de datos en la dirección especificada (CMD 0x10)."""
        addr_bytes = bytes([
            address & 0xFF,
            (address >> 8) & 0xFF,
            (address >> 16) & 0xFF
        ])
        payload = addr_bytes + data
        resp = self._send_frame(CMD_RX_DATA_BLOCK, payload)
        return self._check_ack(resp, f"RX_DATA_BLOCK @0x{address:05X}")

    def crc_check(self, address: int, length: int) -> bool:
        """Verifica CRC de una región de memoria (CMD 0x16)."""
        print(f"[BSL] CRC Check @0x{address:05X}, len={length}...")
        payload = bytes([
            address & 0xFF,
            (address >> 8) & 0xFF,
            (address >> 16) & 0xFF,
            length & 0xFF,
            (length >> 8) & 0xFF
        ])
        resp = self._send_frame(CMD_CRC_CHECK, payload)
        return self._check_ack(resp, "CRC_CHECK")

    def load_pc(self, address: int) -> bool:
        """Ejecuta el programa saltando a la dirección dada (CMD 0x17)."""
        print(f"[BSL] Load PC → 0x{address:05X}...")
        payload = bytes([
            address & 0xFF,
            (address >> 8) & 0xFF,
            (address >> 16) & 0xFF
        ])
        resp = self._send_frame(CMD_LOAD_PC, payload, expect_response=False)
        print("  [LOAD_PC] ✓ Comando enviado — el target debería arrancar")
        return True

    # ─────────────────────────────────────────
    #  COMANDOS DE INTERACCIÓN MISSION BOSS
    # ─────────────────────────────────────────

    def _gs_build_frame(self, subsystem_id: int, cmd: int, data: bytes) -> bytes:
        """
        Construye un frame para interacción con Mission Boss.
        El formato es el mismo que el BSL, pero se puede usar
        para enviar comandos personalizados o recibir datos.

        [SUBSYSTEM_ID] [CMD_ID] [DATA_SIZE] [DATA...] [CRC_MSB] [CRC_LSB] [STOP_BYTE]

        """
        data_size = len(data)
        # Revisar calculo de CRC
        crc = cmd ^ data_size
        for b in data:
            crc ^= b

        frame = bytes([
            subsystem_id,
            cmd,
            data_size & 0xFF,
            (data_size >> 8) & 0xFF,
        ]) + data + bytes([crc & 0xFF, (crc >> 8) & 0xFF, STOP_BYTE])

        return frame

    def send_mb_command(self, cmd: int, data: bytes) -> bool:
        """Envía un comando personalizado a Mission Boss."""
        frame = self._gs_build_frame(PC_SUBSYSTEM_ID, cmd, data)
        self.ser.write(frame)
        self.ser.flush()
        print(f"[Mission Boss] Comando 0x{cmd:02X} enviado con {len(data)} bytes de datos")
        return True
    
    def request_mb_bypass_uart_mode(self) -> bool:
        """Ejemplo de comando personalizado para pedir a Mission Boss que active modo bypass UART."""
        print("[Mission Boss] Solicitando modo bypass UART...")

        self.open()
        self.ser.write(bytes([0x05]))
        response = self.ser.read(5)
        if response:
            print(f"[Mission Boss] Respuesta recibida: {response.hex()}")
            for b in response:
                if b == MB_ACK:  # ACK esperado
                    print("[Mission Boss] ✓ Modo bypass UART activado")
                    return True
            print("[Mission Boss] ✗ Respuesta no reconocida o ACK no encontrado")
        else:
            print("[Mission Boss] ✗ Sin respuesta (timeout)")

        self.ser.close()
        return False 

    # ─────────────────────────────────────────
    #  FLUJO COMPLETO DE PROGRAMACIÓN
    # ─────────────────────────────────────────

    def program_ti_txt(self, filepath: str) -> bool:
        """
        Proceso completo de programación desde un archivo TI-TXT:
          1. Mass-erase (password incorrecta)
          2. Unlock (password correcta = 0xFF x 32)
          3. Escritura de todos los bloques
          4. Load PC al primer bloque
        """
        print(f"\n{'═'*55}")
        print(f"  MSP430 BSL Programmer")
        print(f"  Archivo: {os.path.basename(filepath)}")
        print(f"{'═'*55}\n")

        # 1. Parsear TI-TXT
        print("[TI-TXT] Parseando archivo...")
        try:
            blocks = parse_ti_txt(filepath)
        except Exception as e:
            print(f"[TI-TXT] ✗ Error al parsear: {e}")
            return False

        if not blocks:
            print("[TI-TXT] ✗ No se encontraron bloques de datos")
            return False

        total_bytes = sum(len(d) for _, d in blocks)
        print(f"[TI-TXT] ✓ {len(blocks)} bloque(s), {total_bytes} bytes totales")
        for addr, data in blocks:
            print(f"          @0x{addr:05X}  ({len(data)} bytes)")

        # 2. Abrir UART
        self.open()
        time.sleep(0.05)

        self.ser.write(bytes(0))  # Enviar algo para "despertar" al MSP
        self.ser.flush()
        print('Esperando respuesta de la MSP', end='', flush=True)
        while(not self.ser.read(1)):
            print('.', end='', flush=True)
        else:
            print('MSP430 en modo BSL detectado, continuando con programación...')
            time.sleep(1)

        try:
            # 3. Mass-erase
            self.mass_erase()
            time.sleep(0.2)

            # 4. Unlock con password correcta
            ok = self.send_password(DEFAULT_PASSWORD)
            if not ok:
                print("\n[BSL] ✗ No se pudo hacer unlock. Abortando.")
                return False
            time.sleep(0.05)

            # 5. Un comando RX_DATA_BLOCK por cada segmento @DIRECCIÓN del TI-TXT.
            #    Si un segmento supera MAX_BLOCK_SIZE se fragmenta en sub-bloques
            #    consecutivos actualizando la dirección base en cada uno.
            print("\n[BSL] Escribiendo bloques de memoria...")
            for addr, data in blocks:
                if len(data) <= MAX_BLOCK_SIZE:
                    # Segmento cabe en un solo comando — caso normal
                    ok = self.write_block(addr, data)
                    if not ok:
                        print(f"\n[BSL] ✗ Fallo al escribir @0x{addr:05X}. Abortando.")
                        return False
                else:
                    # Segmento demasiado grande: partir en sub-bloques
                    print(f"  [INFO] Segmento @0x{addr:05X} ({len(data)} bytes) "
                          f"supera {MAX_BLOCK_SIZE} bytes → fragmentando...")
                    offset = 0
                    while offset < len(data):
                        chunk = data[offset : offset + MAX_BLOCK_SIZE]
                        chunk_addr = addr + offset
                        ok = self.write_block(chunk_addr, chunk)
                        if not ok:
                            print(f"\n[BSL] ✗ Fallo al escribir sub-bloque "
                                  f"@0x{chunk_addr:05X}. Abortando.")
                            return False
                        offset += len(chunk)
                        time.sleep(0.01)
                time.sleep(0.01)

            # 6. Ejecutar programa (Load PC al primer bloque)
            first_addr = blocks[0][0]
            print()
            self.load_pc(first_addr)

            print(f"\n{'═'*55}")
            print("  ✓ Programación completada exitosamente")
            print(f"{'═'*55}\n")
            return True

        finally:
            self.close()

# ══════════════════════════════════════════════
#  UTILIDADES DE SELECCIÓN INTERACTIVA
# ══════════════════════════════════════════════

def list_serial_ports() -> list[str]:
    """Retorna lista de puertos serie disponibles."""
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return sorted(ports)

def select_port_interactive() -> str:
    """Muestra los puertos disponibles y pide al usuario que seleccione uno."""
    ports = list_serial_ports()
    if not ports:
        print("✗ No se encontraron puertos serie disponibles.")
        sys.exit(1)

    print("\nPuertos serie disponibles:")
    for i, p in enumerate(ports):
        info = serial.tools.list_ports.comports()
        desc = next((x.description for x in info if x.device == p), "")
        print(f"  [{i+1}] {p}  —  {desc}")

    while True:
        try:
            sel = int(input("\nSelecciona puerto [número]: ")) - 1
            if 0 <= sel < len(ports):
                return ports[sel]
        except (ValueError, KeyboardInterrupt):
            pass
        print("  Selección inválida, intenta de nuevo.")

def select_file_interactive(search_dirs: list[str] | None = None) -> str:
    """
    Busca archivos TI-TXT (.txt) en los directorios indicados
    (o en el directorio actual si no se especifican) y pide
    al usuario que seleccione uno.
    """
    if search_dirs is None:
        search_dirs = ["."]

    found = []
    for d in search_dirs:
        found += glob.glob(os.path.join(d, "*.txt"))
        found += glob.glob(os.path.join(d, "*.TXT"))

    # Eliminar duplicados y ordenar
    found = sorted(set(found))

    if not found:
        print(f"✗ No se encontraron archivos .txt en: {', '.join(search_dirs)}")
        manual = input("Ingresa la ruta del archivo TI-TXT manualmente: ").strip()
        if os.path.isfile(manual):
            return manual
        print("✗ Archivo no encontrado. Abortando.")
        sys.exit(1)

    print("\nArchivos TI-TXT encontrados:")
    for i, f in enumerate(found):
        size = os.path.getsize(f)
        print(f"  [{i+1}] {f}  ({size} bytes)")

    while True:
        try:
            sel = int(input("\nSelecciona archivo [número]: ")) - 1
            if 0 <= sel < len(found):
                return found[sel]
        except (ValueError, KeyboardInterrupt):
            pass
        print("  Selección inválida, intenta de nuevo.")

# ══════════════════════════════════════════════
#  PUNTO DE ENTRADA
# ══════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="MSP430 BSL Programmer — programa via UART backchannel usando archivos TI-TXT"
    )
    parser.add_argument("-p", "--port",
        help="Puerto serie (ej. COM3, /dev/ttyUSB0)")
    parser.add_argument("-f", "--file",
        help="Archivo TI-TXT a cargar (.txt)")
    parser.add_argument("-b", "--baud", type=int, default=9600,
        help="Baudrate (default: 9600)")
    parser.add_argument("--search-dir", default="./titxt_files",
        help="Directorio donde buscar archivos .txt (default: directorio actual)")
    parser.add_argument("--list-ports", action="store_true",
        help="Lista los puertos serie disponibles y sale")
    args = parser.parse_args()

    # Solo listar puertos
    if args.list_ports:
        ports = list_serial_ports()
        if ports:
            print("Puertos disponibles:")
            for p in ports:
                print(f"  {p}")
        else:
            print("No se encontraron puertos.")
        return

    # Selección de puerto
    port = args.port or select_port_interactive()

    # Selección de archivo
    filepath = args.file or select_file_interactive([args.search_dir])

    if not os.path.isfile(filepath):
        print(f"✗ Archivo no encontrado: {filepath}")
        sys.exit(1)

    bsl = MSP430_BSL(port=port, baudrate=args.baud)

    # Solicitar modo bypass UART a Mission Boss
    if (bsl.request_mb_bypass_uart_mode()):
        print("[Mission Boss] Modo bypass UART activado, continuando con programación...")
        time.sleep(1)

        # Ejecutar programación
        success = bsl.program_ti_txt(filepath)
        sys.exit(0 if success else 1)
    else:
        print("[Mission Boss] No se pudo activar modo bypass UART. Abortando.")
        sys.exit(1)

main() 