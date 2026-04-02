import os
from intelhex import IntelHex

CARPETA_ENTRADA = "intelhex_files"
CARPETA_SALIDA  = "hex2bsl"

def listar_archivos_hex(carpeta):
    archivos = [f for f in os.listdir(carpeta) if f.lower().endswith(".hex")]
    return archivos

def seleccionar_archivo(archivos):
    print("\nArchivos .hex disponibles en la carpeta 'intelhex':")
    for i, nombre in enumerate(archivos, 1):
        print(f"  [{i}] {nombre}")
    
    while True:
        entrada = input("\nEscribe el número del archivo que deseas convertir: ").strip()
        if entrada.isdigit():
            idx = int(entrada) - 1
            if 0 <= idx < len(archivos):
                return archivos[idx]
        print("Opción inválida, intenta de nuevo.")

def generar_archivos_firmware():
    # Verificar que exista la carpeta de entrada
    if not os.path.isdir(CARPETA_ENTRADA):
        print(f"[ERROR] No se encontró la carpeta '{CARPETA_ENTRADA}'.")
        print("Crea la carpeta y coloca tus archivos .hex dentro de ella.")
        return

    # Listar archivos disponibles
    archivos = listar_archivos_hex(CARPETA_ENTRADA)
    if not archivos:
        print(f"[ERROR] No hay archivos .hex dentro de la carpeta '{CARPETA_ENTRADA}'.")
        return

    # Selección por terminal
    archivo_seleccionado = seleccionar_archivo(archivos)
    ruta_hex = os.path.join(CARPETA_ENTRADA, archivo_seleccionado)

    # Crear carpeta de salida si no existe
    os.makedirs(CARPETA_SALIDA, exist_ok=True)

    try:
        nombre_base = os.path.splitext(archivo_seleccionado)[0]
        ruta_bin = os.path.join(CARPETA_SALIDA, f"{nombre_base}.bin")
        ruta_h   = os.path.join(CARPETA_SALIDA, f"{nombre_base}.h")

        # 1. Convertir Hex a Bin crudo
        ih = IntelHex(ruta_hex)
        ih.tobinfile(ruta_bin)

        # 2. Leer el Bin para generar el arreglo en C
        with open(ruta_bin, 'rb') as f_bin:
            datos_bin = f_bin.read()

        tamano_bin = len(datos_bin)

        # 3. Escribir el archivo .h
        with open(ruta_h, 'w') as f_h:
            f_h.write("#ifndef FIRMWARE_MSB_H\n")
            f_h.write("#define FIRMWARE_MSB_H\n\n")
            f_h.write("#include <stdint.h>\n\n")

            f_h.write(f"#define FIRMWARE_SIZE {tamano_bin}\n")
            f_h.write(f"#define FIRMWARE_PART1_SIZE {tamano_bin}\n\n")

            f_h.write(f"const uint8_t firmware_part1[{tamano_bin}] = {{\n")

            for i in range(0, tamano_bin, 12):
                bloque = datos_bin[i:i+12]
                linea = ", ".join([f"0x{b:02X}" for b in bloque])
                if i + 12 < tamano_bin:
                    f_h.write(f"    {linea},\n")
                else:
                    f_h.write(f"    {linea}\n")

            f_h.write("};\n\n")
            f_h.write("#endif // FIRMWARE_MSB_H\n")

        # Resumen en terminal
        peso_hex = os.path.getsize(ruta_hex)
        print("\n¡Conversión exitosa!")
        print(f"  Peso original (.hex) : {peso_hex:,} bytes")
        print(f"  Peso real    (.bin)  : {tamano_bin:,} bytes")
        print(f"\nArchivos generados en '{CARPETA_SALIDA}/':")
        print(f"  -> {os.path.basename(ruta_bin)}")
        print(f"  -> {os.path.basename(ruta_h)}")
        print("\nEl archivo .h está listo para usar.")

    except Exception as e:
        print(f"\n[ERROR] Ocurrió un problema al procesar el archivo:\n  {e}")

if __name__ == "__main__":
    generar_archivos_firmware()