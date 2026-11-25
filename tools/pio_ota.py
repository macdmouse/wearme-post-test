# Tarea de PlatformIO "ota": compila, arma firmware.zip y lo envía por Bluetooth a un collar en modo AdaDFU
# (tools/ota_dfu.py). En VS Code: PlatformIO -> Project Tasks -> Custom -> "OTA por Bluetooth", o `pio run -t ota`.
# Antes hay que habilitar el modo en el collar por LoRa: TTN downlink {"cmd":"dfu"} (o DFU:OTA por serie).
# Opciones en platformio.ini (o variables de entorno OTA_ADDRESS / OTA_SERIAL):
#   custom_ota_address = CF:46:AF:60:35:5F   ; collar a actualizar si hay varios en alcance
#   custom_ota_serial  = COM62               ; banco: manda DFU:OTA por ese puerto antes (sin downlink)
import os

Import("env")  # noqa: F821 (lo define SCons)

script = os.path.join(env.subst("$PROJECT_DIR"), "tools", "ota_dfu.py")  # noqa: F821
address = os.environ.get("OTA_ADDRESS") or env.GetProjectOption("custom_ota_address", "")  # noqa: F821
serial_port = os.environ.get("OTA_SERIAL") or env.GetProjectOption("custom_ota_serial", "")  # noqa: F821

flags = []
if address:
    flags += ["--address", address]
if serial_port:
    flags += ["--serial", serial_port]

# Tamaño máximo del firmware: el bootloader (Nordic SDK 11, doble banco) escribe el nuevo desde 0x89000 (mitad de la
# zona 0x26000-0xED000 menos los 28 KB de InternalFS) y LoRa Basics Modem guarda su contexto (claves, DevNonce, LR1MAC,
# modem) en 0xE9000-0xED000, DENTRO de esa zona. Más de 96 páginas (393216 B) y cada carga, por USB o por Bluetooth,
# borraría el contexto: DevNonce de vuelta a 0 y joins rechazados ("DevNonce already used"). Ver device_info.md.
FIRMWARE_MAX_BYTES = 0xE9000 - 0x89000


def check_firmware_size(target, source, env):  # noqa: ARG001
    import zipfile

    with zipfile.ZipFile(str(target[0])) as z:
        size = z.getinfo("firmware.bin").file_size
    margin = FIRMWARE_MAX_BYTES - size
    if margin < 0:
        print("\n*** ERROR: firmware de %d B: pasa el limite de %d B por %d B. Cargarlo (USB u OTA) borraria el contexto "
              "de LoRaWAN (0xE9000-0xED000): reducir el firmware (ver device_info.md, 'Mapa de la memoria flash').\n"
              % (size, FIRMWARE_MAX_BYTES, -margin))
        env.Exit(1)
    print("Firmware %d B: quedan %d B antes del contexto de LoRaWAN (limite %d B)" % (size, margin, FIRMWARE_MAX_BYTES))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.zip", check_firmware_size)  # noqa: F821

env.AddCustomTarget(  # noqa: F821
    name="ota",
    dependencies="$BUILD_DIR/${PROGNAME}.zip",
    actions='"$PYTHONEXE" "%s" "$BUILD_DIR/${PROGNAME}.zip" %s' % (script, " ".join(flags)),
    title="OTA por Bluetooth",
    description="Envia el firmware por Bluetooth a un collar en modo AdaDFU (habilitarlo por LoRa: {\"cmd\":\"dfu\"})",
)
