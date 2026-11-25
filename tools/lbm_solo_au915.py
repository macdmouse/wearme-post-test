# LoRa Basics Modem (LBM_WM1110 del framework) solo con la región AU915: deja fuera del build los region_*.c de las
# demás regiones (~13 KB de flash). Es la configuración que Semtech prevé (su makefile compila solo las regiones
# elegidas): smtc_real.c y smtc_real_defs.h ya están protegidos con #if defined(REGION_xx); los region_*.c no, y por
# eso con las macros de platformio.ini quitadas no compilan si no se excluyen aquí.
# Con una sola región, LBM arranca en ella (lr1mac_core.c: smtc_real_region_list[0]) y el número de la región
# (SMTC_MODEM_REGION_AU_915 = 4) no cambia: el contexto guardado en la flash sigue valiendo.
# Para volver a todas las regiones: quitar este script de extra_scripts y restaurar los -DREGION_xx en platformio.ini.
import os

Import("env")  # noqa: F821 (lo define SCons)

KEEP = ("region_au_915.c",)


def skip_other_regions(node):
    name = os.path.basename(node.get_path())
    if name.startswith("region_") and name.endswith(".c") and name not in KEEP:
        return None  # no se compila
    return node


env.AddBuildMiddleware(skip_other_regions, "*smtc_real*region_*.c")  # noqa: F821
