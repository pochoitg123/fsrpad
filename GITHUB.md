# Actualizaciones del FSRpad WROOM-32E-N8

El pad consulta https://raw.githubusercontent.com/pochoitg123/fsrpad/main/ota/manifest.json . No es necesario habilitar Pages. El canal existe al publicar la primera versión real. No publiques un manifiesto de ejemplo.

1. Descarga el proyecto configurado para GitHub y conserva la carpeta completa. En ota_config.h establece PAD_BUILD en un entero creciente (1 para la instalación inicial, 2 para la primera actualización). Compila para ESP32 Dev Module / flash 8 MB y exporta los binarios compilados en Arduino IDE.
2. En Windows, abre publicar.cmd y selecciona el archivo FSRpad_N8.ino.bin. Requiere Python 3 en el PATH (py o python). La herramienta pide el número de PAD_BUILD usado al compilar. En otros sistemas: python3 tools/preparar_version.py RUTA/FSRpad_N8.ino.bin 2 --out ota
3. En https://github.com/pochoitg123/fsrpad abre la rama main. Usa Add file → Upload files. Arrastra la carpeta ota generada a la raíz del repositorio, o, si ya abriste la carpeta ota en GitHub, arrastra solo los dos archivos. Verifica las rutas ota/manifest.json y ota/firmware-2.bin (para build 2). Confirma ambos en el mismo commit. Usa un número nuevo para cada binario; conserva los binarios anteriores por la caché de GitHub.
4. Abre el enlace del manifiesto y después firmware-2.bin en la misma ruta. Cuando ambos estén publicados, entra en Ajustes → Actualizaciones → Buscar actualización → Instalar y reiniciar. El build debe ser mayor que el instalado. Si el reloj se está sincronizando, espera y reintenta. GitHub puede tardar unos minutos en refrescar la caché.
5. Si el pad tiene el firmware OTA anterior con el canal vacío, puedes instalar la edición configurada desde Actualizaciones → Desde un archivo, sin USB. Si nunca instalaste el firmware OTA con dos particiones, la primera carga se hace por USB siguiendo LEEME.txt.

Publica solo los archivos de distribución. Nunca subas claves privadas, contraseñas ni tokens. El ESP32 valida TLS, modelo, tamaño y SHA-256.
