# FSR Dance Pad Firmware

Firmware para pad de baile arcade con sensores FSR, arquitectura de doble microcontrolador.

## Arquitectura del sistema

```
┌─────────────────────────────────────────────────────────┐
│                    FSR Dance Pad                        │
│                                                         │
│  ┌──────────────┐   UART 921600  ┌──────────────────┐  │
│  │    RP2040    │ ◄────────────► │    ESP32-S3      │  │
│  │  1 kHz HID  │                │  WiFi + Web UI   │  │
│  └──────────────┘                └──────────────────┘  │
│        │                                  │            │
│   MCP3008 SPI                        WebSocket         │
│   16 sensores FSR                    Monitor UI        │
└─────────────────────────────────────────────────────────┘
```

## Módulos

### [`rp2040/`](./rp2040/) — Firmware RP2040 (tiempo real)
- Lectura de 16 sensores FSR vía SPI (MCP3008) a **1 kHz**
- HID Joystick USB con poll interval de 1 ms
- Anti-Held Miss: retrigger de flecha si se activa un sensor diferente
- Stats packet binario (14 bytes, tipo `0x02`) a 1 Hz → ESP32
- Protocolo UART binario: telemetría a 50 Hz, comandos event-driven

### [`esp32s3/`](./esp32s3/) — Firmware ESP32-S3 (WiFi / UI)
- Access Point WiFi + servidor WebSocket
- Web UI con tabs: configuración de umbrales, monitor de jitter
- Parseo de tramas STATS (`0x02`) del RP2040
- Configuración Anti-Held Miss persistente en NVS
- Comandos WS: `set_antiheld`, `reset_stats`

## Protocolo UART binario (RP2040 ↔ ESP32-S3)

| Dirección      | Tipo         | Tamaño | Frecuencia          |
|----------------|--------------|--------|---------------------|
| RP2040 → ESP32 | Telemetría   | 37 B   | 50 Hz (con cliente) |
| RP2040 → ESP32 | Stats        | 14 B   | 1 Hz (con cliente)  |
| ESP32 → RP2040 | Comando      | 6 B    | Event-driven        |

### Comandos ESP32 → RP2040

| Código | Nombre               | Descripción                    |
|--------|----------------------|--------------------------------|
| `0x10` | `CMD_SET_THR`        | Ajustar umbral de sensor       |
| `0x20` | `CMD_START_STREAM`   | Iniciar telemetría             |
| `0x21` | `CMD_STOP_STREAM`    | Detener telemetría             |
| `0x30` | `CMD_RECALIBRATE`    | Recalibrar sensores            |
| `0x41` | `CMD_SET_ANTIHELD_EN`| Activar/desactivar Anti-Held   |
| `0x42` | `CMD_SET_ANTIHELD_DB`| Debounce Anti-Held (ms)        |
| `0x43` | `CMD_SET_ANTIHELD_ST`| Settle window Anti-Held (ms)   |
| `0x44` | `CMD_SET_ANTIHELD_GR`| Growth retrigger (ms)          |
| `0x50` | `CMD_RESET_STATS`    | Resetear contadores de jitter  |

## Pines

### RP2040
| Pin   | Función            |
|-------|--------------------|
| GP12  | UART TX → ESP32    |
| GP13  | UART RX ← ESP32    |
| SPI   | MCP3008 (16 FSR)   |

### ESP32-S3
| Pin | Función          |
|-----|------------------|
| 43  | UART TX → RP2040 |
| 44  | UART RX ← RP2040 |

## Dependencias

### RP2040
- [arduino-pico](https://github.com/earlephilhower/arduino-pico) core
- `Joystick` library
- `TinyUSB` (tusb.h)

### ESP32-S3
- `WiFi`, `DNSServer`, `WebServer`
- [WebSocketsServer](https://github.com/Links2004/arduinoWebSockets)
- [ArduinoJson](https://arduinojson.org/)
- `Preferences` (NVS)

## Historial de versiones

| Versión | Módulo  | Cambios principales |
|---------|---------|---------------------|
| v3.1    | ESP32   | Monitor de jitter, Anti-Held Miss configurable desde UI |
| v4 pro  | RP2040  | SPI.beginTransaction() fuera de ISR (−8-15µs jitter), Anti-Held Miss, stats packet |
| v3 pro  | RP2040  | Base de referencia |
| v2.0    | ESP32   | Versión anterior sin monitor de stats |

## Compilación

1. Abrir el `.ino` correspondiente en Arduino IDE
2. Seleccionar la placa correcta:
   - RP2040: `Raspberry Pi Pico` (core arduino-pico)
   - ESP32-S3: `ESP32S3 Dev Module`
3. Instalar dependencias desde el Library Manager
4. Compilar y subir
