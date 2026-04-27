// =============================================================================
//  rp2040_mcp3008_16fsr_1khz_pro4.ino
//  Pro Arcade FSR Pad – RP2040  |  1 kHz HID · Anti-Held Miss · Jitter Monitor
// =============================================================================
//
//  CORRECCIONES vs pro3:
//    1. SPI.beginTransaction() llamado UNA SOLA VEZ en setup1() — fuera de la ISR
//       Esto elimina el mutex lock/unlock + reconfig HW cada 250µs (mayor fuente
//       de jitter identificada: ~8-15µs por ciclo de ISR)
//    2. Anti-Held Miss: re-trigger de flecha si un sensor DISTINTO se activa
//       mientras la flecha sigue pisada. Debounce configurable vía WiFi/Serial.
//    3. Stats packet binario (14 bytes, tipo 0x02) enviado cada 1 s al ESP32
//       para el monitor de jitter sin impacto en el gameplay.
//
//  PROTOCOLO UART BINARIO (RP2040 ↔ ESP32-S3):
//    RP2040→ESP32  Telemetría (37 bytes, tipo 0x01)  @ 50 Hz (cuando hay cliente)
//    RP2040→ESP32  Stats      (14 bytes, tipo 0x02)  @ 1 Hz  (cuando hay cliente)
//    ESP32→RP2040  Comando    ( 6 bytes)             event-driven
//
//  COMANDOS NUEVOS:
//    0x41  CMD_SET_ANTIHELD_EN   valor: 0=off / 1=on
//    0x42  CMD_SET_ANTIHELD_DB   valor: debounce en ms (0-500)
//    0x43  CMD_SET_ANTIHELD_ST   valor: settle window en ms (0-500)
//    0x44  CMD_SET_ANTIHELD_GR   valor: growth retrig en ms (0-500)
//    0x50  CMD_RESET_STATS       resetea contadores de jitter
//
//  PINES UART bridge (GP12=TX, GP13=RX)
// =============================================================================

// ─── Feature flag: comentar para compilar sin bridge WiFi ────────────────────
#define ENABLE_WIFI_BRIDGE

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <hardware/timer.h>
#include <hardware/sync.h>
#include <pico/multicore.h>

#if !defined(ARDUINO_ARCH_RP2040) && !defined(PICO_BOARD)
  #error "Solo para RP2040 con arduino-pico core."
#endif

#include <Joystick.h>
#include "tusb.h"

int usb_hid_poll_interval = 1;   // 1 kHz HID

// =============================================================================
//  PINES SPI
// =============================================================================
#define PIN_SCK   2
#define PIN_MOSI  3
#define PIN_MISO  4
#define PIN_CS1   5
#define PIN_CS2   6

// =============================================================================
//  PINES LED (una luz por flecha)
//  Arrow 0 → GP29 | Arrow 1 → GP28 | Arrow 2 → GP27 | Arrow 3 → GP26
// =============================================================================
#define PIN_LED_ARROW0  29
#define PIN_LED_ARROW1  28
#define PIN_LED_ARROW2  27
#define PIN_LED_ARROW3  26

static const uint8_t kLedPins[4] = {
  PIN_LED_ARROW0, PIN_LED_ARROW1, PIN_LED_ARROW2, PIN_LED_ARROW3
};

// =============================================================================
//  PINES UART bridge
// =============================================================================
#ifdef ENABLE_WIFI_BRIDGE
  #define UART_BRIDGE    Serial1
  #define UART_TX_PIN    12
  #define UART_RX_PIN    13
  #define UART_BAUD      921600
#endif

// =============================================================================
//  PARÁMETROS DSP
// =============================================================================
static const uint8_t  kNumArrows       = 4;
static const uint8_t  kSensorsPerArrow = 4;
static const uint8_t  kNumSensors      = 16;
static const uint32_t kSetPeriodUs     = 250;
static const uint8_t  kWindowSize      = 8;
static const int16_t  kDefaultThr      = 100;
static const int16_t  kHysteresis      = 8;

// SPI: 2 MHz – seguro para MCP3008 @ 3.3V (max 3.6 MHz).
// beginTransaction/endTransaction ya NO se llaman en la ISR.
static const SPISettings kSpiCfg(2000000, MSBFIRST, SPI_MODE0);

// =============================================================================
//  TABLAS
// =============================================================================
static const uint8_t SENSOR_TO_ARROW[kNumSensors] = {
  0,0,0,0, 1,1,1,1, 2,2,2,2, 3,3,3,3
};
static const uint8_t SET_SENSORS[4][4] = {
  { 0, 4,  8, 12},
  { 1, 5,  9, 13},
  { 2, 6, 10, 14},
  { 3, 7, 11, 15}
};

// =============================================================================
//  UMBRALES  (atómico ARM 16-bit alineado)
// =============================================================================
volatile int16_t gThresholds[kNumSensors];

// =============================================================================
//  COMUNICACIÓN INTER-CORE
// =============================================================================
static volatile uint8_t gButtonState = 0;
static volatile bool    gReportReady = false;

static inline void publishReport(uint8_t btns) {
  gButtonState = btns;
  __dmb();
  gReportReady = true;
}
static inline bool consumeReport(uint8_t* btns) {
  if (!gReportReady) return false;
  __dmb();
  *btns = gButtonState;
  gReportReady = false;
  return true;
}

// =============================================================================
//  MÉTRICAS DE JITTER  (escritas en Core 1 ISR, leídas en Core 0)
// =============================================================================
static volatile bool     gCalibrationDone = false;  // Core 1 lo setea al terminar calibración
static volatile uint32_t gMaxJitterUs   = 0;
static volatile uint32_t gMissedAlarms  = 0;
static volatile uint32_t gTotalCycles   = 0;

// =============================================================================
//  ANTI-HELD MISS
//  Permite re-marcar la misma flecha si un sensor distinto se activa mientras
//  la flecha sigue pisada (técnica de dos pies en la misma flecha).
//
//  Mecanismo (versión robusta):
//    - Al primer press se guarda un "footprint" inicial (holdMask)
//    - Durante una ventana corta de asentamiento, cualquier sensor nuevo
//      de esa misma flecha se absorbe SIN retrigger (mismo stomp rodado)
//    - Fuera de esa ventana, solo se permite retrigger si el patrón de
//      sensores cambió de verdad (hay sensores nuevos Y se soltaron otros)
//      → evita que una pisada que solo "crece" dispare doble hit falso
//    - Cuando se acepta un retrigger:
//        → fuerza un release de 1 frame en la próxima publicación
//        → el juego ve: ON, OFF(1ms), ON → registra 2 presses
// =============================================================================
struct ArrowAHM {
  bool     held            = false;
  uint8_t  holdMask        = 0;    // footprint de referencia del hold actual
  uint32_t lastCycle       = 0;    // gTotalCycles del press / retrigger aceptado
  uint32_t settleStartCycle = 0;   // inicio de la ventana de asentamiento
};
static ArrowAHM gAHM[kNumArrows];

// Configurables vía WiFi/Serial (escritas en Core 0, leídas en Core 1 ISR)
volatile bool     gAntiheldEnable      = true;
volatile uint16_t gAntiheldDebounceMs  = 80;   // debounce entre re-triggers válidos
volatile uint16_t gAntiheldSettleMs    = 110;   // absorbe crecimiento del mismo stomp
volatile uint16_t gAntiheldGrowthRetrigMs = 180; // permite re-trigger si la huella solo crece, pero tras un retardo mayor
volatile uint32_t gAntiheldRetriggers  = 0;    // contador para stats

// =============================================================================
//  WMA / HMA
// =============================================================================
class WMA {
 public:
  explicit WMA(uint8_t n)
    : n_(n < 1 ? 1u : (n > kWindowSize ? (uint8_t)kWindowSize : n)),
      sum_(0), wsum_(0), head_(0) {
    denom_ = (int32_t)n_ * (n_ + 1) / 2;
    memset(buf_, 0, sizeof(buf_));
  }
  void prime(int16_t v) {
    sum_ = 0; wsum_ = 0;
    for (uint8_t i = 0; i < n_; ++i) {
      buf_[i] = v; sum_ += v; wsum_ += (int32_t)(i+1)*v;
    }
    head_ = 0;
  }
  int16_t update(int16_t v) {
    int32_t ns  = sum_  + v - buf_[head_];
    int32_t nws = wsum_ + (int32_t)n_ * v - sum_;
    sum_ = ns; wsum_ = nws;
    buf_[head_] = v;
    if (++head_ >= n_) head_ = 0;
    return (int16_t)(nws / denom_);
  }
 private:
  uint8_t n_; int32_t sum_, wsum_, denom_;
  int16_t buf_[kWindowSize]; uint8_t head_;
};

class HMA {
 public:
  explicit HMA(uint8_t n)
    : wma_half_((uint8_t)(n/2 > 0 ? n/2 : 1)),
      wma_full_((uint8_t)(n > 0 ? n : 1)),
      wma_hull_((uint8_t)((uint8_t)sqrtf((float)n) > 0 ? (uint8_t)sqrtf((float)n) : 1)) {}
  void prime(int16_t v) { wma_half_.prime(v); wma_full_.prime(v); wma_hull_.prime(v); }
  int16_t update(int16_t v) {
    int16_t h = wma_half_.update(v), f = wma_full_.update(v);
    return wma_hull_.update((int16_t)(2*h - f));
  }
 private:
  WMA wma_half_, wma_full_, wma_hull_;
};

// =============================================================================
//  ESTADO DSP POR SENSOR  (Core 1 únicamente)
// =============================================================================
struct SensorDSP {
  HMA     hma{kWindowSize};
  int16_t offset   = 0;
  int16_t curValue = 0;   // atómico ARM 16-bit, leído por Core 0 para stats/UART
  bool    pressed  = false;
};
static SensorDSP gDSP[kNumSensors];

// =============================================================================
//  LECTURA SPI MCP3008
//  beginTransaction/endTransaction se llaman UNA VEZ en setup1(),
//  NO dentro de esta función ni de la ISR → elimina overhead de mutex por ciclo.
// =============================================================================
static inline uint16_t ReadMcp3008(uint8_t cs, uint8_t ch) {
  digitalWrite(cs, LOW);
  SPI.transfer(0x01);
  uint8_t hi = SPI.transfer((uint8_t)(0x80 | (ch << 4)));
  uint8_t lo = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  return (uint16_t)(((hi & 0x03) << 8) | lo);
}

// =============================================================================
//  HARDWARE ALARM  (Core 1 ISR – todo el hot path aquí)
// =============================================================================
static uint8_t  gAlarmNum;
static uint64_t gNextAlarmUs;

static void alarmCallback(uint alarm_num);
static inline void armAlarm() {
  hardware_alarm_set_target(gAlarmNum, from_us_since_boot(gNextAlarmUs));
}

static void alarmCallback(uint /*alarm_num*/) {
  static uint8_t  set            = 0;
  static uint8_t  sRetrigRelease = 0;  // fuerza OFF en el publish de este ciclo

  // ── Reprogramar PRIMERO → sin drift acumulativo ──────────────────────────
  uint64_t now  = to_us_since_boot(get_absolute_time());
  int64_t  late = (int64_t)(now - gNextAlarmUs);
  if (late > 0) {
    if ((uint32_t)late > gMaxJitterUs) gMaxJitterUs = (uint32_t)late;
    if (late > (int64_t)kSetPeriodUs) gMissedAlarms++;
  }
  gNextAlarmUs += kSetPeriodUs;
  armAlarm();

  // ── Leer 4 sensores del set (SIN beginTransaction/endTransaction) ─────────
  uint16_t raw[4];
  raw[0] = ReadMcp3008(PIN_CS1, set);
  raw[1] = ReadMcp3008(PIN_CS1, (uint8_t)(4 + set));
  raw[2] = ReadMcp3008(PIN_CS2, set);
  raw[3] = ReadMcp3008(PIN_CS2, (uint8_t)(4 + set));

  // ── DSP ──────────────────────────────────────────────────────────────────
  const uint8_t* idx = SET_SENSORS[set];
  for (uint8_t i = 0; i < 4; ++i) {
    SensorDSP& d = gDSP[idx[i]];
    int16_t filt = d.hma.update((int16_t)raw[i]) - d.offset;
    if (filt < 0)    filt = 0;
    if (filt > 1023) filt = 1023;
    d.curValue = filt;
    int16_t thr = gThresholds[idx[i]];
    if (!d.pressed && filt >= thr + kHysteresis) d.pressed = true;
    else if (d.pressed && filt < thr - kHysteresis) d.pressed = false;
  }

  set = (set + 1) & 0x03;

  // ── Publicar al completar ciclo de 1 ms (4 sets) ─────────────────────────
  if (set == 0) {

    // 1. Calcular bitmask de sensores presionados por flecha
    uint8_t arrowMask[kNumArrows] = {0, 0, 0, 0};
    for (uint8_t si = 0; si < kNumSensors; ++si) {
      if (gDSP[si].pressed)
        arrowMask[SENSOR_TO_ARROW[si]] |= (uint8_t)(1u << (si & 3u));
    }

    // 2. Bitmask de flechas activas
    uint8_t buttons = 0;
    for (uint8_t a = 0; a < kNumArrows; ++a)
      if (arrowMask[a]) buttons |= (uint8_t)(1u << a);

    // 3. Aplicar release forzado del ciclo anterior (anti-held miss gap)
    uint8_t publishBtns = buttons & ~sRetrigRelease;
    sRetrigRelease = 0;   // consumido

    // 4. Anti-Held Miss detection (robusto contra multi-sensor escalonado)
    if (gAntiheldEnable) {
      for (uint8_t a = 0; a < kNumArrows; ++a) {
        ArrowAHM& ahm = gAHM[a];
        const uint8_t curMask = arrowMask[a];

        if (curMask == 0) {
          // Flecha liberada → resetear estado
          ahm.held = false;
          ahm.holdMask = 0;
          ahm.lastCycle = gTotalCycles;
          ahm.settleStartCycle = gTotalCycles;

        } else if (!ahm.held) {
          // Flecha recién presionada → registrar footprint inicial
          ahm.held = true;
          ahm.holdMask = curMask;
          ahm.lastCycle = gTotalCycles;
          ahm.settleStartCycle = gTotalCycles;

        } else {
          const uint8_t newSensors  = curMask & (uint8_t)~ahm.holdMask;
          const uint8_t lostSensors = ahm.holdMask & (uint8_t)~curMask;

          // Ventana corta de asentamiento: el mismo stomp puede tocar más
          // sensores en ms consecutivos. Se absorbe sin retrigger.
          const uint32_t settleElapsed = gTotalCycles - ahm.settleStartCycle;
          if (settleElapsed < (uint32_t)gAntiheldSettleMs) {
            ahm.holdMask = curMask;
            continue;
          }

          if (newSensors) {
            const uint32_t elapsed = gTotalCycles - ahm.lastCycle;
            if (!lostSensors) {
              // La huella solo creció. Para evitar el doble falso por el mismo stomp,
              // normalmente se absorbe como el mismo golpe. Pero si pasó suficiente
              // tiempo, permitir un re-trigger para casos de dos pies en la misma
              // flecha manteniendo un sensor ya activo (ej. borde derecho -> izquierdo).
              if (elapsed >= (uint32_t)gAntiheldGrowthRetrigMs) {
                ahm.holdMask = curMask;
                ahm.lastCycle = gTotalCycles;
                ahm.settleStartCycle = gTotalCycles;
                sRetrigRelease |= (uint8_t)(1u << a);
                gAntiheldRetriggers++;
              } else {
                ahm.holdMask = curMask;
              }
            } else {
              // Cambio real de footprint: posible segundo golpe / footswitch
              if (elapsed >= (uint32_t)gAntiheldDebounceMs) {
                ahm.holdMask = curMask;
                ahm.lastCycle = gTotalCycles;
                ahm.settleStartCycle = gTotalCycles;          // reabrir asentamiento
                sRetrigRelease |= (uint8_t)(1u << a);         // forzar OFF próximo publish
                gAntiheldRetriggers++;
              } else {
                // Muy pronto para otro retrigger: actualizar referencia sin generar OFF/ON
                ahm.holdMask = curMask;
              }
            }
          } else if (lostSensors) {
            // La huella se achicó o se desplazó, pero sin sensores nuevos.
            // Mantener referencia actualizada evita que queden bits "viejos".
            ahm.holdMask = curMask;
          }
        }
      }
    }

    // 5. Publicar
    publishReport(publishBtns);
    gTotalCycles++;
  }
}

// =============================================================================
//  CALIBRACIÓN
// =============================================================================
static void CalibrateSensors() {
  const uint16_t kPasses = 64;
  int32_t sums[kNumSensors] = {};
  for (uint16_t pass = 0; pass < kPasses; ++pass) {
    for (uint8_t s = 0; s < 4; ++s) {
      uint16_t raw[4];
      raw[0] = ReadMcp3008(PIN_CS1, s);
      raw[1] = ReadMcp3008(PIN_CS1, (uint8_t)(4 + s));
      raw[2] = ReadMcp3008(PIN_CS2, s);
      raw[3] = ReadMcp3008(PIN_CS2, (uint8_t)(4 + s));
      const uint8_t* idx = SET_SENSORS[s];
      for (uint8_t i = 0; i < 4; ++i) {
        sums[idx[i]] += raw[i];
        gDSP[idx[i]].hma.update((int16_t)raw[i]);
      }
    }
    sleep_us(2000);
  }
  for (uint8_t i = 0; i < kNumSensors; ++i) {
    int16_t base = (int16_t)(sums[i] / kPasses);
    gDSP[i].offset = base;
    gDSP[i].hma.prime(base);
    gDSP[i].curValue = 0;
    gDSP[i].pressed  = false;
  }
  // Resetear estado anti-held
  for (uint8_t a = 0; a < kNumArrows; ++a) gAHM[a] = ArrowAHM{};
}

// =============================================================================
//  CORE 1
// =============================================================================
void setup1() {
  // ── 1. Inicializar umbrales ───────────────────────────────────────────────
  for (uint8_t i = 0; i < kNumSensors; ++i) {
    gThresholds[i] = kDefaultThr;
  }

  // ── 2. CS HIGH antes de cualquier actividad SPI ───────────────────────────
  //    CRÍTICO: si CS flota LOW durante SPI.begin(), el MCP3008 responde al
  //    reloj y su FSM interna queda corrupta → sensor no detectado al boot.
  pinMode(PIN_CS1, OUTPUT); digitalWrite(PIN_CS1, HIGH);
  pinMode(PIN_CS2, OUTPUT); digitalWrite(PIN_CS2, HIGH);

  // ── 3. Settling time para que los MCP3008 estabilicen su alimentación ─────
  sleep_ms(100);

  // ── 4. Inicializar SPI con CS ya en HIGH ─────────────────────────────────
  SPI.setSCK(PIN_SCK); SPI.setTX(PIN_MOSI); SPI.setRX(PIN_MISO);
  SPI.begin();

  // ▼▼▼ CLAVE: beginTransaction UNA SOLA VEZ – Core 1 es dueño exclusivo de SPI
  //     Elimina ~8-15µs de overhead por ISR (mutex + reconfig HW cada 250µs)
  SPI.beginTransaction(kSpiCfg);

  // ── 5. Calibrar (64 passes × 2ms = ~128ms) ───────────────────────────────
  CalibrateSensors();

  // ── 6. Señalizar a Core 0 que la calibración terminó ─────────────────────
  __dmb();
  gCalibrationDone = true;

  // ── 7. Arrancar el alarm ISR ──────────────────────────────────────────────
  gAlarmNum    = hardware_alarm_claim_unused(true);
  hardware_alarm_set_callback(gAlarmNum, alarmCallback);
  gNextAlarmUs = to_us_since_boot(get_absolute_time()) + kSetPeriodUs;
  armAlarm();
}

void loop1() { tight_loop_contents(); }

// =============================================================================
//  PROTOCOLO BINARIO UART
// =============================================================================
#ifdef ENABLE_WIFI_BRIDGE

#define PKT_START         0xAA
#define PKT_END           0x55
#define PKT_TYPE_TELEM    0x01
#define PKT_TYPE_STATS    0x02   // nuevo: trama de métricas de jitter
#define CMD_SET_THR       0x10
#define CMD_START_STREAM  0x20
#define CMD_STOP_STREAM   0x21
#define CMD_RECALIBRATE   0x30
#define CMD_SET_ANTIHELD_EN  0x41  // nuevo: habilitar/deshabilitar anti-held
#define CMD_SET_ANTIHELD_DB  0x42  // nuevo: debounce en ms
#define CMD_SET_ANTIHELD_ST  0x43  // nuevo: settle window en ms
#define CMD_SET_ANTIHELD_GR  0x44  // growth retrigger en ms
#define CMD_RESET_STATS      0x50  // nuevo: resetear contadores de jitter

#define TELEM_LEN   37
#define STATS_LEN   14
#define CMD_LEN      6

// ─── Estado bridge ────────────────────────────────────────────────────────────
static bool     gStreamActive      = false;
static uint32_t gLastTxMs          = 0;
static uint32_t gLastStatsTxMs     = 0;
static const uint32_t kTxPeriodMs  = 20;    // telemetría @ 50 Hz
static const uint32_t kStatsPeriodMs = 1000; // stats @ 1 Hz

static uint8_t gTxBuf[TELEM_LEN];

// ─── Parser RX (máquina de estados, no bloqueante) ────────────────────────────
static struct { uint8_t buf[CMD_LEN]; uint8_t idx; } gRxState;

// ─── Telemetría ───────────────────────────────────────────────────────────────
static void uartSendTelemetry() {
  gTxBuf[0] = PKT_START;
  gTxBuf[1] = PKT_TYPE_TELEM;
  uint8_t xorSum = PKT_TYPE_TELEM;
  uint8_t off = 2;
  for (uint8_t i = 0; i < kNumSensors; ++i) {
    int16_t v  = gDSP[i].curValue;
    uint8_t hi = (uint8_t)(v >> 8), lo = (uint8_t)(v & 0xFF);
    gTxBuf[off++] = hi; gTxBuf[off++] = lo;
    xorSum ^= hi; xorSum ^= lo;
  }
  uint8_t btn  = gButtonState;
  gTxBuf[off++] = btn;  xorSum ^= btn;
  gTxBuf[off++] = xorSum;
  gTxBuf[off]   = PKT_END;
  UART_BRIDGE.write(gTxBuf, TELEM_LEN);
}

// ─── Stats (jitter monitor) ───────────────────────────────────────────────────
// Trama 14 bytes:
//  [0xAA][0x02][maxJitter16 BE][missed16 BE][cycles32 BE][retrig16 BE][XOR][0x55]
static void uartSendStats() {
  uint8_t buf[STATS_LEN];
  buf[0] = PKT_START;
  buf[1] = PKT_TYPE_STATS;

  uint16_t j = (gMaxJitterUs  > 0xFFFF) ? 0xFFFF : (uint16_t)gMaxJitterUs;
  uint16_t m = (gMissedAlarms > 0xFFFF) ? 0xFFFF : (uint16_t)gMissedAlarms;
  uint32_t c = gTotalCycles;
  uint16_t r = (gAntiheldRetriggers > 0xFFFF) ? 0xFFFF : (uint16_t)gAntiheldRetriggers;

  buf[2]  = j >> 8;       buf[3]  = j & 0xFF;
  buf[4]  = m >> 8;       buf[5]  = m & 0xFF;
  buf[6]  = (c>>24)&0xFF; buf[7]  = (c>>16)&0xFF;
  buf[8]  = (c>> 8)&0xFF; buf[9]  = c & 0xFF;
  buf[10] = r >> 8;       buf[11] = r & 0xFF;

  uint8_t xc = 0;
  for (int i = 1; i <= 11; i++) xc ^= buf[i];
  buf[12] = xc;
  buf[13] = PKT_END;

  UART_BRIDGE.write(buf, STATS_LEN);
}

// ─── Ejecutar comando del ESP32 ───────────────────────────────────────────────
static void executeCommand(const uint8_t* cmd) {
  uint16_t val = ((uint16_t)cmd[3] << 8) | cmd[4];
  switch (cmd[1]) {
    case CMD_SET_THR:
      if (cmd[2] < kNumSensors) {
        int16_t v = (int16_t)val;
        if (v < 0) v = 0; if (v > 1023) v = 1023;
        gThresholds[cmd[2]] = v;
      }
      break;
    case CMD_START_STREAM:
      gStreamActive = true;
      break;
    case CMD_STOP_STREAM:
      gStreamActive = false;
      break;
    case CMD_RECALIBRATE:
      watchdog_reboot(0, 0, 0);
      break;
    case CMD_SET_ANTIHELD_EN:
      gAntiheldEnable = (cmd[4] != 0);
      break;
    case CMD_SET_ANTIHELD_DB:
      gAntiheldDebounceMs = val;   // ms
      break;
    case CMD_SET_ANTIHELD_ST:
      gAntiheldSettleMs = (val > 500) ? 500 : val;   // ms
      break;
    case CMD_SET_ANTIHELD_GR:
      gAntiheldGrowthRetrigMs = (val > 500) ? 500 : val;   // ms
      break;
    case CMD_RESET_STATS:
      gMaxJitterUs = 0; gMissedAlarms = 0; gAntiheldRetriggers = 0;
      break;
  }
}

// ─── RX no bloqueante ─────────────────────────────────────────────────────────
static void uartRxProcess() {
  while (UART_BRIDGE.available()) {
    uint8_t b = (uint8_t)UART_BRIDGE.read();
    if (gRxState.idx == 0) {
      if (b == PKT_START) { gRxState.buf[0] = b; gRxState.idx = 1; }
    } else {
      gRxState.buf[gRxState.idx++] = b;
      if (gRxState.idx == CMD_LEN) {
        gRxState.idx = 0;
        if (gRxState.buf[CMD_LEN - 1] == PKT_END) executeCommand(gRxState.buf);
      }
    }
  }
}

// ─── Tick principal del bridge (llamado en loop() DESPUÉS del HID) ────────────
static void uartBridgeTick() {
  uartRxProcess();               // siempre: procesar comandos entrantes

  if (!gStreamActive) return;    // sin clientes → no gastar bytes de UART

  uint32_t now = millis();

  // Telemetría @ 50 Hz
  if ((now - gLastTxMs) >= kTxPeriodMs) {
    gLastTxMs = now;
    uartSendTelemetry();
  }

  // Stats @ 1 Hz
  if ((now - gLastStatsTxMs) >= kStatsPeriodMs) {
    gLastStatsTxMs = now;
    uartSendStats();
  }
}

#endif  // ENABLE_WIFI_BRIDGE

// =============================================================================
//  CORE 0  –  USB HID (máxima prioridad) + Bridge (baja prioridad)
// =============================================================================
void setup() {
  Serial.begin(115200);
  Joystick.begin();
  Joystick.useManualSend(true);

#ifdef ENABLE_WIFI_BRIDGE
  UART_BRIDGE.setTX(UART_TX_PIN);
  UART_BRIDGE.setRX(UART_RX_PIN);
  UART_BRIDGE.begin(UART_BAUD);
  gRxState.idx  = 0;
  gStreamActive = false;
#endif

  // ── Inicializar LEDs de flechas ─────────────────────────────────────────────
  for (uint8_t a = 0; a < kNumArrows; ++a) {
    pinMode(kLedPins[a], OUTPUT);
    digitalWrite(kLedPins[a], LOW);
  }

  // Esperar a que Core 1 termine la calibración (flag en vez de delay ciego)
  // CS bug + settling: calibración real toma ~230ms pero esperamos con certeza.
  while (!gCalibrationDone) { delay(1); }
}

void loop() {
  // ── 1. HID — MÁXIMA PRIORIDAD ─────────────────────────────────────────────
  uint8_t btns;
  if (consumeReport(&btns)) {
    Joystick.button(1, (btns >> 0) & 1);
    Joystick.button(2, (btns >> 1) & 1);
    Joystick.button(3, (btns >> 2) & 1);
    Joystick.button(4, (btns >> 3) & 1);
    if (tud_hid_ready()) Joystick.send_now();

    // ── Actualizar LEDs (refleja el estado de cada flecha) ──────────────────
    for (uint8_t a = 0; a < kNumArrows; ++a)
      digitalWrite(kLedPins[a], (btns >> a) & 1 ? HIGH : LOW);
  }

  // ── 2. UART bridge — DESPUÉS del HID, nunca bloquea ──────────────────────
#ifdef ENABLE_WIFI_BRIDGE
  uartBridgeTick();
#endif

  // ── 3. Debug Serial — mínima prioridad ───────────────────────────────────
  handleDebugSerial();
}

// =============================================================================
//  DEBUG SERIAL USB
// =============================================================================
static void handleDebugSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (!line.length()) return;

  if (line == "t") {
    // Imprimir umbrales
    for (uint8_t i = 0; i < kNumSensors; ++i) {
      Serial.print(gThresholds[i]);
      Serial.print(i < kNumSensors-1 ? ',' : '\n');
    }

  } else if (line == "p") {
    // Imprimir valores raw de sensores
    for (uint8_t i = 0; i < kNumSensors; ++i) {
      Serial.print(gDSP[i].curValue);
      Serial.print(i < kNumSensors-1 ? ',' : '\n');
    }

  } else if (line == "j") {
    // Jitter stats
    Serial.print(F("maxJitter=")); Serial.print(gMaxJitterUs);
    Serial.print(F("us missed=")); Serial.print(gMissedAlarms);
    Serial.print(F(" cycles="));   Serial.print(gTotalCycles);
    Serial.print(F(" retrig="));   Serial.println(gAntiheldRetriggers);

  } else if (line == "ah") {
    // Imprimir config actual anti-held
    Serial.print(F("anti-held=")); Serial.print(gAntiheldEnable ? F("ON") : F("OFF"));
    Serial.print(F("  db="));     Serial.print(gAntiheldDebounceMs);  Serial.print(F("ms"));
    Serial.print(F("  st="));     Serial.print(gAntiheldSettleMs);    Serial.print(F("ms"));
    Serial.print(F("  gr="));     Serial.print(gAntiheldGrowthRetrigMs); Serial.println(F("ms"));

  } else if (line == "ah0") {
    gAntiheldEnable = false; Serial.println(F("Anti-held OFF"));

  } else if (line == "ah1") {
    gAntiheldEnable = true;  Serial.println(F("Anti-held ON"));

  } else if (line == "c") {
    Serial.println(F("Recalibrando...")); delay(100); watchdog_reboot(0,0,0);

  } else if (line == "help") {
    Serial.println(F("=== COMANDOS SERIAL ==="));
    Serial.println(F("  t             -> umbrales actuales"));
    Serial.println(F("  p             -> valores raw sensores"));
    Serial.println(F("  j             -> stats jitter"));
    Serial.println(F("  ah            -> config anti-held actual"));
    Serial.println(F("  ah0 / ah1     -> anti-held OFF / ON"));
    Serial.println(F("  ah_db <ms>    -> debounce (0-500)"));
    Serial.println(F("  ah_st <ms>    -> settle window (0-500)"));
    Serial.println(F("  ah_gr <ms>    -> growth retrig (0-500)"));
    Serial.println(F("  c             -> recalibrar"));
    Serial.println(F("  <s> <thr>     -> sensor s umbral thr"));

  } else {
    int sp = line.indexOf(' ');
    if (sp > 0) {
      String cmd = line.substring(0, sp);
      int val    = line.substring(sp + 1).toInt();

      if (cmd == "ah_db") {
        if (val < 0) val = 0; if (val > 500) val = 500;
        gAntiheldDebounceMs = (uint16_t)val;
        Serial.print(F("ah_db=")); Serial.println(val);

      } else if (cmd == "ah_st") {
        if (val < 0) val = 0; if (val > 500) val = 500;
        gAntiheldSettleMs = (uint16_t)val;
        Serial.print(F("ah_st=")); Serial.println(val);

      } else if (cmd == "ah_gr") {
        if (val < 0) val = 0; if (val > 500) val = 500;
        gAntiheldGrowthRetrigMs = (uint16_t)val;
        Serial.print(F("ah_gr=")); Serial.println(val);

      } else {
        // Comando umbral: <sensor_idx> <threshold>
        int si = cmd.toInt();
        if (si >= 0 && si < kNumSensors && val >= 0 && val <= 1023) {
          gThresholds[si] = (int16_t)val;
          Serial.print(F("OK s=")); Serial.print(si);
          Serial.print(F(" thr=")); Serial.println(val);
        }
      }
    }
  }
}
