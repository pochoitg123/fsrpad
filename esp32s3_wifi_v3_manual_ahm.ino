/*
 * FSR Dance Pad — ESP32-S3 Firmware v3.1
 * ══════════════════════════════════════════════════════════════════════════════
 * NOVEDADES vs v2.0:
 *   - Parsea trama STATS (tipo 0x02) del RP2040 → Monitor de jitter en tiempo real
 *   - Tab MONITOR en la web UI: maxJitter, missed alarms, cycles, retriggers
 *   - Configuración Anti-Held Miss (enable/disable + debounce + settle) guardada en NVS
 *   - Debounce y settle anti-held ajustables desde la UI web
 *   - Nuevo comando WS: {t:"set_antiheld", enable:bool, debounce:ms, settle:ms}
 *   - Nuevo comando WS: {t:"reset_stats"}
 *   - Al conectar cliente: envía configuración antiheld y stats iniciales
 * ══════════════════════════════════════════════════════════════════════════════
 */

#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_mac.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ─── UART al RP2040 ───────────────────────────────────────────────────────────
#define RP_UART    Serial1
#define RP_TX_PIN  43
#define RP_RX_PIN  44
#define RP_BAUD    921600

// ─── Protocolo binario ────────────────────────────────────────────────────────
#define PKT_START            0xAA
#define PKT_END              0x55
#define PKT_TYPE_TELEM       0x01
#define PKT_TYPE_STATS       0x02
#define CMD_SET_THR          0x10
#define CMD_START_STREAM     0x20
#define CMD_STOP_STREAM      0x21
#define CMD_RECALIBRATE      0x30
#define CMD_SET_ANTIHELD_EN  0x41
#define CMD_SET_ANTIHELD_DB  0x42
#define CMD_SET_ANTIHELD_ST  0x43
#define CMD_SET_ANTIHELD_GR  0x44
#define CMD_RESET_STATS      0x50
#define TELEM_LEN            37
#define STATS_LEN            14
#define CMD_LEN               6



// ─── Forward declarations ─────────────────────────────────────────────────────
void setupHTTP(); void serveSplashPage(); void serveIosCaptive();
void serveConfigPage(); void redirectToPortal(); void addCORS();
void sendJSON(int code, String body);
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len);
void handleWsCmd(uint8_t num, String& json);
void sendWsData(); void sendWsStatus(uint8_t num); void sendWsStats(uint8_t num);
void broadcastThresholds(); void broadcastSensorMode(); void broadcastAntiheld();
void sendProfiles(uint8_t num); void saveThr(int a, int s, int v);
void loadNVS(); void initIdentity();
void wifiTask(void*); void uartRxTask(void*);
void sendCmdToRP2040(uint8_t cmd, uint8_t idx, uint16_t val);
void syncThresholdsToRP2040(); void syncAntiheldToRP2040();
void notifyStreamState(bool active);

// ─── WiFi ─────────────────────────────────────────────────────────────────────
bool wifiEnabled = true;
char AP_SSID[32] = "";
const char* AP_PASSWORD = "fsrpad123";
IPAddress AP_IP, AP_GW;
const IPAddress AP_SN(255,255,255,0);
DNSServer dnsServer; WebServer httpServer(80); WebSocketsServer wsServer(81);
Preferences prefs;

void initIdentity() {
  uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  String custom = prefs.getString("devname","");
  if (custom.length()) snprintf(AP_SSID,sizeof(AP_SSID),"%s",custom.c_str());
  else snprintf(AP_SSID,sizeof(AP_SSID),"FSR-Pad-%02X%02X",mac[4],mac[5]);
  AP_IP = IPAddress(192,168,4,1); AP_GW = IPAddress(192,168,4,1);
}

// ─── Estado compartido (RP2040 → ESP32) ──────────────────────────────────────
SemaphoreHandle_t dataMutex;
volatile uint16_t sm[4][4] = {};
volatile bool     fa[4][4] = {};
volatile bool     btn[4]   = {};

// ─── Stats (recibidas del RP2040 via STATS packet) ────────────────────────────
struct PadStats {
  uint16_t maxJitterUs   = 0;
  uint16_t missedAlarms  = 0;
  uint32_t totalCycles   = 0;
  uint16_t antiheldRetrig= 0;
  uint32_t lastUpdateMs  = 0;
};
static PadStats gStats;

// ─── Configuración ────────────────────────────────────────────────────────────
int     THR[4][4] = {{100,100,100,100},{100,100,100,100},{100,100,100,100},{100,100,100,100}};
uint8_t sensorMode      = 0;
bool     antiheldEnable    = true;
uint16_t antiheldDebounceMs = 80;
uint16_t antiheldSettleMs   = 110;
uint16_t antiheldGrowthRetrigMs = 180;
char    activeProfile[32] = "";

static volatile bool gStreamActive = false;
static int           gPrevClients  = -1;
const uint32_t WS_MS = 50;
TaskHandle_t wifiTaskHandle = nullptr, uartRxTaskHandle = nullptr;

// =============================================================================
//  NVS
// =============================================================================
void saveThr(int a, int s, int v) { char k[7]; snprintf(k,7,"t%d%d",a,s); prefs.putInt(k,v); }
void loadNVS() {
  for (int a=0;a<4;a++) for (int s=0;s<4;s++) {
    char k[7]; snprintf(k,7,"t%d%d",a,s); THR[a][s]=prefs.getInt(k,100);
  }
  String ap=prefs.getString("activeprofile","");
  if (ap.length()) snprintf(activeProfile,sizeof(activeProfile),"%s",ap.c_str());
  sensorMode          = prefs.getUChar("sensormode",0);
  antiheldEnable      = prefs.getBool("ah_en",true);
  antiheldDebounceMs     = (uint16_t)prefs.getUInt("ah_db",80);
  antiheldSettleMs       = (uint16_t)prefs.getUInt("ah_st",110);
  antiheldGrowthRetrigMs = (uint16_t)prefs.getUInt("ah_gr",180);
}

// =============================================================================
//  ENVÍO BINARIO AL RP2040
// =============================================================================
void sendCmdToRP2040(uint8_t cmd, uint8_t idx, uint16_t val) {
  uint8_t pkt[CMD_LEN] = {PKT_START, cmd, idx, (uint8_t)(val>>8), (uint8_t)(val&0xFF), PKT_END};
  RP_UART.write(pkt, CMD_LEN);
}
void syncThresholdsToRP2040() {
  for (int a=0;a<4;a++) for (int s=0;s<4;s++) {
    sendCmdToRP2040(CMD_SET_THR,(uint8_t)(a*4+s),(uint16_t)THR[a][s]);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
void syncAntiheldToRP2040() {
  sendCmdToRP2040(CMD_SET_ANTIHELD_EN, 0xFF, antiheldEnable ? 1 : 0);
  vTaskDelay(pdMS_TO_TICKS(1));
  sendCmdToRP2040(CMD_SET_ANTIHELD_DB, 0xFF, antiheldDebounceMs);
  vTaskDelay(pdMS_TO_TICKS(1));
  sendCmdToRP2040(CMD_SET_ANTIHELD_ST, 0xFF, antiheldSettleMs);
  vTaskDelay(pdMS_TO_TICKS(1));
  sendCmdToRP2040(CMD_SET_ANTIHELD_GR, 0xFF, antiheldGrowthRetrigMs);
}
void notifyStreamState(bool active) {
  if (gStreamActive == active) return;
  gStreamActive = active;
  sendCmdToRP2040(active ? CMD_START_STREAM : CMD_STOP_STREAM, 0xFF, 0);
}

// =============================================================================
//  UART RX — parser binario variable-length (sin JSON, sin malloc)
// =============================================================================
static uint8_t uartRxBuf[TELEM_LEN];  // mayor de los dos
static uint8_t uartRxIdx  = 0;
static uint8_t uartExpLen = 0;

static void processTelemetryFrame(const uint8_t* f) {
  if (f[TELEM_LEN-1] != PKT_END) return;
  uint8_t xc = 0; for (int i=1;i<TELEM_LEN-1;i++) xc^=f[i]; if (xc) return;
  if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(2))!=pdTRUE) return;
  uint8_t bm = f[34];
  for (int a=0;a<4;a++) {
    btn[a] = (bm>>a)&1;
    for (int s=0;s<4;s++) {
      uint8_t si=(uint8_t)(a*4+s), off=(uint8_t)(2+si*2);
      uint16_t v=((uint16_t)f[off]<<8)|f[off+1];
      sm[a][s]=v; fa[a][s]=(v>=(uint16_t)THR[a][s]);
    }
  }
  xSemaphoreGive(dataMutex);
}

static void processStatsFrame(const uint8_t* f) {
  if (f[STATS_LEN-1] != PKT_END) return;
  uint8_t xc=0; for (int i=1;i<STATS_LEN-1;i++) xc^=f[i]; if (xc) return;
  gStats.maxJitterUs   = ((uint16_t)f[2]<<8)|f[3];
  gStats.missedAlarms  = ((uint16_t)f[4]<<8)|f[5];
  gStats.totalCycles   = ((uint32_t)f[6]<<24)|((uint32_t)f[7]<<16)|((uint32_t)f[8]<<8)|f[9];
  gStats.antiheldRetrig= ((uint16_t)f[10]<<8)|f[11];
  gStats.lastUpdateMs  = (uint32_t)millis();
  // Broadcast stats a todos los clientes WebSocket
  StaticJsonDocument<128> doc;
  doc["t"]        = "stats";
  doc["jitter"]   = gStats.maxJitterUs;
  doc["missed"]   = gStats.missedAlarms;
  doc["cycles"]   = gStats.totalCycles;
  doc["retrig"]   = gStats.antiheldRetrig;
  doc["uptime_s"] = gStats.totalCycles / 1000;
  String msg; serializeJson(doc, msg);
  wsServer.broadcastTXT(msg);
}

void uartRxTask(void*) {
  for (;;) {
    while (RP_UART.available()) {
      uint8_t b = (uint8_t)RP_UART.read();
      if (uartRxIdx == 0) {
        if (b == PKT_START) { uartRxBuf[0]=b; uartRxIdx=1; uartExpLen=0; }
      } else if (uartRxIdx == 1) {
        uartRxBuf[1] = b; uartRxIdx = 2;
        if      (b == PKT_TYPE_TELEM) uartExpLen = TELEM_LEN;
        else if (b == PKT_TYPE_STATS) uartExpLen = STATS_LEN;
        else    uartRxIdx = 0;  // tipo desconocido → resincronizar
      } else {
        uartRxBuf[uartRxIdx++] = b;
        if (uartExpLen > 0 && uartRxIdx == uartExpLen) {
          uartRxIdx = 0;
          if      (uartRxBuf[1]==PKT_TYPE_TELEM) processTelemetryFrame(uartRxBuf);
          else if (uartRxBuf[1]==PKT_TYPE_STATS)  processStatsFrame(uartRxBuf);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// =============================================================================
//  HTML: Splash
// =============================================================================
const char SPLASH_HTML[] PROGMEM = R"SPLASH(
<!DOCTYPE html><html lang="es"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>FSR Pad</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{min-height:100dvh;background:#080810;display:flex;flex-direction:column;align-items:center;justify-content:center;font-family:system-ui,sans-serif;padding:24px;text-align:center}
.logo{font-size:48px;margin-bottom:8px;filter:drop-shadow(0 0 20px #a855f7)}
.brand{font-size:28px;font-weight:900;color:#fff;letter-spacing:4px;margin-bottom:4px}
.sub{font-size:12px;color:#ffffff55;letter-spacing:2px;margin-bottom:36px}
.card{background:#0f0f1a;border:1px solid #ffffff14;border-radius:20px;padding:28px 24px;max-width:340px;width:100%;margin-bottom:24px}
.wifi-icon{font-size:36px;margin-bottom:12px}
.msg{font-size:16px;color:#e8e8f0;line-height:1.55;margin-bottom:20px}
.url-chip{display:inline-block;background:#7c3aed22;border:1.5px solid #a855f7;border-radius:12px;padding:10px 22px;font-size:20px;font-family:monospace;font-weight:800;color:#c084fc;letter-spacing:1px;margin-bottom:20px;cursor:pointer}
.open-btn{display:block;width:100%;background:linear-gradient(135deg,#7c3aed,#a855f7);border:none;border-radius:14px;color:#fff;font-size:16px;font-weight:800;padding:16px;cursor:pointer;text-decoration:none}
.hint{font-size:11px;color:#ffffff33;margin-top:20px;line-height:1.6}
.pulse{animation:pulse 2s ease-in-out infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.5}}
</style></head><body>
<div class="logo">🎮</div><div class="brand">FSR PAD</div><div class="sub">DANCE PAD CONTROLLER</div>
<div class="card"><div class="wifi-icon">📶</div>
<div class="msg">¡Conectado!<br>Abre el panel en:</div>
<div class="url-chip" onclick="window.location='http://192.168.4.1/'">192.168.4.1</div>
<a class="open-btn" href="http://192.168.4.1/">Abrir configuración →</a></div>
<div class="hint pulse">Si no abre, escribe <b style="color:#a855f7;font-family:monospace">192.168.4.1</b> en tu navegador</div>
</body></html>
)SPLASH";

// =============================================================================
//  HTML: Config (CONFIG + TEST + MONITOR + PERFILES)
// =============================================================================
const char CONFIG_HTML[] PROGMEM = R"HTMLEOF(
<!DOCTYPE html><html lang="es"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>FSR Pad</title>
<style>
:root{--bg:#080810;--bg2:#0f0f1a;--bg3:#16162a;--border:#ffffff12;--text:#e8e8f0;--muted:#ffffff44;--accent:#a855f7}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{background:var(--bg);color:var(--text);font-family:system-ui,sans-serif;min-height:100dvh;max-width:680px;margin:0 auto;display:flex;flex-direction:column}
.header{display:flex;align-items:center;gap:10px;padding:13px 16px;border-bottom:1px solid var(--border);flex-shrink:0}
.dot{width:8px;height:8px;border-radius:50%;background:#f43f6e;flex-shrink:0;transition:background .3s}
.dot.ok{background:#22d3a5;box-shadow:0 0 8px #22d3a5}.dot.wait{background:#fbbf24}
.htitle{font-size:13px;font-weight:800;letter-spacing:2px;flex:1}
.hsub{font-size:10px;color:var(--muted);font-family:monospace;margin-top:1px}
.tabs{display:flex;border-bottom:1px solid var(--border);flex-shrink:0}
.tab{flex:1;padding:11px;text-align:center;font-size:10px;font-weight:700;letter-spacing:.5px;color:var(--muted);cursor:pointer;position:relative;transition:color .15s}
.tab.active{color:var(--accent)}.tab.active::after{content:'';position:absolute;bottom:0;left:50%;transform:translateX(-50%);width:24px;height:2px;background:var(--accent);border-radius:1px}
.content{flex:1;overflow-y:auto;padding:14px;display:flex;flex-direction:column;gap:11px}
.map-card{background:linear-gradient(180deg,#111426,#0c1020);border:1px solid var(--border);border-radius:12px;padding:12px;display:flex;flex-direction:column;gap:8px}
.map-title{font-size:10px;font-weight:800;letter-spacing:1.6px;color:var(--muted)}
.cfg-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
@media(max-width:640px){.cfg-grid{grid-template-columns:1fr}}
.acard{background:var(--bg2);border:1px solid var(--border);border-radius:12px;padding:13px;transition:border-color .15s}
.acard.active{border-color:var(--ac,#38bdf8)}
.ahead{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px}
.atitle{display:flex;align-items:center;gap:8px}
.adot{width:9px;height:9px;border-radius:50%;background:#ffffff22;transition:background .1s,box-shadow .1s;flex-shrink:0}
.adot.on{box-shadow:0 0 8px var(--ac,#38bdf8)}
.aname{font-size:15px;font-weight:800;color:#ffffff88;transition:color .1s}
.astatus{font-size:12px;font-family:monospace;font-weight:700;color:#ffffff22;transition:color .1s}
.sensor-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:6px}
.srow{display:flex;align-items:center;gap:8px;background:var(--bg3);border:1px solid #ffffff10;border-radius:10px;padding:8px}
.slabel{font-size:10px;font-family:monospace;color:#ffffff44;width:26px;flex-shrink:0}
input[type=range]{flex:1;height:32px;-webkit-appearance:none;appearance:none;background:transparent;cursor:pointer;touch-action:none}
input[type=range]::-webkit-slider-runnable-track{height:4px;border-radius:2px;background:#ffffff12}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:22px;height:22px;border-radius:50%;background:var(--tc,#38bdf8);margin-top:-9px;box-shadow:0 0 6px var(--tc,#38bdf8)66;transition:transform .1s}
.sval{font-size:13px;font-family:monospace;font-weight:800;width:44px;text-align:right;flex-shrink:0}
.fsr-bar-wrap{position:relative;height:5px;border-radius:3px;background:#ffffff0f;margin:1px 0 4px 0}
.fsr-bar-fill{height:100%;border-radius:3px;background:var(--tc,#38bdf8);width:0%;transition:width .04s linear}
.fsr-bar-thr{position:absolute;top:-3px;width:2px;height:11px;border-radius:1px;background:#ffffff55;pointer-events:none}
.pbadge{display:flex;align-items:center;gap:8px;background:#7c3aed18;border:1px solid #7c3aed44;border-radius:10px;padding:9px 12px}
.pbadge-dot{width:7px;height:7px;border-radius:50%;background:var(--accent);flex-shrink:0}
.pbadge-lbl{font-size:9px;font-weight:700;color:var(--accent);letter-spacing:1.5px;flex-shrink:0}
.pbadge-name{font-size:14px;font-weight:800;flex:1}
.pbadge-none{padding:8px 12px;background:#ffffff08;border-radius:10px;border:1px solid #ffffff10;font-size:11px;color:var(--muted);font-family:monospace}
.save-row{display:flex;gap:8px}
.ninput{flex:1;background:var(--bg2);border:1px solid #ffffff20;border-radius:10px;padding:11px;color:var(--text);font-size:14px;outline:none}
.ninput:focus{border-color:#ffffff44}
.sbtn{background:var(--accent);border:none;border-radius:10px;color:#fff;font-weight:700;font-size:13px;padding:0 18px;cursor:pointer}
.sbtn:disabled{opacity:.4}
.prow{background:var(--bg2);border:1px solid var(--border);border-radius:11px;padding:12px 14px;display:flex;align-items:center;gap:10px}
.prow.ap{border-color:#7c3aed66;background:#7c3aed0a}
.pinfo{flex:1;min-width:0}
.pname{font-size:15px;font-weight:700;display:flex;align-items:center;gap:6px;flex-wrap:wrap}
.pthr{font-size:9px;font-family:monospace;color:var(--muted);margin-top:2px}
.pill{border-radius:5px;padding:1px 6px;font-size:9px;font-weight:800;letter-spacing:.5px;border:1px solid}
.pa{background:#7c3aed33;border-color:var(--accent);color:var(--accent)}
.pacts{display:flex;gap:6px;flex-shrink:0}
.lbtn{background:#7c3aed22;border:1px solid #7c3aed66;border-radius:7px;color:#a855f7;font-size:12px;font-weight:700;padding:5px 12px;cursor:pointer}
.lbtn:disabled{opacity:.35;cursor:default}
.dbtn{background:#f43f6e15;border:1px solid #f43f6e33;border-radius:7px;color:#f43f6e;font-size:13px;width:30px;height:30px;cursor:pointer;display:flex;align-items:center;justify-content:center}
.empty{text-align:center;padding:32px;color:var(--muted);font-size:14px}
.toast{position:fixed;bottom:24px;left:50%;transform:translateX(-50%) translateY(20px);background:var(--bg3);border:1px solid var(--border);border-radius:20px;padding:8px 20px;font-size:12px;font-family:monospace;opacity:0;transition:opacity .2s,transform .2s;pointer-events:none;white-space:nowrap;z-index:99}
.toast.show{opacity:1;transform:translateX(-50%) translateY(0)}
.test-pad-wrap{display:flex;flex-direction:column;align-items:center;gap:0;padding:8px 0}
.test-pad-row{display:flex;gap:0;justify-content:center}
.test-cell{width:110px;height:110px;position:relative;display:flex;align-items:center;justify-content:center}
.test-cell-empty{width:110px;height:110px}
.tap{width:96px;height:96px;position:relative;border-radius:16px;background:#0d0d1c;display:flex;align-items:center;justify-content:center;overflow:visible;transition:background .1s}
.tap.firing{background:#111828}
.tap-edge{position:absolute;border-radius:3px;background:#ffffff10;transition:background .08s,box-shadow .08s;display:flex;align-items:center;justify-content:center;overflow:visible}
.tap-edge.top{top:0;left:8px;right:8px;height:5px}.tap-edge.right{top:8px;bottom:8px;right:0;width:5px}
.tap-edge.bottom{bottom:0;left:8px;right:8px;height:5px}.tap-edge.left{top:8px;bottom:8px;left:0;width:5px}
.tap-edge.lit{background:var(--ac,#38bdf8);box-shadow:0 0 10px var(--ac,#38bdf8),0 0 22px var(--ac,#38bdf8)88}
.tap-edge-lbl{position:absolute;font-size:7px;font-family:monospace;font-weight:800;color:#ffffff33;letter-spacing:.3px;white-space:nowrap;pointer-events:none;transition:color .08s}
.tap-edge.top .tap-edge-lbl{top:-11px;left:50%;transform:translateX(-50%)}
.tap-edge.bottom .tap-edge-lbl{bottom:-11px;left:50%;transform:translateX(-50%)}
.tap-edge.left .tap-edge-lbl{left:-18px;top:50%;transform:translateY(-50%) rotate(-90deg)}
.tap-edge.right .tap-edge-lbl{right:-18px;top:50%;transform:translateY(-50%) rotate(90deg)}
.tap-edge.lit .tap-edge-lbl{color:var(--ac,#38bdf8)}
.tap-icon{width:52px;height:52px;opacity:.25;transition:opacity .1s,filter .1s}
.tap.firing .tap-icon{opacity:1;filter:drop-shadow(0 0 10px var(--ac,#38bdf8))}
.tap-vals{position:absolute;bottom:6px;left:0;right:0;display:flex;justify-content:center;gap:3px;flex-wrap:wrap;padding:0 4px}
.tap-val{font-size:7px;font-family:monospace;font-weight:800;color:#ffffff22;background:#ffffff08;border-radius:4px;padding:1px 3px;min-width:24px;text-align:center;transition:color .08s,background .08s;line-height:1.3}
.tap-val.lit{color:var(--ac,#38bdf8);background:color-mix(in srgb,var(--ac,#38bdf8) 20%,transparent)}
.test-hint{text-align:center;font-size:11px;color:var(--muted);font-family:monospace;padding:4px}
.mode-row{display:flex;gap:8px;margin-bottom:2px}
.mbtn{flex:1;border:1.5px solid #ffffff18;border-radius:10px;padding:10px 6px;background:var(--bg3);font-size:12px;font-weight:700;color:#ffffff55;cursor:pointer;transition:all .15s;text-align:center}
.mbtn.on{border-color:var(--ac,#38bdf8);color:var(--ac,#38bdf8);background:color-mix(in srgb,var(--ac,#38bdf8) 12%,transparent)}
.wifi-btn{width:100%;border:1.5px solid #f43f6e55;border-radius:10px;padding:10px;background:#f43f6e10;font-size:12px;font-weight:700;color:#f43f6e;cursor:pointer;transition:all .15s;text-align:center;letter-spacing:.5px}
.wifi-btn.on{border-color:#22d3a555;color:#22d3a5;background:#22d3a510}
/* Monitor */
.stat-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.stat-box{background:var(--bg2);border:1px solid var(--border);border-radius:12px;padding:14px;text-align:center}
.stat-label{font-size:9px;font-weight:800;letter-spacing:1.5px;color:var(--muted);margin-bottom:6px}
.stat-value{font-size:26px;font-weight:900;font-family:monospace;transition:color .3s}
.stat-unit{font-size:10px;color:var(--muted);margin-top:2px}
.stat-ok{color:#22d3a5}.stat-warn{color:#fbbf24}.stat-err{color:#f43f6e}
.jitter-bar{height:8px;border-radius:4px;background:#ffffff0a;margin-top:8px;overflow:hidden}
.jitter-fill{height:100%;border-radius:4px;transition:width .5s,background .5s;width:0%}
.reset-btn{width:100%;border:1.5px solid #f43f6e55;border-radius:10px;padding:11px;background:#f43f6e08;font-size:12px;font-weight:700;color:#f43f6e77;cursor:pointer;transition:all .15s;letter-spacing:.5px}
.reset-btn:hover{border-color:#f43f6e88;color:#f43f6e;background:#f43f6e15}
.ah-row{display:flex;align-items:center;gap:10px}
.toggle-btn{flex:0 0 auto;border:1.5px solid #f59e0b55;border-radius:8px;padding:6px 14px;background:#f59e0b0a;font-size:11px;font-weight:800;color:#f59e0b55;cursor:pointer;transition:all .15s;letter-spacing:.5px}
.toggle-btn.on{border-color:#f59e0b;color:#f59e0b;background:#f59e0b15}
.ah-hint{font-size:10px;color:var(--muted);line-height:1.4;flex:1}
.preset-row{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:8px 0 10px}
.preset-btn{border:1.5px solid #ffffff18;border-radius:12px;padding:10px 8px;background:var(--bg3);cursor:pointer;transition:all .15s;text-align:left}
.preset-btn strong{display:block;font-size:12px;color:#ffffffcc;margin-bottom:4px}
.preset-btn span{display:block;font-size:10px;color:#ffffff55;line-height:1.35}
.preset-btn.active{border-color:#a855f7;background:#a855f714;box-shadow:0 0 0 1px #a855f733 inset}
.preset-btn.active strong{color:#c084fc}
.preset-note{font-size:10px;color:#ffffff66;line-height:1.45;margin-top:2px}
.ah-summary{background:#0b1220;border:1px solid #1d4ed833;border-radius:12px;padding:10px 12px;margin:4px 0 10px}
.ah-summary-title{font-size:10px;font-weight:800;letter-spacing:1px;color:#93c5fd;margin-bottom:4px}
.ah-summary-text{font-size:11px;color:#dbeafe;line-height:1.45}
.adv-toggle{width:100%;border:1.5px solid #ffffff18;border-radius:10px;padding:10px;background:#ffffff06;font-size:11px;font-weight:800;color:#ffffff88;cursor:pointer;transition:all .15s;letter-spacing:.4px}
.adv-toggle.on{border-color:#22d3a555;color:#22d3a5;background:#22d3a510}
.adv-box{display:none;margin-top:10px;padding-top:10px;border-top:1px dashed #ffffff14}
.adv-box.open{display:block}
</style></head><body>
<div class="header">
  <div class="dot wait" id="dot"></div>
  <div style="flex:1">
    <div class="htitle">PAD CONTROLLER <span style="font-size:10px;font-weight:400;color:var(--muted)" id="hmode">FSR</span></div>
    <div class="hsub" id="hsub">4 grupos · 4 sensores · 16 canales</div>
  </div>
  <button class="wifi-btn" id="wifi-btn" onclick="toggleWifi()" style="width:auto;padding:6px 14px;font-size:11px">WiFi ON</button>
</div>
<div class="tabs">
  <div class="tab active" onclick="showTab('config')"   id="tab-config">CONFIG</div>
  <div class="tab"        onclick="showTab('test')"     id="tab-test">TEST</div>
  <div class="tab"        onclick="showTab('monitor')"  id="tab-monitor">MONITOR</div>
  <div class="tab"        onclick="showTab('profiles')" id="tab-profiles">PERFILES</div>
</div>

<!-- CONFIG TAB -->
<div class="content" id="tab-config-c">
  <div class="map-card">
    <div class="map-title">MODO SENSOR</div>
    <div class="mode-row">
      <button class="mbtn on" id="mbtn-fsr" style="--ac:#a855f7" onclick="setSensorMode(0)">⚡ FSR</button>
      <button class="mbtn"    id="mbtn-dig" style="--ac:#38bdf8" onclick="setSensorMode(1)">◉ Binario</button>
      <button class="mbtn"    id="mbtn-swi" style="--ac:#22d3a5" onclick="setSensorMode(2)">🚀 CONTACTO</button>
    </div>
    <div style="font-size:10px;color:#ffffff44;font-family:monospace;padding:2px 0" id="mode-hint">EMA activo · threshold con histéresis</div>
  </div>
  <!-- ANTI HELD MISS – AJUSTE MANUAL -->
  <div class="map-card">
    <div class="map-title">ANTI-HELD MISS</div>
    <div class="ah-row" style="margin-bottom:12px">
      <button class="toggle-btn on" id="ah-btn" onclick="toggleAntiheld()">⚡ Activado</button>
      <span class="ah-hint">Evita que una pisada con varios sensores active la flecha dos veces. Ajusta los ms según tu técnica.</span>
    </div>
    <div class="srow" style="margin-bottom:6px">
      <span class="slabel" style="color:#22d3a588;width:70px;font-size:9px">SETTLE<br><span style="color:#ffffff33">absorbe pisada</span></span>
      <input type="range" id="ah-st" min="0" max="500" step="5" value="110"
        style="--tc:#22d3a5;flex:1"
        oninput="document.getElementById('ah-st-val').textContent=this.value+' ms'"
        onchange="setAntiheldSettle(+this.value)">
      <span class="sval" id="ah-st-val" style="color:#22d3a5">110 ms</span>
    </div>
    <div class="srow" style="margin-bottom:6px">
      <span class="slabel" style="color:#f59e0b88;width:70px;font-size:9px">DEBOUNCE<br><span style="color:#ffffff33">entre re-triggers</span></span>
      <input type="range" id="ah-db" min="0" max="500" step="5" value="80"
        style="--tc:#f59e0b;flex:1"
        oninput="document.getElementById('ah-db-val').textContent=this.value+' ms'"
        onchange="setAntiheldDebounce(+this.value)">
      <span class="sval" id="ah-db-val" style="color:#f59e0b">80 ms</span>
    </div>
    <div class="srow" style="margin-bottom:6px">
      <span class="slabel" style="color:#c084fc88;width:70px;font-size:9px">GROWTH<br><span style="color:#ffffff33">huella creciente</span></span>
      <input type="range" id="ah-gr" min="0" max="500" step="5" value="180"
        style="--tc:#c084fc;flex:1"
        oninput="document.getElementById('ah-gr-val').textContent=this.value+' ms'"
        onchange="setAntiheldGrowth(+this.value)">
      <span class="sval" id="ah-gr-val" style="color:#c084fc">180 ms</span>
    </div>
    <div style="font-size:9px;color:#ffffff33;font-family:monospace;line-height:1.7;padding:4px 0">
      SETTLE = ventana donde se absorbe la pisada gradual (sube si hay falsos positivos con 1 pie)<br>
      DEBOUNCE = tiempo mínimo entre dos presses válidos distintos<br>
      GROWTH = retardo para re-trigger cuando la huella solo crece sin soltar sensores
    </div>
  </div>
  <div id="pb-config"></div>
  <div id="arrows-list" class="cfg-grid"></div>
</div>

<!-- TEST TAB -->
<div class="content" id="tab-test-c" style="display:none">
  <div class="map-card">
    <div class="map-title">TEST EN TIEMPO REAL</div>
    <div style="font-size:12px;color:#ffffffb0;line-height:1.45">Los 4 bordes de cada flecha = 4 FSR. Se iluminan al superar el threshold.</div>
  </div>
  <div class="test-pad-wrap" id="test-pad"></div>
  <div class="test-hint">Cada borde = 1 sensor · etiqueta indica canal MCP3008</div>
</div>

<!-- MONITOR TAB -->
<div class="content" id="tab-monitor-c" style="display:none">
  <div class="map-card">
    <div class="map-title">TIMING MONITOR — RP2040</div>
    <div style="font-size:10px;color:#ffffff55;line-height:1.5">Actualiza cada ~1 s. <b style="color:#22d3a5">Verde</b> &lt;50µs · <b style="color:#fbbf24">Amarillo</b> 50-100µs · <b style="color:#f43f6e">Rojo</b> &gt;100µs</div>
  </div>
  <div class="stat-grid">
    <div class="stat-box">
      <div class="stat-label">MAX JITTER</div>
      <div class="stat-value stat-ok" id="m-jitter">—</div>
      <div class="stat-unit">microsegundos</div>
      <div class="jitter-bar"><div class="jitter-fill" id="m-jbar" style="background:#22d3a5"></div></div>
    </div>
    <div class="stat-box">
      <div class="stat-label">MISSED ALARMS</div>
      <div class="stat-value stat-ok" id="m-missed">—</div>
      <div class="stat-unit">slots perdidos (debe ser 0)</div>
    </div>
    <div class="stat-box">
      <div class="stat-label">RUNTIME</div>
      <div class="stat-value" style="color:#a855f7" id="m-uptime">—</div>
      <div class="stat-unit">segundos activo</div>
    </div>
    <div class="stat-box">
      <div class="stat-label">ANTI-HELD RETRIG</div>
      <div class="stat-value" style="color:#f59e0b" id="m-retrig">—</div>
      <div class="stat-unit">re-triggers totales</div>
    </div>
  </div>
  <div class="map-card" style="gap:6px">
    <div class="stat-label">EXPLICACIÓN DE OVERRUN</div>
    <div style="font-size:10px;color:#ffffff66;line-height:1.6">
      <b style="color:#f43f6e">Missed Alarms &gt; 0</b> = la ISR tardó más de 250µs → el RP2040 no pudo leer sensores a tiempo.<br>
      <b style="color:#fbbf24">Jitter &gt; 50µs</b> = el alarm se disparó tarde → lectura desplazada en el tiempo.<br>
      Ambos pueden causar que un press registre 1-2ms tarde. Si ves esto, reduce el baud UART o la frecuencia SPI.
    </div>
  </div>
  <div style="font-size:9px;color:#ffffff33;font-family:monospace;text-align:center;padding:4px" id="m-lastupdate">Sin datos aún</div>
  <button class="reset-btn" onclick="resetStats()">⟳ RESETEAR CONTADORES</button>
</div>

<!-- PERFILES TAB -->
<div class="content" id="tab-profiles-c" style="display:none">
  <div id="pb-profiles"></div>
  <div id="thr-summary" style="font-size:10px;color:var(--muted);font-family:monospace;padding:2px 0"></div>
  <div class="save-row">
    <input class="ninput" id="pname" placeholder="nombre del perfil" oninput="document.getElementById('save-btn').disabled=!this.value.trim()">
    <button class="sbtn" id="save-btn" onclick="saveProfile()" disabled>Guardar</button>
  </div>
  <div id="plist"></div>
</div>
<div class="toast" id="toast"></div>

<script>
const C4=['#38bdf8','#f43f6e','#4ade80','#e879f9'];
const NAMES4=['← Izquierda','↓ Abajo','↑ Arriba','→ Derecha'];
const SENSOR_LABELS=['S1(A1)','S2(A1)','S3(A2)','S4(A2)'];
let thr=[[100,100,100,100],[100,100,100,100],[100,100,100,100],[100,100,100,100]];
let btns=[false,false,false,false],faArr=[[false,false,false,false],[false,false,false,false],[false,false,false,false],[false,false,false,false]];
let liveD=[[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]];
let activeProfile='',profiles=[],curSensorMode=0,curWifi=true;
let curAhEnable=true,curAhDebounce=80,curAhSettle=110,curAhGrowth=180;
let ws;
function connect(){
  ws=new WebSocket(`ws://${location.hostname}:81`);
  ws.onopen=()=>{document.getElementById('dot').className='dot ok';document.getElementById('hsub').textContent='RP2040+ESP32-S3 · 2×MCP3008 · 16 FSR';curWifi=true;updateWifiBtn();};
  ws.onclose=()=>{document.getElementById('dot').className='dot wait';setTimeout(connect,1500);};
  ws.onmessage=e=>{try{handleMsg(JSON.parse(e.data));}catch(x){}};
}
function sendJSON(o){if(ws&&ws.readyState===1)ws.send(JSON.stringify(o));}
function handleMsg(d){
  if(d.t==='status'||d.t==='thresholds'){
    if(d.profile!==undefined)activeProfile=d.profile??'';
    if(d.thresholds)thr=d.thresholds.map(r=>r.map(Number));
    if(d.sensormode!==undefined){curSensorMode=d.sensormode;updateModeUI();}
    if(d.ah_enable!==undefined){curAhEnable=d.ah_enable;updateAhUI();}
    if(d.ah_debounce!==undefined){curAhDebounce=d.ah_debounce;updateAhUI();}
    if(d.ah_settle!==undefined){curAhSettle=d.ah_settle;updateAhUI();}
    if(d.ah_growth!==undefined){curAhGrowth=d.ah_growth;updateAhUI();}
    rebuildConfig();updateBadge();updateThrSummary();buildTestGrid();
  }else if(d.t==='s'){
    if(d.profile!==undefined)activeProfile=d.profile??'';
    btns=d.b??btns;faArr=d.fa??faArr;liveD=d.d??liveD;
    updateConfigStates();updateTestStates();
  }else if(d.t==='profiles'){
    profiles=d.profiles??[];activeProfile=d.active??'';renderProfiles();updateBadge();
  }else if(d.t==='sensormode'){
    curSensorMode=d.sensormode??0;updateModeUI();
  }else if(d.t==='antiheld'){
    curAhEnable=d.enable??true;curAhDebounce=d.debounce??80;curAhSettle=d.settle??110;curAhGrowth=d.growth??180;updateAhUI();
  }else if(d.t==='wifi'){
    curWifi=d.on??true;updateWifiBtn();
  }else if(d.t==='stats'){
    updateMonitor(d);
  }
}
// ── Anti-Held Miss ───────────────────────────────────────────────────────────
function sendAntiheld(){sendJSON({t:'set_antiheld',enable:curAhEnable,debounce:curAhDebounce,settle:curAhSettle,growth:curAhGrowth});}
function toggleAntiheld(){curAhEnable=!curAhEnable;sendAntiheld();updateAhUI();}
function setAntiheldDebounce(v){curAhDebounce=v;sendAntiheld();updateAhUI();}
function setAntiheldSettle(v){curAhSettle=v;sendAntiheld();updateAhUI();}
function setAntiheldGrowth(v){curAhGrowth=v;sendAntiheld();updateAhUI();}
function updateAhUI(){
  const btn=document.getElementById('ah-btn');
  if(btn){btn.textContent=curAhEnable?'⚡ Activado':'○ Desactivado';btn.classList.toggle('on',curAhEnable);}
  const slDb=document.getElementById('ah-db');
  if(slDb){slDb.value=curAhDebounce;document.getElementById('ah-db-val').textContent=curAhDebounce+' ms';}
  const slSt=document.getElementById('ah-st');
  if(slSt){slSt.value=curAhSettle;document.getElementById('ah-st-val').textContent=curAhSettle+' ms';}
  const slGr=document.getElementById('ah-gr');
  if(slGr){slGr.value=curAhGrowth;document.getElementById('ah-gr-val').textContent=curAhGrowth+' ms';}
}
// ── Monitor ──────────────────────────────────────────────────────────────────
function updateMonitor(d){
  const jit=d.jitter??0,mis=d.missed??0,up=d.uptime_s??0,ret=d.retrig??0;
  const jEl=document.getElementById('m-jitter');
  const jClass=jit<50?'stat-ok':(jit<100?'stat-warn':'stat-err');
  jEl.textContent=jit; jEl.className='stat-value '+jClass;
  const jFill=document.getElementById('m-jbar');
  const pct=Math.min(100,jit/2); // 200µs = 100%
  jFill.style.width=pct+'%';
  jFill.style.background=jit<50?'#22d3a5':(jit<100?'#fbbf24':'#f43f6e');
  const mEl=document.getElementById('m-missed');
  mEl.textContent=mis;mEl.className='stat-value '+(mis===0?'stat-ok':'stat-err');
  document.getElementById('m-uptime').textContent=up;
  document.getElementById('m-retrig').textContent=ret;
  const now=new Date();
  document.getElementById('m-lastupdate').textContent='Última actualización: '+now.toLocaleTimeString();
}
function resetStats(){sendJSON({t:'reset_stats'});toast('Contadores reseteados');}
// ── Sensor mode ──────────────────────────────────────────────────────────────
function setSensorMode(m){sendJSON({t:'set_sensor_mode',sensormode:m});}
function toggleWifi(){sendJSON({t:'wifi_toggle'});}
function updateModeUI(){
  document.getElementById('mbtn-fsr').classList.toggle('on',curSensorMode===0);
  document.getElementById('mbtn-dig').classList.toggle('on',curSensorMode===1);
  document.getElementById('mbtn-swi').classList.toggle('on',curSensorMode===2);
  document.getElementById('hmode').textContent=curSensorMode===0?'FSR':(curSensorMode===1?'BINARIO':'CONTACTO');
  const hints=['Analógico: HMA activo · threshold con histéresis','Binario: Lectura ADC cruda','Contacto: Digital puro'];
  document.getElementById('mode-hint').textContent=hints[curSensorMode]||'';
}
function updateWifiBtn(){const b=document.getElementById('wifi-btn');b.textContent=curWifi?'WiFi ON':'WiFi OFF';b.classList.toggle('on',curWifi);}
function showTab(t){
  ['config','test','monitor','profiles'].forEach(id=>{
    document.getElementById('tab-'+id+'-c').style.display=id===t?'flex':'none';
    document.getElementById('tab-'+id).classList.toggle('active',id===t);
  });
  if(t==='profiles')sendJSON({t:'get_profiles'});
}
function rebuildConfig(){const list=document.getElementById('arrows-list');list.innerHTML='';for(let i=0;i<4;i++)list.appendChild(buildArrowCard(i));}
function buildArrowCard(i){
  const c=C4[i];const d=document.createElement('div');
  d.className='acard';d.id='card-'+i;d.style.setProperty('--ac',c);
  d.innerHTML=`<div class="ahead"><div class="atitle"><div class="adot" id="adot-${i}"></div><span class="aname" id="aname-${i}">${NAMES4[i]}</span></div><span class="astatus" id="ast-${i}">○</span></div><div class="sensor-grid" id="sensor-grid-${i}">`;
  for(let s=0;s<4;s++){
    const id=`r-${i}-${s}`,vid=`v-${i}-${s}`,bid=`bar-${i}-${s}`,tid=`thr-${i}-${s}`,val=thr[i][s]??100;
    d.innerHTML+=`<div class="srow"><span class="slabel" style="color:${c}88">${SENSOR_LABELS[s]}</span><div style="flex:1;display:flex;flex-direction:column;gap:0"><div style="display:flex;align-items:center;gap:6px"><input type="range" id="${id}" min="0" max="1023" step="1" value="${val}" style="--tc:${c};flex:1" oninput="document.getElementById('${vid}').textContent=this.value;document.getElementById('${tid}').style.left=((+this.value/1023)*100)+'%'" onchange="onSlider(${i},${s},+this.value)"><span class="sval" id="${vid}" style="color:${c}">${val}</span></div><div class="fsr-bar-wrap" style="--tc:${c}"><div class="fsr-bar-fill" id="${bid}"></div><div class="fsr-bar-thr" id="${tid}" style="left:${(val/1023)*100}%"></div></div></div></div>`;
  }
  d.innerHTML+=`</div>`;return d;
}
function onSlider(i,s,v){thr[i][s]=v;sendJSON({t:'set_threshold',arrow:i,sensor:s,value:v});activeProfile='';updateBadge();}
function updateConfigStates(){
  for(let i=0;i<4;i++){
    const c=C4[i],p=btns[i]??false,dot=document.getElementById('adot-'+i);if(!dot)continue;
    dot.style.background=p?c:'#ffffff22';dot.classList.toggle('on',p);
    document.getElementById('aname-'+i).style.color=p?c:'#ffffff88';
    const st=document.getElementById('ast-'+i);st.textContent=p?'● PRESS':'○';st.style.color=p?c:'#ffffff22';
    const card=document.getElementById('card-'+i);card.classList.toggle('active',p);card.style.borderColor=p?c+'88':'';
    for(let s=0;s<4;s++){
      const raw=(liveD[i]??[])[s]??0,af=(faArr[i]??[])[s]??false,bar=document.getElementById(`bar-${i}-${s}`);
      if(bar){bar.style.width=(raw/10.23)+'%';bar.style.background=af?c:c+'55';}
    }
  }
}
const ARROW_SVGS=[
  `<svg class="tap-icon" viewBox="0 0 52 52" fill="none"><path d="M32 10L16 26L32 42" stroke="currentColor" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"/></svg>`,
  `<svg class="tap-icon" viewBox="0 0 52 52" fill="none"><path d="M10 20L26 36L42 20" stroke="currentColor" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"/></svg>`,
  `<svg class="tap-icon" viewBox="0 0 52 52" fill="none"><path d="M10 32L26 16L42 32" stroke="currentColor" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"/></svg>`,
  `<svg class="tap-icon" viewBox="0 0 52 52" fill="none"><path d="M20 10L36 26L20 42" stroke="currentColor" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"/></svg>`
];
const ADC_LABEL=['A1','A1','A2','A2'];
function makeTile(i){
  const c=C4[i],edgeCls=['top','right','bottom','left'];
  const div=document.createElement('div');div.className='test-cell';
  const tap=document.createElement('div');tap.className='tap';tap.id='tap-'+i;tap.style.setProperty('--ac',c);tap.style.color=c;
  const edges=edgeCls.map((cls,s)=>{
    const lbl=`${ADC_LABEL[s]}·CH${s<2?(i*2+s):(i*2+s-2)}`;
    return `<div class="tap-edge ${cls}" id="te-${i}-${s}"><span class="tap-edge-lbl">${lbl}</span></div>`;
  }).join('');
  const chips=[0,1,2,3].map(s=>`<span class="tap-val" id="tv-${i}-${s}"><span id="tvv-${i}-${s}">0</span></span>`).join('');
  tap.innerHTML=edges+ARROW_SVGS[i]+`<div class="tap-vals">${chips}</div>`;div.appendChild(tap);return div;
}
function buildTestGrid(){
  const pad=document.getElementById('test-pad');pad.innerHTML='';
  const r0=document.createElement('div');r0.className='test-pad-row';
  r0.appendChild(document.createElement('div')).className='test-cell-empty';
  r0.appendChild(makeTile(2));r0.appendChild(document.createElement('div')).className='test-cell-empty';pad.appendChild(r0);
  const r1=document.createElement('div');r1.className='test-pad-row';
  r1.appendChild(makeTile(0));r1.appendChild(document.createElement('div')).className='test-cell-empty';r1.appendChild(makeTile(3));pad.appendChild(r1);
  const r2=document.createElement('div');r2.className='test-pad-row';
  r2.appendChild(document.createElement('div')).className='test-cell-empty';
  r2.appendChild(makeTile(1));r2.appendChild(document.createElement('div')).className='test-cell-empty';pad.appendChild(r2);
}
function updateTestStates(){
  for(let i=0;i<4;i++){
    const anyOn=btns[i]??false,tap=document.getElementById('tap-'+i);if(!tap)continue;
    tap.classList.toggle('firing',anyOn);
    for(let s=0;s<4;s++){
      const raw=(liveD[i]??[])[s]??0,af=(faArr[i]??[])[s]??false;
      const edge=document.getElementById(`te-${i}-${s}`),chip=document.getElementById(`tv-${i}-${s}`),val=document.getElementById(`tvv-${i}-${s}`);
      if(edge)edge.classList.toggle('lit',af);if(chip)chip.classList.toggle('lit',af);if(val)val.textContent=raw;
    }
  }
}
function updateBadge(){
  ['config','profiles'].forEach(t=>{
    const el=document.getElementById('pb-'+t);if(!el)return;
    el.innerHTML=activeProfile?`<div class="pbadge"><div class="pbadge-dot"></div><span class="pbadge-lbl">PERFIL ACTIVO</span><span class="pbadge-name">${activeProfile}</span></div>`:`<div class="pbadge-none">Sin perfil activo</div>`;
  });
}
function updateThrSummary(){const el=document.getElementById('thr-summary');if(!el)return;el.textContent='4 FSR por flecha · '+thr.map((r,i)=>`${NAMES4[i]}:[${r.join(',')}]`).join('  ');}
function saveProfile(){
  const name=document.getElementById('pname').value.trim();if(!name)return;
  sendJSON({t:'save_profile',name,thresholds:thr,sensormode:curSensorMode});
  activeProfile=name;updateBadge();document.getElementById('pname').value='';document.getElementById('save-btn').disabled=true;
  toast(`✓ Perfil "${name}" guardado`);setTimeout(()=>sendJSON({t:'get_profiles'}),300);
}
function renderProfiles(){
  const list=document.getElementById('plist');
  if(!profiles.length){list.innerHTML='<div class="empty">Sin perfiles guardados</div>';return;}
  list.innerHTML='';
  profiles.forEach(p=>{
    const isA=p.name===activeProfile,row=document.createElement('div');row.className='prow'+(isA?' ap':'');
    row.innerHTML=`<div class="pinfo"><div class="pname">${p.name} ${isA?'<span class="pill pa">ACTIVO</span>':''}</div><div class="pthr">${fmtThr(p)}</div></div><div class="pacts"><button class="lbtn" ${isA?'disabled':''} onclick="loadProfile('${p.name}')">${isA?'✓':'Cargar'}</button><button class="dbtn" onclick="delProfile('${p.name}')">✕</button></div>`;
    list.appendChild(row);
  });
}
function fmtThr(p){return !p.thresholds?'':p.thresholds.map((r,i)=>`A${i+1}:[${Array.isArray(r)?r.join(','):r}]`).join('  ');}
function loadProfile(name){sendJSON({t:'load_profile',name});toast(`Cargando "${name}"...`);}
function delProfile(name){
  if(!confirm(`¿Eliminar "${name}"?`))return;sendJSON({t:'delete_profile',name});
  if(activeProfile===name){activeProfile='';updateBadge();}setTimeout(()=>sendJSON({t:'get_profiles'}),300);
}
let toastT;
function toast(msg){const el=document.getElementById('toast');el.textContent=msg;el.classList.add('show');clearTimeout(toastT);toastT=setTimeout(()=>el.classList.remove('show'),2500);}
connect();buildTestGrid();updateModeUI();updateWifiBtn();updateAhUI();
</script></body></html>
)HTMLEOF";

// =============================================================================
//  WebSocket helpers
// =============================================================================
void sendWsData() {
  if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(2))!=pdTRUE) return;
  StaticJsonDocument<768> doc; doc["t"]="s"; doc["profile"]=strlen(activeProfile)?activeProfile:"";
  JsonArray d=doc.createNestedArray("d"),b=doc.createNestedArray("b"),fa_arr=doc.createNestedArray("fa");
  for(int i=0;i<4;i++){
    JsonArray pd=d.createNestedArray(),pf=fa_arr.createNestedArray();
    for(int s=0;s<4;s++){pd.add((int)sm[i][s]);pf.add(fa[i][s]?1:0);}
    b.add(btn[i]?1:0);
  }
  xSemaphoreGive(dataMutex);
  String msg; serializeJson(doc,msg); wsServer.broadcastTXT(msg);
}

void sendWsStatus(uint8_t num) {
  StaticJsonDocument<896> doc;
  doc["t"]="status"; doc["ip"]=AP_IP.toString(); doc["ssid"]=AP_SSID;
  doc["mode"]="4fsr_16ch"; doc["profile"]=strlen(activeProfile)?activeProfile:"";
  doc["sensormode"]=sensorMode;
  doc["ah_enable"]=antiheldEnable; doc["ah_debounce"]=antiheldDebounceMs; doc["ah_settle"]=antiheldSettleMs; doc["ah_growth"]=antiheldGrowthRetrigMs;
  JsonArray t=doc.createNestedArray("thresholds");
  for(int i=0;i<4;i++){JsonArray r=t.createNestedArray();for(int s=0;s<4;s++)r.add(THR[i][s]);}
  String msg; serializeJson(doc,msg); wsServer.sendTXT(num,msg);
}

void sendWsStats(uint8_t num) {
  StaticJsonDocument<128> doc;
  doc["t"]="stats"; doc["jitter"]=gStats.maxJitterUs; doc["missed"]=gStats.missedAlarms;
  doc["cycles"]=gStats.totalCycles; doc["retrig"]=gStats.antiheldRetrig;
  doc["uptime_s"]=gStats.totalCycles/1000;
  String msg; serializeJson(doc,msg); wsServer.sendTXT(num,msg);
}

void broadcastThresholds() {
  StaticJsonDocument<512> doc; doc["t"]="thresholds"; doc["profile"]=strlen(activeProfile)?activeProfile:"";
  JsonArray v=doc.createNestedArray("thresholds");
  for(int i=0;i<4;i++){JsonArray r=v.createNestedArray();for(int s=0;s<4;s++)r.add(THR[i][s]);}
  String msg; serializeJson(doc,msg); wsServer.broadcastTXT(msg);
}

void broadcastSensorMode() {
  StaticJsonDocument<64> doc; doc["t"]="sensormode"; doc["sensormode"]=sensorMode;
  String msg; serializeJson(doc,msg); wsServer.broadcastTXT(msg);
}

void broadcastAntiheld() {
  StaticJsonDocument<128> doc; doc["t"]="antiheld"; doc["enable"]=antiheldEnable; doc["debounce"]=antiheldDebounceMs; doc["settle"]=antiheldSettleMs; doc["growth"]=antiheldGrowthRetrigMs;
  String msg; serializeJson(doc,msg); wsServer.broadcastTXT(msg);
}

void sendProfiles(uint8_t num) {
  String names=prefs.getString("pnames","");
  StaticJsonDocument<2048> doc; doc["t"]="profiles"; doc["active"]=strlen(activeProfile)?activeProfile:"";
  JsonArray arr=doc.createNestedArray("profiles");
  int start=0;
  while(start<(int)names.length()){
    int end=names.indexOf(',',start);if(end<0)end=names.length();
    String name=names.substring(start,end),data=prefs.getString(("p_"+name).c_str(),"");
    if(data.length()){StaticJsonDocument<512> p;deserializeJson(p,data);JsonObject o=arr.createNestedObject();o["name"]=name;o["thresholds"]=p["thresholds"];}
    start=end+1;
  }
  String msg; serializeJson(doc,msg); wsServer.sendTXT(num,msg);
}

// =============================================================================
//  WebSocket handler
// =============================================================================
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  if (type==WStype_CONNECTED) { sendWsStatus(num); sendWsStats(num); }
  else if (type==WStype_TEXT) { String msg; msg.reserve(len); for(size_t i=0;i<len;i++)msg+=(char)payload[i]; handleWsCmd(num,msg); }
}

static String toast_str = "";


void handleWsCmd(uint8_t num, String& json) {
  StaticJsonDocument<512> doc; if(deserializeJson(doc,json))return;
  String t=doc["t"].as<String>();

  if (t=="set_threshold") {
    int a=doc["arrow"]|-1,s=doc["sensor"]|-1,v=doc["value"]|-1;
    if(a<0||a>3||s<0||s>3||v<0||v>1023)return;
    THR[a][s]=v; saveThr(a,s,v); activeProfile[0]='\0'; prefs.putString("activeprofile","");
    sendCmdToRP2040(CMD_SET_THR,(uint8_t)(a*4+s),(uint16_t)v);
    broadcastThresholds();

  } else if (t=="set_sensor_mode") {
    uint8_t m=doc["sensormode"]|(uint8_t)0; if(m>2)return;
    sensorMode=m; prefs.putUChar("sensormode",m); broadcastSensorMode();

  } else if (t=="set_antiheld") {
    antiheldEnable         = doc["enable"]|true;
    antiheldDebounceMs     = (uint16_t)(doc["debounce"]|(int)80);
    antiheldSettleMs       = (uint16_t)(doc["settle"]|(int)110);
    antiheldGrowthRetrigMs = (uint16_t)(doc["growth"]|(int)180);
    if (antiheldDebounceMs     > 500) antiheldDebounceMs     = 500;
    if (antiheldSettleMs       > 500) antiheldSettleMs       = 500;
    if (antiheldGrowthRetrigMs > 500) antiheldGrowthRetrigMs = 500;
    prefs.putBool("ah_en",antiheldEnable);
    prefs.putUInt("ah_db",(uint32_t)antiheldDebounceMs);
    prefs.putUInt("ah_st",(uint32_t)antiheldSettleMs);
    prefs.putUInt("ah_gr",(uint32_t)antiheldGrowthRetrigMs);
    syncAntiheldToRP2040();
    broadcastAntiheld();
    toast_str = String("AH DB=")+antiheldDebounceMs+"ms ST="+antiheldSettleMs+"ms GR="+antiheldGrowthRetrigMs+"ms";

  } else if (t=="reset_stats") {
    sendCmdToRP2040(CMD_RESET_STATS,0xFF,0);
    gStats = PadStats{};
    // Broadcast stats vacias
    StaticJsonDocument<64> r; r["t"]="stats"; r["jitter"]=0; r["missed"]=0; r["cycles"]=0; r["retrig"]=0; r["uptime_s"]=0;
    String msg; serializeJson(r,msg); wsServer.broadcastTXT(msg);

  } else if (t=="wifi_toggle") {
    wifiEnabled=!wifiEnabled;
    if(wifiEnabled){WiFi.softAP(AP_SSID,AP_PASSWORD);dnsServer.start(53,"*",AP_IP);StaticJsonDocument<32>r;r["t"]="wifi";r["on"]=true;String msg;serializeJson(r,msg);wsServer.broadcastTXT(msg);}
    else{StaticJsonDocument<32>r;r["t"]="wifi";r["on"]=false;String msg;serializeJson(r,msg);wsServer.broadcastTXT(msg);vTaskDelay(pdMS_TO_TICKS(100));dnsServer.stop();WiFi.softAPdisconnect(true);}

  } else if (t=="get_profiles") { sendProfiles(num);

  } else if (t=="save_profile") {
    String name=doc["name"].as<String>();name.replace(",","_");if(!name.length())return;
    String pData;serializeJson(doc,pData);prefs.putString(("p_"+name).c_str(),pData);
    String names=prefs.getString("pnames","");
    if(names.indexOf(name)<0){if(names.length())names+=",";names+=name;prefs.putString("pnames",names);}
    snprintf(activeProfile,sizeof(activeProfile),"%s",name.c_str());prefs.putString("activeprofile",name);sendProfiles(num);

  } else if (t=="load_profile") {
    String name=doc["name"].as<String>(),data=prefs.getString(("p_"+name).c_str(),"");if(!data.length())return;
    StaticJsonDocument<512>p;deserializeJson(p,data);
    if(p.containsKey("sensormode")){sensorMode=p["sensormode"].as<uint8_t>();prefs.putUChar("sensormode",sensorMode);broadcastSensorMode();}
    JsonArray th=p["thresholds"];
    for(int a=0;a<4&&a<(int)th.size();a++){JsonArray row=th[a];for(int s=0;s<4&&s<(int)row.size();s++){THR[a][s]=row[s].as<int>();saveThr(a,s,THR[a][s]);}}
    snprintf(activeProfile,sizeof(activeProfile),"%s",name.c_str());prefs.putString("activeprofile",name);
    syncThresholdsToRP2040();broadcastThresholds();

  } else if (t=="delete_profile") {
    String name=doc["name"].as<String>();prefs.remove(("p_"+name).c_str());
    if(String(activeProfile)==name){activeProfile[0]='\0';prefs.putString("activeprofile","");}
    String names=prefs.getString("pnames","");int idx=names.indexOf(name);
    if(idx>=0){int end=idx+name.length();if(idx>0)idx--;else if(end<(int)names.length())end++;names=names.substring(0,idx)+names.substring(end);prefs.putString("pnames",names);}
    sendProfiles(num);
  }
}



// =============================================================================
//  HTTP
// =============================================================================
void addCORS(){httpServer.sendHeader("Access-Control-Allow-Origin","*");httpServer.sendHeader("Access-Control-Allow-Methods","GET,POST,OPTIONS");httpServer.sendHeader("Access-Control-Allow-Headers","Content-Type");}
void sendJSON(int code,String body){httpServer.send(code,"application/json",body);}
void serveConfigPage(){httpServer.send_P(200,"text/html",CONFIG_HTML);}
void serveSplashPage(){httpServer.send_P(200,"text/html",SPLASH_HTML);}
void redirectToPortal(){httpServer.sendHeader("Location","http://192.168.4.1/splash",true);httpServer.send(302,"text/plain","");}
void serveIosCaptive(){httpServer.send(200,"text/html",F("<HTML><HEAD><meta http-equiv='refresh' content='0;url=http://192.168.4.1/splash'></HEAD><BODY></BODY></HTML>"));}

void setupHTTP() {
  httpServer.on("/",HTTP_GET,serveConfigPage);httpServer.on("/index.html",HTTP_GET,serveConfigPage);
  httpServer.on("/splash",HTTP_GET,serveSplashPage);
  httpServer.on("/hotspot-detect.html",HTTP_GET,[](){serveIosCaptive();});
  httpServer.on("/library/test/success.html",HTTP_GET,[](){serveIosCaptive();});
  httpServer.on("/bag",HTTP_GET,[](){serveIosCaptive();});
  httpServer.on("/generate_204",HTTP_GET,[](){redirectToPortal();});httpServer.on("/gen_204",HTTP_GET,[](){redirectToPortal();});
  httpServer.on("/mobile/status.php",HTTP_GET,[](){redirectToPortal();});httpServer.on("/connecttest.txt",HTTP_GET,[](){redirectToPortal();});
  httpServer.on("/ncsi.txt",HTTP_GET,[](){redirectToPortal();});httpServer.on("/redirect",HTTP_GET,[](){redirectToPortal();});
  httpServer.onNotFound([](){addCORS();if(httpServer.method()==HTTP_OPTIONS){httpServer.send(204);return;}redirectToPortal();});
  httpServer.on("/status",HTTP_GET,[](){
    addCORS();StaticJsonDocument<700>doc;
    doc["firmware"]="rp2040-esp32s3-v3.0";doc["ip"]="192.168.4.1";doc["mode"]="4fsr_16ch_binary";
    doc["profile"]=strlen(activeProfile)?activeProfile:"";doc["sensormode"]=sensorMode;
    doc["wifi_clients"]=WiFi.softAPgetStationNum();doc["stream_active"]=gStreamActive;
    doc["heap_free"]=ESP.getFreeHeap();doc["ah_enable"]=antiheldEnable;doc["ah_debounce"]=antiheldDebounceMs;doc["ah_settle"]=antiheldSettleMs;doc["ah_growth"]=antiheldGrowthRetrigMs;
    doc["jitter_us"]=gStats.maxJitterUs;doc["missed_alarms"]=gStats.missedAlarms;
    JsonArray t=doc.createNestedArray("thresholds");
    for(int i=0;i<4;i++){JsonArray r=t.createNestedArray();for(int s=0;s<4;s++)r.add(THR[i][s]);}
    String res;serializeJson(doc,res);sendJSON(200,res);
  });
}

// =============================================================================
//  WiFi Task
// =============================================================================
void wifiTask(void*) {
  uint32_t localLastWs=0;
  for(;;){
    dnsServer.processNextRequest(); httpServer.handleClient(); wsServer.loop();
    uint32_t ms=millis(); int clients=wsServer.connectedClients();
    if(clients!=gPrevClients){gPrevClients=clients;notifyStreamState(clients>0);}
    if(ms-localLastWs>=WS_MS&&clients>0){localLastWs=ms;if(ESP.getFreeHeap()>40000)sendWsData();}
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// =============================================================================
//  Setup & Loop
// =============================================================================
void setup() {
  Serial.begin(115200);
  RP_UART.begin(RP_BAUD,SERIAL_8N1,RP_RX_PIN,RP_TX_PIN);
  dataMutex=xSemaphoreCreateMutex();
  prefs.begin("fsrpad",false); loadNVS(); initIdentity();
  WiFi.mode(WIFI_AP); WiFi.softAPConfig(AP_IP,AP_GW,AP_SN); WiFi.softAP(AP_SSID,AP_PASSWORD); WiFi.setTxPower(WIFI_POWER_8_5dBm);
  dnsServer.start(53,"*",AP_IP); setupHTTP(); httpServer.begin();
  wsServer.begin(); wsServer.enableHeartbeat(10000,3000,2); wsServer.onEvent(onWsEvent);
  vTaskDelay(pdMS_TO_TICKS(700));
  syncThresholdsToRP2040();
  syncAntiheldToRP2040();
  notifyStreamState(false);
  xTaskCreatePinnedToCore(uartRxTask,"UART_RX",4096,nullptr,2,&uartRxTaskHandle,1);
  xTaskCreatePinnedToCore(wifiTask,  "WIFI",   8192,nullptr,1,&wifiTaskHandle,  0);
}
void loop(){vTaskDelay(portMAX_DELAY);}
