/*
* DonutShop platform layer for the Raspberry Pi Pico 2 W (arduino-pico core)
* Copyright(C) 2026 @Donutswdad and contributors
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation,either version 3 of the License,or
*(at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not,see <http://www.gnu.org/licenses/>.
*/

#if defined(ARDUINO_ARCH_RP2040)

#include "platform.h"
#include <WiFi.h>
#include <LittleFS.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#if DS_RT4K_USB
#include <Adafruit_TinyUSB.h>
#endif
#include <semphr.h>
#include <stream_buffer.h>
#include <lwip_wrap.h> // lwip_callback()
#include <pico/cyw43_arch.h>
#include <algorithm>
#include <vector>

// Boot logs on the USB serial console, debug build only. (This file does not see the sketch's
// "#define Serial Serial2", so Serial here is the USB CDC port.)
#if DS_RT4K_USB
#define DS_LOG(...) do{}while(0)
#else
#define DS_LOG(fmt, ...) Serial.printf("[ds %lu] " fmt "\r\n", (unsigned long)millis(), ##__VA_ARGS__)
#endif
#define DS_LOG_STACK(where) DS_LOG("%s: stack free %lu bytes", where, (unsigned long)uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t))

Rt4kUsbSerial CdcSerial;
SerialPIO SerialPIO2(DS_EXTRON2_TX_PIN, DS_EXTRON2_RX_PIN);

// CYW43 power pin (WL_REG_ON, GP23 on the Pico 2 W).
#if defined(CYW43_DEFAULT_PIN_WL_REG_ON)
#define DS_WL_REG_ON CYW43_DEFAULT_PIN_WL_REG_ON
#else
#define DS_WL_REG_ON 23
#endif

// A software reset restarts the RP2350 but may leave the CYW43 powered with its old state, and the
// next boot then brings it up badly (slow init, "F2 not ready", a soft AP that never beacons).
// Power the chip off first, so every reboot starts it from scratch like a cold boot.
[[noreturn]] static void rebootClean(){
  gpio_put(DS_WL_REG_ON, 0);
  delay(150);
  rp2040.reboot();
  for(;;){}
}

////////////////////////////////////////////////////////////////////////////////////
// RT4K USB (Pico = USB host, RT4K = CDC-ACM / FTDI device)

#if DS_RT4K_USB

static Adafruit_USBH_Host usbHost;
static Adafruit_USBH_CDC usbCdc;
static StreamBufferHandle_t usbTx = nullptr;
static SemaphoreHandle_t usbTxLock = nullptr;
static volatile bool usbStarted = false;

static const size_t USB_TX_BUFFER = 1024;
static const TickType_t USB_TX_WAIT = pdMS_TO_TICKS(50);

// Called from setup(), i.e. from the loop() task, which is the only task that touches TinyUSB.
void Rt4kUsbSerial::begin(unsigned long baud){
  if(usbStarted) return;
  usbTxLock = xSemaphoreCreateMutex();
  usbTx = xStreamBufferCreate(USB_TX_BUFFER, 1);
  if(!usbTxLock || !usbTx) return;
  usbCdc.begin(baud); // applied to the RT4K when it mounts
  usbHost.begin(0);   // rhport 0 = native USB port
  usbStarted = true;  // publish last, writers check it
}

size_t Rt4kUsbSerial::write(uint8_t c){
  return write(&c, 1);
}

// Any task. Stream buffers allow a single writer at a time, hence the mutex.
size_t Rt4kUsbSerial::write(const uint8_t *buffer, size_t size){
  if(!usbStarted || size == 0) return 0;
  if(xSemaphoreTake(usbTxLock, USB_TX_WAIT) != pdTRUE) return 0;
  size_t sent = xStreamBufferSend(usbTx, buffer, size, USB_TX_WAIT);
  xSemaphoreGive(usbTxLock);
  return sent;
}

static void usbService(){
  usbHost.task(0);
  uint8_t chunk[64];
  bool wrote = false;
  for(;;){
    size_t room = sizeof(chunk);
    const bool mounted = usbCdc.mounted();
    if(mounted){
      int afw = usbCdc.availableForWrite();
      if(afw <= 0) break;
      if((size_t)afw < room) room = afw;
    }
    size_t got = xStreamBufferReceive(usbTx, chunk, room, 0);
    if(got == 0) break;
    if(mounted){ // otherwise the RT4K is unplugged and the bytes are dropped
      usbCdc.write(chunk, got);
      wrote = true;
    }
  }
  if(wrote) usbCdc.flush();
}

extern "C" void tuh_cdc_mount_cb(uint8_t idx){
  usbCdc.mount(idx);
}

extern "C" void tuh_cdc_umount_cb(uint8_t idx){
  usbCdc.umount(idx);
}

#else // debug build: no USB host, RT4K output is dropped

static const bool usbStarted = false;
void Rt4kUsbSerial::begin(unsigned long baud){ (void)baud; }
size_t Rt4kUsbSerial::write(uint8_t c){ (void)c; return 1; }
size_t Rt4kUsbSerial::write(const uint8_t *buffer, size_t size){ (void)buffer; return size; }
static void usbService(){}

#endif // DS_RT4K_USB

////////////////////////////////////////////////////////////////////////////////////
// Status LED. The Pico 2 W has one LED (on the CYW43) where the Nano ESP32 has an RGB LED, so
// states become blink patterns (README "Status LED"):
//   joining Wi-Fi: fast blink (5 Hz)   setup portal: slow blink (1 Hz)   connected: solid on
//   while connected: gameID query started = short wink, console did not answer = double wink,
//   profile sent to the RT4K = 3 quick blinks.
// Its own task drives it, since WiFi.begin() blocks for many seconds. Events come from any task
// (DDloop/GIDloop may run on the other core) through an atomic bitmask.

enum LedMode : uint8_t { LED_JOINING, LED_PORTAL, LED_CONNECTED };
static volatile LedMode ledMode = LED_JOINING;
static volatile bool ledWifiUp = false; // set once wifiBegin() returns; loss of Wi-Fi then shows as joining
static uint32_t ledEvents = 0;          // bit per platform::StatusEvent, accessed with __atomic builtins

void platform::statusEvent(StatusEvent event){
  __atomic_fetch_or(&ledEvents, 1u << (uint8_t)event, __ATOMIC_RELEASE);
}

// Patterns shown over the solid "connected" state: durations in ms, alternating off/on, starting off.
static const uint16_t LED_WINK[] = {80};
static const uint16_t LED_DOUBLE_WINK[] = {80, 150, 80};
static const uint16_t LED_PROFILE[] = {100, 100, 100, 100, 100};

static void ledTask(void*){
  const uint16_t* pattern = nullptr;
  uint8_t patternLen = 0, step = 0;
  uint32_t stepStart = 0, lastWrite = 0, lastWifiCheck = 0;
  bool wrote = false;
  for(;;){
    const uint32_t now = millis();
    LedMode mode = ledMode;
    if(ledWifiUp){
      static bool linkUp = true;
      if(now - lastWifiCheck >= 500){ // WiFi.status() goes through the LWIP task; don't hammer it
        lastWifiCheck = now;
        linkUp = WiFi.status() == WL_CONNECTED;
      }
      mode = linkUp ? LED_CONNECTED : LED_JOINING;
    }

    const uint32_t ev = __atomic_exchange_n(&ledEvents, 0u, __ATOMIC_ACQUIRE);
    auto start = [&](const uint16_t* p, uint8_t n){ pattern = p; patternLen = n; step = 0; stepStart = now; };
    if(mode == LED_CONNECTED){
      const bool showingProfile = pattern == LED_PROFILE;
      if(ev & (1u << (uint8_t)platform::StatusEvent::ProfileSent)) start(LED_PROFILE, 5);
      else if(!showingProfile && (ev & (1u << (uint8_t)platform::StatusEvent::QueryFailed))) start(LED_DOUBLE_WINK, 3);
      else if(!pattern && (ev & (1u << (uint8_t)platform::StatusEvent::QueryStart))) start(LED_WINK, 1);
    }
    else{
      pattern = nullptr;
    }

    bool on;
    if(mode == LED_JOINING) on = (now / 100) % 2;
    else if(mode == LED_PORTAL) on = (now / 500) % 2;
    else{
      on = true;
      if(pattern){
        while(pattern && now - stepStart >= pattern[step]){
          stepStart += pattern[step];
          if(++step >= patternLen) pattern = nullptr;
        }
        if(pattern) on = (step % 2) == 1; // even steps are "off"
      }
    }

    // Write on change, plus once a second so the sketch's own LED_BUILTIN writes (Nano logic) don't
    // stick. Each write is a CYW43 bus transaction, so keep them rare.
    static bool lastOn = false;
    if(!wrote || on != lastOn || now - lastWrite >= 1000){
      digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
      lastOn = on;
      lastWrite = now;
      wrote = true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void ledBegin(){
  static bool started = false;
  if(started) return;
  started = true;
  pinMode(LED_BUILTIN, OUTPUT);
  xTaskCreate(ledTask, "DSLED", 1024, nullptr, tskIDLE_PRIORITY + 2, nullptr);
}

////////////////////////////////////////////////////////////////////////////////////
// Core services

bool platform::fsBegin(){
  if(LittleFS.begin()) return true;
  LittleFS.format();
  return LittleFS.begin();
}

void platform::restart(){
  rebootClean();
}

#if !DS_RT4K_USB
static void debugSerialCommands(); // Wi-Fi section below

// Debug build: every 10 s, the numbers that tell a slow leak or a stack overflow from a Wi-Fi drop.
static void debugHeartbeat(){
  static uint32_t last = 0;
  if(millis() - last < 10000) return;
  last = millis();
  auto stackFree = [](const char* name) -> long {
    TaskHandle_t t = xTaskGetHandle(name);
    return t ? (long)(uxTaskGetStackHighWaterMark(t) * sizeof(StackType_t)) : -1;
  };
  DS_LOG("alive: heap free %d, wifi %d rssi %ld ip %s, stack free DDloop %ld GIDloop %ld loop %ld",
    rp2040.getFreeHeap(), (int)WiFi.status(), (long)WiFi.RSSI(), WiFi.localIP().toString().c_str(),
    stackFree("DDloop"), stackFree("GIDloop"),
    (long)(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)));
}
#endif

void platform::loopHook(){
  if(usbStarted) usbService();
  MDNS.update();
#if !DS_RT4K_USB
  debugHeartbeat();
  debugSerialCommands();
#endif
  delay(1); // loop() outranks DDloop/GIDloop, it must block to let them run
}

////////////////////////////////////////////////////////////////////////////////////
// Firmware update. arduino-pico stages the new image in LittleFS and the bootloader
// copies it over on the next reboot, so the size limit is the free FS space.

static bool updStarted = false;
static bool updRejected = false;
static bool updDone = false;

void platform::updateUpload(HTTPUpload& upload){
  if(upload.status == UPLOAD_FILE_START){
    updStarted = updRejected = updDone = false;
  }
  else if(upload.status == UPLOAD_FILE_WRITE){
    if(updRejected || upload.currentSize == 0) return;
    if(!updStarted){
      // 0xE9 is the ESP32 image magic byte: never stage an image for the wrong chip.
      if(upload.buf[0] == 0xE9){
        updRejected = true;
        return;
      }
      FSInfo info;
      LittleFS.info(info);
      if(!Update.begin(info.totalBytes - info.usedBytes, U_FLASH)){
        updRejected = true;
        return;
      }
      updStarted = true;
    }
    if(Update.write(upload.buf, upload.currentSize) != upload.currentSize){
      updRejected = true;
    }
  }
  else if(upload.status == UPLOAD_FILE_END){
    if(updStarted && !updRejected) updDone = Update.end(true);
  }
  else if(upload.status == UPLOAD_FILE_ABORTED){
    if(updStarted) Update.end();
    updRejected = true;
  }
}

bool platform::updateFailed(){
  return !updDone || updRejected || Update.hasError();
}

////////////////////////////////////////////////////////////////////////////////////
// Wi-Fi: stored credentials, else a captive portal (stand-in for WiFiManager, which is ESP-only)

static const char* WIFI_CREDS_FILE = "/wifi.json"; // not part of /exportAll
static const uint32_t STA_CONNECT_TIMEOUT = 20000;
static const uint32_t PORTAL_IDLE_REBOOT = 300000;  // only when saved credentials exist

static bool loadCreds(String& ssid, String& pass){
  File f = LittleFS.open(WIFI_CREDS_FILE, "r");
  if(!f) return false;
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if(err) return false;
  ssid = doc["ssid"] | "";
  pass = doc["pass"] | "";
  return ssid.length() > 0;
}

static bool saveCreds(const String& ssid, const String& pass){
  File f = LittleFS.open(WIFI_CREDS_FILE, "w");
  if(!f) return false;
  JsonDocument doc;
  doc["ssid"] = ssid;
  doc["pass"] = pass;
  bool ok = serializeJson(doc, f) > 0;
  f.close();
  return ok;
}

// Boot straight into the portal: the radio then starts from a clean state. Switching the CYW43
// from STA (a failed join, or a scan) to AP in place left the AP without working DHCP.
static const char* PORTAL_FLAG_FILE = "/portal.flag";

#if !DS_RT4K_USB
// Debug build: commands typed on the USB serial console. "portal" reboots into the setup portal
// (keeping the saved network), "flag" only arms that for the next boot (e.g. a power cycle),
// "reboot" just reboots.
static void debugSerialCommands(){
  static char line[16];
  static uint8_t len = 0;
  while(Serial.available()){
    const char c = Serial.read();
    if(c != '\r' && c != '\n'){
      if(len < sizeof(line) - 1) line[len++] = c;
      continue;
    }
    line[len] = 0;
    len = 0;
    if(!strcmp(line, "portal")){
      DS_LOG("command: portal, rebooting into the setup portal");
      File f = LittleFS.open(PORTAL_FLAG_FILE, "w");
      f.close();
      delay(100);
      rebootClean();
    }
    else if(!strcmp(line, "flag")){
      DS_LOG("command: flag, the next boot starts the setup portal");
      File f = LittleFS.open(PORTAL_FLAG_FILE, "w");
      f.close();
    }
    else if(!strcmp(line, "reboot")){
      DS_LOG("command: reboot");
      delay(100);
      rebootClean();
    }
    else if(line[0]){
      DS_LOG("unknown command '%s' (portal, flag, reboot)", line);
    }
  }
}
#endif

static bool connectSta(const char* hostname, const String& ssid, const String& pass){
  DS_LOG("joining %s", ssid.c_str());
  ledMode = LED_JOINING;
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);
  if(pass.length()) WiFi.begin(ssid.c_str(), pass.c_str());
  else WiFi.begin(ssid.c_str());
  uint32_t start = millis();
  while(WiFi.status() != WL_CONNECTED && millis() - start < STA_CONNECT_TIMEOUT){
    delay(100);
  }
  if(WiFi.status() != WL_CONNECTED) return false;
  WiFi.noLowPowerMode(); // same as esp_wifi_set_ps(WIFI_PS_NONE) on the ESP32
  return true;
}

struct SoftApArgs {
  const char* name;
  IPAddress ip;
  bool ok;
};

// Runs in the lwIP thread (see runPortal). The AP path of CYW43::begin() does not block.
static void startSoftAp(void* param){
  SoftApArgs* a = static_cast<SoftApArgs*>(param);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(a->ip, a->ip, IPAddress(255, 255, 255, 0));
  a->ok = WiFi.softAP(a->name);
}

static String htmlEscape(const String& in){
  String out;
  out.reserve(in.length() + 8);
  for(size_t i = 0; i < in.length(); i++){
    char c = in[i];
    switch(c){
      case '&': out += F("&amp;"); break;
      case '<': out += F("&lt;"); break;
      case '>': out += F("&gt;"); break;
      case '"': out += F("&quot;"); break;
      case '\'': out += F("&#39;"); break;
      default: out += c;
    }
  }
  return out;
}

// Nearby networks as tappable rows (like WiFiManager): strongest first, one row per SSID.
// Runs before the AP starts; scanning uses the STA interface, which startSoftAp() tears down.
static String scanNetworkList(){
  WiFi.mode(WIFI_STA);
  const int found = WiFi.scanNetworks();
  DS_LOG("scan: %d network(s)", found);
  struct Net { String ssid; int32_t rssi; bool open; };
  std::vector<Net> nets;
  for(int i = 0; i < found; i++){
    String ssid = WiFi.SSID(i);
    if(ssid.length() == 0) continue; // hidden network
    const int32_t rssi = WiFi.RSSI(i);
    auto it = std::find_if(nets.begin(), nets.end(), [&](const Net& n){ return n.ssid == ssid; });
    if(it == nets.end()) nets.push_back({ssid, rssi, WiFi.encryptionType(i) == ENC_TYPE_NONE});
    else if(rssi > it->rssi) it->rssi = rssi;
  }
  std::sort(nets.begin(), nets.end(), [](const Net& a, const Net& b){ return a.rssi > b.rssi; });
  String rows;
  const size_t maxRows = 20;
  for(size_t i = 0; i < nets.size() && i < maxRows; i++){
    const int32_t q = constrain(2 * (nets[i].rssi + 100), 0, 100); // rough dBm -> %
    rows += "<a href=# onclick='return pick(this)'>" + htmlEscape(nets[i].ssid) + "<span>" +
            (nets[i].open ? "" : "&#128274; ") + String(q) + "%</span></a>";
  }
  if(rows.length() == 0) rows = F("<p>No networks found. Type the name below.</p>");
  return rows;
}

[[noreturn]] static void runPortal(const char* portalName, bool haveCreds){
  ledMode = LED_PORTAL;
  const String networks = scanNetworkList();

  // Same sequence as arduino-pico's DNSServer/CaptivePortal example.
  DS_LOG("starting AP %s", portalName);
  SoftApArgs ap{portalName, IPAddress(192, 168, 4, 1), false};
  // arduino-pico <= 6.1.1 FreeRTOS bug: bringing the AP netif up from a user task ends in
  // netif_set_default(), which the lwIP thread does not implement ("Unimplemented LWIP thread
  // action" panic). Inside the lwIP thread the wrapped lwIP calls run directly, so start it there.
  // extras/arduino-pico-patches fixes the core; this keeps unpatched cores working too.
  lwip_callback(startSoftAp, &ap);
  const IPAddress ip = ap.ip;
  DS_LOG("softAP %s, IP %s", ap.ok ? "ok" : "FAILED", WiFi.softAPIP().toString().c_str());
  DS_LOG_STACK("after softAP");

  // Heap allocated: they live for the rest of this boot (the portal ends with a reboot).
  DNSServer* dns = new DNSServer();
  dns->start(53, "*", ip);
  WebServer* portal = new WebServer(80);

  const String page = String(F(
    "<!DOCTYPE html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>DonutShop Wi-Fi setup</title><style>"
    "body{font-family:sans-serif;text-align:center;max-width:360px;margin:auto;padding:16px}"
    "input{width:100%;box-sizing:border-box;padding:10px;margin:6px 0}"
    "button{width:100%;padding:12px;background:#4CAF50;color:#fff;border:0;border-radius:4px;font-size:1em}"
    "#nets a{display:flex;justify-content:space-between;padding:10px 4px;border-bottom:1px solid #ddd;"
    "color:inherit;text-decoration:none;text-align:left}"
    "#nets span{white-space:nowrap;margin-left:8px;color:#666}"
    "</style><script>function pick(a){document.getElementsByName('s')[0].value=a.firstChild.nodeValue;"
    "document.getElementsByName('p')[0].focus();return false;}</script>"
    "</head><body><h2>DonutShop Wi-Fi setup</h2><div id=nets>")) + networks + F("</div>"
    "<form method=POST action=/save>"
    "<input name=s placeholder='Network name (SSID), 2.4 GHz' required autocomplete=off autocapitalize=none>"
    "<input name=p type=password placeholder='Password'>"
    "<button type=submit>Save</button></form></body></html>");

  uint32_t lastActivity = millis();
  bool saved = false;

  portal->on("/", HTTP_GET, [&](){
    lastActivity = millis();
    portal->send(200, "text/html", page);
  });
  portal->on("/save", HTTP_POST, [&](){
    lastActivity = millis();
    const String ssid = portal->arg("s");
    if(ssid.length() == 0 || ssid.length() > 32 || portal->arg("p").length() > 64){
      portal->send(400, "text/plain", "Invalid network name or password");
      return;
    }
    if(!saveCreds(ssid, portal->arg("p"))){
      portal->send(500, "text/plain", "Could not save credentials");
      return;
    }
    portal->send(200, "text/html", String(F("<html><body style='font-family:sans-serif;text-align:center'>"
      "<h3>Saved. DonutShop is rebooting and joining the network.</h3></body></html>")));
    saved = true;
  });
  portal->onNotFound([&](){ // captive portal detection: send everything to the form
    lastActivity = millis();
    portal->sendHeader("Location", String("http://") + ip.toString() + "/", true);
    portal->send(302, "text/plain", "");
  });
  portal->begin();
  DS_LOG("portal web server + DNS running");
  DS_LOG_STACK("portal ready");

  uint32_t lastBeat = millis();
  for(;;){
    if(millis() - lastBeat > 10000){
      lastBeat = millis();
      DS_LOG("portal alive, %d client(s)", WiFi.softAPgetStationNum());
      DS_LOG_STACK("portal");
    }
#if !DS_RT4K_USB
    debugSerialCommands();
#endif
    dns->processNextRequest();
    portal->handleClient();
    if(saved){
      delay(1500); // let the response go out
      rebootClean();
    }
    // A router that is down at boot must not strand the device in the portal.
    if(haveCreds && millis() - lastActivity > PORTAL_IDLE_REBOOT) rebootClean();
    delay(2);
  }
}

static void wifiBeginImpl(const char* hostname, const char* portalName){
  DS_LOG_STACK("wifi task start");
  // Bring the CYW43 up (firmware download) from this task before the status LED task starts: the LED
  // is on the CYW43, and LED writes run in the LWIP task. If the first CYW43 access happens there, the
  // download waits on events that the busy LWIP task must deliver, and the chip is left half up
  // ("F2 not ready": no scan results, no AP, no LED).
  cyw43_arch_enable_sta_mode();
  DS_LOG("CYW43 up");
  ledBegin();
  bool fsOk = platform::fsBegin(); // credentials live in LittleFS; setup() mounts it later again, which is a no-op
  DS_LOG("LittleFS %s", fsOk ? "mounted" : "FAILED");
  String ssid, pass;
  const bool haveCreds = loadCreds(ssid, pass);
  DS_LOG("saved network: %s", haveCreds ? ssid.c_str() : "(none)");
  if(LittleFS.exists(PORTAL_FLAG_FILE)){
    DS_LOG("portal flag set, starting portal");
    LittleFS.remove(PORTAL_FLAG_FILE);
    runPortal(portalName, haveCreds);
  }
  if(!haveCreds) runPortal(portalName, false); // first boot: radio untouched so far
  if(connectSta(hostname, ssid, pass)){
    DS_LOG("connected, IP %s", WiFi.localIP().toString().c_str());
    ledMode = LED_CONNECTED;
    ledWifiUp = true;
    return;
  }
  DS_LOG("join failed, rebooting into the portal");
  File f = LittleFS.open(PORTAL_FLAG_FILE, "w");
  f.close();
  rebootClean();
}

// Wi-Fi setup runs on a task of our own while setup() waits. Run directly in setup() (arduino-pico's
// loop() task) the join hung on hardware, although the measured stack need (~1.1 KB peak, portal
// included) fits its 4 KB; the cause is not known. 16 KB leaves room.
static const configSTACK_DEPTH_TYPE WIFI_TASK_STACK = 4096; // words = 16 KB

struct WifiTaskArgs {
  const char* hostname;
  const char* portalName;
  TaskHandle_t waiter;
};

static void wifiTask(void* param){
  WifiTaskArgs* args = static_cast<WifiTaskArgs*>(param);
  wifiBeginImpl(args->hostname, args->portalName);
  DS_LOG_STACK("wifi task end");
  xTaskNotifyGive(args->waiter);
  vTaskDelete(nullptr);
}

void platform::wifiBegin(const char* hostname, const char* portalName){
#if !DS_RT4K_USB
  Serial.begin(115200);
  uint32_t t0 = millis();
  while(!Serial && millis() - t0 < 8000) delay(10); // give the serial monitor time to attach
  DS_LOG("DonutShop Pico 2 W debug build");
  DS_LOG_STACK("setup");
#endif
  WifiTaskArgs args{hostname, portalName, xTaskGetCurrentTaskHandle()};
  TaskHandle_t task = nullptr;
  if(xTaskCreate(wifiTask, "DSWIFI", WIFI_TASK_STACK, &args, uxTaskPriorityGet(nullptr), &task) != pdPASS){
    DS_LOG("could not create the Wi-Fi task, running inline");
    wifiBeginImpl(hostname, portalName);
    return;
  }
  vTaskCoreAffinitySet(task, 1 << 0); // same core as the loop() task it replaces
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

#endif // ARDUINO_ARCH_RP2040
