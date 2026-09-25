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
// Core services

bool platform::fsBegin(){
  if(LittleFS.begin()) return true;
  LittleFS.format();
  return LittleFS.begin();
}

void platform::restart(){
  rp2040.reboot();
}

void platform::loopHook(){
  if(usbStarted) usbService();
  MDNS.update();
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

static void blinkLed(uint32_t periodMs){
  static uint32_t last = 0;
  if(millis() - last >= periodMs / 2){
    last = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

static bool connectSta(const char* hostname, const String& ssid, const String& pass){
  DS_LOG("joining %s", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);
  if(pass.length()) WiFi.begin(ssid.c_str(), pass.c_str());
  else WiFi.begin(ssid.c_str());
  uint32_t start = millis();
  while(WiFi.status() != WL_CONNECTED && millis() - start < STA_CONNECT_TIMEOUT){
    blinkLed(200); // fast blink: joining
    delay(20);
  }
  digitalWrite(LED_BUILTIN, LOW);
  if(WiFi.status() != WL_CONNECTED) return false;
  WiFi.noLowPowerMode(); // same as esp_wifi_set_ps(WIFI_PS_NONE) on the ESP32
  return true;
}

[[noreturn]] static void runPortal(const char* portalName, bool haveCreds){
  // Same sequence as arduino-pico's DNSServer/CaptivePortal example.
  const IPAddress ip(192, 168, 4, 1);
  DS_LOG("starting AP %s", portalName);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
  bool apOk = WiFi.softAP(portalName);
  DS_LOG("softAP %s, IP %s", apOk ? "ok" : "FAILED", WiFi.softAPIP().toString().c_str());
  DS_LOG_STACK("after softAP");

  // Heap allocated: they live for the rest of this boot (the portal ends with a reboot).
  DNSServer* dns = new DNSServer();
  dns->start(53, "*", ip);
  WebServer* portal = new WebServer(80);

  const String page = F(
    "<!DOCTYPE html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>DonutShop Wi-Fi setup</title><style>"
    "body{font-family:sans-serif;text-align:center;max-width:360px;margin:auto;padding:16px}"
    "input{width:100%;box-sizing:border-box;padding:10px;margin:6px 0}"
    "button{width:100%;padding:12px;background:#4CAF50;color:#fff;border:0;border-radius:4px;font-size:1em}"
    "</style></head><body><h2>DonutShop Wi-Fi setup</h2>"
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
    }
    dns->processNextRequest();
    portal->handleClient();
    blinkLed(1000); // slow blink: setup portal active
    if(saved){
      delay(1500); // let the response go out
      rp2040.reboot();
    }
    // A router that is down at boot must not strand the device in the portal.
    if(haveCreds && millis() - lastActivity > PORTAL_IDLE_REBOOT) rp2040.reboot();
    delay(2);
  }
}

static void wifiBeginImpl(const char* hostname, const char* portalName){
  DS_LOG_STACK("wifi task start");
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
    return;
  }
  DS_LOG("join failed, rebooting into the portal");
  File f = LittleFS.open(PORTAL_FLAG_FILE, "w");
  f.close();
  rp2040.reboot();
}

// setup() runs on arduino-pico's 4 KB loop() task stack, too small for Wi-Fi + LittleFS + the portal
// web server. Run them on a task of our own and block setup() until Wi-Fi is up.
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
