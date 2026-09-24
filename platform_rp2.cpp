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
#include <Adafruit_TinyUSB.h>
#include <semphr.h>
#include <stream_buffer.h>

Rt4kUsbSerial CdcSerial;
SerialPIO SerialPIO2(DS_EXTRON2_TX_PIN, DS_EXTRON2_RX_PIN);

////////////////////////////////////////////////////////////////////////////////////
// RT4K USB (Pico = USB host, RT4K = CDC-ACM / FTDI device)

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

static bool connectSta(const char* hostname, const String& ssid, const String& pass){
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

[[noreturn]] static void runPortal(const char* portalName, bool haveCreds){
  WiFi.mode(WIFI_STA);
  int found = WiFi.scanNetworks(); // scan before going AP
  String options;
  for(int i = 0; i < found; i++){
    String s = WiFi.SSID(i);
    if(s.length()) options += "<option value=\"" + htmlEscape(s) + "\">";
  }
  WiFi.disconnect();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(portalName);
  const IPAddress ip = WiFi.softAPIP();

  // Heap allocated: setup() runs on the 4 KB loop() task stack.
  DNSServer* dns = new DNSServer();
  dns->start(53, "*", ip);
  WebServer* portal = new WebServer(80);

  const String page = String(F(
    "<!DOCTYPE html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>DonutShop Wi-Fi setup</title><style>"
    "body{font-family:sans-serif;text-align:center;max-width:360px;margin:auto;padding:16px}"
    "input{width:100%;box-sizing:border-box;padding:10px;margin:6px 0}"
    "button{width:100%;padding:12px;background:#4CAF50;color:#fff;border:0;border-radius:4px;font-size:1em}"
    "</style></head><body><h2>DonutShop Wi-Fi setup</h2>"
    "<form method=POST action=/save>"
    "<input name=s list=nets placeholder='Network (SSID)' required autocomplete=off>"
    "<datalist id=nets>")) + options + F("</datalist>"
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

  for(;;){
    dns->processNextRequest();
    portal->handleClient();
    if(saved){
      delay(1500); // let the response go out
      rp2040.reboot();
    }
    // A router that is down at boot must not strand the device in the portal.
    if(haveCreds && millis() - lastActivity > PORTAL_IDLE_REBOOT) rp2040.reboot();
    delay(2);
  }
}

void platform::wifiBegin(const char* hostname, const char* portalName){
  fsBegin(); // credentials live in LittleFS; setup() mounts it later again, which is a no-op
  String ssid, pass;
  const bool haveCreds = loadCreds(ssid, pass);
  if(haveCreds && connectSta(hostname, ssid, pass)) return;
  runPortal(portalName, haveCreds);
}

#endif // ARDUINO_ARCH_RP2040
