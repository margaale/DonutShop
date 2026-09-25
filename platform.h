/*
* DonutShop platform abstraction (multi-target)
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

// Included at the very top of DonutShop.ino, BEFORE the "#define Serial ..." line,
// so nothing in here may rely on that remap.
//
// Supported targets:
//   - Arduino Nano ESP32 (ESP32-S3)          -> ARDUINO_ARCH_ESP32
//   - Raspberry Pi Pico 2 W (RP2350+CYW43)   -> ARDUINO_ARCH_RP2040 (arduino-pico defines it for RP2350 too)
//
// Everything the ESP32 build needs stays inline in DonutShop.ino (upstream code, untouched),
// so on ESP32 this header only defines a couple of constants.

#pragma once

#include <Arduino.h>

// GitHub repo the WebCtl updater checks for releases (both targets). CI release builds pass
// -DDS_RELEASE_REPO_ID=owner/repo (unquoted, avoids shell quoting), see .github/workflows/release-pico2w.yml.
#define DS_STR_(x) #x
#define DS_STR(x) DS_STR_(x)
#if defined(DS_RELEASE_REPO_ID)
#define DS_RELEASE_REPO DS_STR(DS_RELEASE_REPO_ID)
#else
#define DS_RELEASE_REPO "svirant/DonutShop"
#endif

#if defined(ARDUINO_ARCH_ESP32)

// ESP-IDF xTaskCreate() stack depth is in BYTES.
#define DS_TASK_STACK 16384

// WebCtl "Check GitHub" updater asset: DonutShop_v<ver>_update.bin
#define DS_UPDATE_ASSET_SUFFIX "_update.bin"

#elif defined(ARDUINO_ARCH_RP2040)

#if !defined(__FREERTOS)
#error "DonutShop on the Pico 2 W needs the FreeRTOS OS option (FQBN os=freertos), see sketch.yaml"
#endif
#if !defined(USE_TINYUSB_HOST)
#error "DonutShop on the Pico 2 W needs the native TinyUSB host stack (FQBN usbstack=tinyusb_host), see sketch.yaml"
#endif

#include <FreeRTOS.h>
#include <task.h>
#include <SimpleMDNS.h>  // global MDNS object; the one arduino-pico's ArduinoOTA links (LEAmDNS would clash)
#include <Updater.h>     // provides the global Update object
#include <WebServer.h>

// ---- Pico 2 W pinout (GPx numbers) -------------------------------------------------------------
#define DS_RT4K_TX_PIN     8   // RT4K HD-15 serial (Serial2 / UART1)
#define DS_RT4K_RX_PIN     9
#define DS_EXTRON1_TX_PIN  0   // Extron sw1 (Serial1 / UART0)
#define DS_EXTRON1_RX_PIN  1
#define DS_EXTRON2_TX_PIN  4   // Extron sw2 (SerialPIO2)
#define DS_EXTRON2_RX_PIN  5
#define IR_RECEIVE_PIN     2   // Optional IR receiver
#define IR_SEND_PIN        3   // Optional IR LED emitter
#define LED_RED           16   // Optional common-anode RGB LED (active low, analogWrite 255 = off)
#define LED_GREEN         17
#define LED_BLUE          18
// LED_BUILTIN is the core's (the CYW43 LED, pin 64); not redefined. It is not a real GPIO, so
// IRremote's digitalWriteFast() feedback LED must stay off.
#define NO_LED_FEEDBACK_CODE
#define NO_LED_RECEIVE_FEEDBACK_CODE
#define NO_LED_SEND_FEEDBACK_CODE

// WebCtl "Check GitHub" updater asset: DonutShop_v<ver>_pico2w_update.bin
// (built and attached to each release by .github/workflows/release-pico2w.yml)
#define DS_UPDATE_ASSET_SUFFIX "_pico2w_update.bin"

// FreeRTOS xTaskCreate() stack depth is in WORDS here: 8192 words = 32 KB.
// Both tasks may run BearSSL (HTTPS console polling / GitHub proxy), which is stack hungry.
#define DS_TASK_STACK 8192

// The ESP32 FS layer defines these, arduino-pico does not.
#ifndef FILE_READ
#define FILE_READ "r"
#endif
#ifndef FILE_WRITE
#define FILE_WRITE "w"
#endif

// Write-only link to the RT4K USB-C port (the RT4K is a CDC-ACM/FTDI device, the Pico is the USB host).
// Any task may print to it. Bytes are queued in a FreeRTOS stream buffer and only the loop() task
// (platform::loopHook) touches TinyUSB. Bytes are dropped while no RT4K is mounted.
class Rt4kUsbSerial : public Print {
public:
  void begin(unsigned long baud);
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *buffer, size_t size) override;
  using Print::write;
};
extern Rt4kUsbSerial CdcSerial;

// 3rd UART (Extron sw2), implemented with PIO on DS_EXTRON2_TX_PIN / DS_EXTRON2_RX_PIN.
extern SerialPIO SerialPIO2;

namespace platform {
  // Mounts LittleFS (formats and retries on failure). Safe to call more than once.
  bool fsBegin();
  // Connects to the stored Wi-Fi network, or runs a captive portal named portalName (never returns
  // from the portal: it reboots after the credentials are saved). Replaces WiFiManager.
  void wifiBegin(const char* hostname, const char* portalName);
  // Called from loop(): services USB host + mDNS and yields (loop runs at a higher priority than DDloop/GIDloop).
  void loopHook();
  void restart();
  // Firmware upload (/update). Stages the image in LittleFS; rejects ESP32 images.
  void updateUpload(HTTPUpload& upload);
  bool updateFailed();
}

#else
#error "Unsupported target: DonutShop builds for the Arduino Nano ESP32 or the Raspberry Pi Pico 2 W"
#endif
