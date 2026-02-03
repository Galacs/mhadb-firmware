#include <Arduino.h>

#include "ESP32_NOW_Serial.h"
#include "MacAddress.h"
#include "WiFi.h"

#include "esp_wifi.h"

// Channel to be used by the ESP-NOW protocol
#define ESPNOW_WIFI_CHANNEL 5

#define ESPNOW_WIFI_MODE WIFI_STA  // WiFi Mode
#define ESPNOW_WIFI_IF WIFI_IF_STA // WiFi Interface

const MacAddress peer_mac({0x20, 0x6E, 0xF1, 0x9E, 0xAD, 0xBC});

ESP_NOW_Serial_Class NowSerial(peer_mac, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IF);

void setup() {
  Serial.begin(115200);

  Serial.print("WiFi Mode: ");
  Serial.println(ESPNOW_WIFI_MODE == WIFI_AP ? "AP" : "Station");
  WiFi.mode(ESPNOW_WIFI_MODE);

  Serial.print("Channel: ");
  Serial.println(ESPNOW_WIFI_CHANNEL);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Start the ESP-NOW communication
  Serial.println("ESP-NOW communication starting...");
  NowSerial.begin(115200);
  Serial.printf("ESP-NOW version: %d, max data length: %d\n",
                ESP_NOW.getVersion(), ESP_NOW.getMaxDataLen());
  Serial.println(
      "You can now send data to the peer device using the Serial Monitor.\n");
}

void loop() {
  while (NowSerial.available()) {
    Serial.write(NowSerial.read());
  }

  while (Serial.available() && NowSerial.availableForWrite()) {
    if (NowSerial.write(Serial.read()) <= 0) {
      Serial.println("Failed to send data over ESP-NOW");
      break;
    }
  }

  delay(1);
}
