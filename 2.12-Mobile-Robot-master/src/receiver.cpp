/*
This code receives data from the remote

Data is sent using the ESP-NOW Protocol
*/
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "receiver.h"
struct_message joyData;
esp_now_peer_info_t peerInfo;


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&joyData, incomingData, sizeof(joyData));
}
//0C:DC:7E:CC:6B:D8
void wirelessSetup(void){

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  // ESP-NOW Setup Complete
  //default the joystick values
  joyData.joyX = 512;
  joyData.joyY = 512;
  joyData.rightPressed = false;
  joyData.downPressed = false;
  joyData.leftPressed = false;
  joyData.upPressed = false;
  joyData.selPressed = false;
}