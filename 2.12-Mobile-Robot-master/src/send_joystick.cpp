#include <Arduino.h>
#include <SPI.h>
//Joystick setup
#include "Adafruit_seesaw.h"


Adafruit_seesaw ss;

#define BUTTON_RIGHT 6
#define BUTTON_DOWN  7
#define BUTTON_LEFT  9
#define BUTTON_UP    10
#define BUTTON_SEL   14
uint32_t button_mask = (1 << BUTTON_RIGHT) | (1 << BUTTON_DOWN) | 
                (1 << BUTTON_LEFT) | (1 << BUTTON_UP) | (1 << BUTTON_SEL);
//ESP-NOW SETUP
#include <esp_now.h>
#include <WiFi.h>
#include "wireless.h"

//TODO set address to robot mac adress 0C:DC:7E:CC:6B:D8
uint8_t broadcastAddress[] = {0x0C, 0xDC, 0x7E, 0xCC, 0x68, 0xD8};

// Structure example to send data
// Must match the receiver structure

joy_message joyData;

odometry_message odom_data;
//flag to see if we have new data
bool freshOdomData = false;
esp_now_peer_info_t peerInfo;
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

    bool success = status == ESP_NOW_SEND_SUCCESS ;
    if (success) {
      Serial.printf("Sent x:%d y:%d\n", joyData.joyX, joyData.joyY);
    } else {
      Serial.println("Failed");
    }

}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&odom_data, incomingData, sizeof(odom_data));
  freshOdomData = true;
}

bool readJoystick();
void sendJoystick();
//void printOdometrySerial();

//Joystick tracking variables
int last_x = 0, last_y = 0;
bool rightPressed = false;
bool downPressed = false;
bool leftPressed = false;
bool upPressed = false;
bool selPressed = false;

void setup(void){
  Serial.begin(115200);
    //ESP_NOW Setup
    // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //Tell the microcontroller which functions to call when
  //data is sent or received
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // ESP-NOW Setup Complete


  //Initialize Joystick
  ss.begin(0x49);
  ss.pinModeBulk(button_mask, INPUT_PULLUP);
  ss.setGPIOInterrupts(button_mask, 1);    

  Serial.println("Finished setup");
}

//minimum time between sending data to the robot
long sendDataDelay = 50; //Millis
long lastSendData = 0;

//minimum time between sending data over serial
long printDataDelay = 20;
long lastPrintData = 0;

void loop(){
    //Send data when the joystick is ready
    if (readJoystick() && (millis() - lastSendData > sendDataDelay)){
      sendJoystick();    
      lastSendData = millis();
    }

    // //only print data if a new reading came in
    // if (freshOdomData && millis() - lastPrintData > printDataDelay){
    //   printOdometrySerial();
    //   lastPrintData = millis();
    //   //wait for the next bit of fresh data
    //   freshOdomData = false;
    // }


}

//Sends the joystick readings to the broadcast address
void sendJoystick(){
  // Set values to send
  joyData.joyX = last_x;
  joyData.joyY = last_y;
  joyData.rightPressed = rightPressed;
  joyData.downPressed = downPressed;
  joyData.leftPressed = leftPressed;
  joyData.upPressed = upPressed;
  joyData.selPressed = selPressed;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &joyData, sizeof(joyData));
}

//Reads the current joystick values and updates the tracking variables
//Returns true if the values have changed
bool readJoystick(){
    //Assume no reading has occured
    bool newReading = false;
    //get newest reading from the joystick
    int x = ss.analogRead(2);
    int y = ss.analogRead(3);
    //If it moved, update the position of the joystick
    if ( (abs(x - last_x) > 3)  ||  (abs(y - last_y) > 3)) {
        newReading = true;
        last_x = x;
        last_y = y;
    }    

    uint32_t buttons = ss.digitalReadBulk(button_mask);
    bool newRightPressed = !(buttons & (1 << BUTTON_RIGHT));
    bool newDownPressed = !(buttons & (1 << BUTTON_DOWN));
    bool newLeftPressed = !(buttons & (1 << BUTTON_LEFT));
    bool newUpPressed = !(buttons & (1 << BUTTON_UP));
    bool newSelPressed = ! (buttons & (1 << BUTTON_SEL));

    //If a button value changed, show that there was a new reading
    if (rightPressed != newRightPressed || 
        downPressed != newDownPressed || 
        leftPressed != newLeftPressed || 
        upPressed != newUpPressed || 
        selPressed != newSelPressed){
        newReading = true;
        rightPressed = newRightPressed;
        downPressed = newDownPressed;
        leftPressed = newLeftPressed;
        upPressed = newSelPressed;
    }

    return newReading;
}

//Prints odometry data to be read by matlab
void printOdometrySerial(){
  Serial.printf("%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n", odom_data.millis/1000.0, odom_data.x, odom_data.y, odom_data.theta, odom_data.pathDistance);
}