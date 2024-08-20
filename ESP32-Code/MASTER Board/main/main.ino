#include <WiFi.h>
#include <esp_now.h>
#define CELLPIN 23

/*
 *  LAP Timing system for Formula Student events
 *  DYNAMIS PRC - 2024
 *  Master board (START/NEW LAP) firmware
 *  
 *  Connect via Bluetooth, then visit: www.test.com
*/

struct elapsedTime_t{
  uint8_t minutes;
  uint8_t seconds;
  uint16_t milliseconds;
};
elapsedTime_t elapsedTime;

typedef enum timingMode_t{
  lap = 0,
  acceleration,
  skidpad
};
timingMode_t timingMode;

//Change to SLAVE MAC address (PIT/END Unit)
uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x4D, 0x2D, 0xBC};
uint32_t startTime, stopTime, deltaTime;
bool isTiming, newLap=false;
uint8_t lapNumber;

esp_now_peer_info_t peerInfo;


void lapSlaveInterrupt(const uint8_t * mac, const uint8_t *incomingData, int len){
  if(isTiming){
    stopTime=millis();
    deltaTime=stopTime-startTime;
    startTime=stopTime;
    newLap=true;
    isTiming=false;
    lapNumber++;
  }
  return;
}

void accSlaveInterrupt(const uint8_t * mac, const uint8_t *incomingData, int len){
  if(isTiming){
    stopTime=millis();
    deltaTime=stopTime-startTime;
    isTiming=false;
    newLap=true;
  }
  return;
}


void IRAM_ATTR lapCellInterrupt(){
  if(isTiming){
    stopTime=millis();
    deltaTime=stopTime-startTime;
    startTime=stopTime;
    newLap=true;
    lapNumber++;
  }
  else{
    startTime=millis();
    isTiming=true;    
    lapNumber=0;
  }
  return;
}

void IRAM_ATTR accCellInterrupt(){
  if(!isTiming){
    startTime=millis();
    isTiming=true;
  }

  return;
}

void IRAM_ATTR skidCellInterrupt(){
  if(isTiming){
    stopTime=millis();
    deltaTime=stopTime-startTime;
    lapNumber++;
    if(lapNumber==5){
      isTiming=false;
    }
    newLap=true;
  }
  else{
    startTime=millis();
    lapNumber=1;
    isTiming=true;
  }
  return;
}



void setup() {
  timingMode=lap;
  elapsedTime.minutes=0;
  elapsedTime.seconds=0;
  elapsedTime.milliseconds=0;
  isTiming=false;

  //COM Init
  Serial.begin(115200);
  WiFiInit();

  pinMode(CELLPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CELLPIN), lapCellInterrupt, FALLING);

  Serial.println("Power On Master Board");
}

void loop() {
  
 if(newLap==true){
  deltaToElapsed(deltaTime);
  lapTimeToBLE();
  lapTimeToDisplay();
  lapTimeToSerial();
  newLap=false;
 }

}

//Starts ESP-NOW communication with slave unit
void WiFiInit(){

    //Start WiFi in ESP-NOW Mode
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    }

    //Set callback functions
    esp_now_register_recv_cb(esp_now_recv_cb_t(lapSlaveInterrupt));

    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    //Add Slave unit
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
    }
  
    return;
}


void deltaToElapsed(uint32_t milliseconds){
  
  elapsedTime.milliseconds=milliseconds%1000;
  milliseconds/=1000;

  elapsedTime.seconds=milliseconds%60;
  elapsedTime.minutes=milliseconds/60;

  return;
}

void lapTimeToBLE(){
  return;
}

void lapTimeToDisplay(){
  return;
}

void lapTimeToSerial(){
  Serial.print("Lap N.");
  Serial.print(lapNumber);
  Serial.print("  ");
  Serial.print(elapsedTime.minutes);
  Serial.print(":");
  Serial.print(elapsedTime.seconds);
  Serial.print(":");
  Serial.println(elapsedTime.milliseconds);
  Serial.println(deltaTime);
  return;
}

void modeToLap(){
  detachInterrupt(CELLPIN);
  attachInterrupt(CELLPIN, lapCellInterrupt, FALLING);

  timingMode=lap;
  
  esp_now_unregister_recv_cb();
  esp_now_register_recv_cb(esp_now_recv_cb_t(lapSlaveInterrupt));

  return;
}

void modeToAcc(){
  detachInterrupt(CELLPIN);
  attachInterrupt(CELLPIN, accCellInterrupt, FALLING);

  timingMode=acceleration;
  
  esp_now_unregister_recv_cb();
  esp_now_register_recv_cb(esp_now_recv_cb_t(accSlaveInterrupt));

  return;
}

void modeToSkid(){
  detachInterrupt(CELLPIN);
  attachInterrupt(CELLPIN, skidCellInterrupt, FALLING);

  timingMode=skidpad;

  esp_now_unregister_recv_cb();

  return;
}
