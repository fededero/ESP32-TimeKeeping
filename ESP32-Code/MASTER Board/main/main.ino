#include <WiFi.h>
#include <esp_now.h>

#include <NimBLEDevice.h>

#define CELLPIN 23

/*
 *  LAP Timing system for Formula Student events
 *  DYNAMIS PRC - 2024
 *  Master board (START/NEW LAP) firmware
 *  
 *  Visit on a BLE Enabled Browser: www.test.com
*/

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;
String sendBLE = "";

#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"

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

typedef struct message_t{
  uint32_t delay;
} message_t;

message_t message;

timingMode_t timingMode;

//Change to SLAVE MAC address (PIT/END Unit)
uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x4D, 0x2D, 0xBC};
uint32_t startTime, stopTime, deltaTime, sumOfLap, lastButtonPress;
bool isTiming, newLap=false, inPit=false, sendPit=false;
uint8_t lapNumber;

esp_now_peer_info_t peerInfo;

void lapSlaveInterrupt(const uint8_t * mac, const uint8_t *incomingData, int len){
  if(isTiming && !inPit){
    memcpy(&message, incomingData, sizeof(message_t));
    stopTime=millis();
    deltaTime=stopTime-startTime-message.delay;
    startTime=stopTime;
    newLap=true;
    inPit=true;
    lapNumber++;
  }
  return;
}

void accSlaveInterrupt(const uint8_t * mac, const uint8_t *incomingData, int len){
  if(isTiming){
    memcpy(&message, incomingData, sizeof(message_t));
    stopTime=millis();
    deltaTime=stopTime-startTime-message.delay;
    isTiming=false;
    newLap=true;
  }
  return;
}


void IRAM_ATTR lapCellInterrupt(){
  if(isTiming){
    if((millis()-startTime)>1000){
      stopTime=millis();
      deltaTime=stopTime-startTime;
      startTime=stopTime;
      if(inPit){
        inPit=false;
        sendPit=true;
      }
      else{
        newLap=true;
        lapNumber++;
      }
    }
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
    if((millis()-startTime)>1000){
      stopTime=millis();
      deltaTime=stopTime-startTime;
      startTime=stopTime;
      lapNumber++;
      if(lapNumber==2 || lapNumber==4) sumOfLap+=deltaTime;
      if(lapNumber==4){
        isTiming=false;
      }
      newLap=true;
    }
  }
  else{
    sumOfLap=0;
    startTime=millis();
    lapNumber=0;
    isTiming=true;
  }
  return;
}

void IRAM_ATTR modeButtonInterrupt(){
  if((millis()-lastButtonPress)>500){
    lastButtonPress=millis();
    switch(timingMode){
      case lap:
        modeToSkid();
      break;
      case skidpad:
        modeToAcc();
      break;
      case acceleration:
        modeToLap();
      break;
    }
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
  BLEInit();

  pinMode(CELLPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CELLPIN), lapCellInterrupt, FALLING);

  Serial.println("Power On Master Board");
}

void loop() {
  
 if(newLap){
    
    newLap=false;
    if(timingMode==acceleration){
      sendDelta("ACC");
    }
    else{
      sendDelta(lapNumberToString());
      if(timingMode==skidpad && lapNumber==4){
        deltaTime=sumOfLap/2;
        sendDelta(String("AVG"));
      }
    }
 }

 if(sendPit){
  sendPit=false;
  sendDelta("PIT");
 }

 
  /*
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Device disconnected.");
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
  }*/

}

String lapNumberToString(){
  String lapStr;
  lapStr = String("").c_str();
  if(lapNumber<10) lapStr.concat("0");
  if(lapNumber<100) lapStr.concat("0");
  lapStr.concat(String(lapNumber).c_str());

  Serial.println(lapStr);
  Serial.println(lapNumber);
  
  return lapStr;
}

void sendDelta(String lapStr){
  deltaToElapsed(deltaTime);
  if(deviceConnected) lapTimeToBLE(lapStr);
  lapTimeToDisplay();
  lapTimeToSerial();
  
  return;
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

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};



void BLEInit(){

  // Create the BLE Device
  BLEDevice::init("TimeKeep");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
                      SENSOR_CHARACTERISTIC_UUID,
                      NIMBLE_PROPERTY::READ   |
                      NIMBLE_PROPERTY::WRITE  |
                      NIMBLE_PROPERTY::NOTIFY |
                      NIMBLE_PROPERTY::INDICATE
                    );

  // Create the ON button Characteristic
  

  // Register the callback for the ON button characteristic
 

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  //pSensorCharacteristic->addDescriptor(new BLE2902());
  

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

  return;
}


void deltaToElapsed(uint32_t milliseconds){
  
  elapsedTime.milliseconds=milliseconds%1000;
  milliseconds/=1000;

  elapsedTime.seconds=milliseconds%60;
  elapsedTime.minutes=milliseconds/60;

  return;
}

void lapTimeToBLE(String lapStr){
    sendBLE = String(deltaTime).c_str();
    sendBLE.concat(lapStr.c_str());
    pSensorCharacteristic->setValue(sendBLE);
    pSensorCharacteristic->notify();
  
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
