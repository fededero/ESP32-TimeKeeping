#include <WiFi.h>
#include <esp_now.h>

#include <LiquidCrystal_I2C.h>

#include <NimBLEDevice.h>

#define CELLPIN 23

/*
 *  LAP Timing system for Formula Student events
 *  DYNAMIS PRC - 2024
 *  Master board (START/NEW LAP) firmware
 *  
 *  Visit on a BLE Enabled Browser: https://federicoderocco.altervista.org/dynamis/TimeKeeping.html
*/

//Change to SLAVE MAC address (PIT/END Unit)
uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x4D, 0x2D, 0xBC};

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

typedef enum checks_t{
  ble=0,
  wifi,
  laser,
  timing
};

typedef struct message_t{
  uint32_t delay;
} message_t;

message_t message;

timingMode_t timingMode;

uint32_t startTime, stopTime, deltaTime, sumOfLap, lastButtonPress, lastWifiCheck;
bool isTiming=false, isWifi=false, newLap=false, inPit=false, sendPit=false;
bool isTimingOld = false, isWifiOld=false, isCellOld=false;
uint8_t lapNumber;

uint32_t dataWifi=0;

esp_now_peer_info_t peerInfo;

int lcdColumns = 16, lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);


byte laserIcon[] = {
  B11111,
  B01110,
  B10101,
  B00100,
  B00000,
  B00100,
  B00000,
  B00100
};

byte wifiIcon[] = {
  B01110,
  B10001,
  B00100,
  B01010,
  B00000,
  B00100,
  B00000,
  B11111
};

byte checkIcon[] = {
  B00000,
  B00000,
  B00000,
  B00001,
  B00010,
  B10100,
  B01000,
  B00000
};

byte clockIcon[] = {
  B01110,
  B10001,
  B10101,
  B10111,
  B10001,
  B10001,
  B01110,
  B00000
};



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

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  isWifi=!status;
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
  DisplayInit();

  pinMode(CELLPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CELLPIN), lapCellInterrupt, FALLING);


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

 
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {

    setCheckFalse(ble);

    delay(50); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising

    oldDeviceConnected = deviceConnected;
  }
  
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    setCheckTrue(ble);
  }

  if(isCellOld != digitalRead(CELLPIN)){
    if(isCellOld)
      setCheckTrue(laser);
    else
      setCheckFalse(laser);
    isCellOld = !isCellOld;
  }

  if(isTiming != isTimingOld){
    if(isTiming)
      setCheckTrue(timing);
    else
      setCheckFalse(timing);
    isTimingOld = isTiming;
  }

  if(isWifi != isWifiOld){
    if(isWifi)
      setCheckTrue(wifi);
    else
      setCheckFalse(wifi);
    isWifiOld = isWifi;
  }

  if(millis()-lastWifiCheck>2000){
    esp_err_t result = esp_now_send(0, (uint8_t *) &dataWifi, sizeof(uint32_t));
    lastWifiCheck=millis();
  }

  
}

void DisplayInit(){
  lcd.init();
  lcd.backlight();
  
  displayMode();
  lapTimeToDisplay("000");

  lcd.createChar(0, checkIcon);
  lcd.createChar(1, wifiIcon);
  lcd.createChar(2, laserIcon);
  lcd.createChar(3, clockIcon);

  lcd.setCursor(12, 0);
  lcd.write('B');
  lcd.write(byte(1));
  lcd.write(byte(2));
  lcd.write(byte(3));

  lcd.setCursor(12, 1);
  lcd.print("xxxx");
  
  return;
}

void setCheckFalse(checks_t check){
  lcd.setCursor(check+12, 1);
  lcd.write('x');
  return;
}

void setCheckTrue(checks_t check){
  lcd.setCursor(check+12, 1);
  lcd.write(byte(0));
  return;
}

void displayMode(){
  lcd.setCursor(0, 0);
  switch(timingMode){
    case lap:
      lcd.print("LAP");
    break;
    case acceleration:
      lcd.print("ACC");
    break;
    case skidpad:
      lcd.print("SKD");
    break;   
  }

  return;
}

String lapNumberToString(){
  String lapStr;
  lapStr = String("").c_str();
  if(lapNumber<10) lapStr.concat("0");
  if(lapNumber<100) lapStr.concat("0");
  lapStr.concat(String(lapNumber).c_str());

  
  return lapStr;
}

void sendDelta(String lapStr){
  deltaToElapsed(deltaTime);
  if(deviceConnected) lapTimeToBLE(lapStr);
  lapTimeToDisplay(lapStr);
  lapTimeToSerial(lapStr);
  
  return;
}

//Starts ESP-NOW communication with slave unit
void WiFiInit(){

    //Start WiFi in ESP-NOW Mode
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {

    }

    //Set callback functions
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(esp_now_recv_cb_t(lapSlaveInterrupt));

    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    //Add Slave unit
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){

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

  

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();

  return;
}


void deltaToElapsed(uint32_t milliseconds){
  
  elapsedTime.milliseconds=milliseconds%1000;
  milliseconds/=1000;

  elapsedTime.seconds=milliseconds%60;
  elapsedTime.minutes=milliseconds/60;

  return;
}

String elapsedToString(elapsedTime_t et){
  String temp = String("").c_str();

  if(et.minutes<10) temp.concat(String(0).c_str());
  temp.concat(String(et.minutes).c_str());

  temp.concat(String(":").c_str());

  if(et.seconds<10) temp.concat(String(0).c_str());
  temp.concat(String(et.seconds).c_str());

  temp.concat(String(":").c_str());

  if(et.milliseconds<10) temp.concat(String(0).c_str());
  if(et.milliseconds<100) temp.concat(String(0).c_str());
  temp.concat(String(et.milliseconds).c_str());


  return temp;
}

void lapTimeToBLE(String lapStr){
    sendBLE = String(deltaTime).c_str();
    sendBLE.concat(lapStr.c_str());
    pSensorCharacteristic->setValue(sendBLE);
    pSensorCharacteristic->notify();
  
  return;
}

void lapTimeToDisplay(String lapStr){
  lcd.setCursor(6, 0);
  lcd.print(lapStr);


  lcd.setCursor(0, 1);
  lcd.print(elapsedToString(elapsedTime));
  
  return;
}

void lapTimeToSerial(String lapStr){
    sendBLE = String(deltaTime).c_str();
    sendBLE.concat(lapStr.c_str());
    Serial.println(sendBLE);
  
  return;
}

void modeToLap(){
  detachInterrupt(CELLPIN);
  attachInterrupt(CELLPIN, lapCellInterrupt, FALLING);

  timingMode=lap;
  
  esp_now_unregister_recv_cb();
  esp_now_register_recv_cb(esp_now_recv_cb_t(lapSlaveInterrupt));

  displayMode();

  return;
}

void modeToAcc(){
  detachInterrupt(CELLPIN);
  attachInterrupt(CELLPIN, accCellInterrupt, FALLING);

  timingMode=acceleration;
  
  esp_now_unregister_recv_cb();
  esp_now_register_recv_cb(esp_now_recv_cb_t(accSlaveInterrupt));

  displayMode();

  return;
}

void modeToSkid(){
  detachInterrupt(CELLPIN);
  attachInterrupt(CELLPIN, skidCellInterrupt, FALLING);

  timingMode=skidpad;

  esp_now_unregister_recv_cb();

  displayMode();

  return;
}
