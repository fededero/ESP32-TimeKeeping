#include <esp_now.h>
#include <WiFi.h>

#define CELLPIN 23

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x4F, 0x90, 0xB8}; //c8:f0:9e:4f:90:b8
uint32_t lastCellTime=0;

typedef struct message_t{
  uint32_t delay;
} message_t;

message_t message;

esp_now_peer_info_t peerInfo;

//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  return;
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print(status);
  Serial.print(", last delay: ");
  Serial.println(millis()-lastCellTime);
  if(status){
    message.delay=millis()-lastCellTime;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message_t));
  }
  return;
  
}

void IRAM_ATTR cellInterrupt(){
  if((millis()-lastCellTime)>500){
    message.delay=0;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*) &message, sizeof(message_t));
    lastCellTime=millis();
  }
  return;
}
 
void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  esp_now_register_send_cb(OnDataSent);

  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  //INTERRUPT
  pinMode(CELLPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CELLPIN), cellInterrupt, FALLING);
}
 
void loop() {

}
