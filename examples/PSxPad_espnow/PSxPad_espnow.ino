/*
 * PSxPad + ESP-NOW example
 * Reads a PSX controller via PsxPad library and sends button events
 * to a peer device over ESP-NOW.
 *
 * Registered by tomorrow56 in 2026
 */

#include <M5Unified.h>
#include <FastLED.h>
// Includes the Psx Library 
#include <PsxPad.h>
#include <esp_now.h>
#include <WiFi.h>

/********************
 * LED Setting
 ********************/
#define NUM_LEDS 1
// ATOM
//#define LED_DATA_PIN 27
// ATOMS3
#define LED_DATA_PIN 35
CRGB leds[NUM_LEDS];

/********************
 * PSxPad parameters
 ********************/
/****************
#define psxLeft		0x0001 
#define psxDown		0x0002
#define psxRight	0x0004
#define psxUp		  0x0008
#define psxStrt		0x0010
#define psxSlct		0x0080

#define psxSqu		0x0100
#define psxX		  0x0200
#define psxO		  0x0400
#define psxTri		0x0800
#define psxR1		  0x1000
#define psxL1		  0x2000
#define psxR2		  0x4000
#define psxL2		  0x8000
***************/

/***** Pin info *****
 M5AtomS3
 3v3
 G5     G39
 G6     G38
 G7     5V
 G8     GND

 M5Atom
 3v3
 G22    G21
 G19    G25
 G23    5V
 G33    GND
*********************/
// For M5Atom
// #define dataPin  22  // brown, pull-up
// #define cmndPin  19  // orange
// #define attPin   23  // yellow
// #define clockPin 33  // blue

// For M5AtomS3
#define dataPin  5  // brown, pull-up
#define cmndPin  6  // orange
#define attPin   7  // yellow
#define clockPin 8  // blue

// Initializes the library
// Defines what each pin is used
// (Data Pin #, Cmnd Pin #, Att Pin #, Clk Pin #, Delay)

PsxPad PsxPad(dataPin, cmndPin, attPin, clockPin, 10);

uint16_t state = 0;

/********************
 * ESPNow settings
 ********************/
esp_now_peer_info_t slave;
esp_err_t result;
boolean sending = false;

// 送信コールバック
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void OnDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  const uint8_t *mac_addr = info->des_addr;
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// 受信コールバック
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int data_len) {
  const uint8_t *mac_addr = info->src_addr;
#else
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
#endif
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.printf("Last Packet Recv from: %s\n", macStr);
  Serial.printf("Last Packet Recv Data(%d): ", data_len);
  for ( int i = 0 ; i < data_len ; i++ ) {
    Serial.print(data[i]);
    Serial.print(" ");
  }
  Serial.println("");
  // LEDを光らせる
  leds[0] = CRGB::Red;
  FastLED.show();
  delay(100);
  leds[0] = CRGB::Blue;
  FastLED.show();
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  PsxPad.begin();

  FastLED.addLeds<WS2811, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(20);

  // LEDを青にする
  leds[0] = CRGB::Blue;
  FastLED.show();
  delay(100);

  Serial.println("PSxPad + ESP-NOW");

  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }

  // Slave登録
  memset(&slave, 0, sizeof(slave));
  // Slave Mac Address: 01:02:03:04:05:06 (暫定値。接続先のMacアドレスに書き換えてください)
  // FF:FF:FF:FF:FF:FF にするとブロードキャストになる
  // Slave Mac Address: 01:02:03:04:05:06 (placeholder value; replace with the peer's actual MAC address)
  // Setting it to FF:FF:FF:FF:FF:FF makes it a broadcast
  slave.peer_addr[0] = (uint8_t)0x01;
  slave.peer_addr[1] = (uint8_t)0x02;
  slave.peer_addr[2] = (uint8_t)0x03;
  slave.peer_addr[3] = (uint8_t)0x04;
  slave.peer_addr[4] = (uint8_t)0x05;
  slave.peer_addr[5] = (uint8_t)0x06;

  esp_err_t addStatus = esp_now_add_peer(&slave);
  if (addStatus == ESP_OK) {
    // Pair success
    Serial.println("Pair success");
  }

  // ESP-NOWコールバック登録
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  M5.update();
  // Psx.read() initiates the PSX controller and returns the button data
  state = PsxPad.read();

  uint8_t data[1] = {0};

  if(state == 0){
    // LED CYAN
    leds[0] = CRGB::Cyan;
    FastLED.show();
    if(sending){
      data[0] = 0;
      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.println("STOP");
      sending = false;
    }
  }else{
    // LED RED
    leds[0] = CRGB::Red;
    FastLED.show();
    sending = true;
    if(state & psxSqu){
      data[0] = 1;
      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.println("PSx_Square");
    }else if(state & psxO){
      data[0] = 2;
      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.println("PSx_Circle");
    }else if(state & psxTri){
//      data[0] = 3;
//      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.println("PSx_Triangle");
    }else if(state & psxX){
//      data[0] = 3;
//      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.println("PSx_Cross");
    }else if(state & psxStrt){
      data[0] = 4;
      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.println("PSx_START");
    }else if(state & psxSlct){
//      data[0] = 3;
//      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.println("PSx_Select");
    }else if(state & psxUp){
      data[0] = 5;
      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.println("PSx_UP");
    }else if(state & psxDown){
      data[0] = 6;
      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.println("PSx_DOWN");
    }else if(state & psxLeft){
      data[0] = 7;
      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.println("PSx_LEFT");
    }else if(state & psxRight){
      data[0] = 8;
      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.println("PSx_RIGHT");
    }else if(state & psxR1){
//      data[0] = 3;
//      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.println("PSx_R1");
    }else if(state & psxL1){
//      data[0] = 3;
//      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.println("PSx_L1");
    }else if(state & psxR2){
//      data[0] = 3;
//      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.println("PSx_R2");
    }else if(state & psxL2){
//      data[0] = 3;
//      result = esp_now_send(slave.peer_addr, data, sizeof(data));
      Serial.println("PSx_L2");
    }else{
      Serial.println("Unknown");
    }
    Serial.print("Send Status: ");
    if (result == ESP_OK) {
      Serial.println("Success");
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
      Serial.println("ESPNOW not Init.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Invalid Argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
      Serial.println("Internal Error");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
      Serial.println("ESP_ERR_ESPNOW_NO_MEM");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      Serial.println("Peer not found.");
    } else {
      Serial.println("Not sure what happened");
    }    
  }
  delay(100);
}
