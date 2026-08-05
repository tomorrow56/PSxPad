/********************
 * PSxPad BLE Gamepad example
 * Copyright(c) tomorrow56 all rights reserved
 *
 * PSxPad Arduino Library
 *  https://github.com/tomorrow56/PSxPad
 *
 * ESP32-BLE-Gamepad library
 *  https://github.com/lemmingDev/ESP32-BLE-Gamepad
 ********************/

#include <M5Unified.h>
#include <PsxPad.h>
#include <BleGamepad.h>

/********************
 * PSxPad parameters
 * PsxPad のピン設定
 ********************/
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

PsxPad PsxPad(dataPin, cmndPin, attPin, clockPin, 10);

/********************
 * BLE Gamepad settings
 * BLE ゲームパッドの設定
 ********************/
BleGamepad bleGamepad("PSxPad BLE");

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);

  PsxPad.begin();

  Serial.println("PSxPad BLE Gamepad");
  Serial.println("Starting BLE Gamepad...");
  // BLE ゲームパッドを開始
  bleGamepad.begin();
  Serial.println("Ready.");
}

void loop() {
  M5.update();

  // PSX コントローラーのボタン状態を取得
  uint16_t state = PsxPad.read();
  Serial.printf("Button state: 0x%04x\r\n", state);

  // ホストに接続されている場合のみレポートを送信
  if (bleGamepad.isConnected()) {
    // Face buttons
    // 各種ボタンをゲームパッドボタンへマッピング
    if (state & psxX) {
      bleGamepad.press(BUTTON_1);
    } else {
      bleGamepad.release(BUTTON_1);
    }

    if (state & psxO) {
      bleGamepad.press(BUTTON_2);
    } else {
      bleGamepad.release(BUTTON_2);
    }

    if (state & psxSqu) {
      bleGamepad.press(BUTTON_3);
    } else {
      bleGamepad.release(BUTTON_3);
    }

    if (state & psxTri) {
      bleGamepad.press(BUTTON_4);
    } else {
      bleGamepad.release(BUTTON_4);
    }

    if (state & psxL1) {
      bleGamepad.press(BUTTON_5);
    } else {
      bleGamepad.release(BUTTON_5);
    }

    if (state & psxR1) {
      bleGamepad.press(BUTTON_6);
    } else {
      bleGamepad.release(BUTTON_6);
    }

    if (state & psxL2) {
      bleGamepad.press(BUTTON_7);
    } else {
      bleGamepad.release(BUTTON_7);
    }

    if (state & psxR2) {
      bleGamepad.press(BUTTON_8);
    } else {
      bleGamepad.release(BUTTON_8);
    }

    if (state & psxSlct) {
      bleGamepad.press(BUTTON_9);
    } else {
      bleGamepad.release(BUTTON_9);
    }

    if (state & psxStrt) {
      bleGamepad.press(BUTTON_10);
    } else {
      bleGamepad.release(BUTTON_10);
    }

    // D-pad
    // 十字キーを hat switch へマッピング
    bool up    = state & psxUp;
    bool down  = state & psxDown;
    bool left  = state & psxLeft;
    bool right = state & psxRight;

    // 0: 中立, 1〜8: 上から時計回りに 8 方向
    int8_t hat = 0; // centered
    if (up && right) {
      hat = 2;
    } else if (right && down) {
      hat = 4;
    } else if (down && left) {
      hat = 6;
    } else if (left && up) {
      hat = 8;
    } else if (up) {
      hat = 1;
    } else if (right) {
      hat = 3;
    } else if (down) {
      hat = 5;
    } else if (left) {
      hat = 7;
    }

    bleGamepad.setHat1(hat);

    // Send the report to the host
    // ホストに HID レポートを送信
    bleGamepad.sendReport();
  }

  delay(10);
}
