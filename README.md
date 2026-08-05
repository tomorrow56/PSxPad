# Arduino Simple PlayStation 1 Pad Controller Decoder Library

Based on PSX Library

http://playground.arduino.cc/Main/PSXLibrary

Written by: Kevin Ahrendt June 22nd, 2008

Registered by tomorrow56 in 2026

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.
If not, see <http://www.gnu.org/licenses/>.

## Overview
PsxPad is an Arduino library for reading button states from a
PlayStation 1 (PSX) controller by directly driving its
Data/Command/Attention/Clock lines. It targets ESP32-based boards
(tested on M5Stack ATOM / ATOMS3) and returns all button states as a
single `uint16_t` bitmask.

## Installation
1. Download or clone this repository.
2. Copy (or symlink) the `PSxPad` folder into your Arduino
   `libraries` directory (e.g. `~/Documents/Arduino/libraries/`).
3. Restart the Arduino IDE, or open the project with the Arduino CLI /
   PlatformIO, then `#include <PsxPad.h>` in your sketch.

### Wire Connection Information
|PSX-con|Sig|Wire|
|-------|---|----|
|1|DATA|BROWN (pull-up)|
|2|CMD|ORANGE|
|3|9V|-|
|4|GND|-|
|5|VCC (3.3-5V)|-|
|6|ATT|YELLOW|
|7|CLK|BLUE|
|8|N/C|-|
|9|Ack|-|

### Example Pin Assignment (M5AtomS3)
|Pin|Signal|
|---|------|
|G5|DATA|
|G6|CMD|
|G7|ATT|
|G8|CLK|

## API

### Constructor
```cpp
PsxPad(byte dataPin, byte cmndPin, byte attPin, byte clockPin, byte delay);
```
Creates a `PsxPad` instance bound to the given GPIO pin numbers for
Data, Command, Attention and Clock, plus a bit-bang delay (in
microseconds) between clock transitions.

### begin()
```cpp
void begin();
```
Configures the pin modes for communicating with the controller. Call
this once in `setup()`.

### read()
```cpp
uint16_t read();
```
Polls the controller and returns the current button state as a
bitmask. Use the constants below with the `&` operator to check
individual buttons.

### Button constants
|Constant|Bit value|Button|
|--------|---------|------|
|`psxLeft`|`0x0001`|Left|
|`psxDown`|`0x0002`|Down|
|`psxRight`|`0x0004`|Right|
|`psxUp`|`0x0008`|Up|
|`psxStrt`|`0x0010`|Start|
|`psxSlct`|`0x0080`|Select|
|`psxSqu`|`0x0100`|Square|
|`psxX`|`0x0200`|Cross|
|`psxO`|`0x0400`|Circle|
|`psxTri`|`0x0800`|Triangle|
|`psxR1`|`0x1000`|R1|
|`psxL1`|`0x2000`|L1|
|`psxR2`|`0x4000`|R2|
|`psxL2`|`0x8000`|L2|

## Usage
```cpp
#include <PsxPad.h>

PsxPad psx = PsxPad(5, 6, 7, 8, 10);

void setup() {
  Serial.begin(115200);
  psx.begin();
}

void loop() {
  uint16_t state = psx.read();
  if (state & psxX) {
    Serial.println("Cross");
  }
  delay(20);
}
```

## Examples
|Example|Description|
|-------|-----------|
|`PsxPad_test`|Prints the name of any pressed button to the Serial monitor.|
|`PSxPad_espnow`|Sends button presses to a paired ESP-NOW peer (placeholder MAC address), with an on-board LED status indicator.|
|`PSxPad_BLE_Gamepad`|Exposes the controller as a BLE HID gamepad using the [ESP32-BLE-Gamepad](https://github.com/lemmingDev/ESP32-BLE-Gamepad) library (buttons + D-pad hat switch).|

## Requirements
- Arduino IDE or Arduino CLI with ESP32 board support
- [M5Unified](https://github.com/m5stack/M5Unified) (for the `PSxPad_espnow` and `PSxPad_BLE_Gamepad` examples)
- [FastLED](https://github.com/FastLED/FastLED) (for the `PSxPad_espnow` example)
- [ESP32-BLE-Gamepad](https://github.com/lemmingDev/ESP32-BLE-Gamepad) (for the `PSxPad_BLE_Gamepad` example)
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) (required by ESP32-BLE-Gamepad, for the `PSxPad_BLE_Gamepad` example)

## License
This project is licensed under the GNU General Public License v3.0.
See [LICENSE](LICENSE) for details.
