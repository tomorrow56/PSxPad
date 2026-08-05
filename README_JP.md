# Arduino Simple PlayStation 1 Pad Controller Decoder Library

PSX Library をベースにしています

http://playground.arduino.cc/Main/PSXLibrary

作者: Kevin Ahrendt 2008年6月22日

本プログラムはフリーソフトウェアです。フリーソフトウェア財団が発行する GNU General Public License（バージョン3、またはそれ以降のいずれかのバージョン）の条件の下で、再頒布や改変を行うことができます。

本プログラムは有用であることを願って頒布されていますが、*全くの無保証*です。商品性や特定目的への適合性についての黙示の保証さえありません。詳しくは GNU General Public License をご覧ください。

本プログラムと一緒に GNU General Public License のコピーを受け取ったはずです。もし受け取っていなければ、<http://www.gnu.org/licenses/> をご覧ください。

## 概要
PsxPad は、PlayStation 1 (PSX) コントローラーの Data/Command/Attention/Clock
ラインを直接制御してボタン状態を読み取るための Arduino ライブラリです。
ESP32系ボード（M5Stack ATOM / ATOMS3 で動作確認済み）を対象としており、
すべてのボタン状態を単一の `uint16_t` ビットマスクとして返します。

## インストール
1. 本リポジトリをダウンロードまたはクローンします。
2. `PSxPad` フォルダを Arduino の `libraries` ディレクトリ
   （例: `~/Documents/Arduino/libraries/`）にコピー（またはシンボリックリンク）します。
3. Arduino IDE を再起動するか、Arduino CLI / PlatformIO でプロジェクトを開き、
   スケッチ内で `#include <PsxPad.h>` します。

### 配線情報
|PSXコネクタ|信号|配線色|
|-------|---|----|
|1|DATA|BROWN（プルアップ）|
|2|CMD|ORANGE|
|3|9V|-|
|4|GND|-|
|5|VCC (3.3-5V)|-|
|6|ATT|YELLOW|
|7|CLK|BLUE|
|8|N/C|-|
|9|Ack|-|

### ピン割当例（M5AtomS3）
|ピン|信号|
|---|------|
|G5|DATA|
|G6|CMD|
|G7|ATT|
|G8|CLK|

## API

### コンストラクタ
```cpp
PsxPad(byte dataPin, byte cmndPin, byte attPin, byte clockPin, byte delay);
```
Data、Command、Attention、Clock の各GPIOピン番号と、クロック遷移間の
ビットバンギング遅延（マイクロ秒）を指定して `PsxPad` インスタンスを生成します。

### begin()
```cpp
void begin();
```
コントローラーと通信するためのピンモードを設定します。`setup()` 内で
一度だけ呼び出してください。

### read()
```cpp
uint16_t read();
```
コントローラーをポーリングし、現在のボタン状態をビットマスクとして
返します。個々のボタンを確認するには、下記の定数を `&` 演算子と
組み合わせて使用します。

### ボタン定数
|定数|ビット値|ボタン|
|--------|---------|------|
|`psxLeft`|`0x0001`|左|
|`psxDown`|`0x0002`|下|
|`psxRight`|`0x0004`|右|
|`psxUp`|`0x0008`|上|
|`psxStrt`|`0x0010`|Start|
|`psxSlct`|`0x0080`|Select|
|`psxSqu`|`0x0100`|□（Square）|
|`psxX`|`0x0200`|×（Cross）|
|`psxO`|`0x0400`|○（Circle）|
|`psxTri`|`0x0800`|△（Triangle）|
|`psxR1`|`0x1000`|R1|
|`psxL1`|`0x2000`|L1|
|`psxR2`|`0x4000`|R2|
|`psxL2`|`0x8000`|L2|

## 使い方
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

## サンプル
|サンプル|説明|
|-------|-----------|
|`PsxPad_test`|押されたボタン名をシリアルモニタに出力します。|
|`PSxPad_espnow`|ボタン入力をペアリング済みのESP-NOWピア（暫定MACアドレス）へ送信し、オンボードLEDでステータスを表示します。|
|`PSxPad_BLE_Gamepad`|[ESP32-BLE-Gamepad](https://github.com/lemmingDev/ESP32-BLE-Gamepad) ライブラリを使用して、コントローラーをBLE HIDゲームパッド（ボタン＋十字キーのhat switch）として公開します。|

## 必要環境
- ESP32ボードサポート付きの Arduino IDE または Arduino CLI
- [M5Unified](https://github.com/m5stack/M5Unified)（`PSxPad_espnow` および `PSxPad_BLE_Gamepad` サンプル用）
- [FastLED](https://github.com/FastLED/FastLED)（`PSxPad_espnow` サンプル用）
- [ESP32-BLE-Gamepad](https://github.com/lemmingDev/ESP32-BLE-Gamepad)（`PSxPad_BLE_Gamepad` サンプル用）
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)（ESP32-BLE-Gamepadが必要とする、`PSxPad_BLE_Gamepad` サンプル用）

## ライセンス
本プロジェクトは GNU General Public License v3.0 の下でライセンスされています。
詳細は [LICENSE](LICENSE) を参照してください。
