// LED / 画面表示と ID 設定 UI。ANCHOR と TAG で共通。
#pragma once

#include <Adafruit_NeoPixel.h>
#include <M5Stamp_UWB.h>

extern Adafruit_NeoPixel rgbLed;
extern bool hasLed;
extern bool hasDisplay;
// UWB リンクが上がっているか。setup() で initUwb() の結果を入れる。
extern bool uwbReady;

// showIdSetup() が表示する ID 範囲。setup() の先頭で役割ごとの ID_MIN/MAX を
// 入れておく (ANCHOR_ID_MIN/MAX または TAG_ID_MIN/MAX)。
extern uint16_t idRangeMin;
extern uint16_t idRangeMax;

// Ok: a distance was measured. Waiting: nothing arrived in time, which is
// normal while idle. Fail: the exchange started but broke down.
enum class DisplayState { Init, Ok, Waiting, Fail };

// M5.begin() の後に呼び、hasDisplay/hasLed を確定させて必要な方を初期化する。
void initStatusHardware();

// The 128x128 display fits about 10 characters per line at text size 2, so the
// library error names are far too long to print as-is.
const char* errorShortName(M5Stamp_UWBError error);

uint16_t stateColor(DisplayState state);

// Repeated writes each drive an RMT frame, so only push the LED when the
// color actually changes.
void setLed(uint8_t red, uint8_t green, uint8_t blue);

// On the screenless Lite boards the whole status is carried by the single RGB
// LED: RED = the UWB transceiver is unavailable, YELLOW = idle/waiting,
// GREEN = actively exchanging distances, MAGENTA = waiting for an ID on the
// serial console (showIdSetup() が直接光らせる)。
void updateLed(DisplayState state);

// ID 設定モードの表示。マゼンタは他のどの状態でも使わないので、画面のない
// Lite 系でも「シリアル入力待ちで止まっている」と一目で分かる。
void showIdSetup(const char* text, bool error);
