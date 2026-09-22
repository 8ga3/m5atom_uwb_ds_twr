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

// Ok: 距離を測定できた。Waiting: 時間内に何も届かなかった状態で、無通信中は
// 正常。Fail: 交信は始まったが途中で破綻した。
enum class DisplayState { Init, Ok, Waiting, Fail };

// M5.begin() の後に呼び、hasDisplay/hasLed を確定させて必要な方を初期化する。
void initStatusHardware();

// 128x128 の画面はテキストサイズ 2 だと 1 行 10 文字程度しか入らないため、
// ライブラリのエラー名はそのまま出すには長すぎる。
const char* errorShortName(M5Stamp_UWBError error);

uint16_t stateColor(DisplayState state);

// 書き込みのたびに RMT フレームが 1 回動くので、色が実際に変わったときだけ
// LED へ反映する。
void setLed(uint8_t red, uint8_t green, uint8_t blue);

// 画面のない Lite 系では状態のすべてを RGB LED 1 つで表す: RED = UWB
// トランシーバーが使えない、YELLOW = 無通信で待機中、GREEN = 測距交信中、
// MAGENTA = シリアルコンソールでの ID 入力待ち (showIdSetup() が直接光らせる)。
void updateLed(DisplayState state);

// ID 設定モードの表示。マゼンタは他のどの状態でも使わないので、画面のない
// Lite 系でも「シリアル入力待ちで止まっている」と一目で分かる。
void showIdSetup(const char* text, bool error);

// Wi-Fi 設定モードの表示。ID 設定と同じマゼンタで入力待ちを示し、見出しの
// 代わりに入力中の項目名 (ssid / pass) を出す。text には表示してよい文字列だけを
// 渡すこと - パスフレーズは呼び出し側で伏せ字に置き換える。
void showWifiSetup(const char* field, const char* text, bool error);
