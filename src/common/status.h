// LED / 画面表示と ID 設定 UI。ANCHOR と TAG で共通。
#pragma once

#include <Adafruit_NeoPixel.h>
#include <M5GFX.h>
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

// 画面 1 枚ぶんの描画を始める。返した描画先は黒で消去済みで、カーソルは左上にある。
// 描き終えたら必ず endDisplayFrame() を呼ぶ。hasDisplay が true のときだけ呼ぶこと。
// 描画先は画面と同じ大きさのオフスクリーンのキャンバスで、endDisplayFrame() で
// まとめて画面へ送る。画面へ直接 fillScreen() してから文字を描くと、消去から
// 描き終わるまでの黒い画面が毎回見えてちらつくため。キャンバスのメモリを確保
// できなかったときは画面そのものを返し、従来どおり直接描く。
// 画面への送り方は起動時に決まり、displayPushMode() で分かる。
// - "task": キャンバスを 2 面確保でき、Core 0 のタスクも作れた場合。画面への転送は
//   そのタスクが行うので、endDisplayFrame() はすぐに戻る。実際に画面が書き換わるのは
//   数 ms 後で、送信中に次の画面を描き終えた場合は最新の 1 枚だけが送られる
// - "direct": 2 面目の確保かタスクの作成に失敗した場合。endDisplayFrame() がその場で
//   画面へ送るので、全画面の転送 (約 7 ms) が終わるまで戻らない。描いた画面は毎回送る。
//   キャンバスを 1 面も確保できなかったときは描画先が画面そのものになり、
//   endDisplayFrame() は何もしない
// どちらの場合も Core 1 (loop()) からだけ呼ぶこと。
lgfx::LovyanGFX& beginDisplayFrame();
void endDisplayFrame();

// 画面への送り方。起動ログ用。"task" = Core 0 のタスク、"direct" = 呼び出し元で
// 直接送る (2 面目かタスクを用意できなかった)、"none" = 画面なし。
const char* displayPushMode();

// 128x128 の画面はテキストサイズ 2 だと 1 行 10 文字程度しか入らないため、
// ライブラリのエラー名はそのまま出すには長すぎる。
const char* errorShortName(M5Stamp_UWBError error);

uint16_t stateColor(DisplayState state);

// 書き込みのたびに RMT フレームが 1 回動くので、色が実際に変わったときだけ
// LED へ反映する。
void setLed(uint8_t red, uint8_t green, uint8_t blue);

// 画面のない Lite 系では状態のすべてを RGB LED 1 つで表す: RED = UWB
// トランシーバーが使えない、または測距を開始できない (DisplayState::Fail。
// サーバー構成が無い場合を含む)、YELLOW = 無通信で待機中、GREEN = 測距交信中、
// MAGENTA = シリアルコンソールでの ID 入力待ち (showIdSetup() が直接光らせる)。
// Atom Matrix では同じ色で自機の ID を描く (setLedId() を参照)。
void updateLed(DisplayState state);

// Atom Matrix の 5x5 LED に表示する自機の ID を設定する。setup() の最後、設定の
// 入力待ちが終わってから呼ぶ。呼ぶまでは 25 個すべてを状態の色で点け、Lite 系の
// LED 1 個と同じ見え方にする。ID が 2 桁以上なら 1 桁ずつ約 1 秒ごとに切り替える。
// Atom Matrix 以外のビルドでは何もしない。
void setLedId(uint16_t id);

// setLedId() で設定した ID の桁の切り替えを進める。loop() から繰り返し呼ぶ。
// 切り替えのたびに LED の書き込みで約 1 ms 止まるので、UWB の交信を終えた直後など、
// 待たせても困らない位置で呼ぶ。Atom Matrix 以外のビルドでは何もしない。
void serviceLed();

// ID 設定モードの表示。マゼンタは他のどの状態でも使わないので、画面のない
// Lite 系でも「シリアル入力待ちで止まっている」と一目で分かる。
void showIdSetup(const char* text, bool error);

// Wi-Fi 設定モードの表示。ID 設定と同じマゼンタで入力待ちを示し、見出しの
// 代わりに入力中の項目名 (ssid / pass) を出す。text には表示してよい文字列だけを
// 渡すこと - パスフレーズは呼び出し側で伏せ字に置き換える。
void showWifiSetup(const char* field, const char* text, bool error);

// 測位サーバーの宛先 (IP / ポート) を入力するときの表示。Wi-Fi 設定と同じ形式で、
// 見出しだけを変える。field には srv_host / srv_port が入る。
void showServerSetup(const char* field, const char* text, bool error);
