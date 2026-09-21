// Stamp-UWB / LED / ボタンの配線定数。ANCHOR と TAG で共通。
#pragma once

// Stamp-UWB のホスト側配線。AtomS3 と AtomS3 Lite は同じブレイクアウトを
// 使えるが、旧世代 ESP32 の Atom Lite はまったく使えない: GPIO6-8 は内蔵 SPI
// フラッシュに配線済み、GPIO38/39 は入力専用のため、汎用ヘッダーピン 6 本から
// 独自にマッピングを組んでいる。
#if defined(CONFIG_IDF_TARGET_ESP32S3)
static constexpr int UWB_PIN_IRQ  = 39;
static constexpr int UWB_PIN_RST  = 38;
static constexpr int UWB_PIN_MISO = 5;
static constexpr int UWB_PIN_MOSI = 6;
static constexpr int UWB_PIN_SCK  = 7;
static constexpr int UWB_PIN_CS   = 8;
#else
static constexpr int UWB_PIN_IRQ  = 21;
static constexpr int UWB_PIN_RST  = 25;
static constexpr int UWB_PIN_MISO = 22;
static constexpr int UWB_PIN_MOSI = 19;
static constexpr int UWB_PIN_SCK  = 23;
static constexpr int UWB_PIN_CS   = 33;
#endif

// M5Unified 0.2.22 の M5.Led は ESP-IDF 5.0 以降の RMT ドライバ専用で、
// espressif32 7.1.3 (IDF 4.4.7) では LedBus_RMT::init() が空実装のまま false を
// 返す。isEnabled() はインスタンスの有無しか見ないので true を返し、一切点灯
// しないことに気付けない。そのため NeoPixel で直接駆動する。
//
// ピンはボード判別ではなくビルドターゲットで決める。M5Unified は G34 が
// フローティングの Atom Lite を AtomU と誤検出することがあり (実測 board=130)、
// getBoard() を信用できない。
#if defined(CONFIG_IDF_TARGET_ESP32S3)
static constexpr int LED_PIN = 35;  // AtomS3 Lite (AtomS3 は画面があり LED 非搭載)
#else
static constexpr int LED_PIN = 27;  // Atom Lite
#endif
// Atom の RGB LED は最大輝度だと直視しづらいほど明るい。
static constexpr uint8_t LED_BRIGHTNESS = 40;

// 本体ボタン。起動時に押されていたら ID 設定モードへ入る。LED と同じ理由で
// getBoard() は信用せず、ビルドターゲットでピンを決める。Atom Lite の G39 は
// 入力専用ピンで内部プルアップを持たないが、基板側にプルアップがある。
#if defined(CONFIG_IDF_TARGET_ESP32S3)
static constexpr int BTN_PIN      = 41;  // AtomS3 / AtomS3 Lite
static constexpr uint8_t BTN_MODE = INPUT_PULLUP;
#else
static constexpr int BTN_PIN      = 39;  // Atom Lite
static constexpr uint8_t BTN_MODE = INPUT;
#endif

// ドライバが 2MHz のプローブレートを抜けた後に使う SPI クロック。QM33120 自体は
// 38MHz まで対応するので、制限はホスト側バスにある: 旧世代 ESP32 では UWB の
// 全信号が GPIO マトリクスを経由するため、全二重転送は 20MHz が上限になる。
// 20MHz は 80MHz の APB クロックをちょうど割り切る値 (80/4) でもあり、ライブラリ
// 既定の 16MHz (80/5) より都合が良い。ESP32-S3 側はこの点で余裕がある。
// 別のレートを試すときは -D UWB_SPI_FAST_HZ=<hz> で上書きする。init() は指定
// レートでの読み戻しが壊れていればライブラリ既定値へフォールバックする。
#ifndef UWB_SPI_FAST_HZ
#define UWB_SPI_FAST_HZ 20000000
#endif
static constexpr uint32_t UWB_SPI_FAST_FALLBACK_HZ = 16000000;
