// Stamp-UWB / LED / ボタンの配線定数。ANCHOR と TAG で共通。
#pragma once

// Stamp-UWB host wiring. AtomS3 and AtomS3 Lite expose the same breakout, but
// the classic ESP32 Atom Lite cannot reuse it at all: GPIO6-8 are wired to the
// internal SPI flash and GPIO38/39 are input-only, so it gets its own mapping
// built from the six general-purpose header pins.
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
// The Atom RGB LED is uncomfortably bright at full scale.
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

// SPI clock used once the driver leaves its 2MHz probe rate. The QM33120 itself
// accepts up to 38MHz, so the host bus is the limit: on the classic ESP32 every
// UWB signal is routed through the GPIO matrix, which caps full-duplex transfers
// at 20MHz. 20MHz is also an exact divider of the 80MHz APB clock (80/4), unlike
// the library default of 16MHz (80/5). The ESP32-S3 has plenty of margin here.
// Override with -D UWB_SPI_FAST_HZ=<hz> to bench another rate; init() falls back
// to the library default if the readback at the requested rate is corrupt.
#ifndef UWB_SPI_FAST_HZ
#define UWB_SPI_FAST_HZ 20000000
#endif
static constexpr uint32_t UWB_SPI_FAST_FALLBACK_HZ = 16000000;
