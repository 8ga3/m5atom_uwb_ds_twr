#include "status.h"

#include <M5Unified.h>

#include "hw_pins.h"

Adafruit_NeoPixel rgbLed(1, LED_PIN, NEO_GRB + NEO_KHZ800);
bool hasLed     = false;
bool hasDisplay = false;
bool uwbReady   = false;

uint16_t idRangeMin = 0;
uint16_t idRangeMax = 0;

namespace {
uint32_t lastLedColor = UINT32_MAX;
}  // namespace

void initStatusHardware()
{
    // AtomS3 は LCD 搭載で RGB LED はなく、Lite 系は RGB LED 搭載で LCD はない。
    // どちらの基板でも同じスケッチで動かせるよう、M5.begin() の後にパネルの
    // 有無を調べる。getDisplayCount() は安全な判定方法 - 画面が検出されて
    // いないとき M5.Display.width() はヌルパネルを参照してしまう。
    hasDisplay = (M5.getDisplayCount() > 0);
    if (hasDisplay) {
        M5.Display.setTextSize(2);
    }
    // 画面があるのは AtomS3 (LED非搭載)、無いのは Lite 系 (LED搭載)。
    hasLed = !hasDisplay;
    if (hasLed) {
        rgbLed.begin();
        rgbLed.setBrightness(LED_BRIGHTNESS);
        rgbLed.clear();
        rgbLed.show();
    }
}

const char* errorShortName(M5Stamp_UWBError error)
{
    switch (error) {
        case M5Stamp_UWBError::Ok:                    return "OK";
        case M5Stamp_UWBError::InvalidConfig:         return "CFG";
        case M5Stamp_UWBError::SpiNotReady:           return "SPI";
        case M5Stamp_UWBError::ProbeFailed:           return "PROBE";
        case M5Stamp_UWBError::DeviceIdMismatch:      return "DEVID";
        case M5Stamp_UWBError::InitFailed:            return "INIT";
        case M5Stamp_UWBError::ConfigFailed:          return "CFGFAIL";
        case M5Stamp_UWBError::TxDataFailed:          return "TXDATA";
        case M5Stamp_UWBError::TxStartFailed:         return "TXSTART";
        case M5Stamp_UWBError::TxTimeout:             return "TXTMO";
        case M5Stamp_UWBError::RxStartFailed:         return "RXSTART";
        case M5Stamp_UWBError::RxTimeout:             return "RXTMO";
        case M5Stamp_UWBError::RxError:               return "RXERR";
        case M5Stamp_UWBError::RxBufferTooSmall:      return "RXBUF";
        case M5Stamp_UWBError::FrameParseFailed:      return "PARSE";
        case M5Stamp_UWBError::RangeTimestampInvalid: return "TSBAD";
        case M5Stamp_UWBError::RangeFrameMismatch:    return "FRMMIS";
        case M5Stamp_UWBError::InvalidArgument:       return "ARG";
        case M5Stamp_UWBError::Busy:                  return "BUSY";
        default:                                      return "UNK";
    }
}

uint16_t stateColor(DisplayState state)
{
    switch (state) {
        case DisplayState::Ok:      return GREEN;
        case DisplayState::Waiting: return YELLOW;
        case DisplayState::Init:    return uwbReady ? GREEN : RED;
        default:                    return RED;
    }
}

void setLed(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!hasLed) return;

    const uint32_t color = rgbLed.Color(red, green, blue);
    if (color == lastLedColor) return;
    lastLedColor = color;
    rgbLed.setPixelColor(0, color);
    rgbLed.show();
}

void updateLed(DisplayState state)
{
    uint8_t red = 0, green = 0;
    if (!uwbReady) {
        red = 255;                 // RED
    } else if (state == DisplayState::Ok) {
        green = 255;                // GREEN
    } else {
        red = green = 255;          // YELLOW
    }
    setLed(red, green, 0);
}

void showIdSetup(const char* text, bool error)
{
    setLed(255, 0, 255);
    if (!hasDisplay) return;

    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setCursor(0, 0);
    M5.Display.setTextColor(error ? RED : MAGENTA);
    M5.Display.println("SET ID");
    M5.Display.setTextColor(WHITE);
    // ANCHOR の ID_MAX は 5 桁あり "min-max" を 1 行に収めると欠ける。TAG は
    // 3 桁で余裕があるが、共通化のため両者とも 2 行表示に揃える。
    M5.Display.printf("%u-\n%u\n", static_cast<unsigned>(idRangeMin), static_cast<unsigned>(idRangeMax));
    M5.Display.println("SERIAL");
    M5.Display.setTextColor(error ? RED : GREEN);
    M5.Display.printf(">%s\n", text);
}

void showWifiSetup(const char* field, const char* text, bool error)
{
    setLed(255, 0, 255);
    if (!hasDisplay) return;

    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setCursor(0, 0);
    M5.Display.setTextColor(error ? RED : MAGENTA);
    M5.Display.println("SET WIFI");
    M5.Display.setTextColor(WHITE);
    M5.Display.println(field);
    M5.Display.println("SERIAL");
    M5.Display.setTextColor(error ? RED : GREEN);
    M5.Display.printf(">%s\n", text);
}
