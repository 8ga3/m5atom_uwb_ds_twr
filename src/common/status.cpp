#include "status.h"

#include <M5Unified.h>

#include "hw_pins.h"

Adafruit_NeoPixel rgbLed(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
bool hasLed     = false;
bool hasDisplay = false;
bool uwbReady   = false;

uint16_t idRangeMin = 0;
uint16_t idRangeMax = 0;

namespace {
uint32_t lastLedColor = UINT32_MAX;

// 画面と同じ大きさのオフスクリーン描画先。initStatusHardware() で確保する。
// 128x128 の 16 ビット色で 32 KB 使う。
M5Canvas frameCanvas;
bool frameCanvasReady = false;

#if defined(UWB_ATOM_MATRIX)
// 3x5 ドットの数字。各行の下位 3 ビットが左から右の列に対応する。5x5 の
// 中央 3 列に描く。
constexpr uint8_t DIGIT_FONT[10][LED_MATRIX_SIZE] = {
    {0b111, 0b101, 0b101, 0b101, 0b111},  // 0
    {0b010, 0b110, 0b010, 0b010, 0b111},  // 1
    {0b111, 0b001, 0b111, 0b100, 0b111},  // 2
    {0b111, 0b001, 0b111, 0b001, 0b111},  // 3
    {0b101, 0b101, 0b111, 0b001, 0b001},  // 4
    {0b111, 0b100, 0b111, 0b001, 0b111},  // 5
    {0b111, 0b100, 0b111, 0b101, 0b111},  // 6
    {0b111, 0b001, 0b010, 0b010, 0b010},  // 7
    {0b111, 0b101, 0b111, 0b101, 0b111},  // 8
    {0b111, 0b101, 0b111, 0b001, 0b111},  // 9
};
constexpr uint8_t DIGIT_WIDTH = 3;

// 複数桁の ID は横スクロールではなく 1 桁ずつ切り替えて出す。スクロールは 1 秒に
// 10 回ほど LED を書き換える必要があり、書き換えの間 (1 回 1 ms 程度) は測距ループが
// 止まるため。切り替えなら書き換えは 1 秒に 2 回で済む。
// 同じ数字が続く ID (11 など) でも桁の区切りが分かるよう、桁の間は必ず一度消す。
// 最後の桁のあとは長めに消して、ID の先頭に戻ることを示す。
constexpr uint32_t DIGIT_ON_MS      = 800;
constexpr uint32_t DIGIT_GAP_MS     = 200;
constexpr uint32_t ID_REPEAT_GAP_MS = 1000;

char idText[6]        = "";  // 表示する ID の 10 進文字列。空の間は全面を点ける
size_t idLen          = 0;
size_t frameIndex     = 0;  // 偶数なら idText[frameIndex / 2] を表示し、奇数なら消灯
uint32_t frameStartMs = 0;

// 表示上の座標 (左上が原点) を LED の並び順へ変換する。LED_MATRIX_ROTATION の
// 1 つごとに表示を時計回りに 90 度回す。
uint16_t matrixIndex(uint8_t x, uint8_t y)
{
    constexpr uint8_t last = LED_MATRIX_SIZE - 1;
    uint8_t px = x, py = y;
    switch (LED_MATRIX_ROTATION & 3) {
        case 1: px = last - y; py = x;        break;
        case 2: px = last - x; py = last - y; break;
        case 3: px = y;        py = last - x; break;
        default:                              break;
    }
    return static_cast<uint16_t>((py * LED_MATRIX_SIZE) + px);
}

uint32_t frameDurationMs(size_t frame)
{
    if ((frame % 2) == 0) return DIGIT_ON_MS;
    return (((frame / 2) + 1) == idLen) ? ID_REPEAT_GAP_MS : DIGIT_GAP_MS;
}

// 今のフレームを lastLedColor の色で描いて LED へ送る。
void renderMatrix()
{
    if (lastLedColor == UINT32_MAX) return;  // まだ一度も色が決まっていない

    rgbLed.clear();
    if (idLen == 0) {
        rgbLed.fill(lastLedColor);
    } else if ((frameIndex % 2) == 0) {
        const uint8_t* glyph = DIGIT_FONT[idText[frameIndex / 2] - '0'];
        constexpr uint8_t left = (LED_MATRIX_SIZE - DIGIT_WIDTH) / 2;
        for (uint8_t y = 0; y < LED_MATRIX_SIZE; ++y) {
            for (uint8_t x = 0; x < DIGIT_WIDTH; ++x) {
                if (glyph[y] & (1u << (DIGIT_WIDTH - 1 - x))) {
                    rgbLed.setPixelColor(matrixIndex(left + x, y), lastLedColor);
                }
            }
        }
    }
    rgbLed.show();
}
#endif
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
        // 画面へ送るときに色深度を変換しなくて済むよう、パネルと同じ 16 ビット色にする。
        frameCanvas.setColorDepth(16);
        frameCanvasReady = (frameCanvas.createSprite(M5.Display.width(), M5.Display.height()) != nullptr);
        if (frameCanvasReady) frameCanvas.setTextSize(2);
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

lgfx::LovyanGFX& beginDisplayFrame()
{
    lgfx::LovyanGFX& gfx = frameCanvasReady ? static_cast<lgfx::LovyanGFX&>(frameCanvas)
                                            : static_cast<lgfx::LovyanGFX&>(M5.Display);
    gfx.fillScreen(TFT_BLACK);
    gfx.setCursor(0, 0);
    return gfx;
}

void endDisplayFrame()
{
    if (frameCanvasReady) frameCanvas.pushSprite(&M5.Display, 0, 0);
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
#if defined(UWB_ATOM_MATRIX)
    renderMatrix();
#else
    rgbLed.setPixelColor(0, color);
    rgbLed.show();
#endif
}

void setLedId(uint16_t id)
{
#if defined(UWB_ATOM_MATRIX)
    snprintf(idText, sizeof(idText), "%u", static_cast<unsigned>(id));
    idLen        = strlen(idText);
    frameIndex   = 0;
    frameStartMs = millis();
    if (hasLed) renderMatrix();
#else
    (void)id;
#endif
}

void serviceLed()
{
#if defined(UWB_ATOM_MATRIX)
    // 1 桁の ID は切り替える必要がないので、点けたままにする。
    if (!hasLed || (idLen <= 1)) return;

    const uint32_t nowMs = millis();
    if ((nowMs - frameStartMs) < frameDurationMs(frameIndex)) return;
    frameIndex   = (frameIndex + 1) % (idLen * 2);
    frameStartMs = nowMs;
    renderMatrix();
#endif
}

void updateLed(DisplayState state)
{
    uint8_t red = 0, green = 0;
    if (!uwbReady || (state == DisplayState::Fail)) {
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

    lgfx::LovyanGFX& gfx = beginDisplayFrame();
    gfx.setTextColor(error ? RED : MAGENTA);
    gfx.println("SET ID");
    gfx.setTextColor(WHITE);
    // ANCHOR の ID_MAX は 5 桁あり "min-max" を 1 行に収めると欠ける。TAG は
    // 3 桁で余裕があるが、共通化のため両者とも 2 行表示に揃える。
    gfx.printf("%u-\n%u\n", static_cast<unsigned>(idRangeMin), static_cast<unsigned>(idRangeMax));
    gfx.println("SERIAL");
    gfx.setTextColor(error ? RED : GREEN);
    gfx.printf(">%s\n", text);
    endDisplayFrame();
}

void showWifiSetup(const char* field, const char* text, bool error)
{
    setLed(255, 0, 255);
    if (!hasDisplay) return;

    lgfx::LovyanGFX& gfx = beginDisplayFrame();
    gfx.setTextColor(error ? RED : MAGENTA);
    gfx.println("SET WIFI");
    gfx.setTextColor(WHITE);
    gfx.println(field);
    gfx.println("SERIAL");
    gfx.setTextColor(error ? RED : GREEN);
    gfx.printf(">%s\n", text);
    endDisplayFrame();
}

void showServerSetup(const char* field, const char* text, bool error)
{
    setLed(255, 0, 255);
    if (!hasDisplay) return;

    lgfx::LovyanGFX& gfx = beginDisplayFrame();
    gfx.setTextColor(error ? RED : MAGENTA);
    gfx.println("SET SRV");
    gfx.setTextColor(WHITE);
    gfx.println(field);
    gfx.println("SERIAL");
    gfx.setTextColor(error ? RED : GREEN);
    gfx.printf(">%s\n", text);
    endDisplayFrame();
}
