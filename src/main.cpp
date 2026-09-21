// M5Atom S3 / AtomS3 Lite / Atom Lite UWB DS-TWR ANCHOR example code

#include <M5Unified.h>
#include <M5Stamp_UWB.h>
#include <Adafruit_NeoPixel.h>

static constexpr uint32_t LOG_INTERVAL = 20;
// The tag ranges every 200ms while our receive window is only 100ms, so quiet
// gaps between exchanges are normal. Hold the last result on screen this long
// before admitting the tag is actually gone.
static constexpr uint32_t IDLE_GRACE_MS = 1000;

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

Adafruit_NeoPixel rgbLed(1, LED_PIN, NEO_GRB + NEO_KHZ800);
bool hasLed = false;

M5Stamp_UWB uwb;
M5Stamp_UWBDSRangeConfig rangeConfig;
bool uwbReady          = false;
uint32_t responseCount = 0;
uint32_t failCount     = 0;
uint32_t noPollCount   = 0;
uint32_t noFinalCount  = 0;

// AtomS3 has an LCD and no RGB LED; the Lite variants have the RGB LED and no
// LCD. Probe for the panel after M5.begin() so one sketch serves either board.
// getDisplayCount() is the safe test - M5.Display.width() dereferences a null
// panel when no display was detected.
bool hasDisplay = false;

static bool initUwb(uint32_t spiFastHz)
{
    // Stamp-UWB connections to the QM33120 UWB transceiver.
    M5Stamp_UWBConfig config;
    config.pin_gp7    = M5STAMP_UWB_PIN_UNUSED;
    config.pin_wakeup = M5STAMP_UWB_PIN_UNUSED;
    config.pin_irq    = UWB_PIN_IRQ;
    config.pin_rst    = UWB_PIN_RST;
    config.pin_miso   = UWB_PIN_MISO;
    config.pin_mosi   = UWB_PIN_MOSI;
    config.pin_sck    = UWB_PIN_SCK;
    config.pin_cs     = UWB_PIN_CS;
    // Keep spi_slow_hz at the library default (2MHz): probing and the reset
    // sequence run before the chip's clock PLL is up, where only slow SPI is
    // guaranteed. Only the post-init rate is raised here.
    config.spi_fast_hz = spiFastHz;

    M5Stamp_UWBPHYConfig phy;
    // Channel 9 is the only UWB channel permitted in Japan; the tag must match.
    phy.channel = M5Stamp_UWBChannel::Channel9;

    // Both devices must use the same network addresses and DS-TWR timing.
    rangeConfig.panId                          = 0xDECA;
    rangeConfig.initiatorAddress               = 0x0001;
    rangeConfig.responderAddress               = 0x0002;
    rangeConfig.responseRxAfterTxDelayUus      = 1500;
    rangeConfig.responseTxDelayUus             = 3000;
    rangeConfig.finalTxDelayUus                = 1800;
    rangeConfig.finalRxAfterResponseTxDelayUus = 500;
    rangeConfig.resultRxAfterFinalTxDelayUus   = 500;
    rangeConfig.rxTimeoutUus                   = 3000;
    rangeConfig.hostTimeoutMs                  = 100;
    rangeConfig.resultRepeatCount              = 3;
    rangeConfig.resultRepeatGapMs              = 3;

    if (!uwb.begin(config, phy)) {
        Serial.printf("UWB_BEGIN,result=FAIL,error=%s\n", uwb.lastErrorName());
        Serial.printf("UWB_RAW_ID,dev_id=0x%08lX\n", static_cast<unsigned long>(uwb.readRawDeviceId()));
        return false;
    }

    const uint32_t devId = uwb.deviceId();
    Serial.printf("UWB_ID,dev_id=0x%08lX,chip=%s\n", static_cast<unsigned long>(devId), uwb.chipName());
    if (devId != M5STAMP_UWB_QM33120_DEVICE_ID) {
        Serial.printf("UWB_ID,result=FAIL,expected=0xDECA0314\n");
        return false;
    }

    // deviceId() is the value cached while probing, when the bus still ran at
    // spi_slow_hz. The driver switched to spi_fast_hz inside begin(), so read
    // the register once more: a corrupt readback means this wiring cannot hold
    // the requested rate, and every later transfer would be silently unreliable.
    const uint32_t fastId = uwb.readRawDeviceId();
    if (fastId != M5STAMP_UWB_QM33120_DEVICE_ID) {
        Serial.printf("UWB_SPI,result=FAIL,fast_hz=%lu,dev_id=0x%08lX\n", static_cast<unsigned long>(spiFastHz),
                      static_cast<unsigned long>(fastId));
        return false;
    }
    Serial.printf("UWB_SPI,result=OK,fast_hz=%lu\n", static_cast<unsigned long>(spiFastHz));

    Serial.printf("UWB_CONFIG,result=OK,ch=%u,plen=%u,rate=6M8,tx_power=0x%08lX\n", static_cast<unsigned>(phy.channel),
                  static_cast<unsigned>(phy.preambleLength), static_cast<unsigned long>(phy.txPower));
    return true;
}

// The 128x128 display fits about 10 characters per line at text size 2, so the
// library error names are far too long to print as-is.
static const char* errorShortName(M5Stamp_UWBError error)
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

// Ok: a distance was measured. Waiting: nothing arrived in time, which is the
// normal idle state for an anchor. Fail: the exchange started but broke down.
enum class DisplayState { Init, Ok, Waiting, Fail };

static uint16_t stateColor(DisplayState state)
{
    switch (state) {
        case DisplayState::Ok:      return GREEN;
        case DisplayState::Waiting: return YELLOW;
        case DisplayState::Init:    return uwbReady ? GREEN : RED;
        default:                    return RED;
    }
}

// Only Poll frames that actually reached us count as attempts; a silent tag
// (NO POLL) would otherwise inflate the denominator forever.
static uint32_t attemptCount()
{
    return responseCount + failCount + noFinalCount;
}

uint32_t lastLedColor = UINT32_MAX;

// On the screenless Lite boards the whole status is carried by the single RGB
// LED: RED = the UWB transceiver is unavailable, YELLOW = no tag ranging with
// us yet, GREEN = exchanging distances with the tag.
static void updateLed(DisplayState state)
{
    if (!hasLed) return;

    uint8_t red = 0, green = 0;
    if (!uwbReady) {
        red = 255;                 // RED
    } else if (state == DisplayState::Ok) {
        green = 255;               // GREEN
    } else {
        red = green = 255;         // YELLOW
    }

    // Ok arrives several times a second and every write drives an RMT frame, so
    // only push the LED when the color actually changes.
    const uint32_t color = rgbLed.Color(red, green, 0);
    if (color == lastLedColor) return;
    lastLedColor = color;
    rgbLed.setPixelColor(0, color);
    rgbLed.show();
}

// Same six-line layout as the tag sketch, with the state carried in the color.
static void updateStatus(DisplayState state, float distanceM, uint32_t elapsedMs, uint16_t sequence,
                         const char* errorText)
{
    updateLed(state);
    if (!hasDisplay) return;

    const uint16_t color = stateColor(state);

    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setCursor(0, 0);
    M5.Display.setTextColor(color);
    M5.Display.println("UWB ANCHOR");
    M5.Display.printf("STA:%s\n", uwbReady ? "OK" : "FAIL");

    if (state == DisplayState::Ok) {
        M5.Display.printf("D:%.3fm\n", distanceM);
        M5.Display.setTextColor(WHITE);
        M5.Display.printf("T:%lums\n", static_cast<unsigned long>(elapsedMs));
    } else {
        M5.Display.println("D:----");
        M5.Display.printf("E:%s\n", errorText);
        M5.Display.setTextColor(WHITE);
    }

    M5.Display.printf("SEQ:%u\n", sequence);
    M5.Display.printf("OK:%lu/%lu\n", static_cast<unsigned long>(responseCount),
                      static_cast<unsigned long>(attemptCount()));
}

DisplayState lastDisplayState = DisplayState::Init;
bool lastWaitingWasNoPoll     = false;
uint32_t lastSuccessMs        = 0;

static void runResponder()
{
    // Wait for a Poll/Final exchange and return the DS-TWR distance result.
    const M5Stamp_UWBDSResponderResult result = uwb.respondDSRange(rangeConfig);
    if (!result.success) {
        // Idle receive timeouts are expected and are not counted as failures.
        if (result.error == M5Stamp_UWBError::RxTimeout) {
            // requester stays 0 when no Poll was received at all; a non-zero value
            // means the Poll/Response exchange started but the Final frame was lost.
            const bool noPoll = (result.requester == 0);
            noPoll ? ++noPollCount : ++noFinalCount;

            if (noPoll) {
                // Normal idle gap: stay quiet and keep the last reading visible.
                if ((millis() - lastSuccessMs) < IDLE_GRACE_MS) return;
            } else if (noFinalCount % LOG_INTERVAL == 0) {
                // A lost Final frame is a real anomaly, so keep a sparse trace of it.
                Serial.printf("DS_RESP_NO_FINAL,requester=0x%X,seq=%u,elapsed_ms=%lu,no_final=%lu\n", result.requester,
                              result.sequence, static_cast<unsigned long>(result.elapsedMs),
                              static_cast<unsigned long>(noFinalCount));
            }

            // Redraw only on state change to avoid flicker.
            if (lastDisplayState != DisplayState::Waiting || lastWaitingWasNoPoll != noPoll) {
                updateStatus(DisplayState::Waiting, 0.0f, result.elapsedMs, result.sequence,
                             noPoll ? "NOPOLL" : "NOFIN");
                lastDisplayState     = DisplayState::Waiting;
                lastWaitingWasNoPoll = noPoll;
            }
            return;
        }

        ++failCount;
        updateStatus(DisplayState::Fail, 0.0f, result.elapsedMs, result.sequence, errorShortName(result.error));
        lastDisplayState = DisplayState::Fail;
        if (failCount % LOG_INTERVAL == 0) {
            Serial.printf("DS_RESP_STAT,count=%lu,fail=%lu,last=FAIL,error=%s\n",
                          static_cast<unsigned long>(responseCount), static_cast<unsigned long>(failCount),
                          uwb.lastErrorName());
        }
        return;
    }

    ++responseCount;
    lastSuccessMs = millis();
    updateStatus(DisplayState::Ok, result.distanceM, result.elapsedMs, result.sequence, nullptr);
    lastDisplayState = DisplayState::Ok;

    // Print accumulated statistics every LOG_INTERVAL responses.
    if (responseCount % LOG_INTERVAL == 0) {
        Serial.printf(
            "DS_RESP_STAT,count=%lu,fail=%lu,no_poll=%lu,no_final=%lu,last=OK,seq=%u,requester=0x%X,distance_mm=%ld,distance_m=%.3f,elapsed_ms=%lu\n",
            static_cast<unsigned long>(responseCount), static_cast<unsigned long>(failCount),
            static_cast<unsigned long>(noPollCount), static_cast<unsigned long>(noFinalCount), result.sequence,
            result.requester, static_cast<long>(result.distanceMm), result.distanceM,
            static_cast<unsigned long>(result.elapsedMs));
    }
}

void setup()
{
    auto cfg = M5.config();
    // 内部I2CバスはUWBのRST/IRQと同じピンを使う (AtomS3系はGPIO38/39、
    // Atom LiteはGPIO21/25)。バス上のデバイスを初期化しなければピンは
    // M5Stamp_UWB 側の pinMode で占有できる。
    cfg.internal_imu = false;
    cfg.internal_rtc = false;
    // M5Unified は Atom Lite を AtomU と誤検出することがある。AtomU の内蔵PDM
    // マイクは pin_data_in = GPIO19 (= UWB の MOSI) なので、内蔵オーディオを
    // 無効にしてピンを確実に空けておく。
    cfg.internal_mic = false;
    cfg.internal_spk = false;
    M5.begin(cfg);

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

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
    // The classic ESP32 build logs over UART0, and arduino-esp32 leaves that
    // port without a TX ring buffer: anything longer than the 128-byte hardware
    // FIFO blocks the caller until the FIFO drains, which is about 10ms for the
    // stat line at 115200. The radio only listens inside respondDSRange(), so
    // such a stall is dead air on the air interface - harmless at the current
    // 5Hz, but a fifth of the budget once the exchange rate goes up. A TX ring
    // buffer turns Serial.printf back into a memcpy. The size can only be set
    // while the driver is down, hence end() first (a no-op before the first
    // begin()). The ESP32-S3 build uses USB-Serial/JTAG, which already has a
    // 256-byte ring buffer and never blocks like this.
    Serial.end();
    Serial.setTxBufferSize(512);
#endif
    Serial.begin(115200);
    // USB CDC drops output until the host opens the port; wait briefly for it.
    const uint32_t serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart) < 3000) {
        delay(10);
    }
    Serial.printf("M5Stamp UWB DS-TWR ANCHOR\n");
    Serial.printf("ROLE,mode=ANCHOR\n");
    Serial.printf("TWR_MODE,mode=DS-TWR\n");
    Serial.printf("HOST,board=%d,display=%d,led=%d\n", static_cast<int>(M5.getBoard()), hasDisplay ? 1 : 0,
                  hasLed ? 1 : 0);

    uwbReady = initUwb(UWB_SPI_FAST_HZ);
    if (!uwbReady && (UWB_SPI_FAST_HZ != UWB_SPI_FAST_FALLBACK_HZ)) {
        // The link either never came up or failed the fast-rate readback. Retry
        // once at the library default so a marginal board still ranges.
        uwb.end();
        uwbReady = initUwb(UWB_SPI_FAST_FALLBACK_HZ);
    }
    Serial.printf("TEST_START,result=%s\n", uwbReady ? "OK" : "FAIL");
    updateStatus(DisplayState::Init, 0.0f, 0, 0, uwbReady ? "----" : errorShortName(uwb.lastError()));
}

void loop()
{
    if (uwbReady) {
        runResponder();
    } else {
        delay(1000);
    }
}
