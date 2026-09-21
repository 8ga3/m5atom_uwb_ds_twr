#include "host_init.h"

#include <M5Unified.h>

#include "hw_pins.h"
#include "status.h"

void beginHost()
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

    initStatusHardware();
}

void beginSerial(const char* role)
{
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
    // The classic ESP32 build logs over UART0, and arduino-esp32 leaves that
    // port without a TX ring buffer: anything longer than the 128-byte hardware
    // FIFO blocks the caller until the FIFO drains, which is about 10ms for the
    // stat line at 115200. The radio only listens while ranging, so such a
    // stall is dead air on the air interface - harmless at the current 5Hz,
    // but a fifth of the budget once the exchange rate goes up. A TX ring
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
    Serial.printf("M5Stamp UWB DS-TWR %s\n", role);
    Serial.printf("ROLE,mode=%s\n", role);
    Serial.printf("TWR_MODE,mode=DS-TWR\n");
    Serial.printf("HOST,board=%d,display=%d,led=%d\n", static_cast<int>(M5.getBoard()), hasDisplay ? 1 : 0,
                  hasLed ? 1 : 0);
}

bool readBootButtonHeld()
{
    // 電源投入 / リセット直後にボタンが押されていたら、設定済みでも ID 設定へ
    // 入る。現場で ID を振り直す唯一の入口。
    pinMode(BTN_PIN, BTN_MODE);
    delay(1);
    const bool held = (digitalRead(BTN_PIN) == LOW);
    Serial.printf("BUTTON,held=%d\n", held ? 1 : 0);
    return held;
}
