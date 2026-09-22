#include "host_init.h"

#include <M5Unified.h>

#include "hw_pins.h"
#include "status.h"
#include "version.h"

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
    // 旧世代 ESP32 ビルドは UART0 でログ出力し、arduino-esp32 はこのポートに
    // TX リングバッファを用意しない。128 バイトのハードウェア FIFO を超える
    // 出力は、FIFO が空くまで呼び出し元をブロックする。115200bps では統計行
    // 1 本で約 10ms かかる計算。無線は測距中しか受信しないので、このブロックは
    // 空中線上では単なる無通信区間となり、現状の 5Hz では実害はないが、交信
    // 頻度を上げると予算の 5 分の 1 を占めるようになる。TX リングバッファを
    // 用意すれば Serial.printf は memcpy 相当に戻る。バッファサイズはドライバ
    // 停止中しか設定できないため、先に end() する (初回 begin() 前は no-op)。
    // ESP32-S3 ビルドは USB-Serial/JTAG を使い、既に 256 バイトのリングバッファ
    // を持つのでこのブロックは発生しない。
    Serial.end();
    Serial.setTxBufferSize(512);
#endif
    Serial.begin(115200);
    // USB CDC はホストがポートを開くまで出力を捨てるので、少し待つ。
    const uint32_t serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart) < 3000) {
        delay(10);
    }
    Serial.printf("M5Stamp UWB DS-TWR %s\n", role);
    Serial.printf("FW_VERSION,version=%s\n", FW_VERSION);
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
