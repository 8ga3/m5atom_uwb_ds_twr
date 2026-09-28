#include "host_init.h"

#include <M5Unified.h>

#include "hw_pins.h"
#include "status.h"
#include "version.h"

// Atom Lite / Atom Matrix の USB シリアル変換 (FTDI を名乗る M5Stack 独自のチップ) は
// 24MHz を 16 で割った 1.5M の整数分の 1 しか正確に出せず、921600 では文字化けした。
// 書き込みにも使う 1500000 なら変換側は誤差なしで、ESP32 側も分数分周で 0.04% に収まる。
static constexpr uint32_t SERIAL_BAUD = 1500000;

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
    // リングバッファが溢れれば結局ブロックするので、ボーレートも 1500000bps に
    // 上げて 1 行あたりの送出時間を 1ms 以下に縮める (SERIAL_BAUD)。
    Serial.end();
    Serial.setTxBufferSize(512);
#else
    // ESP32-S3 ビルドは USB-Serial/JTAG (HWCDC) を使う。既定の TX リングバッファ
    // は 256 バイトしかなく、ホストが引き取りを少し止めただけで満杯になる。満杯の
    // ときの書き込みは tx_timeout_ms (既定 100ms) まで待つので、統計行を出した周期
    // だけ 200ms が 256ms に延びていた (テレメトリの t_tag_ms で確認)。
    //
    // バッファを大きくして満杯になりにくくし、それでも満杯になったら 2ms で諦めて
    // ログを捨てる。測距ループを止めないことをログの完全性より優先する
    // (doc/server-design.md 7.1)。0 にしないのは、HWCDC::write() がこの値を
    // 「進捗なしで待つ回数」にも使っており、0 だと減算で桁あふれして逆に際限なく
    // 待つため。諦めたあとは HWCDC がホストの切断とみなして書き込みを捨て、
    // ホストが再び引き取り始めた時点で送信に戻る。
    //
    // M5.begin() は cfg.serial_baudrate が 0 (既定) なので Serial を開始しない。
    // ここはまだ begin() 前で、ISR もバッファを使っていないため end() は要らない
    // (HWCDC::end() は USB の再列挙を起こすので呼ばない)。
    Serial.setTxBufferSize(4096);
    Serial.setTxTimeoutMs(2);
#endif
    // HWCDC (ESP32-S3) はボーレートを使わないが、UART0 の旧世代 ESP32 では
    // これがそのまま送出速度になる。platformio.ini の monitor_speed と揃える。
    Serial.begin(SERIAL_BAUD);
    // USB CDC はホストがポートを開くまで出力を捨てるので、少し待つ。
    const uint32_t serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart) < 3000) {
        delay(10);
    }
    Serial.printf("M5Stamp UWB DS-TWR %s\n", role);
    Serial.printf("FW_VERSION,version=%s\n", FW_VERSION);
    Serial.printf("ROLE,mode=%s\n", role);
    Serial.printf("TWR_MODE,mode=DS-TWR\n");
    Serial.printf("HOST,board=%d,display=%d,led=%d,display_push=%s\n", static_cast<int>(M5.getBoard()),
                  hasDisplay ? 1 : 0, hasLed ? 1 : 0, displayPushMode());
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
