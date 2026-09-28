#include "uwb_link.h"

#include <Arduino.h>

#include "hw_pins.h"

M5Stamp_UWB uwb;
M5Stamp_UWBDSRangeConfig rangeConfig;

bool initUwb(uint32_t spiFastHz)
{
    // Stamp-UWB から QM33120 UWB トランシーバーへの接続設定。
    M5Stamp_UWBConfig config;
    config.pin_gp7    = M5STAMP_UWB_PIN_UNUSED;
    config.pin_wakeup = M5STAMP_UWB_PIN_UNUSED;
    config.pin_irq    = UWB_PIN_IRQ;
    config.pin_rst    = UWB_PIN_RST;
    config.pin_miso   = UWB_PIN_MISO;
    config.pin_mosi   = UWB_PIN_MOSI;
    config.pin_sck    = UWB_PIN_SCK;
    config.pin_cs     = UWB_PIN_CS;
    // spi_slow_hz はライブラリ既定値 (2MHz) のままにする: プローブとリセット
    // シーケンスはチップのクロック PLL が上がる前に走り、そこでは低速 SPI しか
    // 保証されない。ここで上げるのは初期化後のレートだけ。
    config.spi_fast_hz = spiFastHz;

    M5Stamp_UWBPHYConfig phy;
    // チャンネル 9 は日本で許可されている唯一の UWB チャンネル。両側で一致させる。
    phy.channel = M5Stamp_UWBChannel::Channel9;

    // 両機とも同じ PAN と DS-TWR タイミングを使う必要がある。panId /
    // initiatorAddress / responderAddress は呼び出し側 (ANCHOR/TAG それぞれの
    // setup()) が initUwb() を呼ぶ前に設定済みという前提。ここで panId を
    // 書き戻すと、TAG がサーバーから受け取った値を握り潰してしまう。
    rangeConfig.responseRxAfterTxDelayUus      = 1500;
    rangeConfig.responseTxDelayUus             = 3000;
    // Final は Response の受信時刻から finalTxDelayUus 後に遅延送信する。受信完了の
    // 検知から送信予約までのホスト側の処理が間に合わず、予約が送信時刻の約 200 us
    // 前を過ぎると TX_START_FAILED になる。上流のライブラリは受信完了を delay(1) で
    // 待ち、SPI も初期化時の 2MHz のまま 1 バイトずつ転送するので、この処理に 1 ms
    // 近くかかり、しかも待ちの遅れがビルドごとに変わって失敗率が 0〜100% の間で振れた。
    // IRQ で待ち、SPI を 20MHz でまとめて転送するフォーク版 (platformio.ini の
    // lib_deps) では、ESP32-S3 で 400〜450 us に収まる。1000 では余裕の最小が 300 us
    // を切る周期があったので 1200 にする (doc/multi-anchor-positioning-design.md 5.2)。
    // この値を使うのはタグだけで、アンカーの Final の受信窓 (Response 送信の
    // 500〜3500 uus 後) にも収まる。
    rangeConfig.finalTxDelayUus                = 1200;
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

    // deviceId() はプローブ中、バスがまだ spi_slow_hz で動いていたときに
    // キャッシュした値。begin() 内でドライバは spi_fast_hz に切り替わったので、
    // レジスタをもう一度読む: この読み戻しが壊れていれば、この配線は要求レート
    // を維持できず、以降のすべての転送が見えないまま不安定になる。タイミングの
    // 際では失敗がときどきしか出ないので、1 回ではなく続けて何度も読む
    // (1 回あたり 20 us 程度なので起動時間への影響はない)。
    static constexpr int FAST_ID_READS = 64;
    for (int i = 0; i < FAST_ID_READS; ++i) {
        const uint32_t fastId = uwb.readRawDeviceId();
        if (fastId != M5STAMP_UWB_QM33120_DEVICE_ID) {
            Serial.printf("UWB_SPI,result=FAIL,fast_hz=%lu,read=%d,dev_id=0x%08lX\n",
                          static_cast<unsigned long>(spiFastHz), i, static_cast<unsigned long>(fastId));
            return false;
        }
    }
    Serial.printf("UWB_SPI,result=OK,fast_hz=%lu\n", static_cast<unsigned long>(spiFastHz));

    // どの待ち方とタイミングで動いているかをログに残す。wait_mode は
    // M5STAMP_UWB_WAIT_MODE (0 = delay(1)、1 = busy-poll、2 = IRQ)。
    Serial.printf("UWB_TIMING,final_tx_delay_uus=%u,wait_mode=%d\n",
                  static_cast<unsigned>(rangeConfig.finalTxDelayUus), M5STAMP_UWB_WAIT_MODE);

    Serial.printf("UWB_CONFIG,result=OK,ch=%u,plen=%u,rate=6M8,tx_power=0x%08lX\n", static_cast<unsigned>(phy.channel),
                  static_cast<unsigned>(phy.preambleLength), static_cast<unsigned long>(phy.txPower));
    return true;
}

bool initUwbWithFallback()
{
    static constexpr uint32_t rates[] = {UWB_SPI_FAST_HZ, UWB_SPI_FALLBACK_HZ, UWB_SPI_SAFE_HZ};
    uint32_t lastTried = 0;
    for (const uint32_t hz : rates) {
        // -D UWB_SPI_FAST_HZ で下位のレートと同じ値にしたときに同じ試行を繰り返さない。
        if (hz == lastTried) continue;
        if (lastTried != 0) {
            // 前のレートでリンクが上がらなかったか、読み戻しに失敗した。
            uwb.end();
        }
        lastTried = hz;
        if (initUwb(hz)) return true;
    }
    return false;
}
