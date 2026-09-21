// M5Atom S3 / AtomS3 Lite / Atom Lite UWB DS-TWR TAG example code

#include <M5Unified.h>
#include <M5Stamp_UWB.h>

#include "common/host_init.h"
#include "common/hw_pins.h"
#include "common/status.h"
#include "common/uwb_link.h"
#include "device_id.h"

// 巡回するアンカーの ID 一覧。仮実装としてソースに直書きする。将来はサーバー
// から ID と設置座標をまとめて取得する (doc/multi-anchor-positioning-design.md)。
static constexpr uint16_t ANCHOR_IDS[] = {0x0100};
static constexpr size_t ANCHOR_COUNT   = sizeof(ANCHOR_IDS) / sizeof(ANCHOR_IDS[0]);

// 1 周期で全アンカーを 1 回ずつ測距する。各アンカーから見た Poll の間隔は
// 単一アンカー時と同じ RANGE_CYCLE_MS のままで、順番に 1 台ずつ叩くこと自体が
// TDMA として働くのでアンカー間の衝突が起きない。
static constexpr uint32_t RANGE_CYCLE_MS = 200;
static constexpr uint32_t RANGE_SLOT_MS  = RANGE_CYCLE_MS / ANCHOR_COUNT;
static constexpr uint32_t LOG_INTERVAL   = 10;

// 自機のタグ ID (= initiatorAddress)。NVS から読むか起動時に設定する。
uint16_t tagId       = DEVICE_ID_UNSET;
uint32_t lastRangeMs = 0;

// 統計と最新距離はアンカーごとに持つ。遮蔽で 1 台だけ落ちるのが普通なので、
// 合算値だけでは切り分けができない。
struct AnchorStat {
    uint32_t attempts;
    uint32_t ok;
    float distanceM;
    bool responded;  // 直近の 1 周期で応答したか (表示と LED 用)
};
AnchorStat anchorStats[ANCHOR_COUNT] = {};
size_t anchorIndex                   = 0;

// 1 周期分の結果をまとめて表示する。アンカーごとに 1 行使うので、交信中の
// 1 台だけを大きく出す単一アンカー時のレイアウトは捨てた。
static void updateStatus(DisplayState state)
{
    updateLed(state);
    if (!hasDisplay) return;

    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setCursor(0, 0);
    M5.Display.setTextColor(stateColor(state));
    M5.Display.println("UWB TAG");

    if (!uwbReady) {
        M5.Display.println("STA:FAIL");
        M5.Display.printf("E:%s\n", errorShortName(uwb.lastError()));
        return;
    }

    M5.Display.setTextColor(WHITE);
    M5.Display.printf("ID:%u\n", static_cast<unsigned>(tagId));

    // 128x128 をテキストサイズ 2 で使うと残りは 4 行。5 台以上に増やしたときは
    // 画面には最初の 4 台しか出ないので、全台分はシリアルログで見る。
    for (size_t i = 0; (i < ANCHOR_COUNT) && (i < 4); ++i) {
        const AnchorStat& stat = anchorStats[i];
        M5.Display.setTextColor(stat.responded ? GREEN : YELLOW);
        if (stat.responded) {
            M5.Display.printf("%u:%.2f\n", static_cast<unsigned>(ANCHOR_IDS[i]), stat.distanceM);
        } else {
            M5.Display.printf("%u:----\n", static_cast<unsigned>(ANCHOR_IDS[i]));
        }
    }
}

// Poll の宛先を 1 台ずつ切り替えながら全アンカーを巡回する。1 スロットにつき
// 1 交信で、周期の先頭に戻ったところで表示を更新する。
static void runRanging()
{
    if (millis() - lastRangeMs < RANGE_SLOT_MS) return;
    lastRangeMs = millis();

    const uint16_t anchorId      = ANCHOR_IDS[anchorIndex];
    rangeConfig.responderAddress = anchorId;

    const M5Stamp_UWBDSRangeResult result = uwb.requestDSRange(rangeConfig);

    AnchorStat& stat = anchorStats[anchorIndex];
    ++stat.attempts;
    stat.ok += result.success;
    stat.responded = result.success;
    if (result.success) stat.distanceM = result.distanceM;

    // Print accumulated statistics every LOG_INTERVAL attempts per anchor.
    if ((stat.attempts % LOG_INTERVAL) == 0) {
        const uint32_t failCount = stat.attempts - stat.ok;
        if (result.success) {
            Serial.printf(
                "DS_RANGE_STAT,anchor=0x%04X,count=%lu,ok=%lu,fail=%lu,last=OK,seq=%u,distance_mm=%ld,distance_m=%.3f,elapsed_ms=%lu\n",
                static_cast<unsigned>(anchorId), static_cast<unsigned long>(stat.attempts),
                static_cast<unsigned long>(stat.ok), static_cast<unsigned long>(failCount), result.sequence,
                static_cast<long>(result.distanceMm), result.distanceM,
                static_cast<unsigned long>(result.elapsedMs));
        } else {
            Serial.printf("DS_RANGE_STAT,anchor=0x%04X,count=%lu,ok=%lu,fail=%lu,last=FAIL,seq=%u,error=%s\n",
                          static_cast<unsigned>(anchorId), static_cast<unsigned long>(stat.attempts),
                          static_cast<unsigned long>(stat.ok), static_cast<unsigned long>(failCount), result.sequence,
                          uwb.lastErrorName());
        }
    }

    anchorIndex = (anchorIndex + 1) % ANCHOR_COUNT;
    if (anchorIndex != 0) return;

    // 1 周終わった。1 台でも応答していれば測距は生きている。
    bool anyResponse = false;
    for (const AnchorStat& s : anchorStats) anyResponse |= s.responded;
    updateStatus(anyResponse ? DisplayState::Ok : DisplayState::Waiting);
}

void setup()
{
    beginHost();
    beginSerial("TAG");
    const bool buttonHeld = readBootButtonHeld();

    // ID が決まるまで UWB は初期化しない。initiatorAddress は下でこの値から
    // 設定する。
    idRangeMin = TAG_ID_MIN;
    idRangeMax = TAG_ID_MAX;
    tagId      = deviceIdSetup("TAG", TAG_ID_MIN, TAG_ID_MAX, buttonHeld, showIdSetup);

    // responderAddress は巡回のたびに runRanging() で対象アンカーの ID に
    // 差し替える。ここでは最初のアンカーを入れておく。
    rangeConfig.initiatorAddress = tagId;
    rangeConfig.responderAddress = ANCHOR_IDS[0];

    uwbReady = initUwb(UWB_SPI_FAST_HZ);
    if (!uwbReady && (UWB_SPI_FAST_HZ != UWB_SPI_FAST_FALLBACK_HZ)) {
        // The link either never came up or failed the fast-rate readback. Retry
        // once at the library default so a marginal board still ranges.
        uwb.end();
        uwbReady = initUwb(UWB_SPI_FAST_FALLBACK_HZ);
    }
    Serial.printf("TEST_START,result=%s\n", uwbReady ? "OK" : "FAIL");
    Serial.printf("ANCHORS,count=%u\n", static_cast<unsigned>(ANCHOR_COUNT));
    updateStatus(DisplayState::Init);
}

void loop()
{
    if (uwbReady) {
        runRanging();
    } else {
        delay(1000);
    }
}
