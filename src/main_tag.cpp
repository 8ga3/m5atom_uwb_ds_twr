// M5Atom S3 / AtomS3 Lite / Atom Lite 用 UWB DS-TWR TAG サンプルコード

#include <M5Unified.h>
#include <M5Stamp_UWB.h>

#include "common/device_id.h"
#include "common/host_init.h"
#include "common/hw_pins.h"
#include "common/server_config.h"
#include "common/status.h"
#include "common/uwb_link.h"
#include "common/wifi_config.h"

// 巡回するアンカーの ID と設置座標は測位サーバーから受け取る
// (doc/server-design.md 5.1)。取得できなければ NVS のキャッシュを使い、それも
// 無ければ巡回先が分からないので測距を始めない。
UwbConfig tagConfig = {};
bool configReady    = false;

// 1 周期で全アンカーを 1 回ずつ測距する。各アンカーから見た Poll の間隔は
// 単一アンカー時と同じ RANGE_CYCLE_MS のままで、順番に 1 台ずつ叩くこと自体が
// TDMA として働くのでアンカー間の衝突が起きない。
static constexpr uint32_t RANGE_CYCLE_MS = 200;
static constexpr uint32_t LOG_INTERVAL   = 10;

// 1 スロットの長さはアンカー台数で決まる。台数はサーバーの構成で変わるので
// 実行時に求める。
uint32_t rangeSlotMs = RANGE_CYCLE_MS;

// 起動時に構成を取りに行く前の Wi-Fi 接続待ち。接続を待つのはここだけで、
// loop() に入ってからは待たない (doc/server-design.md 7.1)。
static constexpr uint32_t CONFIG_WIFI_WAIT_MS = 10000;

// 構成を持たないまま起動したときの再取得間隔。Wi-Fi や サーバーが後から
// 立ち上がる場合に、電源を入れ直さずに走り出せるようにする。
static constexpr uint32_t CONFIG_RETRY_MS = 10000;

// 自機のタグ ID (= initiatorAddress)。NVS から読むか起動時に設定する。
uint16_t tagId           = DEVICE_ID_UNSET;
uint32_t lastRangeMs     = 0;
uint32_t lastConfigTryMs = 0;

// 統計と最新距離はアンカーごとに持つ。遮蔽で 1 台だけ落ちるのが普通なので、
// 合算値だけでは切り分けができない。
struct AnchorStat {
    uint32_t attempts;
    uint32_t ok;
    float distanceM;
    bool responded;  // 直近の 1 周期で応答したか (表示と LED 用)
};
AnchorStat anchorStats[UWB_ANCHOR_MAX] = {};
size_t anchorIndex                     = 0;

// 構成を受け取った直後の反映。巡回の状態と統計を作り直し、1 スロットの長さを
// 台数から求める。走行中に rev が変わることは想定していないが、起動直後に
// キャッシュからサーバーの値へ入れ替わる経路があるのでまとめておく。
static void applyConfig()
{
    configReady = (tagConfig.anchorCount > 0);
    if (!configReady) {
        rangeSlotMs = RANGE_CYCLE_MS;
        return;
    }

    rangeSlotMs = RANGE_CYCLE_MS / tagConfig.anchorCount;
    if (rangeSlotMs == 0) rangeSlotMs = 1;

    for (AnchorStat& stat : anchorStats) stat = AnchorStat{};
    anchorIndex = 0;

    // PAN ID もサーバーが配る。ANCHOR 側は uwb_link.cpp の既定値 (0xDECA) で
    // 動くので、サーバー側で変えるときはアンカーのファームも合わせる必要がある。
    rangeConfig.panId            = tagConfig.panId;
    rangeConfig.responderAddress = tagConfig.anchors[0].id;
}

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

    // Wi-Fi の状態。OFF = 未設定、-- = 設定済みだが接続できていない、OK = 接続中。
    M5.Display.setTextColor(wifiIsConnected() ? GREEN : YELLOW);
    M5.Display.printf("W:%s\n", wifiShortState());

    if (!configReady) {
        // 構成が無い状態は測距そのものが始められないので、距離の代わりに
        // その旨だけを出す。
        M5.Display.setTextColor(RED);
        M5.Display.println("CFG:NONE");
        return;
    }

    // 128x128 をテキストサイズ 2 で使うと Wi-Fi 行のぶんを引いて残りは 3 行。
    // 4 台以上に増やしたときは画面には最初の 3 台しか出ないので、全台分は
    // シリアルログで見る。
    for (size_t i = 0; (i < tagConfig.anchorCount) && (i < 3); ++i) {
        const AnchorStat& stat = anchorStats[i];
        M5.Display.setTextColor(stat.responded ? GREEN : YELLOW);
        if (stat.responded) {
            M5.Display.printf("%u:%.2f\n", static_cast<unsigned>(tagConfig.anchors[i].id), stat.distanceM);
        } else {
            M5.Display.printf("%u:----\n", static_cast<unsigned>(tagConfig.anchors[i].id));
        }
    }
}

// Poll の宛先を 1 台ずつ切り替えながら全アンカーを巡回する。1 スロットにつき
// 1 交信で、周期の先頭に戻ったところで表示を更新する。
static void runRanging()
{
    if (millis() - lastRangeMs < rangeSlotMs) return;
    lastRangeMs = millis();

    const uint16_t anchorId      = tagConfig.anchors[anchorIndex].id;
    rangeConfig.responderAddress = anchorId;

    const M5Stamp_UWBDSRangeResult result = uwb.requestDSRange(rangeConfig);

    AnchorStat& stat = anchorStats[anchorIndex];
    ++stat.attempts;
    stat.ok += result.success;
    stat.responded = result.success;
    if (result.success) stat.distanceM = result.distanceM;

    // アンカーごとに LOG_INTERVAL 回試行するたびに累積統計を出力する。
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

    anchorIndex = (anchorIndex + 1) % tagConfig.anchorCount;
    if (anchorIndex != 0) return;

    // 1 周終わった。1 台でも応答していれば測距は生きている。
    bool anyResponse = false;
    for (size_t i = 0; i < tagConfig.anchorCount; ++i) anyResponse |= anchorStats[i].responded;
    updateStatus(anyResponse ? DisplayState::Ok : DisplayState::Waiting);
}

// 起動時の構成取得。キャッシュを先に読んでから、Wi-Fi が上がっていればサーバーへ
// 問い合わせる。キャッシュの rev を持って行くので、変更が無ければ 304 で済む。
static void setupConfig()
{
    if (configCacheLoad(tagConfig)) {
        configLogSummary(tagConfig, "cache");
    } else {
        Serial.println("CONFIG,source=none");
        tagConfig = UwbConfig{};
    }

    if (serverConfigured && wifiConfigured) {
        // 構成取得のためにここでだけ接続を待つ。走行中は待たない。
        const uint32_t waitStart = millis();
        while (!wifiMaintain() && ((millis() - waitStart) < CONFIG_WIFI_WAIT_MS)) delay(100);
        if (!wifiIsConnected()) Serial.println("CONFIG,warn=wifi_not_connected");
    }

    configFetch(tagId, tagConfig);
    lastConfigTryMs = millis();
    applyConfig();

    if (!configReady) {
        // アンカーが 1 台も分からない。測距は始められないので、原因が構成に
        // あることをログへ残して loop() の再取得へ任せる。
        Serial.println("CONFIG,result=UNAVAILABLE");
    }
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

    // Wi-Fi は ID と同じ起動ボタン押下で設定に入る。SSID とパスフレーズはソース
    // にもリポジトリにも置かず、ここでシリアルから入れた値を NVS に保存する。
    // 接続の完了は待たない (setupConfig() が構成取得の直前だけ待つ)。
    wifiSetup(buttonHeld, showWifiSetup);

    // 測位サーバーの宛先も同じ入り口で設定する。mDNS は使わず IP を直接持つ。
    serverEndpointSetup(buttonHeld, showServerSetup);

    // アンカーの巡回先は構成から決まる。responderAddress は巡回のたびに
    // runRanging() で差し替えるので、ここでは先頭のアンカーが入る。
    rangeConfig.initiatorAddress = tagId;
    setupConfig();

    uwbReady = initUwb(UWB_SPI_FAST_HZ);
    if (!uwbReady && (UWB_SPI_FAST_HZ != UWB_SPI_FAST_FALLBACK_HZ)) {
        // リンクが上がらなかったか、高速レートでの読み戻しに失敗した。ライブラリ
        // 既定のクロックで 1 回だけ再試行し、際 (きわ) の基板でも測距できるようにする。
        uwb.end();
        uwbReady = initUwb(UWB_SPI_FAST_FALLBACK_HZ);
    }
    Serial.printf("TEST_START,result=%s\n", uwbReady ? "OK" : "FAIL");
    Serial.printf("ANCHORS,count=%u\n", static_cast<unsigned>(tagConfig.anchorCount));
    updateStatus(configReady ? DisplayState::Init : DisplayState::Fail);
}

void loop()
{
    // 接続状態の確認だけを行う。再接続は WiFi ライブラリ側のタスクが受け持つので、
    // ここで待たされることはない。
    wifiMaintain();

    if (!configReady) {
        // 構成が無い間は測距に入れない。Wi-Fi やサーバーが後から立ち上がる場合に
        // 備えて、一定間隔で取得をやり直す。
        if ((millis() - lastConfigTryMs) >= CONFIG_RETRY_MS) {
            lastConfigTryMs = millis();
            if (configFetch(tagId, tagConfig) == ConfigFetchResult::Updated) {
                applyConfig();
                Serial.printf("ANCHORS,count=%u\n", static_cast<unsigned>(tagConfig.anchorCount));
            }
            // 画面の書き換えは取得を試みたときだけにする。100ms ごとに全画面を
            // 塗り直すとちらつくうえ、状態は何も変わっていない。
            if (!configReady) updateStatus(DisplayState::Fail);
        }
        delay(100);
        return;
    }

    if (uwbReady) {
        runRanging();
    } else {
        delay(1000);
    }
}
