// M5Atom S3 / AtomS3 Lite / Atom Lite 用 UWB DS-TWR TAG サンプルコード

#include <M5Unified.h>
#include <M5Stamp_UWB.h>

#include "common/device_id.h"
#include "common/host_init.h"
#include "common/hw_pins.h"
#include "common/server_config.h"
#include "common/status.h"
#include "common/telemetry.h"
#include "common/uwb_link.h"
#include "common/wifi_config.h"

// 測距ループは Arduino の loop() で回す。loopTask は Arduino コアが
// xTaskCreatePinnedToCore で ARDUINO_RUNNING_CORE に固定して作るので、ここが 1 なら
// 測距は Core 1 に固定される。Wi-Fi ドライバと TCP/IP、Wi-Fi の監視タスク
// (wifi_config.h) は Core 0 に置き、測距の待機が押しのけられる経路を断つ
// (doc/server-design.md 7.1)。
static_assert(ARDUINO_RUNNING_CORE == 1, "UWB ranging loop must run on Core 1 (doc/server-design.md 7.1)");

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

// 起動ごとに作る 32bit 乱数。サーバーは (tag_id, boot_id) でセッションを区切り、
// millis() が 0 へ戻ったことを知る (doc/server-design.md 4.2)。
uint32_t bootId = 0;
bool helloSent  = false;
// 周期の通番。起動ごとに 0 から数え、1 周期終えるたびに 1 進める。テレメトリの
// 送信を止めていても進めるので、サーバー側では欠番がそのまま欠測を表す。
uint32_t cycleSeq           = 0;
TelemetryCycle currentCycle = {};

// 統計と最新距離はアンカーごとに持つ。遮蔽で 1 台だけ落ちるのが普通なので、
// 合算値だけでは切り分けができない。
struct AnchorStat {
    uint32_t attempts;
    uint32_t ok;
    float distanceM;
    bool responded;  // 直近の 1 周期で応答したか (表示と LED 用)
    // 前回の DS_RANGE_STAT 以降で、Final の遅延送信を予約した時点で送信時刻まで
    // 残っていた時間の最小値。約 200 us を切ると TX_START_FAILED になる。
    int32_t minMarginUs;
    bool marginSeen;
};
AnchorStat anchorStats[UWB_ANCHOR_MAX] = {};
size_t anchorIndex                     = 0;

// 周期の統計。CYCLE_STAT_INTERVAL_MS ごとに 1 行にまとめてシリアルへ出す。
// テレメトリの t_tag_ms と同じ情報を Wi-Fi なしでも見られるようにするためのもので、
// Wi-Fi の有無や AP の切断で周期が乱れるかをシリアルだけで比べられる
// (doc/server-design.md 7.1)。
static constexpr uint32_t CYCLE_STAT_INTERVAL_MS = 10000;
struct CycleStat {
    uint32_t cycles;         // 間隔を測れた周期の数 (前の周期の先頭がある周期だけ数える)
    uint32_t periodMinMs;    // 周期の先頭スロットどうしの間隔
    uint32_t periodMaxMs;
    uint32_t periodSumMs;
    uint32_t elapsedMaxMs;   // requestDSRange() 1 回にかかった時間の最大値
    uint32_t slotLateCount;  // 予定より遅れて始まったスロットの数
    uint32_t slotLateMaxMs;  // スロットの開始が予定より遅れた時間の最大値
};
CycleStat cycleStat      = {};
uint32_t cycleStartMs    = 0;
bool cycleStartValid     = false;
uint32_t lastCycleStatMs = 0;

static void cycleStatReset()
{
    cycleStat             = CycleStat{};
    cycleStat.periodMinMs = UINT32_MAX;
}

// 周期の先頭スロットを始めるときに呼ぶ。前の周期の先頭からの間隔を記録し、
// 出力の間隔が過ぎていれば統計を出して数え直す。
static void cycleStatOnCycleStart(uint32_t startMs)
{
    if (cycleStartValid) {
        const uint32_t period = startMs - cycleStartMs;
        ++cycleStat.cycles;
        cycleStat.periodSumMs += period;
        if (period < cycleStat.periodMinMs) cycleStat.periodMinMs = period;
        if (period > cycleStat.periodMaxMs) cycleStat.periodMaxMs = period;
    }
    cycleStartMs    = startMs;
    cycleStartValid = true;

    if ((startMs - lastCycleStatMs) < CYCLE_STAT_INTERVAL_MS) return;
    lastCycleStatMs = startMs;
    if (cycleStat.cycles > 0) {
        Serial.printf(
            "CYCLE_STAT,seq=%lu,cycles=%lu,period_min_ms=%lu,period_max_ms=%lu,period_avg_ms=%.1f,elapsed_max_ms=%lu,"
            "slot_late=%lu,slot_late_max_ms=%lu,wifi=%s\n",
            static_cast<unsigned long>(cycleSeq), static_cast<unsigned long>(cycleStat.cycles),
            static_cast<unsigned long>(cycleStat.periodMinMs), static_cast<unsigned long>(cycleStat.periodMaxMs),
            static_cast<double>(cycleStat.periodSumMs) / cycleStat.cycles,
            static_cast<unsigned long>(cycleStat.elapsedMaxMs), static_cast<unsigned long>(cycleStat.slotLateCount),
            static_cast<unsigned long>(cycleStat.slotLateMaxMs), wifiShortState());
    }
    cycleStatReset();
}

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

    // 周期の統計も巡回の状態と一緒に数え直す。構成を待っていた間の空白を
    // 周期の間隔として数えないよう、最初の周期の先頭から測り始める。
    cycleStatReset();
    cycleStartValid = false;
    lastCycleStatMs = millis();

    // PAN ID もサーバーが配る。ANCHOR 側は UWB_DEFAULT_PAN_ID で動くので、
    // サーバー側で変えるときはアンカーのファームも合わせる必要がある。
    rangeConfig.panId            = tagConfig.panId;
    rangeConfig.responderAddress = tagConfig.anchors[0].id;

    telemetryConfigure(tagId, bootId, tagConfig);
}

// 構成を得たあとに 1 回だけセッション開始を通知する。HTTP の応答を待つので、
// 測距を始める前 (起動時と構成の再取得時) にしか呼ばない。
static void sendHelloOnce()
{
    if (helloSent || !configReady) return;
    helloSent = serverHello(tagId, bootId, tagConfig.rev);
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

    // 128x128 をテキストサイズ 2 で使うと 1 画面は 8 行で、見出しと ID と Wi-Fi の
    // 3 行を引くと残りは 5 行。入りきらない台数のときは画面の下端で打ち切るので、
    // 全台分はシリアルログで見る。
    for (size_t i = 0; (i < tagConfig.anchorCount)
                       && ((M5.Display.getCursorY() + M5.Display.fontHeight()) <= M5.Display.height());
         ++i) {
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
    const uint32_t nowMs   = millis();
    const uint32_t sinceMs = nowMs - lastRangeMs;
    if (sinceMs < rangeSlotMs) return;
    // 予定 (前のスロットの開始 + rangeSlotMs) からの遅れ。直前のスロットの処理が
    // 1 スロットより長引くと、その分だけここが 0 より大きくなる。
    const uint32_t lateMs = sinceMs - rangeSlotMs;
    if (cycleStartValid && (lateMs > 0)) {
        ++cycleStat.slotLateCount;
        if (lateMs > cycleStat.slotLateMaxMs) cycleStat.slotLateMaxMs = lateMs;
    }
    lastRangeMs = nowMs;

    if (anchorIndex == 0) {
        cycleStatOnCycleStart(lastRangeMs);
        // テレメトリの送信は巡回の先頭スロットの直前だけで行い、測距と測距の間に
        // 割り込ませない (doc/server-design.md 7.1)。スロットの基準時刻は送信前に
        // 取っているので、送信にかかった時間で周期がずれることはない。
        telemetryService();
        currentCycle     = TelemetryCycle{};
        currentCycle.seq = cycleSeq;
        currentCycle.tMs = lastRangeMs;
    }

    const uint16_t anchorId      = tagConfig.anchors[anchorIndex].id;
    rangeConfig.responderAddress = anchorId;

    const M5Stamp_UWBDSRangeResult result = uwb.requestDSRange(rangeConfig);

    // テレメトリの測距レコード。失敗なのにエラーコードが Ok のままという組み合わせは
    // ライブラリの仕様上ないはずだが、サーバー側で成功と取り違えないよう 0 以外にする。
    TelemetryRange& record = currentCycle.ranges[anchorIndex];
    record.anchorId        = anchorId;
    record.status          = result.success ? 0
                             : (result.error == M5Stamp_UWBError::Ok) ? 0xFF
                                                                       : static_cast<uint8_t>(result.error);
    record.elapsedMs  = (result.elapsedMs > TELEMETRY_ELAPSED_MAX) ? TELEMETRY_ELAPSED_MAX
                                                                   : static_cast<uint8_t>(result.elapsedMs);
    record.distanceMm = result.success ? result.distanceMm : 0;
    if (result.elapsedMs > cycleStat.elapsedMaxMs) cycleStat.elapsedMaxMs = result.elapsedMs;

    AnchorStat& stat = anchorStats[anchorIndex];
    ++stat.attempts;
    stat.ok += result.success;
    stat.responded = result.success;
    if (result.success) stat.distanceM = result.distanceM;

    const int32_t marginUs = result.txMarginUs;
    if ((marginUs != M5STAMP_UWB_TX_MARGIN_UNKNOWN) && (!stat.marginSeen || (marginUs < stat.minMarginUs))) {
        stat.minMarginUs = marginUs;
        stat.marginSeen  = true;
    }

    // 失敗は間引かずに毎回出す。累積統計だけでは失敗が周期的に起きているか
    // (何回目・何 ms 時点で落ちたか) が分からないため。
    if (!result.success) {
        Serial.printf("DS_RANGE_FAIL,anchor=0x%04X,attempt=%lu,start_ms=%lu,error=%s",
                      static_cast<unsigned>(anchorId), static_cast<unsigned long>(stat.attempts),
                      static_cast<unsigned long>(lastRangeMs), uwb.lastErrorName());
        if (marginUs != M5STAMP_UWB_TX_MARGIN_UNKNOWN) Serial.printf(",margin_us=%ld", static_cast<long>(marginUs));
        Serial.print('\n');
    }

    // アンカーごとに LOG_INTERVAL 回試行するたびに累積統計を出力する。
    if ((stat.attempts % LOG_INTERVAL) == 0) {
        const uint32_t failCount = stat.attempts - stat.ok;
        if (result.success) {
            Serial.printf(
                "DS_RANGE_STAT,anchor=0x%04X,count=%lu,ok=%lu,fail=%lu,last=OK,seq=%u,distance_mm=%ld,distance_m=%.3f,elapsed_ms=%lu",
                static_cast<unsigned>(anchorId), static_cast<unsigned long>(stat.attempts),
                static_cast<unsigned long>(stat.ok), static_cast<unsigned long>(failCount), result.sequence,
                static_cast<long>(result.distanceMm), result.distanceM,
                static_cast<unsigned long>(result.elapsedMs));
        } else {
            Serial.printf("DS_RANGE_STAT,anchor=0x%04X,count=%lu,ok=%lu,fail=%lu,last=FAIL,seq=%u,error=%s",
                          static_cast<unsigned>(anchorId), static_cast<unsigned long>(stat.attempts),
                          static_cast<unsigned long>(stat.ok), static_cast<unsigned long>(failCount), result.sequence,
                          uwb.lastErrorName());
        }
        if (stat.marginSeen) Serial.printf(",min_margin_us=%ld", static_cast<long>(stat.minMarginUs));
        Serial.print('\n');
        stat.marginSeen = false;
    }

    anchorIndex = (anchorIndex + 1) % tagConfig.anchorCount;
    if (anchorIndex != 0) return;

    // 1 周期ぶんの記録をテレメトリへ積む。自己位置推定はまだ無いので測位欄は空のまま。
    telemetryPush(currentCycle);
    ++cycleSeq;

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
    sendHelloOnce();

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

    // boot_id は Wi-Fi を起動したあとで作る。esp_random() は RF が動いている間は
    // ハードウェアの真性乱数を返すので、起動をまたいで値が重なることは実質的にない。
    // Wi-Fi が未設定だと RF が動いておらず、値は疑似乱数になりうる。ただし boot_id を
    // 使うのはテレメトリと hello だけで、どちらも Wi-Fi が無ければ送られないので害はない。
    bootId = esp_random();
    Serial.printf("BOOT,boot_id=0x%08lX\n", static_cast<unsigned long>(bootId));

    // 測位サーバーの宛先も同じ入り口で設定する。mDNS は使わず IP を直接持つ。
    serverEndpointSetup(buttonHeld, showServerSetup);

    // アンカーの巡回先は構成から決まる。responderAddress は巡回のたびに
    // runRanging() で差し替えるので、ここでは先頭のアンカーが入る。PAN ID は
    // 構成を取れなかったときのために既定値を入れておき、取れたら applyConfig()
    // が上書きする。initUwb() はどちらの値にも触れない。
    rangeConfig.panId            = UWB_DEFAULT_PAN_ID;
    rangeConfig.initiatorAddress = tagId;
    setupConfig();

    // ここから先は Wi-Fi の監視と再接続を Core 0 のタスクへ任せ、測距ループ側では
    // 接続の状態を読むだけにする。
    wifiStartMonitor();
    Serial.printf("TASK,ranging_core=%d\n", xPortGetCoreID());

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
    // 通常は何もしない。監視タスクを作れなかったときだけ、ここで接続状態を確認する。
    wifiPollFromLoop();

    if (!configReady) {
        // 構成が無い間は測距に入れない。Wi-Fi やサーバーが後から立ち上がる場合に
        // 備えて、一定間隔で取得をやり直す。
        if ((millis() - lastConfigTryMs) >= CONFIG_RETRY_MS) {
            lastConfigTryMs = millis();
            if (configFetch(tagId, tagConfig) == ConfigFetchResult::Updated) {
                applyConfig();
                sendHelloOnce();
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
