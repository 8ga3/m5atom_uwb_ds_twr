// M5Atom S3 / AtomS3R / AtomS3 Lite / Atom Lite / Atom Matrix 用 UWB DS-TWR TAG サンプルコード

#include <M5Unified.h>
#include <M5Stamp_UWB.h>

#include "common/device_id.h"
#include "common/host_init.h"
#include "common/hw_pins.h"
#include "common/server_config.h"
#include "common/status.h"
#include "common/telemetry.h"
#include "common/trilateration.h"
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
// 周期はビルドフラグ -D UWB_RANGE_CYCLE_MS=<ms> で変えられる。レートを詰めるときの
// 検討は doc/multi-anchor-positioning-design.md の 5 章を参照。
#ifndef UWB_RANGE_CYCLE_MS
#define UWB_RANGE_CYCLE_MS 100
#endif
// uint32_t へ変換する前のマクロの値で検査する。変換後だと -1 などの負の値が大きな
// 正の数になって検査を通ってしまう。
static_assert((UWB_RANGE_CYCLE_MS) > 0, "UWB_RANGE_CYCLE_MS must be positive");
static constexpr uint32_t RANGE_CYCLE_MS = UWB_RANGE_CYCLE_MS;

// DS_RANGE_STAT はアンカーごとにおよそこの間隔で 1 行出す。試行回数で間引くと、
// 周期を詰めたぶんだけ行数が増えてシリアルの送信が測距ループを止めうるので、
// 周期から回数を逆算して時間あたりの行数を一定にする。
static constexpr uint32_t STAT_LOG_PERIOD_MS = 2000;
static constexpr uint32_t LOG_INTERVAL =
    (STAT_LOG_PERIOD_MS / RANGE_CYCLE_MS) > 0 ? (STAT_LOG_PERIOD_MS / RANGE_CYCLE_MS) : 1;

// 画面の書き換えの最短間隔。128x128 の全画面を送るのに数 ms かかり、その間は
// 次のスロットが始められない。周期を詰めても書き換えはこの間隔までに抑える。
static constexpr uint32_t DISPLAY_INTERVAL_MS = 200;
uint32_t lastDisplayMs                        = 0;

// 画面の下端に測位結果を出す欄の高さ。テキストサイズ 2 (16 px) で 3 行。
static constexpr int32_t POSITION_AREA_HEIGHT = 48;

// requestDSRange() の各受信待ちの上限。実際の受信待ちはトランシーバーの
// rxTimeoutUus で数 ms のうちに終わるので、これはその通知が来なかったときの保険。
// 既定の 100 ms のままだと、その 1 回で周期がまるごと潰れる
// (doc/multi-anchor-positioning-design.md 5.2)。
static constexpr uint32_t TAG_HOST_TIMEOUT_MS = 10;

// タグ (ローバーに載せた Stamp-UWB のアンテナ) の床からの高さ。2D 測位では未知数を
// x と y の 2 つにし、高さはこの値に固定して解く (doc/multi-anchor-positioning-design.md 3.5)。
// アンカーは高い位置に付けるので、この値が実際とずれると測距値の水平成分の見積もりが
// ずれ、位置が外側または内側へ偏る。載せる機体ごとに実測してビルドフラグ
// -D UWB_TAG_Z_MM=<mm> で与える。アンカーの z と同じ座標系 (サーバーの座標表) で測る。
#ifndef UWB_TAG_Z_MM
#define UWB_TAG_Z_MM 0
#endif
// テレメトリの z_mm は int16 なので、その範囲に収まることを確かめる (doc/server-design.md 6.2)。
static_assert(((UWB_TAG_Z_MM) >= INT16_MIN) && ((UWB_TAG_Z_MM) <= INT16_MAX), "UWB_TAG_Z_MM must fit in int16");
static constexpr int32_t TAG_Z_MM = UWB_TAG_Z_MM;
static_assert(TRILAT_ANCHOR_MAX >= UWB_ANCHOR_MAX, "trilateration must accept every anchor in the config");

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

// 直近の周期の測位結果。画面の表示と POS 行に使う。
TrilatResult lastFix = {};
bool lastFixValid    = false;  // 起動してから 1 度でも測位を試みたか

// 測位の統計。POS 行を出すたびに数え直す。
struct PositionStat {
    uint32_t fixes;         // 解が出た周期の数
    uint32_t notConverged;  // そのうち反復の上限までに収束しなかった数
    uint32_t tooFew;        // 解に渡せる測距が 3 本に満たなかった周期の数
    uint32_t badGeometry;   // 配置が悪く解けなかった周期の数
    uint32_t nonFinite;     // 計算が NaN / 無限大になった周期の数
    uint32_t rejected;      // 成功扱いでも値が有り得ないので解から外した測距の数
    uint32_t solveMaxUs;    // trilaterate2d() 1 回にかかった時間の最大値
};
PositionStat positionStat = {};
uint32_t lastPosLogMs     = 0;

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

// メートルの float をテレメトリの int32 ミリメートルへ直す。発散した値もそのまま
// 送る (doc/server-design.md 6.2) が、int32 に収まらない値 (±2147 km 超) は変換が
// 未定義になるので端に寄せる。端の値で送ることは同じ節に書いてある。
static int32_t positionMetersToMm(float meters)
{
    const float mm = meters * 1000.0f;
    // float で表せる 2^31 未満の最大値。これ以上は int32 に収まらない。
    if (mm >= 2147483520.0f) return INT32_MAX;
    if (mm <= -2147483648.0f) return INT32_MIN;
    return static_cast<int32_t>(lroundf(mm));
}

// 解に使う測距値の下限。近距離では雑音で 0 を少し下回ることがあるので、その分は
// 残す。ライブラリが成功として返した測距に、まれに -14 m や -36 m のような値が
// 混ざる (0x0100、30Hz、1 分に 1〜2 回)。そのまま解に入れると初期値が遠くへ飛び、
// 配置の判定で周期ごと落ちるので、この値を下回る測距は外して残りで解く。
// テレメトリには生の値をそのまま送る。
static constexpr int32_t POSITION_RANGE_MIN_MM = -500;

// POS_REJECT と POS_FAIL の行は、種類ごとにこの間隔より短くは出さない。どちらも
// 普段は滅多に起きないが、アンカーの置き方が悪いままだと毎周期起きうる。30Hz で
// 毎周期シリアルへ書くと送信バッファが詰まり、測距ループが止まる。出さなかった分も
// POS 行の rejected / geometry / non_finite に件数として残る。
static constexpr uint32_t POS_DETAIL_LOG_INTERVAL_MS = 1000;
uint32_t lastPosRejectLogMs = 0;
uint32_t lastPosFailLogMs   = 0;
bool posRejectLogged        = false;
bool posFailLogged          = false;

// 詳細行を今出してよいか。出してよければ時刻を記録して true を返す。
static bool posDetailLogDue(uint32_t& lastMs, bool& logged)
{
    const uint32_t nowMs = millis();
    if (logged && ((nowMs - lastMs) < POS_DETAIL_LOG_INTERVAL_MS)) return false;
    lastMs = nowMs;
    logged = true;
    return true;
}

// 1 周期ぶんの測距から自己位置を求め、テレメトリの測位欄を埋める。巡回の最後の
// スロットを終えた直後、テレメトリへ積む前に呼ぶ。
static void solvePosition(TelemetryCycle& cycle)
{
    // 応答したアンカーだけを使う。ranges[] の添字は tagConfig.anchors[] と同じ並び。
    // 測距値には構成の bias_mm を足す (doc/server-design.md 4.1)。
    TrilatInput inputs[UWB_ANCHOR_MAX];
    size_t count = 0;
    for (size_t i = 0; i < tagConfig.anchorCount; ++i) {
        const TelemetryRange& range = cycle.ranges[i];
        if (range.status != 0) continue;
        if (range.distanceMm < POSITION_RANGE_MIN_MM) {
            // 起きたときにどの測距だったかを残す。間隔は POS_DETAIL_LOG_INTERVAL_MS で抑える。
            ++positionStat.rejected;
            if (posDetailLogDue(lastPosRejectLogMs, posRejectLogged)) {
                Serial.printf("POS_REJECT,seq=%lu,anchor=0x%04X,distance_mm=%ld\n",
                              static_cast<unsigned long>(cycle.seq), static_cast<unsigned>(range.anchorId),
                              static_cast<long>(range.distanceMm));
            }
            continue;
        }
        const AnchorConfig& anchor = tagConfig.anchors[i];
        inputs[count].x            = static_cast<float>(anchor.xMm) / 1000.0f;
        inputs[count].y            = static_cast<float>(anchor.yMm) / 1000.0f;
        inputs[count].z            = static_cast<float>(anchor.zMm) / 1000.0f;
        // int32 のまま足すと、bias_mm の値によっては符号付き整数のオーバーフローになる。
        // 構成の bias_mm は範囲を絞らずに受け付けているので、float にしてから足す。
        inputs[count].range =
            (static_cast<float>(range.distanceMm) + static_cast<float>(tagConfig.biasMm)) / 1000.0f;
        ++count;
    }

    const uint32_t startUs = micros();
    const TrilatResult fix = trilaterate2d(inputs, count, static_cast<float>(TAG_Z_MM) / 1000.0f);
    const uint32_t solveUs = micros() - startUs;
    if (solveUs > positionStat.solveMaxUs) positionStat.solveMaxUs = solveUs;

    lastFix      = fix;
    lastFixValid = true;

    // 解けなかった周期は fix_flags を 0 にし、座標欄は 0 のまま送る。サーバーは
    // この周期の座標を NULL で保存する。used_count には解に渡した測距の数を入れ、
    // 何台足りなかったかを追えるようにする。
    cycle.usedCount = fix.used;

    // 測距が揃っているのに解けなかった周期は、その場で測距値を残す。POS 行は間引いて
    // いるので、あとから原因の測距を追えない。間隔は POS_DETAIL_LOG_INTERVAL_MS で抑える。
    // 測距の不足 (TooFewRanges) は遮蔽で続けて起きうるので件数だけを数える。
    if (((fix.status == TrilatStatus::BadGeometry) || (fix.status == TrilatStatus::NonFinite))
        && posDetailLogDue(lastPosFailLogMs, posFailLogged)) {
        Serial.printf("POS_FAIL,seq=%lu,status=%s,used=%u,iter=%u", static_cast<unsigned long>(cycle.seq),
                      trilatStatusName(fix.status), static_cast<unsigned>(fix.used),
                      static_cast<unsigned>(fix.iterations));
        for (size_t i = 0; i < tagConfig.anchorCount; ++i) {
            const TelemetryRange& range = cycle.ranges[i];
            Serial.printf(",0x%04X=%ld", static_cast<unsigned>(range.anchorId),
                          (range.status == 0) ? static_cast<long>(range.distanceMm) : -1L);
        }
        Serial.print('\n');
    }

    switch (fix.status) {
        case TrilatStatus::Ok:
            break;
        case TrilatStatus::TooFewRanges:
            ++positionStat.tooFew;
            return;
        case TrilatStatus::BadGeometry:
            ++positionStat.badGeometry;
            return;
        case TrilatStatus::NonFinite:
            ++positionStat.nonFinite;
            return;
    }

    ++positionStat.fixes;
    if (!fix.converged) ++positionStat.notConverged;

    const float residualMm = fix.residualRms * 1000.0f;
    cycle.fixFlags         = TELEMETRY_FIX_OK;
    cycle.xMm              = positionMetersToMm(fix.x);
    cycle.yMm              = positionMetersToMm(fix.y);
    // 2D 測位では高さは解かないので、固定したタグの高さをそのまま入れる。
    cycle.zMm        = static_cast<int16_t>(TAG_Z_MM);
    cycle.residualMm = (residualMm >= 65535.0f) ? 65535 : static_cast<uint16_t>(lroundf(residualMm));
}

// 測位の結果と統計を STAT_LOG_PERIOD_MS ごとに 1 行出す。周期ごとに出すと 30Hz では
// 行数が多すぎ、シリアルの送信が測距ループを止めうる (DS_RANGE_STAT と同じ理由)。
static void logPosition(const TelemetryCycle& cycle)
{
    const uint32_t nowMs = millis();
    if ((nowMs - lastPosLogMs) < STAT_LOG_PERIOD_MS) return;
    lastPosLogMs = nowMs;

    Serial.printf("POS,seq=%lu,status=%s,used=%u", static_cast<unsigned long>(cycle.seq),
                  trilatStatusName(lastFix.status), static_cast<unsigned>(lastFix.used));
    if (lastFix.status == TrilatStatus::Ok) {
        Serial.printf(",x_mm=%ld,y_mm=%ld,z_mm=%ld,resid_mm=%u,iter=%u,converged=%d", static_cast<long>(cycle.xMm),
                      static_cast<long>(cycle.yMm), static_cast<long>(cycle.zMm),
                      static_cast<unsigned>(cycle.residualMm), static_cast<unsigned>(lastFix.iterations),
                      lastFix.converged ? 1 : 0);
    }
    Serial.printf(
        ",fixes=%lu,not_converged=%lu,too_few=%lu,geometry=%lu,non_finite=%lu,rejected=%lu,solve_max_us=%lu\n",
        static_cast<unsigned long>(positionStat.fixes), static_cast<unsigned long>(positionStat.notConverged),
        static_cast<unsigned long>(positionStat.tooFew), static_cast<unsigned long>(positionStat.badGeometry),
        static_cast<unsigned long>(positionStat.nonFinite), static_cast<unsigned long>(positionStat.rejected),
        static_cast<unsigned long>(positionStat.solveMaxUs));
    positionStat = PositionStat{};
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

    // 測位の結果も前の構成のアンカーで出したものなので捨てる。
    lastFix      = TrilatResult{};
    lastFixValid = false;
    positionStat = PositionStat{};
    lastPosLogMs = millis();

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

// Wi-Fi の状態を、1 行目 (高さ 16 px) の右端に扇形のアイコンで出す。緑 = 接続中、
// 黄 = 設定済みだが接続できていない、灰色に赤の斜線 = 未設定。文字で出すと 1 行を
// まるごと使うので、見出しの行の空きに収める。幅は 20 px で、見出しは "TAG 255" でも
// 84 px なので重ならない。
static void drawWifiIcon(lgfx::LovyanGFX& gfx, int32_t top)
{
    const uint16_t color = !wifiConfigured ? DARKGREY : (wifiIsConnected() ? GREEN : YELLOW);
    // 扇の要を下端に置き、上向き (225〜315 度。0 度が右で時計回り) に 3 本の弧を描く。
    const int32_t cx = gfx.width() - 11;
    const int32_t cy = top + 14;
    gfx.fillCircle(cx, cy, 1, color);
    gfx.fillArc(cx, cy, 4, 5, 225, 315, color);
    gfx.fillArc(cx, cy, 8, 9, 225, 315, color);
    gfx.fillArc(cx, cy, 12, 13, 225, 315, color);
    if (!wifiConfigured) gfx.drawLine(cx - 9, top + 1, cx + 9, cy, RED);
}

// 1 周期分の結果をまとめて表示する。アンカーごとに 1 行使うので、交信中の
// 1 台だけを大きく出す単一アンカー時のレイアウトは捨てた。
static void updateStatus(DisplayState state)
{
    updateLed(state);
    if (!hasDisplay) return;

    lgfx::LovyanGFX& gfx = beginDisplayFrame();
    // 見出しと自機の ID を 1 行にまとめ、右端に Wi-Fi のアイコンを置く。見出しの色で
    // 状態を表す。
    gfx.setTextColor(stateColor(state));
    drawWifiIcon(gfx, gfx.getCursorY());
    gfx.printf("TAG %u\n", static_cast<unsigned>(tagId));

    if (!uwbReady) {
        gfx.println("STA:FAIL");
        gfx.printf("E:%s\n", errorShortName(uwbInitError()));
        endDisplayFrame();
        return;
    }

    if (!configReady) {
        // 構成が無い状態は測距そのものが始められないので、距離の代わりに
        // その旨だけを出す。
        gfx.setTextColor(RED);
        gfx.println("CFG:NONE");
        endDisplayFrame();
        return;
    }

    // 128x128 をテキストサイズ 2 で使うと 1 画面は 8 行で、見出しの 1 行を引くと残りは
    // 7 行。そのうち最後の 3 行は測位結果に使うので、アンカーは 4 台まで出せる。入りきらない台数のときは測位の欄の手前で打ち切るので、全台分は
    // シリアルログで見る。
    const int32_t anchorAreaBottom = gfx.height() - POSITION_AREA_HEIGHT;
    for (size_t i = 0; (i < tagConfig.anchorCount)
                       && ((gfx.getCursorY() + gfx.fontHeight()) <= anchorAreaBottom);
         ++i) {
        const AnchorStat& stat = anchorStats[i];
        gfx.setTextColor(stat.responded ? GREEN : YELLOW);
        if (stat.responded) {
            gfx.printf("%u:%.2f\n", static_cast<unsigned>(tagConfig.anchors[i].id), stat.distanceM);
        } else {
            gfx.printf("%u:----\n", static_cast<unsigned>(tagConfig.anchors[i].id));
        }
    }

    // 測位結果。座標 (メートル) を x, y, z の 1 行ずつに出す。z は解かずに固定値
    // (UWB_TAG_Z_MM) を使っているので、白で出して解いた値と区別する。使ったアンカーの
    // 数と残差はシリアルの POS 行で見る。解けなかった周期は、x と y の 2 行の代わりに
    // その旨と理由を出す。z は既知の値なので、解けたかどうかによらず 3 行目に出す。
    gfx.setCursor(0, anchorAreaBottom);
    if (!lastFixValid) {
        gfx.setTextColor(YELLOW);
        gfx.println("X:----");
        gfx.println("Y:----");
    } else if (lastFix.status == TrilatStatus::Ok) {
        gfx.setTextColor(lastFix.converged ? GREEN : YELLOW);
        gfx.printf("X:%.2f\n", lastFix.x);
        gfx.printf("Y:%.2f\n", lastFix.y);
    } else {
        gfx.setTextColor(YELLOW);
        gfx.println("POS:NG");
        gfx.println(trilatStatusName(lastFix.status));
    }
    gfx.setTextColor(WHITE);
    gfx.printf("Z:%.2f\n", static_cast<float>(TAG_Z_MM) / 1000.0f);
    endDisplayFrame();
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

    // LED の書き換えは交信を終えたこの位置で行い、スロットの開始を遅らせない。
    serviceLed();

    anchorIndex = (anchorIndex + 1) % tagConfig.anchorCount;
    if (anchorIndex != 0) return;

    // 1 周期ぶんの測距がそろったので自己位置を求め、測位欄を埋めてからテレメトリへ積む。
    solvePosition(currentCycle);
    logPosition(currentCycle);
    telemetryPush(currentCycle);
    ++cycleSeq;

    // 1 周終わった。1 台でも応答していれば測距は生きている。
    bool anyResponse = false;
    for (size_t i = 0; i < tagConfig.anchorCount; ++i) anyResponse |= anchorStats[i].responded;
    const uint32_t doneMs = millis();
    if ((doneMs - lastDisplayMs) < DISPLAY_INTERVAL_MS) return;
    lastDisplayMs = doneMs;
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

    // 失敗したら SPI の速度を下げて再試行することはせず、エラー表示で止める。
    // 測距のタイミングは 20MHz 前提で、遅いレートでは測距にならない (hw_pins.h)。
    uwbReady = initUwb(UWB_SPI_FAST_HZ);
    // initUwb() はアンカーと共通の値を入れるので、タグだけの上書きはその後で行う。
    // アンカー側の hostTimeoutMs は Poll を待つ窓の長さも兼ねるので、共通値は変えない。
    rangeConfig.hostTimeoutMs = TAG_HOST_TIMEOUT_MS;
    Serial.printf("TEST_START,result=%s\n", uwbReady ? "OK" : "FAIL");
    Serial.printf("RANGE_CYCLE,cycle_ms=%lu,host_timeout_ms=%lu\n", static_cast<unsigned long>(RANGE_CYCLE_MS),
                  static_cast<unsigned long>(rangeConfig.hostTimeoutMs));
    Serial.printf("ANCHORS,count=%u\n", static_cast<unsigned>(tagConfig.anchorCount));
    Serial.printf("POS_CONFIG,method=trilat2d,tag_z_mm=%ld,min_ranges=%u\n", static_cast<long>(TAG_Z_MM),
                  static_cast<unsigned>(TRILAT_MIN_RANGES));
    setLedId(tagId);
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
        serviceLed();
        delay(100);
        return;
    }

    if (uwbReady) {
        // 測距中の LED の書き換えは runRanging() の中で行う。
        runRanging();
    } else {
        // LED の ID 表示を切り替え続けられるよう、短い間隔で回す。
        serviceLed();
        delay(100);
    }
}
