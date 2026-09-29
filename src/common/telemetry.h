// 測位サーバーへのテレメトリ送信 (UDP)。
//
// 1 周期ぶんの測距結果をリングバッファへ積み、構成で指示された batch_cycles 周期ぶん
// たまったら 1 パケットにまとめて UDP で投げる (doc/server-design.md 6 節 / 7 節)。
// 再送も ACK 待ちも行わない。落ちたパケットはサーバー側で seq の欠番として数えられる。
//
// 送信は測距ループの中の 1 か所 (巡回の先頭スロットの直前) だけで行い、測距と測距の
// 間には割り込ませない。ソケットは非ブロッキングで使い、sendto() がソケットバッファへ
// コピーして即座に戻るようにする。Wi-Fi が切れている間は送信を飛ばし、リングバッファが
// 溢れたら古い周期から捨てる。再接続後にまとめて吐き出すと、その瞬間に測距ループが
// 止まるためである (doc/server-design.md 7.1)。
//
// ヘッダのみで実装しているのは wifi_config.h と同じ理由による。使うのは TAG だけで、
// common/ に .cpp を置くと ANCHOR 側のビルドにも入ってしまう (platformio.ini 参照)。
#pragma once

#include <Arduino.h>
#include <lwip/sockets.h>

#include <cerrno>

#include "server_config.h"
#include "wifi_config.h"

// パケット形式 (doc/server-design.md 6.2)。すべてリトルエンディアンで、パディングは入れない。
static constexpr uint32_t TELEMETRY_MAGIC        = 0x54425755;  // 'U' 'W' 'B' 'T'
static constexpr uint8_t TELEMETRY_VERSION       = 1;
static constexpr size_t TELEMETRY_HEADER_SIZE    = 24;
static constexpr size_t TELEMETRY_CYCLE_SIZE     = 16;
static constexpr size_t TELEMETRY_RANGE_SIZE     = 8;
static constexpr uint8_t TELEMETRY_COUNT_MAX     = 16;  // 1 パケットに詰められるサイクル数の上限
static constexpr uint8_t TELEMETRY_FIX_OK        = 0x01;
static constexpr uint8_t TELEMETRY_FIX_3D        = 0x02;
static constexpr uint8_t TELEMETRY_ELAPSED_MAX   = 255;    // elapsed_ms の飽和値
static constexpr uint16_t TELEMETRY_DT_MAX       = 65535;  // dt_ms の飽和値
static constexpr size_t TELEMETRY_PACKET_MAX =
    TELEMETRY_HEADER_SIZE + (TELEMETRY_COUNT_MAX * (TELEMETRY_CYCLE_SIZE + (TELEMETRY_RANGE_SIZE * UWB_ANCHOR_MAX)));

// 未送信の周期を溜めておける数。1 パケットの上限と同じにしておき、Wi-Fi が切れている
// 間に溜まるのは最大でも 1 パケットぶんとする。
static constexpr size_t TELEMETRY_RING_SIZE = TELEMETRY_COUNT_MAX;

// 送信の統計をシリアルへ出す間隔。
static constexpr uint32_t TELEMETRY_LOG_INTERVAL_MS = 10000;

struct TelemetryRange {
    uint16_t anchorId;
    uint8_t status;  // 0 = OK、それ以外は M5Stamp_UWBError の値
    uint8_t elapsedMs;
    int32_t distanceMm;  // status != 0 のときは 0
};

// 1 周期ぶんの記録。測位欄 (fixFlags 以降) は main_tag.cpp の solvePosition() が埋める。
// 解けなかった周期は fixFlags を 0 にして座標欄を 0 のまま送る。測距の結果だけでも
// サーバー側で周期と欠測を追える。
struct TelemetryCycle {
    uint32_t seq;  // 周期の通番。起動ごとに 0 から数える
    uint32_t tMs;  // 周期の先頭スロットを始めた millis()
    uint8_t fixFlags;
    uint8_t usedCount;
    int32_t xMm;
    int32_t yMm;
    int16_t zMm;
    uint16_t residualMm;
    TelemetryRange ranges[UWB_ANCHOR_MAX];
};

struct TelemetryStats {
    uint32_t sentPackets;
    uint32_t sendFailed;     // sendto() が失敗した回数 (その束は捨てる)
    uint32_t offlineSkips;   // 送るぶんが溜まっていたのに Wi-Fi が未接続だった回数
    uint32_t droppedCycles;  // リングバッファが溢れて捨てた周期の数
    uint32_t maxSendUs;      // 前回の統計出力以降で sendto() にかかった時間の最大値
};

static bool telemetryEnabled        = false;
static int telemetrySocket          = -1;
static sockaddr_in telemetryDest    = {};
static uint16_t telemetryTagId      = 0;
static uint32_t telemetryBootId     = 0;
static uint8_t telemetryAnchorN     = 0;
static uint8_t telemetryBatch       = 1;
static TelemetryCycle telemetryRing[TELEMETRY_RING_SIZE];
static size_t telemetryHead         = 0;  // 最も古い周期の位置
static size_t telemetryCount        = 0;
static TelemetryStats telemetryStats = {};
static uint32_t telemetryLastLogMs  = 0;
static uint8_t telemetryPacket[TELEMETRY_PACKET_MAX];
static bool telemetrySocketErrorLogged = false;

// 構成を反映する。宛先・束ねる周期数・アンカー台数のどれかが変わると 1 パケット内の
// レコード数や宛先が揃わなくなるので、溜まっているぶんは捨てて作り直す。
static void telemetryConfigure(uint16_t tagId, uint32_t bootId, const UwbConfig& config)
{
    telemetryEnabled = false;
    telemetryHead    = 0;
    telemetryCount   = 0;
    telemetryTagId   = tagId;
    telemetryBootId  = bootId;
    telemetryAnchorN = config.anchorCount;

    // サーバー側は batch_cycles をパケット形式の上限 (16) までに制限しているが、
    // 古いサーバーから受け取ったキャッシュでも壊れたパケットを作らないようここでも抑える。
    telemetryBatch = config.batchCycles;
    if (telemetryBatch == 0) telemetryBatch = 1;
    if (telemetryBatch > TELEMETRY_COUNT_MAX) {
        Serial.printf("TELEMETRY,warn=batch_clamped,requested=%u,max=%u\n", static_cast<unsigned>(telemetryBatch),
                      static_cast<unsigned>(TELEMETRY_COUNT_MAX));
        telemetryBatch = TELEMETRY_COUNT_MAX;
    }

    if ((config.telemetryPort == 0) || (config.telemetryHost[0] == '\0')) {
        Serial.println("TELEMETRY,state=disabled,reason=no_destination");
        return;
    }
    if (config.anchorCount == 0) {
        Serial.println("TELEMETRY,state=disabled,reason=no_anchor");
        return;
    }

    IPAddress host;
    if (!host.fromString(config.telemetryHost)) {
        Serial.printf("TELEMETRY,state=disabled,reason=bad_host,host=%s\n", config.telemetryHost);
        return;
    }
    telemetryDest                 = {};
    telemetryDest.sin_family      = AF_INET;
    telemetryDest.sin_port        = htons(config.telemetryPort);
    telemetryDest.sin_addr.s_addr = static_cast<uint32_t>(host);

    telemetryEnabled   = true;
    telemetryLastLogMs = millis();
    Serial.printf("TELEMETRY,state=enabled,host=%s,port=%u,batch_cycles=%u,boot_id=0x%08lX\n", config.telemetryHost,
                  static_cast<unsigned>(config.telemetryPort), static_cast<unsigned>(telemetryBatch),
                  static_cast<unsigned long>(bootId));
}

// 1 周期ぶんの記録を積む。溢れたら最も古い周期を捨てる。残った周期の seq は連続した
// ままなので、1 パケットに詰めたときに先頭の seq からの連番で表せる。
static void telemetryPush(const TelemetryCycle& cycle)
{
    if (!telemetryEnabled) return;
    if (telemetryCount == TELEMETRY_RING_SIZE) {
        telemetryHead = (telemetryHead + 1) % TELEMETRY_RING_SIZE;
        --telemetryCount;
        ++telemetryStats.droppedCycles;
    }
    telemetryRing[(telemetryHead + telemetryCount) % TELEMETRY_RING_SIZE] = cycle;
    ++telemetryCount;
}

static uint8_t* telemetryPut8(uint8_t* p, uint8_t v)
{
    *p = v;
    return p + 1;
}

static uint8_t* telemetryPut16(uint8_t* p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    return p + 2;
}

static uint8_t* telemetryPut32(uint8_t* p, uint32_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
    return p + 4;
}

// リングバッファの先頭から count 周期ぶんをパケットへ詰め、そのバイト数を返す。
static size_t telemetryBuildPacket(uint8_t count)
{
    const TelemetryCycle& first = telemetryRing[telemetryHead];

    uint8_t* p = telemetryPacket;
    p          = telemetryPut32(p, TELEMETRY_MAGIC);
    p          = telemetryPut8(p, TELEMETRY_VERSION);
    p          = telemetryPut8(p, 0);  // flags: タグは走り続けるので終了フラグは立てない
    p          = telemetryPut16(p, telemetryTagId);
    p          = telemetryPut32(p, telemetryBootId);
    p          = telemetryPut32(p, first.seq);
    p          = telemetryPut32(p, first.tMs);
    p          = telemetryPut8(p, count);
    p          = telemetryPut8(p, telemetryAnchorN);
    p          = telemetryPut16(p, 0);  // reserved

    for (uint8_t i = 0; i < count; ++i) {
        const TelemetryCycle& cycle = telemetryRing[(telemetryHead + i) % TELEMETRY_RING_SIZE];
        const uint32_t dt           = cycle.tMs - first.tMs;
        p = telemetryPut16(p, (dt > TELEMETRY_DT_MAX) ? TELEMETRY_DT_MAX : static_cast<uint16_t>(dt));
        p = telemetryPut8(p, cycle.fixFlags);
        p = telemetryPut8(p, cycle.usedCount);
        p = telemetryPut32(p, static_cast<uint32_t>(cycle.xMm));
        p = telemetryPut32(p, static_cast<uint32_t>(cycle.yMm));
        p = telemetryPut16(p, static_cast<uint16_t>(cycle.zMm));
        p = telemetryPut16(p, cycle.residualMm);
        for (uint8_t a = 0; a < telemetryAnchorN; ++a) {
            const TelemetryRange& range = cycle.ranges[a];
            p                           = telemetryPut16(p, range.anchorId);
            p                           = telemetryPut8(p, range.status);
            p                           = telemetryPut8(p, range.elapsedMs);
            p                           = telemetryPut32(p, static_cast<uint32_t>(range.distanceMm));
        }
    }
    return static_cast<size_t>(p - telemetryPacket);
}

static void telemetryLogStats()
{
    Serial.printf("TELEMETRY_STAT,sent=%lu,send_fail=%lu,offline_skip=%lu,dropped_cycles=%lu,pending=%u,max_send_us=%lu\n",
                  static_cast<unsigned long>(telemetryStats.sentPackets),
                  static_cast<unsigned long>(telemetryStats.sendFailed),
                  static_cast<unsigned long>(telemetryStats.offlineSkips),
                  static_cast<unsigned long>(telemetryStats.droppedCycles), static_cast<unsigned>(telemetryCount),
                  static_cast<unsigned long>(telemetryStats.maxSendUs));
    telemetryStats.maxSendUs = 0;
}

// 巡回の先頭スロットの直前に呼ぶ。batch_cycles 周期ぶん溜まっていれば 1 パケットだけ
// 送る。1 回の呼び出しで送るのは多くても 1 パケットで、未送信が複数パケットぶん
// 溜まっていても次の周期へ回す。
static void telemetryService()
{
    if (!telemetryEnabled) return;

    if ((millis() - telemetryLastLogMs) >= TELEMETRY_LOG_INTERVAL_MS) {
        telemetryLastLogMs = millis();
        telemetryLogStats();
    }

    if (telemetryCount < telemetryBatch) return;
    if (!wifiIsConnected()) {
        // 送らずに次へ進む。溜まったぶんは telemetryPush() が古いほうから捨てる。
        ++telemetryStats.offlineSkips;
        return;
    }

    if (telemetrySocket < 0) {
        telemetrySocket = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (telemetrySocket < 0) {
            // 作れない状態が続くと周期ごとにここへ来るので、理由は最初の 1 回だけ残し、
            // 以降の件数は TELEMETRY_STAT の send_fail で追う。溜まった周期はそのまま
            // 残り、溢れたぶんは telemetryPush() が古いほうから捨てる。
            ++telemetryStats.sendFailed;
            if (!telemetrySocketErrorLogged) {
                Serial.printf("TELEMETRY,result=ERR,reason=socket,errno=%d\n", errno);
                telemetrySocketErrorLogged = true;
            }
            return;
        }
    }

    const size_t length = telemetryBuildPacket(telemetryBatch);
    const uint32_t startUs = micros();
    // MSG_DONTWAIT: バッファが空いていなければ待たずに失敗させる。待つくらいなら捨てる。
    const int sent = lwip_sendto(telemetrySocket, telemetryPacket, length, MSG_DONTWAIT,
                                 reinterpret_cast<const sockaddr*>(&telemetryDest), sizeof(telemetryDest));
    const uint32_t elapsedUs = micros() - startUs;
    if (elapsedUs > telemetryStats.maxSendUs) telemetryStats.maxSendUs = elapsedUs;

    if (sent < 0) {
        // 失敗した束も捨てる。再送しないのは設計どおり (doc/server-design.md 3.3)。
        // 最初の 1 回だけ理由を残し、以降の件数は TELEMETRY_STAT で追う。
        if (telemetryStats.sendFailed == 0) Serial.printf("TELEMETRY,result=ERR,reason=sendto,errno=%d\n", errno);
        ++telemetryStats.sendFailed;
    } else {
        ++telemetryStats.sentPackets;
    }
    telemetryHead = (telemetryHead + telemetryBatch) % TELEMETRY_RING_SIZE;
    telemetryCount -= telemetryBatch;
}
