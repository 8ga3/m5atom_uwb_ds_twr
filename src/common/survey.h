// アンカー座標の自動推定 (self-survey) のためのアンカー間の相互測距。ANCHOR だけが使う。
// 手順とフレームの形式は doc/multi-anchor-positioning-design.md の 4.2 を参照。
//
// 1 台 (PC につないだアンカー) がシリアルの `survey` コマンドでコーディネータになり、
// 起床ビーコンで他のアンカーを survey モードへ入れる。コーディネータは全ての順序付きの組
// (i, j) について i に「j と測距せよ」と指示し、i から返ってきた測距値をシリアルへ
// SURVEY_PAIR 行として出す。座標の計算は PC 側 (location_server_uwb の tools/survey_solve.py)
// で行う。
//
// survey モードのアンカーは普段 receiveFrame() で制御フレームを待ち、自機宛ての「応答せよ」
// 要求を受けたときだけ respondDSRange() を回す。respondDSRange() は自機宛ての Poll 以外の
// フレームを受けると payload を捨てて戻るので、応答ループを回したままでは制御フレームを
// 受けられないためである。
#pragma once

#include <Arduino.h>
#include <M5Stamp_UWB.h>
#include <string.h>

#include "device_id.h"
#include "status.h"
#include "uwb_link.h"

// survey のフレームだけが使う PAN ID。タグの測距 (UWB_DEFAULT_PAN_ID か、サーバーが配る値) と
// 分けておくことで、タグが動いたままでも survey の応答側がタグの Poll に答えることがなく、
// survey 中のアンカーがタグのフレームを制御フレームと取り違えることもない。
static constexpr uint16_t SURVEY_PAN_ID    = 0x5356;  // "SV"
static constexpr uint16_t SURVEY_BROADCAST = 0xFFFF;

// 起床ビーコンの送信元アドレスとシーケンス番号。通常の応答ループ (respondDSRange()) は、
// 自機宛ての Poll 以外のフレームを受けると payload を捨てて RangeFrameMismatch を返し、
// 送信元 (requester) とシーケンス番号だけを残す。0xFFFF はブロードキャストアドレスで、
// どの機体の ID にもならない (device_id.h) ので、送信元が 0xFFFF でシーケンス番号が
// 下の値なら起床ビーコンと見なす。
static constexpr uint16_t SURVEY_WAKE_SRC = 0xFFFF;
static constexpr uint8_t SURVEY_WAKE_SEQ  = 0xA5;

// 1 組あたりの測距回数の既定値と上限。上限は報告フレーム (SVD) に入る測距値の数で決まる。
// payload は 116 バイトまでで、見出し 9 バイト + 4 バイト × 24 = 105 バイト。
static constexpr uint8_t SURVEY_SAMPLES_DEFAULT = 20;
static constexpr uint8_t SURVEY_SAMPLES_MAX     = 24;
// コーディネータが扱えるアンカーの数。順序付きの組は N(N-1) 個で、16 台なら 240 組。
static constexpr size_t SURVEY_ANCHORS_MAX = 16;

// 起床ビーコンを送り続ける時間と間隔。通常の応答ループは他の交信のフレームを受けるたびに
// 戻って受信をやり直すので、ビーコンを短い間隔で繰り返せば、そのうちのどれかを受ける。
static constexpr uint32_t SURVEY_WAKE_MS          = 1500;
static constexpr uint32_t SURVEY_REWAKE_MS        = 1000;
static constexpr uint32_t SURVEY_WAKE_INTERVAL_MS = 10;
static constexpr int SURVEY_PING_ROUNDS           = 3;
// 要求を受けてから応答 (SVA など) を送るまでの待ち。要求の送り手は送信完了を確かめてから
// 受信を始めるので、すぐに返すと受信を始める前に応答が終わってしまうことがある。
static constexpr uint32_t SURVEY_REPLY_DELAY_US = 2000;
// 要求 1 回あたりの応答待ちと、送り直す回数。合計 (500 ms) は下の SURVEY_RESPOND_IDLE_MS より
// 長くしておく。直前の組で応答側だったアンカーは、測距が終わってもしばらく応答ループに
// 残っていて制御フレームを受けられないため。
static constexpr uint32_t SURVEY_ACK_TIMEOUT_MS = 50;
static constexpr int SURVEY_REQUEST_TRIES       = 10;
// 応答側が最後の Poll からこの時間だけ何も受けなければ、応答ループを抜けて制御フレームの
// 待ちに戻る。要求側が測距を終えたことは通知されないので、応答側は必ずこの時間だけ残る。
static constexpr uint32_t SURVEY_RESPOND_IDLE_MS = 300;
// 応答ループに留まる時間の上限。測距 1 回は数 ms なので、48 回でも 1 秒かからない。
static constexpr uint32_t SURVEY_RESPOND_MAX_MS = 3000;
// コーディネータが測距の指示 (SVI) を出してから報告 (SVD) を待つ時間。指示への応答が
// 無かった場合も、指示自体は届いていて測距中かもしれないので、短めの時間だけ待つ。
static constexpr uint32_t SURVEY_REPORT_TIMEOUT_MS       = 6000;
static constexpr uint32_t SURVEY_REPORT_TIMEOUT_NOACK_MS = 1500;
// survey モードのアンカーが、survey の PAN のフレームをこの時間受けなければ通常の応答へ戻る。
// コーディネータが途中で止まった (USB を抜いたなど) ときに、survey モードに残り続けないように
// するため。
static constexpr uint32_t SURVEY_MEMBER_IDLE_MS = 15000;
// 測距の Final を Response の受信から送るまでの遅延。タグの値 (1200) は ESP32-S3 で測った
// ホストの処理時間に合わせた値で、Atom Lite / Atom Matrix (ESP32) を測距の要求側にしたときの
// 処理時間は測っていない。survey は測距のレートを求めないので、余裕を持たせる。応答側の
// Final の受信窓 (Response の送信の 500〜3500 uus 後) には収まる。
static constexpr uint32_t SURVEY_FINAL_TX_DELAY_UUS = 2000;
// 測距の試行の間隔。応答側が距離通知を送ってから次の Poll の受信を始めるまでの時間を空ける。
static constexpr uint32_t SURVEY_ATTEMPT_GAP_MS = 3;
// 応答側の応答が確認できないまま測距を試みたとき、この回数続けて失敗したら諦める。
static constexpr uint8_t SURVEY_UNACKED_TRIES = 3;

// 制御フレームの種類。payload は "SV" + 種類の 1 文字 + 本体。
enum : char {
    SURVEY_MSG_WAKE = 'W',  // 起床ビーコン。本体なし。ブロードキャスト
    SURVEY_MSG_PING = 'P',  // 在否の確認。本体なし
    SURVEY_MSG_INIT = 'I',  // 測距の指示。本体: 相手の ID (2)、回数 (1)
    SURVEY_MSG_RESP = 'R',  // 応答ループへ入る要求。本体: 要求側が試す回数の上限 (1)
    SURVEY_MSG_DATA = 'D',  // 測距の報告。本体は surveyEncodeReport() を参照
    SURVEY_MSG_ACK  = 'A',  // 応答。本体: 応答した要求の種類 (1)、そのシーケンス番号 (1)
    SURVEY_MSG_END  = 'E',  // survey の終了。本体なし。ブロードキャスト
};

// 1 組の測距の結果の状態。報告フレームとシリアル出力に使う。
enum class SurveyPairStatus : uint8_t {
    Ok = 0,       // 1 回以上測距できた
    Failed,       // 応答側は要求に応えたが、測距が 1 回も成功しなかった
    NoResponder,  // 応答側が要求に応えず、測距も成功しなかった
    NoAck,        // 測距の要求側 (i) が指示に応えず、報告も届かなかった (コーディネータだけが付ける)
    NoReport,     // 指示には応えたが、報告が届かなかった (コーディネータだけが付ける)
};

struct SurveyPairResult {
    uint16_t initiator         = 0;
    uint16_t responder         = 0;
    uint8_t tries              = 0;
    uint8_t ok                 = 0;
    SurveyPairStatus status    = SurveyPairStatus::Failed;
    M5Stamp_UWBError lastError = M5Stamp_UWBError::Ok;
    int32_t mm[SURVEY_SAMPLES_MAX] = {0};
};

struct SurveyFrame {
    uint16_t src   = 0;
    uint16_t dst   = 0;
    uint8_t seq    = 0;
    char kind      = 0;
    size_t bodyLen = 0;
    // payload 全体 (116 バイトまで) から "SV" と種類の 3 バイトを除いた本体。
    uint8_t body[116] = {0};
};

static uint16_t surveySelf          = DEVICE_ID_UNSET;
static uint8_t surveyTxSeq          = 0;
static uint32_t surveyLastFrameMs   = 0;
static const char* surveyRoleText   = "";
// 直前に受けた測距の指示と、それに対して送った報告。指示への応答が失われてコーディネータが
// 指示を送り直してきたときに、測距をやり直さずに同じ報告を返す。
static uint16_t surveyLastInitSrc   = 0;
static uint8_t surveyLastInitSeq    = 0;
static bool surveyLastInitValid     = false;
static uint8_t surveyLastReport[116] = {0};
static size_t surveyLastReportLen   = 0;

static const char* surveyStatusName(SurveyPairStatus status)
{
    switch (status) {
        case SurveyPairStatus::Ok:          return "OK";
        case SurveyPairStatus::Failed:      return "FAILED";
        case SurveyPairStatus::NoResponder: return "NO_RESPONDER";
        case SurveyPairStatus::NoAck:       return "NO_ACK";
        case SurveyPairStatus::NoReport:    return "NO_REPORT";
        default:                            return "UNKNOWN";
    }
}

static void surveyPut16(uint8_t* dst, uint16_t value)
{
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>(value >> 8);
}

static uint16_t surveyGet16(const uint8_t* src)
{
    return static_cast<uint16_t>(src[0] | (static_cast<uint16_t>(src[1]) << 8));
}

static void surveyPut32(uint8_t* dst, int32_t value)
{
    const uint32_t v = static_cast<uint32_t>(value);
    for (int i = 0; i < 4; ++i) dst[i] = static_cast<uint8_t>(v >> (8 * i));
}

static int32_t surveyGet32(const uint8_t* src)
{
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= static_cast<uint32_t>(src[i]) << (8 * i);
    return static_cast<int32_t>(v);
}

// 画面と LED に survey 中であることを出す。青は通常の状態表示で使わない色なので、
// 画面のない Lite 系でも survey モードに入ったことが分かる。
static void surveyShow(const char* detail)
{
    setLed(0, 0, 255);
    if (!hasDisplay) return;

    lgfx::LovyanGFX& gfx = beginDisplayFrame();
    gfx.setTextColor(CYAN);
    gfx.printf("ANC %u\n", static_cast<unsigned>(surveySelf));
    gfx.println("SURVEY");
    gfx.setTextColor(WHITE);
    gfx.println(surveyRoleText);
    gfx.println(detail);
    endDisplayFrame();
}

// 通常の応答ループの結果が起床ビーコンを受けたことを表すか。
static bool surveyIsWakeBeacon(const M5Stamp_UWBDSResponderResult& result)
{
    return !result.success && (result.error == M5Stamp_UWBError::RangeFrameMismatch) &&
           (result.requester == SURVEY_WAKE_SRC) && (result.sequence == SURVEY_WAKE_SEQ);
}

static bool surveySendRaw(uint16_t src, uint16_t dst, uint8_t seq, char kind, const uint8_t* body, size_t bodyLen)
{
    uint8_t payload[116];
    if (bodyLen + 3 > sizeof(payload)) return false;
    payload[0] = 'S';
    payload[1] = 'V';
    payload[2] = static_cast<uint8_t>(kind);
    if (bodyLen > 0) memcpy(&payload[3], body, bodyLen);

    M5Stamp_UWBFrameConfig frame;
    frame.panId       = SURVEY_PAN_ID;
    frame.src         = src;
    frame.dst         = dst;
    frame.sequence    = seq;
    frame.useSequence = true;
    return uwb.sendFrame(payload, bodyLen + 3, frame, 10).success;
}

// survey の制御フレームを 1 つ受ける。自機宛てかブロードキャストの制御フレームを受けたら
// true を返す。それ以外 (タイムアウト、受信エラー、他の PAN や他機宛てのフレーム) は false。
// survey の PAN のフレームは、他機どうしの測距のフレームも含めて、受けた時刻を
// surveyLastFrameMs に記録する。
//
// receiveFrame() は直前の測距で設定された受信タイムアウト (rxTimeoutUus = 3 ms) を
// 解除しないので、測距の後は数 ms ごとに受信をやり直すことになる。やり直しの間の
// わずかな時間に来たフレームは取りこぼすが、要求は送り直すので問題にならない。
static bool surveyReceive(SurveyFrame& frame, uint32_t timeoutMs)
{
    uint8_t payload[sizeof(frame.body) + 3];
    const M5Stamp_UWBRxResult rx = uwb.receiveFrame(payload, sizeof(payload), timeoutMs);
    if (!rx.success || (rx.panId != SURVEY_PAN_ID)) return false;
    surveyLastFrameMs = millis();
    if ((rx.payloadLength < 3) || (payload[0] != 'S') || (payload[1] != 'V')) return false;
    if ((rx.dst != surveySelf) && (rx.dst != SURVEY_BROADCAST)) return false;

    frame.src     = rx.src;
    frame.dst     = rx.dst;
    frame.seq     = rx.sequence;
    frame.kind    = static_cast<char>(payload[2]);
    frame.bodyLen = rx.payloadLength - 3;
    memcpy(frame.body, &payload[3], frame.bodyLen);
    return true;
}

static void surveyAck(const SurveyFrame& request)
{
    const uint8_t body[2] = {static_cast<uint8_t>(request.kind), request.seq};
    delayMicroseconds(SURVEY_REPLY_DELAY_US);
    surveySendRaw(surveySelf, request.src, request.seq, SURVEY_MSG_ACK, body, sizeof(body));
}

static bool surveyWaitAck(uint16_t from, char kind, uint8_t seq, uint32_t timeoutMs)
{
    const uint32_t startMs = millis();
    while ((millis() - startMs) < timeoutMs) {
        SurveyFrame frame;
        if (!surveyReceive(frame, timeoutMs - (millis() - startMs))) continue;
        if ((frame.src == from) && (frame.kind == SURVEY_MSG_ACK) && (frame.bodyLen >= 2) &&
            (frame.body[0] == static_cast<uint8_t>(kind)) && (frame.body[1] == seq)) {
            return true;
        }
    }
    return false;
}

// 応答を求める要求を送り、応答が来るまで送り直す。seqOut には要求のシーケンス番号を返す。
static bool surveyRequest(uint16_t dst, char kind, const uint8_t* body, size_t bodyLen, uint8_t* seqOut = nullptr)
{
    const uint8_t seq = ++surveyTxSeq;
    if (seqOut != nullptr) *seqOut = seq;
    for (int i = 0; i < SURVEY_REQUEST_TRIES; ++i) {
        surveySendRaw(surveySelf, dst, seq, kind, body, bodyLen);
        if (surveyWaitAck(dst, kind, seq, SURVEY_ACK_TIMEOUT_MS)) return true;
    }
    return false;
}

static M5Stamp_UWBDSRangeConfig surveyRangeConfig(uint16_t initiator, uint16_t responder)
{
    M5Stamp_UWBDSRangeConfig config = rangeConfig;
    config.panId            = SURVEY_PAN_ID;
    config.initiatorAddress = initiator;
    config.responderAddress = responder;
    config.finalTxDelayUus  = SURVEY_FINAL_TX_DELAY_UUS;
    return config;
}

// 応答側として、要求側 (initiator) の Poll が途絶えるまで respondDSRange() を回す。
// 応答側で成功した測距でも、距離通知が失われると要求側では失敗になり、要求側は試行を続ける。
// そのため成功の回数では抜けず、要求側が試す回数の上限 (maxTries) に達したときだけ早く抜ける。
static void surveyRespond(uint16_t initiator, uint8_t maxTries)
{
    M5Stamp_UWBDSRangeConfig config = surveyRangeConfig(0x0000, surveySelf);
    config.hostTimeoutMs            = SURVEY_RESPOND_IDLE_MS;

    uint8_t served          = 0;
    bool ranging            = false;
    const uint32_t startMs  = millis();
    uint32_t lastActivityMs = startMs;
    while ((served < maxTries) && ((millis() - lastActivityMs) < SURVEY_RESPOND_IDLE_MS) &&
           ((millis() - startMs) < SURVEY_RESPOND_MAX_MS)) {
        const M5Stamp_UWBDSResponderResult result = uwb.respondDSRange(config);
        if (result.requester != initiator) continue;
        // Poll を受けて Response を予約したかは txMarginUs で分かる (main_anchor.cpp の runResponder())。
        const bool polled = result.success || (result.txMarginUs != M5STAMP_UWB_TX_MARGIN_UNKNOWN);
        // 測距が始まる前は、要求側からのフレームをすべて活動として数える。応答ループへ入る要求 (SVR)
        // への応答が失われると、要求側は SVR を送り直してから測距を始めるため。測距が始まった後は
        // Poll だけを数える。測距を終えた要求側はコーディネータへ報告 (SVD) を送り直し続けるので、
        // それも数えると、自機がコーディネータのときに応答ループから抜けられず報告を受けそこねる。
        if (ranging && !polled) continue;
        ranging        = ranging || polled;
        lastActivityMs = millis();
        if (result.success) ++served;
    }
}

// 要求側として相手 (target) と samples 回の測距を試みる。
static void surveyRangePair(uint16_t target, uint8_t samples, SurveyPairResult& out)
{
    out           = SurveyPairResult();
    out.initiator = surveySelf;
    out.responder = target;

    // 失敗した試行も数えて、成功の 2 倍まで試す。
    const uint8_t maxTries    = static_cast<uint8_t>(samples * 2);
    const uint8_t respBody[1] = {maxTries};
    const bool acked          = surveyRequest(target, SURVEY_MSG_RESP, respBody, sizeof(respBody));

    const M5Stamp_UWBDSRangeConfig config = surveyRangeConfig(surveySelf, target);
    while ((out.ok < samples) && (out.tries < maxTries)) {
        // 応答が確認できていない相手は、応答ループに入っていないかもしれない。
        // 続けて失敗したら諦める。
        if (!acked && (out.ok == 0) && (out.tries >= SURVEY_UNACKED_TRIES)) break;
        delay(SURVEY_ATTEMPT_GAP_MS);
        const M5Stamp_UWBDSRangeResult result = uwb.requestDSRange(config);
        ++out.tries;
        if (result.success) {
            out.mm[out.ok++] = result.distanceMm;
        } else {
            out.lastError = result.error;
        }
    }

    if (out.ok > 0) {
        out.status = SurveyPairStatus::Ok;
    } else {
        out.status = acked ? SurveyPairStatus::Failed : SurveyPairStatus::NoResponder;
    }
}

// 報告フレーム (SVD) の本体: 相手の ID (2)、指示のシーケンス番号 (1)、状態 (1)、
// 試行回数 (1)、成功回数 (1)、最後の失敗の理由 (1)、予約 (2)、測距値 (int32 × 成功回数)。
static constexpr size_t SURVEY_REPORT_HEADER_LEN = 9;

static size_t surveyEncodeReport(const SurveyPairResult& result, uint8_t initSeq, uint8_t* body)
{
    surveyPut16(&body[0], result.responder);
    body[2] = initSeq;
    body[3] = static_cast<uint8_t>(result.status);
    body[4] = result.tries;
    body[5] = result.ok;
    body[6] = static_cast<uint8_t>(result.lastError);
    body[7] = 0;
    body[8] = 0;
    for (uint8_t i = 0; i < result.ok; ++i) surveyPut32(&body[SURVEY_REPORT_HEADER_LEN + 4 * i], result.mm[i]);
    return SURVEY_REPORT_HEADER_LEN + 4 * static_cast<size_t>(result.ok);
}

static bool surveyDecodeReport(const SurveyFrame& frame, uint16_t initiator, SurveyPairResult& out, uint8_t& initSeq)
{
    if (frame.bodyLen < SURVEY_REPORT_HEADER_LEN) return false;
    const uint8_t ok = frame.body[5];
    if ((ok > SURVEY_SAMPLES_MAX) || (frame.bodyLen != SURVEY_REPORT_HEADER_LEN + 4 * static_cast<size_t>(ok))) {
        return false;
    }

    out           = SurveyPairResult();
    out.initiator = initiator;
    out.responder = surveyGet16(&frame.body[0]);
    initSeq       = frame.body[2];
    out.status    = static_cast<SurveyPairStatus>(frame.body[3]);
    out.tries     = frame.body[4];
    out.ok        = ok;
    out.lastError = static_cast<M5Stamp_UWBError>(frame.body[6]);
    for (uint8_t i = 0; i < ok; ++i) out.mm[i] = surveyGet32(&frame.body[SURVEY_REPORT_HEADER_LEN + 4 * i]);
    return true;
}

// 測距の指示 (SVI) を受けた側の処理。相手と測距し、結果をコーディネータへ報告する。
static void surveyHandleInit(const SurveyFrame& frame)
{
    if (frame.bodyLen < 3) return;

    // 指示への応答が失われて同じ指示が届いた場合は、測距をやり直さずに同じ報告を返す。
    const bool duplicate = surveyLastInitValid && (surveyLastInitSrc == frame.src) && (surveyLastInitSeq == frame.seq);
    surveyAck(frame);
    if (!duplicate) {
        const uint16_t target = surveyGet16(&frame.body[0]);
        uint8_t samples       = frame.body[2];
        if (samples > SURVEY_SAMPLES_MAX) samples = SURVEY_SAMPLES_MAX;

        char detail[16];
        snprintf(detail, sizeof(detail), "->%u", static_cast<unsigned>(target));
        surveyShow(detail);

        SurveyPairResult result;
        surveyRangePair(target, samples, result);
        surveyLastReportLen = surveyEncodeReport(result, frame.seq, surveyLastReport);
        surveyLastInitSrc   = frame.src;
        surveyLastInitSeq   = frame.seq;
        surveyLastInitValid = true;
        Serial.printf("SURVEY_RANGE,i=0x%04X,j=0x%04X,status=%s,try=%u,ok=%u\n", static_cast<unsigned>(surveySelf),
                      static_cast<unsigned>(target), surveyStatusName(result.status),
                      static_cast<unsigned>(result.tries), static_cast<unsigned>(result.ok));
    }
    surveyRequest(frame.src, SURVEY_MSG_DATA, surveyLastReport, surveyLastReportLen);
}

// 自機宛ての制御フレームを処理する。survey を終える (SVE を受けた) ときに false を返す。
// コーディネータも、報告を待っている間に届いた要求をここで処理する。
static bool surveyHandleFrame(const SurveyFrame& frame)
{
    switch (frame.kind) {
        case SURVEY_MSG_PING:
            surveyAck(frame);
            return true;
        case SURVEY_MSG_RESP: {
            if (frame.bodyLen < 1) return true;
            surveyAck(frame);
            char detail[16];
            snprintf(detail, sizeof(detail), "<-%u", static_cast<unsigned>(frame.src));
            surveyShow(detail);
            surveyRespond(frame.src, frame.body[0]);
            return true;
        }
        case SURVEY_MSG_INIT:
            surveyHandleInit(frame);
            return true;
        case SURVEY_MSG_END:
            return false;
        default:
            // 起床ビーコンの残りや、宛先が自機になっている応答の再送などは無視する。
            return true;
    }
}

// 起床ビーコンを受けたアンカーの survey モード。SVE を受けるか、survey のフレームが
// SURVEY_MEMBER_IDLE_MS のあいだ途絶えるまで戻らない。
static void surveyRunMember(uint16_t selfId)
{
    surveySelf          = selfId;
    surveyRoleText      = "MEMBER";
    surveyLastInitValid = false;
    surveyLastFrameMs   = millis();
    Serial.printf("SURVEY_MEMBER,state=enter,id=0x%04X\n", static_cast<unsigned>(selfId));
    surveyShow("WAIT");

    bool ended = false;
    while (!ended && ((millis() - surveyLastFrameMs) < SURVEY_MEMBER_IDLE_MS)) {
        SurveyFrame frame;
        if (!surveyReceive(frame, 100)) continue;
        ended = !surveyHandleFrame(frame);
    }
    Serial.printf("SURVEY_MEMBER,state=exit,reason=%s\n", ended ? "end" : "idle");
}

static void surveyWake(uint32_t durationMs)
{
    const uint32_t startMs = millis();
    while ((millis() - startMs) < durationMs) {
        surveySendRaw(SURVEY_WAKE_SRC, SURVEY_BROADCAST, SURVEY_WAKE_SEQ, SURVEY_MSG_WAKE, nullptr, 0);
        delay(SURVEY_WAKE_INTERVAL_MS);
    }
}

static void surveyPrintPair(const SurveyPairResult& result)
{
    // 中央値は目視の確認用。PC 側は個々の測距値 (mm=) から計算し直す。
    int32_t sorted[SURVEY_SAMPLES_MAX];
    memcpy(sorted, result.mm, sizeof(int32_t) * result.ok);
    for (uint8_t a = 1; a < result.ok; ++a) {
        const int32_t v = sorted[a];
        int b           = a - 1;
        while ((b >= 0) && (sorted[b] > v)) {
            sorted[b + 1] = sorted[b];
            --b;
        }
        sorted[b + 1] = v;
    }

    Serial.printf("SURVEY_PAIR,i=0x%04X,j=0x%04X,status=%s,try=%u,ok=%u", static_cast<unsigned>(result.initiator),
                  static_cast<unsigned>(result.responder), surveyStatusName(result.status),
                  static_cast<unsigned>(result.tries), static_cast<unsigned>(result.ok));
    if (result.ok > 0) {
        const int32_t median = (result.ok % 2 == 1)
                                   ? sorted[result.ok / 2]
                                   : static_cast<int32_t>((static_cast<int64_t>(sorted[result.ok / 2 - 1]) +
                                                           sorted[result.ok / 2]) / 2);
        Serial.printf(",median_mm=%ld,mm=", static_cast<long>(median));
        for (uint8_t i = 0; i < result.ok; ++i) {
            Serial.printf(i == 0 ? "%ld" : ";%ld", static_cast<long>(result.mm[i]));
        }
    }
    if (result.status != SurveyPairStatus::Ok) Serial.printf(",error=%s", errorShortName(result.lastError));
    Serial.print('\n');
}

// コーディネータとして、自機以外の initiator に target との測距を指示し、報告を待つ。
static void surveyCollectPair(uint16_t initiator, uint16_t target, uint8_t samples, SurveyPairResult& out)
{
    uint8_t body[3];
    surveyPut16(&body[0], target);
    body[2] = samples;
    uint8_t initSeq  = 0;
    const bool acked = surveyRequest(initiator, SURVEY_MSG_INIT, body, sizeof(body), &initSeq);

    out           = SurveyPairResult();
    out.initiator = initiator;
    out.responder = target;
    out.status    = acked ? SurveyPairStatus::NoReport : SurveyPairStatus::NoAck;

    const uint32_t timeoutMs = acked ? SURVEY_REPORT_TIMEOUT_MS : SURVEY_REPORT_TIMEOUT_NOACK_MS;
    const uint32_t startMs   = millis();
    while ((millis() - startMs) < timeoutMs) {
        SurveyFrame frame;
        if (!surveyReceive(frame, 100)) continue;
        if ((frame.src == initiator) && (frame.kind == SURVEY_MSG_DATA)) {
            SurveyPairResult report;
            uint8_t reportSeq = 0;
            if (!surveyDecodeReport(frame, initiator, report, reportSeq)) continue;
            surveyAck(frame);
            // 前の組の報告の再送 (コーディネータの応答が失われた場合) は捨てる。
            if ((reportSeq != initSeq) || (report.responder != target)) continue;
            out = report;
            return;
        }
        // 自機が応答側の組では、initiator から応答ループへ入る要求 (SVR) が届く。
        surveyHandleFrame(frame);
    }
}

// コーディネータとして survey を実行する。ids は自機を含むアンカー ID の一覧。
static void surveyRunCoordinator(uint16_t selfId, const uint16_t* ids, size_t count, uint8_t samples)
{
    surveySelf          = selfId;
    surveyRoleText      = "COORD";
    surveyLastInitValid = false;
    const uint32_t startMs = millis();

    Serial.printf("SURVEY_START,coordinator=0x%04X,samples=%u,anchors=", static_cast<unsigned>(selfId),
                  static_cast<unsigned>(samples));
    for (size_t i = 0; i < count; ++i) Serial.printf(i == 0 ? "0x%04X" : ";0x%04X", static_cast<unsigned>(ids[i]));
    Serial.print('\n');

    surveyShow("WAKE");
    surveyWake(SURVEY_WAKE_MS);

    // 起床ビーコンを受けそこねたアンカーがあれば、ビーコンを送り直して確かめ直す。
    bool present[SURVEY_ANCHORS_MAX] = {false};
    for (int round = 0; round < SURVEY_PING_ROUNDS; ++round) {
        bool missing = false;
        for (size_t i = 0; i < count; ++i) {
            if (present[i]) continue;
            present[i] = (ids[i] == selfId) || surveyRequest(ids[i], SURVEY_MSG_PING, nullptr, 0);
            missing    = missing || !present[i];
        }
        if (!missing || (round + 1 == SURVEY_PING_ROUNDS)) break;
        surveyWake(SURVEY_REWAKE_MS);
    }
    for (size_t i = 0; i < count; ++i) {
        Serial.printf("SURVEY_ANCHOR,id=0x%04X,present=%d\n", static_cast<unsigned>(ids[i]), present[i] ? 1 : 0);
    }

    unsigned pairs   = 0;
    unsigned okPairs = 0;
    for (size_t a = 0; a < count; ++a) {
        for (size_t b = 0; b < count; ++b) {
            if ((a == b) || !present[a] || !present[b]) continue;

            char detail[16];
            snprintf(detail, sizeof(detail), "%u>%u", static_cast<unsigned>(ids[a] & 0xFFF),
                     static_cast<unsigned>(ids[b] & 0xFFF));
            surveyShow(detail);

            SurveyPairResult result;
            if (ids[a] == selfId) {
                surveyRangePair(ids[b], samples, result);
            } else {
                surveyCollectPair(ids[a], ids[b], samples, result);
            }
            surveyPrintPair(result);
            ++pairs;
            if (result.status == SurveyPairStatus::Ok) ++okPairs;
        }
    }

    // 終了の通知は応答を求めないので、何度か繰り返す。受けそこねたアンカーも
    // SURVEY_MEMBER_IDLE_MS で通常の応答へ戻る。
    for (int i = 0; i < 10; ++i) {
        surveySendRaw(selfId, SURVEY_BROADCAST, ++surveyTxSeq, SURVEY_MSG_END, nullptr, 0);
        delay(SURVEY_WAKE_INTERVAL_MS);
    }
    Serial.printf("SURVEY_END,pairs=%u,ok_pairs=%u,elapsed_ms=%lu\n", pairs, okPairs,
                  static_cast<unsigned long>(millis() - startMs));
}

// シリアルのコマンド行を解釈する。形式は `survey [n=<回数>] <ID> <ID> ...` で、ID は
// ID の設定 (device_id.h) と同じく 10 進数だけを受け付ける。自機の ID が一覧に無ければ
// 先頭に足す。解釈できなければ reason に理由を入れて false を返す。
static bool surveyParseCommand(const char* line, uint16_t selfId, uint16_t* ids, size_t& count, uint8_t& samples,
                               const char*& reason)
{
    count   = 0;
    samples = SURVEY_SAMPLES_DEFAULT;

    const char* p = line;
    while (*p == ' ') ++p;
    if (strncmp(p, "survey", 6) != 0 || ((p[6] != ' ') && (p[6] != '\0'))) {
        reason = "unknown_command";
        return false;
    }
    p += 6;

    ids[count++] = selfId;
    for (;;) {
        while (*p == ' ') ++p;
        if (*p == '\0') break;

        const bool isSamples = (strncmp(p, "n=", 2) == 0);
        if (isSamples) p += 2;
        uint32_t value = 0;
        int digits     = 0;
        while ((*p >= '0') && (*p <= '9')) {
            value = value * 10 + static_cast<uint32_t>(*p - '0');
            if (value > 0xFFFFUL) {
                reason = "out_of_range";
                return false;
            }
            ++p;
            ++digits;
        }
        if ((digits == 0) || ((*p != ' ') && (*p != '\0'))) {
            reason = "format";
            return false;
        }

        if (isSamples) {
            if ((value < 1) || (value > SURVEY_SAMPLES_MAX)) {
                reason = "samples_range";
                return false;
            }
            samples = static_cast<uint8_t>(value);
            continue;
        }
        if ((value < ANCHOR_ID_MIN) || (value > ANCHOR_ID_MAX)) {
            reason = "id_range";
            return false;
        }
        bool seen = false;
        for (size_t i = 0; i < count; ++i) seen = seen || (ids[i] == value);
        if (seen) continue;
        if (count >= SURVEY_ANCHORS_MAX) {
            reason = "too_many_anchors";
            return false;
        }
        ids[count++] = static_cast<uint16_t>(value);
    }

    if (count < 2) {
        reason = "too_few_anchors";
        return false;
    }
    return true;
}
