// M5Atom S3 / AtomS3 Lite / Atom Lite 用 UWB DS-TWR ANCHOR サンプルコード

#include <M5Unified.h>
#include <M5Stamp_UWB.h>

#include "common/device_id.h"
#include "common/host_init.h"
#include "common/hw_pins.h"
#include "common/status.h"
#include "common/uwb_link.h"

static constexpr uint32_t LOG_INTERVAL = 20;
// タグ側の測距周期は 200ms、受信ウィンドウは 100ms しかないため、交信の間に無通信の
// 間隔ができるのは正常。タグが本当にいなくなったと判断するまで、最後の結果を
// この時間だけ画面に残す。
static constexpr uint32_t IDLE_GRACE_MS = 1000;

// 自機のアンカー ID (= responderAddress)。NVS から読むか起動時に設定する。
uint16_t anchorId      = DEVICE_ID_UNSET;
uint32_t responseCount = 0;
uint32_t failCount     = 0;
uint32_t noPollCount   = 0;
uint32_t noFinalCount  = 0;

// 実際に届いた Poll フレームだけを試行回数として数える。無音のタグ (NO POLL) まで
// 数えると、分母が際限なく膨らんでしまう。
static uint32_t attemptCount()
{
    return responseCount + failCount + noFinalCount;
}

// タグ側スケッチと同じ 6 行レイアウト。状態は文字色で表す。
static void updateStatus(DisplayState state, float distanceM, uint32_t elapsedMs, uint16_t requester,
                         const char* errorText)
{
    updateLed(state);
    if (!hasDisplay) return;

    const uint16_t color = stateColor(state);

    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setCursor(0, 0);
    M5.Display.setTextColor(color);
    M5.Display.println("UWB ANCHOR");
    // 自機 ID は設定時と同じ 10 進で出す。UWB が上がっていないときは ID より
    // 故障の方が知りたいので、その行を潰して FAIL を出す。
    if (uwbReady) {
        M5.Display.printf("ID:%u\n", static_cast<unsigned>(anchorId));
    } else {
        M5.Display.println("STA:FAIL");
    }

    if (state == DisplayState::Ok) {
        M5.Display.printf("D:%.3fm\n", distanceM);
        M5.Display.setTextColor(WHITE);
        M5.Display.printf("T:%lums\n", static_cast<unsigned long>(elapsedMs));
    } else {
        M5.Display.println("D:----");
        M5.Display.printf("E:%s\n", errorText);
        M5.Display.setTextColor(WHITE);
    }

    // 測距してきたタグの ID。タグは当面 1 台だが、複数台になったときに
    // どのタグと交信しているかはここでしか分からない。
    M5.Display.printf("TG:%u\n", static_cast<unsigned>(requester));
    M5.Display.printf("OK:%lu/%lu\n", static_cast<unsigned long>(responseCount),
                      static_cast<unsigned long>(attemptCount()));
}

DisplayState lastDisplayState = DisplayState::Init;
bool lastWaitingWasNoPoll     = false;
uint32_t lastSuccessMs        = 0;

static void runResponder()
{
    // Poll/Final の交信を待ち、DS-TWR の測距結果を返す。
    const M5Stamp_UWBDSResponderResult result = uwb.respondDSRange(rangeConfig);
    if (!result.success) {
        // 無通信による受信タイムアウトは想定内であり、失敗とはカウントしない。
        if (result.error == M5Stamp_UWBError::RxTimeout) {
            // Poll が全く受信できなかった場合 requester は 0 のまま。0 以外の値なら
            // Poll/Response の交信は始まったが Final フレームが届かなかったことを示す。
            const bool noPoll = (result.requester == 0);
            noPoll ? ++noPollCount : ++noFinalCount;

            if (noPoll) {
                // 通常の無通信区間なので何もせず、最後の測距結果を画面に残す。
                if ((millis() - lastSuccessMs) < IDLE_GRACE_MS) return;
            } else if (noFinalCount % LOG_INTERVAL == 0) {
                // Final フレームの欠落は本当の異常なので、間引いて記録しておく。
                Serial.printf("DS_RESP_NO_FINAL,requester=0x%X,seq=%u,elapsed_ms=%lu,no_final=%lu\n", result.requester,
                              result.sequence, static_cast<unsigned long>(result.elapsedMs),
                              static_cast<unsigned long>(noFinalCount));
            }

            // ちらつきを避けるため、状態が変わったときだけ再描画する。
            if (lastDisplayState != DisplayState::Waiting || lastWaitingWasNoPoll != noPoll) {
                updateStatus(DisplayState::Waiting, 0.0f, result.elapsedMs, result.requester,
                             noPoll ? "NOPOLL" : "NOFIN");
                lastDisplayState     = DisplayState::Waiting;
                lastWaitingWasNoPoll = noPoll;
            }
            return;
        }

        ++failCount;
        updateStatus(DisplayState::Fail, 0.0f, result.elapsedMs, result.requester, errorShortName(result.error));
        lastDisplayState = DisplayState::Fail;
        if (failCount % LOG_INTERVAL == 0) {
            Serial.printf("DS_RESP_STAT,count=%lu,fail=%lu,last=FAIL,error=%s\n",
                          static_cast<unsigned long>(responseCount), static_cast<unsigned long>(failCount),
                          uwb.lastErrorName());
        }
        return;
    }

    ++responseCount;
    lastSuccessMs = millis();
    updateStatus(DisplayState::Ok, result.distanceM, result.elapsedMs, result.requester, nullptr);
    lastDisplayState = DisplayState::Ok;

    // LOG_INTERVAL 回応答するごとに累積統計を出力する。
    if (responseCount % LOG_INTERVAL == 0) {
        Serial.printf(
            "DS_RESP_STAT,count=%lu,fail=%lu,no_poll=%lu,no_final=%lu,last=OK,seq=%u,requester=0x%X,distance_mm=%ld,distance_m=%.3f,elapsed_ms=%lu\n",
            static_cast<unsigned long>(responseCount), static_cast<unsigned long>(failCount),
            static_cast<unsigned long>(noPollCount), static_cast<unsigned long>(noFinalCount), result.sequence,
            result.requester, static_cast<long>(result.distanceMm), result.distanceM,
            static_cast<unsigned long>(result.elapsedMs));
    }
}

void setup()
{
    beginHost();
    beginSerial("ANCHOR");
    const bool buttonHeld = readBootButtonHeld();

    // ID が決まるまで UWB は初期化しない。responderAddress は下で
    // この値から設定する。
    idRangeMin = ANCHOR_ID_MIN;
    idRangeMax = ANCHOR_ID_MAX;
    anchorId   = deviceIdSetup("ANCHOR", ANCHOR_ID_MIN, ANCHOR_ID_MAX, buttonHeld, showIdSetup);

    // 応答側は受信フレームの src を見ておらず (dst と panId のみ照合)、応答は
    // 常に Poll の送信元へ返す。したがってタグ側の ID を知る必要はなく、この
    // フィールドは responder では未使用。responderAddress はこのアンカーの自機 ID
    // であり、respondDSRange() は dst が一致しないフレームをすべて捨てる。これに
    // よって、タグが順に呼びかけていく中で複数アンカーが同時に応答しないようになる。
    rangeConfig.initiatorAddress = 0x0000;
    rangeConfig.responderAddress = anchorId;

    uwbReady = initUwb(UWB_SPI_FAST_HZ);
    if (!uwbReady && (UWB_SPI_FAST_HZ != UWB_SPI_FAST_FALLBACK_HZ)) {
        // リンクが上がらなかったか、高速レートでの読み戻しに失敗した。ライブラリ
        // 既定のクロックで 1 回だけ再試行し、際 (きわ) の基板でも測距できるようにする。
        uwb.end();
        uwbReady = initUwb(UWB_SPI_FAST_FALLBACK_HZ);
    }
    Serial.printf("TEST_START,result=%s\n", uwbReady ? "OK" : "FAIL");
    updateStatus(DisplayState::Init, 0.0f, 0, 0, uwbReady ? "----" : errorShortName(uwb.lastError()));
}

void loop()
{
    if (uwbReady) {
        runResponder();
    } else {
        delay(1000);
    }
}
