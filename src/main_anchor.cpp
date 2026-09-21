// M5Atom S3 / AtomS3 Lite / Atom Lite UWB DS-TWR ANCHOR example code

#include <M5Unified.h>
#include <M5Stamp_UWB.h>

#include "common/host_init.h"
#include "common/hw_pins.h"
#include "common/status.h"
#include "common/uwb_link.h"
#include "device_id.h"

static constexpr uint32_t LOG_INTERVAL = 20;
// The tag ranges every 200ms while our receive window is only 100ms, so quiet
// gaps between exchanges are normal. Hold the last result on screen this long
// before admitting the tag is actually gone.
static constexpr uint32_t IDLE_GRACE_MS = 1000;

// 自機のアンカー ID (= responderAddress)。NVS から読むか起動時に設定する。
uint16_t anchorId      = DEVICE_ID_UNSET;
uint32_t responseCount = 0;
uint32_t failCount     = 0;
uint32_t noPollCount   = 0;
uint32_t noFinalCount  = 0;

// Only Poll frames that actually reached us count as attempts; a silent tag
// (NO POLL) would otherwise inflate the denominator forever.
static uint32_t attemptCount()
{
    return responseCount + failCount + noFinalCount;
}

// Same six-line layout as the tag sketch, with the state carried in the color.
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
    // Wait for a Poll/Final exchange and return the DS-TWR distance result.
    const M5Stamp_UWBDSResponderResult result = uwb.respondDSRange(rangeConfig);
    if (!result.success) {
        // Idle receive timeouts are expected and are not counted as failures.
        if (result.error == M5Stamp_UWBError::RxTimeout) {
            // requester stays 0 when no Poll was received at all; a non-zero value
            // means the Poll/Response exchange started but the Final frame was lost.
            const bool noPoll = (result.requester == 0);
            noPoll ? ++noPollCount : ++noFinalCount;

            if (noPoll) {
                // Normal idle gap: stay quiet and keep the last reading visible.
                if ((millis() - lastSuccessMs) < IDLE_GRACE_MS) return;
            } else if (noFinalCount % LOG_INTERVAL == 0) {
                // A lost Final frame is a real anomaly, so keep a sparse trace of it.
                Serial.printf("DS_RESP_NO_FINAL,requester=0x%X,seq=%u,elapsed_ms=%lu,no_final=%lu\n", result.requester,
                              result.sequence, static_cast<unsigned long>(result.elapsedMs),
                              static_cast<unsigned long>(noFinalCount));
            }

            // Redraw only on state change to avoid flicker.
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

    // Print accumulated statistics every LOG_INTERVAL responses.
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
    // フィールドは responder では未使用。responderAddress はこのアンカーの
    // 自機 ID: respondDSRange() drops every frame whose dst differs, which is
    // what keeps the tag's sweep from being answered by more than one anchor
    // at a time.
    rangeConfig.initiatorAddress = 0x0000;
    rangeConfig.responderAddress = anchorId;

    uwbReady = initUwb(UWB_SPI_FAST_HZ);
    if (!uwbReady && (UWB_SPI_FAST_HZ != UWB_SPI_FAST_FALLBACK_HZ)) {
        // The link either never came up or failed the fast-rate readback. Retry
        // once at the library default so a marginal board still ranges.
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
