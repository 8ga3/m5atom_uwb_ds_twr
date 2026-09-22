// UWB 16bit ショートアドレスの保存 (NVS) と、シリアルからの設定 UI。
// ANCHOR (main_anchor.cpp) と TAG (main_tag.cpp) の両方から使う共通ヘッダ。
#pragma once

#include <Arduino.h>
#include <Preferences.h>

// アドレスは役割ごとに範囲を分ける。フレームのアドレスを見ただけで送信元の
// 役割が分かり、タグとアンカーで ID 空間が衝突しない。
//   タグ   0x0001..0x00FF  当面 1 台だが DL-TDoA を見据えて 255 台分予約
//   アンカー 0x0100..0xFFFE  0xFFFF は 802.15.4 のブロードキャストアドレス
static constexpr uint16_t TAG_ID_MIN    = 0x0001;
static constexpr uint16_t TAG_ID_MAX    = 0x00FF;
static constexpr uint16_t ANCHOR_ID_MIN = 0x0100;
static constexpr uint16_t ANCHOR_ID_MAX = 0xFFFE;

// 0 はどちらの範囲にも入らないので「未設定」を表せる。
static constexpr uint16_t DEVICE_ID_UNSET = 0;

// NVS は再フラッシュしても消えない (erase_flash しない限り)。全機に同じファーム
// を書き込んでも、各機は一度設定した ID を保持し続ける。
static constexpr char DEVICE_ID_NVS_NAMESPACE[] = "uwb";
static constexpr char DEVICE_ID_NVS_KEY[]       = "id";

// 設定 UI の表示コールバック。画面のある AtomS3 は LCD へ、画面のない Lite 系は
// LED だけで状態を出す。text は入力中の文字列、error は直前の入力が不正だった
// ことを示す。
using DeviceIdMessageFn = void (*)(const char* text, bool error);

static uint16_t deviceIdLoad(uint16_t minId, uint16_t maxId)
{
    Preferences prefs;
    // 名前空間がまだ作られていない (初回起動) 場合は read-only の begin が失敗する。
    if (!prefs.begin(DEVICE_ID_NVS_NAMESPACE, true)) return DEVICE_ID_UNSET;
    const uint16_t id = prefs.getUShort(DEVICE_ID_NVS_KEY, DEVICE_ID_UNSET);
    prefs.end();

    // 範囲外の値は、同じ基板にもう一方の役割のファームを書いた痕跡。設定済みとは
    // 見なさず、その ID で送信もしない。
    if ((id < minId) || (id > maxId)) return DEVICE_ID_UNSET;
    return id;
}

static bool deviceIdSave(uint16_t id)
{
    Preferences prefs;
    if (!prefs.begin(DEVICE_ID_NVS_NAMESPACE, false)) return false;
    const bool ok = (prefs.putUShort(DEVICE_ID_NVS_KEY, id) == sizeof(uint16_t));
    prefs.end();
    return ok;
}

// 入力は 10 進数のみ。16 進や前置記号を受け付けると現場で取り違える。
static bool deviceIdParse(const char* text, uint16_t minId, uint16_t maxId, uint16_t& out)
{
    if ((text == nullptr) || (*text == '\0')) return false;

    uint32_t value = 0;
    for (const char* p = text; *p != '\0'; ++p) {
        if ((*p < '0') || (*p > '9')) return false;
        value = (value * 10) + static_cast<uint32_t>(*p - '0');
        if (value > 0xFFFFUL) return false;  // 桁あふれ前に打ち切る
    }
    if ((value < minId) || (value > maxId)) return false;

    out = static_cast<uint16_t>(value);
    return true;
}

// 有効な ID が入力されるまでブロックする。不正値は理由を返して再要求する。
static uint16_t deviceIdPromptSerial(const char* role, uint16_t minId, uint16_t maxId, uint16_t currentId,
                                     DeviceIdMessageFn onMessage)
{
    char line[8]    = {0};  // 最大 5 桁 + 終端。余裕を見て 8
    size_t len      = 0;
    bool overflow   = false;
    bool needPrompt = true;

    for (;;) {
        if (needPrompt) {
            Serial.printf("ID_INPUT,role=%s,min=%u,max=%u,current=%u,format=decimal\n", role,
                          static_cast<unsigned>(minId), static_cast<unsigned>(maxId),
                          static_cast<unsigned>(currentId));
            needPrompt = false;
            len        = 0;
            overflow   = false;
            line[0]    = '\0';
        }

        const int c = Serial.read();
        if (c < 0) {
            delay(10);
            continue;
        }

        if ((c != '\r') && (c != '\n')) {
            if (len + 1 < sizeof(line)) {
                line[len++] = static_cast<char>(c);
                line[len]   = '\0';
                if (onMessage) onMessage(line, false);
            } else {
                // 桁数オーバー。以降の文字は捨て、改行時にエラーとして扱う。
                overflow = true;
            }
            continue;
        }

        // CRLF の 2 文字目など、空行は入力の区切りとして無視する。
        if ((len == 0) && !overflow) continue;

        uint16_t id = DEVICE_ID_UNSET;
        if (!overflow && deviceIdParse(line, minId, maxId, id)) {
            Serial.printf("ID_INPUT,result=OK,id=%u\n", static_cast<unsigned>(id));
            return id;
        }

        Serial.printf("ID_INPUT,result=ERR,input=%s%s,reason=range_or_format\n", line, overflow ? "..." : "");
        if (onMessage) onMessage("ERR", true);
        needPrompt = true;
    }
}

// 起動時の ID 確定。未設定、範囲外、あるいはボタン押下による強制設定のときだけ
// シリアル入力を待つ。それ以外は NVS の値をそのまま返す。
static uint16_t deviceIdSetup(const char* role, uint16_t minId, uint16_t maxId, bool forceSetup,
                              DeviceIdMessageFn onMessage)
{
    const uint16_t stored = deviceIdLoad(minId, maxId);
    if ((stored != DEVICE_ID_UNSET) && !forceSetup) {
        Serial.printf("DEVICE_ID,role=%s,id=%u,id_hex=0x%04X,source=nvs\n", role, static_cast<unsigned>(stored),
                      static_cast<unsigned>(stored));
        return stored;
    }

    Serial.printf("ID_SETUP,role=%s,reason=%s\n", role, forceSetup ? "button" : "unset");
    if (onMessage) onMessage("", false);

    const uint16_t id = deviceIdPromptSerial(role, minId, maxId, stored, onMessage);
    const bool saved  = deviceIdSave(id);
    Serial.printf("DEVICE_ID,role=%s,id=%u,id_hex=0x%04X,source=serial,saved=%d\n", role, static_cast<unsigned>(id),
                  static_cast<unsigned>(id), saved ? 1 : 0);
    return id;
}
