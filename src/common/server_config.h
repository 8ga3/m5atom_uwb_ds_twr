// 測位サーバーからの構成取得 (GET /api/v1/config) と、その NVS キャッシュ。
//
// タグは起動時に 1 回だけサーバーへ問い合わせ、アンカーの ID と設置座標・バイアス
// 補正値・テレメトリ送信先を受け取る (doc/server-design.md 5.1 / 7 節)。取得した
// 構成は NVS へ保存し、次回以降の起動でサーバーへ届かなかったときはキャッシュで
// 動作する。キャッシュも無ければ巡回するアンカーが分からないので測距を開始しない。
//
// サーバーの IP とポートも NVS に持たせる。mDNS は ESP32 側の解決が不安定なので
// 使わず、Wi-Fi の資格情報と同じく起動ボタン押下時にシリアルから入力する
// (doc/server-design.md 9 節)。
//
// ヘッダのみで実装しているのは wifi_config.h と同じ理由による。common/ へ .cpp を
// 置くと build_src_filter が ANCHOR 側のビルドでも拾ってしまい、Wi-Fi と HTTP の
// コードが不要なファームへ入る (platformio.ini 参照)。
//
// 入力プロンプトの実装 (wifiPromptLine) は wifi_config.h のものを使い回す。設定 UI
// の操作方法をサーバー設定と Wi-Fi 設定で揃えるためで、両方とも TAG からしか
// include しない。
#pragma once

#include <ArduinoJson.h>
#include <Arduino.h>
#include <HTTPClient.h>
#include <Preferences.h>

#include "device_id.h"
#include "wifi_config.h"

// Wi-Fi (WIFI_NVS_NAMESPACE) や ID (DEVICE_ID_NVS_NAMESPACE) とは別の名前空間に
// 置く。サーバーを変えたときに Wi-Fi の設定を巻き込まずに消せる。
static constexpr char SERVER_NVS_NAMESPACE[]  = "srv";
static constexpr char SERVER_NVS_KEY_HOST[]   = "host";
static constexpr char SERVER_NVS_KEY_PORT[]   = "port";
static constexpr char SERVER_NVS_KEY_CONFIG[] = "config";

// IPv4 のドット記法の最大長 ("255.255.255.255")。名前解決は行わない。
static constexpr size_t SERVER_HOST_MAX       = 15;

// テレメトリ送信先 (doc/server-design.md 5.1 の telemetry.host) の長さ上限。
// サーバー側の検証は IPv6 も通すが、タグ側の送信経路は IPv4 前提なので同じ長さに
// 揃える。収まらない宛先を受け取ったときは切り詰めずにテレメトリを止める。
static constexpr size_t TELEMETRY_HOST_MAX    = SERVER_HOST_MAX;
static constexpr uint16_t SERVER_PORT_DEFAULT = 8000;  // サーバー側 settings.py の既定値

// 1 タグが巡回するアンカーの上限。4 台構成 (doc/multi-anchor-positioning-design.md)
// に対して余裕を持たせた値で、これを超えるぶんはサーバーが返しても捨てる。
static constexpr size_t UWB_ANCHOR_MAX = 8;

// リビジョン 0 は「構成を持っていない」ことを表す。サーバーの rev は 1 から始まる。
static constexpr uint32_t CONFIG_REV_UNSET = 0;

// キャッシュの構造体をそのまま NVS のブロブへ書くので、メンバを変えたらこの値を
// 上げる。古い形式を読み込んだら構成なしとして扱う。
static constexpr uint8_t CONFIG_CACHE_FORMAT = 1;

// HTTP の応答を待つ上限。走行前の起動時にしか呼ばないが、サーバーが落ちている
// ときに何十秒も止まらないよう短く切る。
static constexpr uint16_t CONFIG_HTTP_TIMEOUT_MS = 3000;

// 座標の妥当性の上限 (メートル)。桁を間違えた入力や壊れた応答を弾くための値で、
// 実験場の広さとしては十分に大きい。
static constexpr float CONFIG_COORD_MAX_M = 1000.0f;

struct AnchorConfig {
    uint16_t id;  // UWB の responderAddress そのもの (doc/server-design.md 5.4)
    int32_t xMm;
    int32_t yMm;
    int32_t zMm;
};

// サーバーから受け取る構成一式。座標は整数ミリメートルで保持する。浮動小数の
// ままキャッシュすると、保存と読み出しで値が変わったように見えることがある。
struct UwbConfig {
    uint8_t format;
    uint8_t anchorCount;
    uint8_t batchCycles;
    uint16_t panId;
    uint32_t rev;
    int32_t biasMm;
    AnchorConfig anchors[UWB_ANCHOR_MAX];
    char telemetryHost[TELEMETRY_HOST_MAX + 1];
    uint16_t telemetryPort;
    // この構成を取得したサーバー。rev はサーバーごとに独立した連番なので、
    // 宛先を変えたあとに前のサーバーの rev をそのまま送ると、値が偶然一致した
    // ときに 304 が返って古いアンカーで走り続けてしまう。
    char sourceHost[SERVER_HOST_MAX + 1];
    uint16_t sourcePort;
};

enum class ConfigFetchResult {
    Updated,      // 200 を受け取り、構成を更新した
    NotModified,  // 304。キャッシュがそのまま有効
    Failed,       // 未接続・未設定・通信失敗・応答が不正
};

static char serverHost[SERVER_HOST_MAX + 1] = {0};
static uint16_t serverPort                  = SERVER_PORT_DEFAULT;
static bool serverConfigured                = false;

// NVS からサーバーの宛先を読む。ホストが入っていれば true。
static bool serverEndpointLoad()
{
    serverHost[0] = '\0';
    serverPort    = SERVER_PORT_DEFAULT;

    Preferences prefs;
    // 名前空間がまだ作られていない (初回起動) 場合は read-only の begin が失敗する。
    if (!prefs.begin(SERVER_NVS_NAMESPACE, true)) return false;
    prefs.getString(SERVER_NVS_KEY_HOST, serverHost, sizeof(serverHost));
    serverPort = prefs.getUShort(SERVER_NVS_KEY_PORT, SERVER_PORT_DEFAULT);
    prefs.end();

    if (serverPort == 0) serverPort = SERVER_PORT_DEFAULT;
    return (serverHost[0] != '\0');
}

static bool serverEndpointSave(const char* host, uint16_t port)
{
    Preferences prefs;
    if (!prefs.begin(SERVER_NVS_NAMESPACE, false)) return false;
    const bool hostOk = (prefs.putString(SERVER_NVS_KEY_HOST, host) == strlen(host));
    const bool portOk = (prefs.putUShort(SERVER_NVS_KEY_PORT, port) == sizeof(uint16_t));
    prefs.end();
    return hostOk && portOk;
}

// 宛先とキャッシュの両方を消す。サーバーを変えたのに古い構成で走り出すのを防ぐ。
static void serverEndpointClear()
{
    Preferences prefs;
    if (!prefs.begin(SERVER_NVS_NAMESPACE, false)) return;
    prefs.clear();
    prefs.end();
}

// NVS のキャッシュを読む。形式が合わないか中身が壊れていれば false。
static bool configCacheLoad(UwbConfig& out)
{
    UwbConfig loaded = {};

    Preferences prefs;
    if (!prefs.begin(SERVER_NVS_NAMESPACE, true)) return false;
    const size_t read = prefs.getBytes(SERVER_NVS_KEY_CONFIG, &loaded, sizeof(loaded));
    prefs.end();

    if (read != sizeof(loaded)) return false;
    if (loaded.format != CONFIG_CACHE_FORMAT) return false;
    if (loaded.rev == CONFIG_REV_UNSET) return false;
    if ((loaded.anchorCount == 0) || (loaded.anchorCount > UWB_ANCHOR_MAX)) return false;

    loaded.telemetryHost[sizeof(loaded.telemetryHost) - 1] = '\0';
    loaded.sourceHost[sizeof(loaded.sourceHost) - 1]       = '\0';

    // 宛先を設定済みで、キャッシュの取得元と違うなら使わない。別のサーバーの
    // 構成で走り出すより、取得できるまで測距を止めるほうが安全である。宛先が
    // 未設定のときは取得しようがないので、そのままキャッシュを使う。
    if (serverConfigured && ((strcmp(loaded.sourceHost, serverHost) != 0) || (loaded.sourcePort != serverPort))) {
        Serial.printf("CONFIG,warn=cache_endpoint_mismatch,cached_host=%s,cached_port=%u\n",
                      (loaded.sourceHost[0] != '\0') ? loaded.sourceHost : "-",
                      static_cast<unsigned>(loaded.sourcePort));
        return false;
    }

    out = loaded;
    return true;
}

static bool configCacheSave(const UwbConfig& config)
{
    Preferences prefs;
    if (!prefs.begin(SERVER_NVS_NAMESPACE, false)) return false;
    const size_t written = prefs.putBytes(SERVER_NVS_KEY_CONFIG, &config, sizeof(config));
    prefs.end();
    return (written == sizeof(config));
}

// "0x0100" と "256" のどちらの表記も受け付ける。サーバーは前者で返すが、管理 API
// が 10 進数も受け付ける (doc/server-design.md 5.3) のに合わせておく。
//
// minId / maxId で用途ごとの範囲を絞る。アンカー ID は device_id.h の採番に従って
// 0x0100..0xFFFE に限る。0xFFFE より上 (0xFFFF) は 802.15.4 のブロードキャスト
// アドレスで、宛先に使うと全アンカーが同時に応答してしまう。
static bool configParseId(const char* text, uint16_t minId, uint16_t maxId, uint16_t& out)
{
    if ((text == nullptr) || (*text == '\0')) return false;

    char* end             = nullptr;
    const unsigned long v = strtoul(text, &end, 0);
    if ((end == text) || (*end != '\0')) return false;
    if ((v < minId) || (v > maxId)) return false;

    out = static_cast<uint16_t>(v);
    return true;
}

// JSON のメートル値を整数ミリメートルへ直す。範囲外と NaN はここで弾く。
static bool configMetersToMm(float meters, int32_t& out)
{
    if (isnan(meters) || isinf(meters)) return false;
    if (fabsf(meters) > CONFIG_COORD_MAX_M) return false;
    out = static_cast<int32_t>(lroundf(meters * 1000.0f));
    return true;
}

// 設定 UI。起動ボタン押下時だけシリアルから宛先を入力させる。Wi-Fi 設定と同じく
// ピリオド 1 文字で「変更しない」、ハイフン 1 文字で「消去」を表す。
//
// 入力を待たずに進む点も wifiSetup() と揃える。サーバーが無くてもキャッシュが
// あれば測距は動くので、コンソールを繋いでいない機体を起動時に止めない。
static bool serverEndpointSetup(bool forceSetup, WifiSetupMessageFn onMessage)
{
    bool stored = serverEndpointLoad();

    if (forceSetup) {
        Serial.printf("SERVER_SETUP,stored=%d,keep=.,clear=-\n", stored ? 1 : 0);

        char host[SERVER_HOST_MAX + 1] = {0};
        for (;;) {
            wifiPromptLine("srv_host", false, host, sizeof(host), onMessage);
            if ((strcmp(host, ".") == 0) || (strcmp(host, "-") == 0)) break;

            IPAddress parsed;
            if (parsed.fromString(host)) break;
            // 名前解決はしない。IP 以外を受け付けると、現場で解決できずに
            // 原因の分からない取得失敗になる。
            Serial.println("SERVER_SETUP,field=srv_host,result=ERR,reason=not_ipv4");
            if (onMessage != nullptr) onMessage("srv_host", "ERR", true);
        }

        if (strcmp(host, "-") == 0) {
            serverEndpointClear();
            serverHost[0] = '\0';
            serverPort    = SERVER_PORT_DEFAULT;
            stored        = false;
            Serial.println("SERVER_SETUP,result=cleared");
        } else if (strcmp(host, ".") == 0) {
            Serial.println("SERVER_SETUP,result=kept");
        } else {
            // ポートはピリオド 1 文字で既定値を選べるようにする。ほとんどの場合は
            // サーバーを既定ポートで動かすので、毎回入力させる必要がない。
            uint16_t port      = SERVER_PORT_DEFAULT;
            char portText[6]   = {0};
            bool portDecided   = false;
            while (!portDecided) {
                wifiPromptLine("srv_port", false, portText, sizeof(portText), onMessage);
                if (strcmp(portText, ".") == 0) {
                    portDecided = true;
                    break;
                }
                uint32_t value = 0;
                bool digitsOk  = true;
                for (const char* p = portText; *p != '\0'; ++p) {
                    if ((*p < '0') || (*p > '9')) {
                        digitsOk = false;
                        break;
                    }
                    value = (value * 10) + static_cast<uint32_t>(*p - '0');
                    if (value > 0xFFFFUL) {
                        digitsOk = false;
                        break;
                    }
                }
                if (digitsOk && (value > 0)) {
                    port        = static_cast<uint16_t>(value);
                    portDecided = true;
                    break;
                }
                Serial.println("SERVER_SETUP,field=srv_port,result=ERR,reason=out_of_range");
                if (onMessage != nullptr) onMessage("srv_port", "ERR", true);
            }

            const bool saved = serverEndpointSave(host, port);
            Serial.printf("SERVER_SETUP,result=%s,host=%s,port=%u\n", saved ? "OK" : "ERR", host,
                          static_cast<unsigned>(port));
            if (saved) {
                strncpy(serverHost, host, sizeof(serverHost) - 1);
                serverHost[sizeof(serverHost) - 1] = '\0';
                serverPort                         = port;
                stored                             = true;
            }
        }
    }

    serverConfigured = stored;
    if (!stored) {
        Serial.println("SERVER,state=unconfigured");
        return false;
    }

    Serial.printf("SERVER,host=%s,port=%u\n", serverHost, static_cast<unsigned>(serverPort));
    return true;
}

// 応答の JSON を構成へ写す。1 つでも必須項目が欠けていれば false を返し、
// 呼び出し側は手前のキャッシュをそのまま使い続ける。
static bool configParseJson(const JsonDocument& doc, UwbConfig& out)
{
    // 既定値で補わずに型と存在を確かめる。欠けた項目を 0 で埋めると、壊れた
    // 200 応答が「bias 0 の正しい構成」として有効なキャッシュを上書きしてしまう。
    JsonVariantConst revVar  = doc["rev"];
    JsonVariantConst biasVar = doc["bias_mm"];
    JsonVariantConst panVar  = doc["pan_id"];
    if (!revVar.is<uint32_t>() || !biasVar.is<int32_t>() || !panVar.is<const char*>()) return false;

    const uint32_t rev = revVar.as<uint32_t>();
    if (rev == CONFIG_REV_UNSET) return false;

    UwbConfig parsed = {};
    parsed.format    = CONFIG_CACHE_FORMAT;
    parsed.rev       = rev;
    parsed.biasMm    = biasVar.as<int32_t>();

    // PAN ID はアドレスではないので範囲を絞らず 16bit 全域を受け取る。
    if (!configParseId(panVar.as<const char*>(), 0x0000, 0xFFFF, parsed.panId)) return false;

    JsonArrayConst anchors = doc["anchors"].as<JsonArrayConst>();
    if (anchors.isNull()) return false;

    for (JsonObjectConst anchor : anchors) {
        if (parsed.anchorCount >= UWB_ANCHOR_MAX) {
            // 上限を超えたぶんは捨てる。巡回できない台数を受け取っても
            // 1 周期が伸びるだけで測位の役に立たない。
            Serial.printf("CONFIG,warn=anchor_overflow,max=%u\n", static_cast<unsigned>(UWB_ANCHOR_MAX));
            break;
        }

        AnchorConfig& slot   = parsed.anchors[parsed.anchorCount];
        JsonVariantConst idVar = anchor["id"];
        JsonVariantConst xVar  = anchor["x"];
        JsonVariantConst yVar  = anchor["y"];
        JsonVariantConst zVar  = anchor["z"];
        if (!idVar.is<const char*>() || !xVar.is<float>() || !yVar.is<float>() || !zVar.is<float>()) return false;
        if (!configParseId(idVar.as<const char*>(), ANCHOR_ID_MIN, ANCHOR_ID_MAX, slot.id)) return false;
        if (!configMetersToMm(xVar.as<float>(), slot.xMm)) return false;
        if (!configMetersToMm(yVar.as<float>(), slot.yMm)) return false;
        if (!configMetersToMm(zVar.as<float>(), slot.zMm)) return false;
        ++parsed.anchorCount;
    }
    if (parsed.anchorCount == 0) return false;

    // telemetry は構成の一部として必ず入る (doc/server-design.md 5.1)。
    JsonObjectConst telemetry = doc["telemetry"].as<JsonObjectConst>();
    if (telemetry.isNull()) return false;

    JsonVariantConst hostVar  = telemetry["host"];
    JsonVariantConst portVar  = telemetry["port"];
    JsonVariantConst batchVar = telemetry["batch_cycles"];
    if (!hostVar.is<const char*>() || !portVar.is<uint16_t>() || !batchVar.is<uint8_t>()) return false;

    const char* host = hostVar.as<const char*>();
    if (strlen(host) > TELEMETRY_HOST_MAX) {
        // サーバー側の検証は IPv6 も通すため、IPv4 に収まらない宛先が届きうる。
        // 切り詰めると別のアドレスへ投げることになるので、宛先を空にして
        // テレメトリだけを止める。測距と測位はこの値に依存しない。
        Serial.printf("CONFIG,warn=telemetry_host_too_long,len=%u,max=%u\n", static_cast<unsigned>(strlen(host)),
                      static_cast<unsigned>(TELEMETRY_HOST_MAX));
        parsed.telemetryHost[0] = '\0';
        parsed.telemetryPort    = 0;
    } else {
        strncpy(parsed.telemetryHost, host, sizeof(parsed.telemetryHost) - 1);
        parsed.telemetryHost[sizeof(parsed.telemetryHost) - 1] = '\0';
        parsed.telemetryPort                                   = portVar.as<uint16_t>();
    }

    parsed.batchCycles = batchVar.as<uint8_t>();
    if (parsed.batchCycles == 0) parsed.batchCycles = 1;

    out = parsed;
    return true;
}

// 取得した構成をシリアルへ残す。走行後にログだけを見て、どの座標表で走ったかを
// 追えるようにしておく。
static void configLogSummary(const UwbConfig& config, const char* source)
{
    Serial.printf("CONFIG,source=%s,rev=%lu,pan_id=0x%04X,bias_mm=%ld,anchors=%u\n", source,
                  static_cast<unsigned long>(config.rev), static_cast<unsigned>(config.panId),
                  static_cast<long>(config.biasMm), static_cast<unsigned>(config.anchorCount));
    for (uint8_t i = 0; i < config.anchorCount; ++i) {
        const AnchorConfig& anchor = config.anchors[i];
        Serial.printf("CONFIG_ANCHOR,id=0x%04X,x_mm=%ld,y_mm=%ld,z_mm=%ld\n", static_cast<unsigned>(anchor.id),
                      static_cast<long>(anchor.xMm), static_cast<long>(anchor.yMm), static_cast<long>(anchor.zMm));
    }
    Serial.printf("CONFIG_TELEMETRY,host=%s,port=%u,batch_cycles=%u\n",
                  (config.telemetryHost[0] != '\0') ? config.telemetryHost : "-",
                  static_cast<unsigned>(config.telemetryPort), static_cast<unsigned>(config.batchCycles));
}

// サーバーから構成を取得する。config に有効なキャッシュが入っていれば、その rev を
// If-None-Match で送って 304 を受けられるようにする。
//
// 成功したときだけ config を書き換える。失敗時に手元の値を壊すと、一度でも通信に
// 失敗した機体がキャッシュを失って走れなくなる。
static ConfigFetchResult configFetch(uint16_t tagId, UwbConfig& config)
{
    if (!serverConfigured) return ConfigFetchResult::Failed;
    if (!wifiIsConnected()) return ConfigFetchResult::Failed;

    char url[64];
    snprintf(url, sizeof(url), "http://%s:%u/api/v1/config?tag_id=%u", serverHost, static_cast<unsigned>(serverPort),
             static_cast<unsigned>(tagId));

    HTTPClient http;
    if (!http.begin(url)) {
        Serial.println("CONFIG,result=ERR,reason=begin");
        return ConfigFetchResult::Failed;
    }
    http.setTimeout(CONFIG_HTTP_TIMEOUT_MS);
    http.setConnectTimeout(CONFIG_HTTP_TIMEOUT_MS);
    // サーバーは Content-Length 付きの 1 レスポンスで返す (doc/server-design.md 5.1)。
    http.useHTTP10(true);

    char etag[24];
    if (config.rev != CONFIG_REV_UNSET) {
        snprintf(etag, sizeof(etag), "\"rev-%lu\"", static_cast<unsigned long>(config.rev));
        http.addHeader("If-None-Match", etag);
    }

    const int status = http.GET();
    if (status == HTTP_CODE_NOT_MODIFIED) {
        http.end();
        Serial.printf("CONFIG,result=NOT_MODIFIED,rev=%lu\n", static_cast<unsigned long>(config.rev));
        return ConfigFetchResult::NotModified;
    }
    if (status != HTTP_CODE_OK) {
        http.end();
        Serial.printf("CONFIG,result=ERR,reason=http,status=%d\n", status);
        return ConfigFetchResult::Failed;
    }

    JsonDocument doc;
    const DeserializationError error = deserializeJson(doc, http.getStream());
    http.end();
    if (error) {
        Serial.printf("CONFIG,result=ERR,reason=json,detail=%s\n", error.c_str());
        return ConfigFetchResult::Failed;
    }

    UwbConfig fetched = {};
    if (!configParseJson(doc, fetched)) {
        Serial.println("CONFIG,result=ERR,reason=invalid");
        return ConfigFetchResult::Failed;
    }

    // どのサーバーから取った構成かを一緒に残す。次回の起動で宛先が変わっていれば
    // configCacheLoad() がこの値を見てキャッシュを捨てる。
    strncpy(fetched.sourceHost, serverHost, sizeof(fetched.sourceHost) - 1);
    fetched.sourceHost[sizeof(fetched.sourceHost) - 1] = '\0';
    fetched.sourcePort                                 = serverPort;

    config = fetched;
    configLogSummary(config, "server");
    // 保存に失敗しても今回の走行は続けられる。次回の起動でキャッシュが古いままに
    // なるだけなので、ログを残して先へ進む。
    if (!configCacheSave(config)) Serial.println("CONFIG,warn=cache_save_failed");
    return ConfigFetchResult::Updated;
}
