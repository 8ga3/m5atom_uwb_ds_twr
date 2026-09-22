// Wi-Fi 接続と、SSID / パスフレーズの保存 (NVS)・シリアルからの設定 UI。
//
// 資格情報はソースコードにもリポジトリにも置かない。実機の NVS だけに持たせ、
// 起動ボタンを押しながら電源を入れたときにシリアルコンソールから入力する。
// device_id.h の ID 設定と同じ考え方で、全機に同じファームを書き込んでも各機の
// 設定は残り、ビルド成果物やリポジトリには秘密が入らない。
//
// 現在使っているのは TAG (main_tag.cpp) だけだが、役割に依存する処理は持たせて
// いない。アンカーの self-survey (doc/multi-anchor-positioning-design.md 4 節) で
// 距離行列をサーバーへ直接送るようになれば、ANCHOR 側からもそのまま使える。
//
// ヘッダのみで実装しているのは、include しないビルドへ Wi-Fi のコードが一切
// 入らないようにするため。common/ へ .cpp を置くと build_src_filter が両方の
// env で拾ってしまう (platformio.ini 参照)。
//
// 注意: ESP32 の NVS は既定では暗号化されない。フラッシュを吸い出せる相手からは
// パスフレーズを読み出せる。ここで防げるのはリポジトリ・配布ファームへの混入で
// あって、端末を物理的に奪われた場合の保護にはならない。
#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>

// ID (device_id.h) とは別の名前空間に置く。用途ごとに分けておくと、Wi-Fi の
// 設定だけを消したいときに ID を巻き込まない。
static constexpr char WIFI_NVS_NAMESPACE[] = "wifi";
static constexpr char WIFI_NVS_KEY_SSID[]  = "ssid";
static constexpr char WIFI_NVS_KEY_PASS[]  = "pass";

static constexpr size_t WIFI_SSID_MAX = 32;  // IEEE 802.11 の SSID 長の上限
static constexpr size_t WIFI_PASS_MAX = 63;  // WPA2 パスフレーズの上限

// 設定 UI の表示コールバック。field は入力中の項目名、text は入力済みの文字列
// (パスフレーズのときは伏せ字に置き換え済み)、error は直前の入力が不正だった
// ことを示す。
using WifiSetupMessageFn = void (*)(const char* field, const char* text, bool error);

// 接続に使う資格情報。パスフレーズは WiFi.begin() を呼んだ直後に消す。
static char wifiSsid[WIFI_SSID_MAX + 1] = {0};
static char wifiPass[WIFI_PASS_MAX + 1] = {0};

static bool wifiConfigured      = false;  // 資格情報があり接続を開始したか
static bool wifiConnected       = false;  // 直近の監視で接続できていたか
static uint32_t wifiLastCheckMs = 0;

// 使い終わったパスフレーズを RAM から消す。memset は「以降読まれない」と判断
// されると最適化で削られることがあるので、volatile 経由で書く。
static void wifiWipe(char* buffer, size_t size)
{
    volatile char* p = buffer;
    for (size_t i = 0; i < size; ++i) p[i] = '\0';
}

// NVS から資格情報を読む。SSID が入っていれば true。
static bool wifiCredentialsLoad()
{
    wifiWipe(wifiSsid, sizeof(wifiSsid));
    wifiWipe(wifiPass, sizeof(wifiPass));

    Preferences prefs;
    // 名前空間がまだ作られていない (初回起動) 場合は read-only の begin が失敗する。
    if (!prefs.begin(WIFI_NVS_NAMESPACE, true)) return false;
    prefs.getString(WIFI_NVS_KEY_SSID, wifiSsid, sizeof(wifiSsid));
    prefs.getString(WIFI_NVS_KEY_PASS, wifiPass, sizeof(wifiPass));
    prefs.end();

    return (wifiSsid[0] != '\0');
}

static bool wifiCredentialsSave(const char* ssid, const char* pass)
{
    Preferences prefs;
    if (!prefs.begin(WIFI_NVS_NAMESPACE, false)) return false;

    const bool ssidOk = (prefs.putString(WIFI_NVS_KEY_SSID, ssid) == strlen(ssid));
    // putString は空文字列を書いたときも 0 を返すので、戻り値だけでは失敗と
    // 区別できない。パスフレーズなしの開放 AP を許すため、キーの有無で判定する。
    prefs.putString(WIFI_NVS_KEY_PASS, pass);
    const bool passOk = prefs.isKey(WIFI_NVS_KEY_PASS);

    prefs.end();
    return ssidOk && passOk;
}

static void wifiCredentialsClear()
{
    Preferences prefs;
    if (!prefs.begin(WIFI_NVS_NAMESPACE, false)) return;
    prefs.clear();
    prefs.end();
}

// 入力中の文字列を表示へ渡す。パスフレーズは画面にも出さず伏せ字に置き換える。
static void wifiShowInput(const char* field, const char* text, bool secret, bool error, WifiSetupMessageFn onMessage)
{
    if (onMessage == nullptr) return;
    if (!secret) {
        onMessage(field, text, error);
        return;
    }

    // 128x128 の画面には長い文字列が収まらないので 8 文字で打ち切る。
    char masked[9];
    const size_t len   = strlen(text);
    const size_t shown = (len < (sizeof(masked) - 1)) ? len : (sizeof(masked) - 1);
    for (size_t i = 0; i < shown; ++i) masked[i] = '*';
    masked[shown] = '\0';
    onMessage(field, masked, error);
}

// 1 行読み取る。長すぎる入力はエラーとして扱い、同じ項目を再要求する。
//
// 空行は入力の区切りとして無視する。改行コードが CRLF のときに 2 行目として
// 空行が届くため、空行に意味を持たせると端末によって挙動が変わってしまう。
// 「変更しない」「パスフレーズなし」はピリオド 1 文字で表す。
static void wifiPromptLine(const char* field, bool secret, char* out, size_t outSize, WifiSetupMessageFn onMessage)
{
    size_t len      = 0;
    bool overflow   = false;
    bool needPrompt = true;

    for (;;) {
        if (needPrompt) {
            // パスフレーズはプロンプト行にも一切出さない。echo=masked は
            // 「入力しても画面・ログに出ない」ことを操作者へ伝えるための表示。
            Serial.printf("WIFI_INPUT,field=%s,max=%u,echo=%s\n", field, static_cast<unsigned>(outSize - 1),
                          secret ? "masked" : "plain");
            needPrompt = false;
            len        = 0;
            overflow   = false;
            out[0]     = '\0';
            wifiShowInput(field, out, secret, false, onMessage);
        }

        const int c = Serial.read();
        if (c < 0) {
            delay(10);
            continue;
        }

        if ((c != '\r') && (c != '\n')) {
            if (len + 1 < outSize) {
                out[len++] = static_cast<char>(c);
                out[len]   = '\0';
                wifiShowInput(field, out, secret, false, onMessage);
            } else {
                // 桁数オーバー。以降の文字は捨て、改行時にエラーとして扱う。
                overflow = true;
            }
            continue;
        }

        if ((len == 0) && !overflow) continue;

        if (!overflow) {
            Serial.printf("WIFI_INPUT,field=%s,result=OK,length=%u\n", field, static_cast<unsigned>(len));
            return;
        }

        Serial.printf("WIFI_INPUT,field=%s,result=ERR,reason=too_long\n", field);
        wifiShowInput(field, "ERR", false, true, onMessage);
        needPrompt = true;
    }
}

static void wifiBeginConnect()
{
    // 資格情報の保存先は自前の NVS (WIFI_NVS_NAMESPACE) に一本化する。WiFi
    // ライブラリ既定の persistent 保存を切り、同じ秘密がもう 1 か所の NVS へ
    // 複製されるのを防ぐ。
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    // モデムスリープは送信のたびに数十ミリ秒の待ちを生むことがあり、測距ループ
    // から見ると突発的な停止として現れる (doc/server-design.md 7.1)。
    WiFi.setSleep(false);
    // 切断後の再接続はライブラリ側のタスクへ任せる。測距ループから
    // WiFi.reconnect() を呼ぶと、その呼び出しの間ずっと測距が止まる。
    WiFi.setAutoReconnect(true);
    WiFi.begin(wifiSsid, wifiPass);
}

// 起動時の Wi-Fi 設定。forceSetup (起動ボタン押下) のときだけシリアル入力を
// 受け付ける。未設定でも入力を待たずに進む - 測距は Wi-Fi なしでも動くので、
// コンソールを繋いでいない機体が起動時に止まるのを避ける。
//
// 接続の完了は待たない。ここでは接続を開始するだけで、成否は loop() から呼ぶ
// wifiMaintain() が報告する。
static bool wifiSetup(bool forceSetup, WifiSetupMessageFn onMessage)
{
    bool stored = wifiCredentialsLoad();

    if (forceSetup) {
        Serial.printf("WIFI_SETUP,stored=%d,keep=.,clear=-\n", stored ? 1 : 0);

        char ssid[WIFI_SSID_MAX + 1] = {0};
        wifiPromptLine("ssid", false, ssid, sizeof(ssid), onMessage);

        if (strcmp(ssid, "-") == 0) {
            wifiCredentialsClear();
            wifiWipe(wifiSsid, sizeof(wifiSsid));
            wifiWipe(wifiPass, sizeof(wifiPass));
            stored = false;
            Serial.println("WIFI_SETUP,result=cleared");
        } else if (strcmp(ssid, ".") == 0) {
            Serial.println("WIFI_SETUP,result=kept");
        } else {
            char pass[WIFI_PASS_MAX + 1] = {0};
            wifiPromptLine("pass", true, pass, sizeof(pass), onMessage);
            // ピリオド 1 文字はパスフレーズなし (開放 AP) を表す。
            if (strcmp(pass, ".") == 0) pass[0] = '\0';

            const bool saved = wifiCredentialsSave(ssid, pass);
            // ログに出すのは SSID までとし、パスフレーズは長さも含めて出さない。
            Serial.printf("WIFI_SETUP,result=%s,ssid=%s\n", saved ? "OK" : "ERR", ssid);
            if (saved) {
                strncpy(wifiSsid, ssid, sizeof(wifiSsid) - 1);
                wifiSsid[sizeof(wifiSsid) - 1] = '\0';
                strncpy(wifiPass, pass, sizeof(wifiPass) - 1);
                wifiPass[sizeof(wifiPass) - 1] = '\0';
                stored                         = true;
            }
            wifiWipe(pass, sizeof(pass));
        }
    }

    if (!stored) {
        // 未設定。測距だけを行い、テレメトリ送信は後段で無効のまま扱う。
        Serial.println("WIFI,state=unconfigured");
        wifiWipe(wifiPass, sizeof(wifiPass));
        return false;
    }

    Serial.printf("WIFI,state=connecting,ssid=%s\n", wifiSsid);
    wifiBeginConnect();
    // WiFi.begin() が内部へ取り込んだので、こちら側の控えは残さない。
    wifiWipe(wifiPass, sizeof(wifiPass));
    wifiConfigured = true;
    return true;
}

// 接続状態を監視する。再接続はライブラリに任せているので、ここでは状態が
// 変わったときにログを出すだけにとどめ、測距ループを止めない。
static bool wifiMaintain()
{
    if (!wifiConfigured) return false;
    if ((millis() - wifiLastCheckMs) < 1000) return wifiConnected;
    wifiLastCheckMs = millis();

    const bool nowConnected = (WiFi.status() == WL_CONNECTED);
    if (nowConnected == wifiConnected) return wifiConnected;

    wifiConnected = nowConnected;
    if (nowConnected) {
        Serial.printf("WIFI,state=connected,ip=%s,rssi=%d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
        Serial.println("WIFI,state=disconnected");
    }
    return wifiConnected;
}

// 直近の監視結果。WiFi.status() を毎回呼ばずに済ませるためのキャッシュ。
static bool wifiIsConnected()
{
    return wifiConnected;
}

// 画面表示用の短い状態文字列。128x128 に収めるため 3 文字以内に抑える。
static const char* wifiShortState()
{
    if (!wifiConfigured) return "OFF";
    return wifiConnected ? "OK" : "--";
}
