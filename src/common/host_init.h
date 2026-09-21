// setup() 冒頭の共通処理 (M5.begin、Serial、起動ボタン読み取り)。
#pragma once

#include <Arduino.h>

// M5.begin() と表示/LED判定 (initStatusHardware()) を済ませる。
void beginHost();

// Serial を初期化し、USB CDC のオープン待ちと起動ログ (ROLE/TWR_MODE/HOST)
// を出す。role は "ANCHOR" または "TAG"。
void beginSerial(const char* role);

// 起動時の本体ボタン状態を返す (押されていたら true)。押下時は ID 設定モード
// への強制入り口として使う。
bool readBootButtonHeld();
