// Stamp-UWB (QM33120) の初期化。ANCHOR と TAG で共通。
#pragma once

#include <M5Stamp_UWB.h>

extern M5Stamp_UWB uwb;
extern M5Stamp_UWBDSRangeConfig rangeConfig;

// 既定の PAN ID。ANCHOR と TAG で一致している必要がある。TAG は測位サーバーから
// 配られた値で上書きすることがある (doc/server-design.md 5.1)。
static constexpr uint16_t UWB_DEFAULT_PAN_ID = 0xDECA;

// rangeConfig.initiatorAddress / responderAddress / panId は呼び出し側の役割に
// 応じて initUwb() を呼ぶ前に設定しておくこと。initUwb() はこれらに触れず、
// DS-TWR のタイミングなど役割に依存しない項目だけを設定する。
bool initUwb(uint32_t spiFastHz);
