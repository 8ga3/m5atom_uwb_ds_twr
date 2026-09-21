// Stamp-UWB (QM33120) の初期化。ANCHOR と TAG で共通。
#pragma once

#include <M5Stamp_UWB.h>

extern M5Stamp_UWB uwb;
extern M5Stamp_UWBDSRangeConfig rangeConfig;

// rangeConfig.initiatorAddress / responderAddress は呼び出し側の役割に応じて
// initUwb() を呼ぶ前に設定しておくこと (panId 等の共通項目は initUwb() 側で
// 設定する)。
bool initUwb(uint32_t spiFastHz);
