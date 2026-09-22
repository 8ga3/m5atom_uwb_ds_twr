// ファームウェアバージョン。単一の情報源は platformio.ini の FW_VERSION ビルドフラグ。
// 運用方針は AGENTS.md の「バージョン管理」を参照。
#pragma once

#ifndef FW_VERSION
// ビルドフラグを経由しない場合 (エディタの構文解析など) のフォールバック。
#define FW_VERSION "unknown"
#endif
