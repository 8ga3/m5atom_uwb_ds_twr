# m5atom_uwb_ds_twr

M5Stamp-UWB (QM33120) を搭載した M5Atom シリーズで、DS-TWR (Double-Sided Two-Way Ranging) による UWB 測距を行うファームウェア。
TAG (測距を要求する側) と ANCHOR (応答する側) の 2 つの役割を、共通コードを共有しつつ 1 つのリポジトリにまとめている。

対応ボードは AtomS3 / AtomS3 Lite / Atom Lite。すべて M5Stamp-UWB ブレイクアウトと組み合わせて使う。

## セットアップ方法

### 1. ビルドとファームウェア書き込み

`platformio.ini` に役割とボードの組み合わせごとの環境を定義している。

- `atoms3-anchor` / `atom-anchor`: ANCHOR
- `atoms3-tag` / `atom-tag`: TAG

PlatformIO で対象の環境を選び、ビルド・書き込みを行う。

```sh
pio run -e atoms3-tag -t upload
```

### 2. ID の設定 (TAG / ANCHOR 共通)

起動ボタンを押しながら電源を入れると、シリアルコンソールから ID 設定モードに入る。

- TAG の ID 範囲: `0x0001`〜`0x00FF`
- ANCHOR の ID 範囲: `0x0100`〜`0xFFFE`

設定した ID は NVS (不揮発ストレージ) に保存され、再フラッシュしても保持される。同じファームウェアを全機に書き込んでも、機体ごとに異なる ID を設定できる。

### 3. Wi-Fi の設定 (TAG のみ)

ID 設定と同じく、起動ボタンを押しながら電源を入れたときにシリアルコンソールから SSID・パスフレーズを入力する。

- 資格情報はソースコードにもリポジトリにも含めない。入力値は NVS にのみ保存される。
- 未設定のままでも測距自体は動作する。Wi-Fi はテレメトリ送信のためのものであり、必須ではない。
- SSID にピリオド 1 文字 (`.`) を入力すると変更なしで進む。ハイフン 1 文字 (`-`) を入力すると保存済みの設定を消去する。

## ドキュメント

詳細な設計は `doc/` 以下を参照。

- [doc/multi-anchor-positioning-design.md](doc/multi-anchor-positioning-design.md) - マルチアンカー UWB 測位システムの設計メモ
- [doc/downlink-tdoa-design.md](doc/downlink-tdoa-design.md) - Downlink-TDoA 方式の設計メモ
- [doc/server-design.md](doc/server-design.md) - 測位サーバーの設計メモ

## License

MIT License。詳細は [LICENSE](LICENSE) を参照。
