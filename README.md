# m5atom_uwb_ds_twr

M5Stamp-UWB (QM33120) を搭載した M5Atom シリーズで、DS-TWR (Double-Sided Two-Way Ranging) による UWB 測距を行うファームウェア。
TAG (測距を要求する側) と ANCHOR (応答する側) の 2 つの役割を、共通コードを共有しつつ 1 つのリポジトリにまとめている。

対応ボードは AtomS3 / AtomS3 Lite / Atom Lite。すべて M5Stamp-UWB ブレイクアウトと組み合わせて使う。

バージョン: `0.1.0-dev`

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
- SSID にピリオド 1 文字 (`.`) を入力すると変更なしで進む。ハイフン 1 文字 (`-`) を入力すると保存済みの設定を消去する。
- Wi-Fi は測位サーバーからアンカー構成を取得するために使う。構成が NVS にキャッシュ済みであれば、Wi-Fi が繋がらない状態でも測距は動作する。

### 4. 測位サーバーの設定 (TAG のみ)

Wi-Fi 設定に続けて、測位サーバー ([location_server_uwb](https://github.com/8ga3/location_server_uwb)) の宛先を入力する。

- `srv_host` には サーバーの IPv4 アドレスを入力する。名前解決は行わないので、ホスト名は受け付けない。
- `srv_port` はピリオド 1 文字 (`.`) で既定値 (`8000`) を選べる。
- `srv_host` にピリオド 1 文字 (`.`) を入力すると変更なしで進む。ハイフン 1 文字 (`-`) を入力すると宛先と構成キャッシュを消去する。

### 5. アンカー構成の取得 (TAG のみ)

TAG は起動時に `GET /api/v1/config` を 1 回呼び、巡回するアンカーの ID・設置座標・PAN ID・バイアス補正値・テレメトリ送信先を受け取る。

- 取得した構成は NVS に保存する。次回以降の起動ではキャッシュの `rev` を `If-None-Match` で送るため、変更が無ければサーバーは `304` を返す。
- サーバーへ届かないときはキャッシュで動作する。キャッシュも無い場合は巡回先が決まらないため測距を開始せず、LED を赤 (画面付きでは `CFG:NONE`) にして 10 秒ごとに取得をやり直す。
- キャッシュにはどのサーバーから取得したかを一緒に記録する。宛先を別のサーバーへ変更した場合、前のサーバーの構成は使わずに取得をやり直す。
- アンカーの座標はサーバー側の CLI (`tools/anchor_cli.py`) または管理 API で登録する。

## ドキュメント

詳細な設計は `doc/` 以下を参照。

- [doc/multi-anchor-positioning-design.md](doc/multi-anchor-positioning-design.md) - マルチアンカー UWB 測位システムの設計メモ
- [doc/downlink-tdoa-design.md](doc/downlink-tdoa-design.md) - Downlink-TDoA 方式の設計メモ
- [doc/server-design.md](doc/server-design.md) - 測位サーバーの設計メモ

`doc/` 以下の設計メモは、サーバー実装側の
[location_server_uwb](https://github.com/8ga3/location_server_uwb) リポジトリと同じ内容を保つ。
片方だけを書き換えない。一致しているかは次のコマンドで確認できる。

```sh
python tools/check_doc_sync.py ../location_server_uwb
```

## License

MIT License。詳細は [LICENSE](LICENSE) を参照。
