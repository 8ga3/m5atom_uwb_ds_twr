# m5atom_uwb_ds_twr

M5Stamp-UWB (QM33120) を搭載した M5Atom シリーズで、DS-TWR (Double-Sided Two-Way Ranging) による UWB 測距を行うファームウェア。
TAG (測距を要求する側) と ANCHOR (応答する側) の 2 つの役割を、共通コードを共有しつつ 1 つのリポジトリにまとめている。

対応ボードは AtomS3 / AtomS3R / AtomS3 Lite / Atom Lite / Atom Matrix。すべて M5Stamp-UWB ブレイクアウトと組み合わせて使う。

バージョン: `0.1.0-dev`

## セットアップ方法

### 1. ビルドとファームウェア書き込み

`platformio.ini` に役割とボードの組み合わせごとの環境を定義している。

- `atoms3-anchor` / `atom-anchor` / `atom-matrix-anchor`: ANCHOR
- `atoms3-tag` / `atom-tag` / `atom-matrix-tag`: TAG

`atoms3-*` は AtomS3 / AtomS3R / AtomS3 Lite の共用、`atom-*` は Atom Lite 用、`atom-matrix-*` は Atom Matrix 用。
Atom Lite と Atom Matrix は実行時に見分けられないため、ビルドを分けている。

PlatformIO で対象の環境を選び、ビルド・書き込みを行う。

```sh
pio run -e atoms3-tag -t upload
```

シリアルは 1500000bps で出力する (`platformio.ini` の `monitor_speed` と同じ)。`pio device monitor` はこの値を使う。

### 2. ID の設定 (TAG / ANCHOR 共通)

起動ボタンを押しながら電源を入れると、シリアルコンソールから ID 設定モードに入る。

- TAG の ID 範囲: `0x0001`〜`0x00FF`
- ANCHOR の ID 範囲: `0x0100`〜`0xFFFE`

設定した ID は NVS (不揮発ストレージ) に保存され、再フラッシュしても保持される。同じファームウェアを全機に書き込んでも、機体ごとに異なる ID を設定できる。

### 3. Wi-Fi の設定 (TAG のみ)

ID 設定と同じく、起動ボタンを押しながら電源を入れたときにシリアルコンソールから SSID・パスフレーズを入力する。

- 資格情報はソースコードにもリポジトリにも含めない。入力値は NVS にのみ保存される。
- SSID にピリオド 1 文字 (`.`) を入力すると変更なしで進む。ハイフン 1 文字 (`-`) を入力すると保存済みの設定を消去する。
- Wi-Fi は測位サーバーからのアンカー構成の取得と、走行中のテレメトリ送信に使う。構成が NVS にキャッシュ済みであれば、Wi-Fi が繋がらない状態でも測距は動作する。
- 走行中も Wi-Fi は切らない。接続の監視と再接続は Core 0 のタスクが行い、Core 1 の測距ループは止めない。

### 4. 測位サーバーの設定 (TAG のみ)

Wi-Fi 設定に続けて、測位サーバー ([location_server_uwb](https://github.com/8ga3/location_server_uwb)) の宛先を入力する。

- `srv_host` にはサーバーの IPv4 アドレスを入力する。名前解決は行わないので、ホスト名は受け付けない。
- `srv_port` はピリオド 1 文字 (`.`) で既定値 (`8000`) を選べる。
- `srv_host` にピリオド 1 文字 (`.`) を入力すると変更なしで進む。ハイフン 1 文字 (`-`) を入力すると宛先と構成キャッシュを消去する。

### 5. アンカー構成の取得 (TAG のみ)

TAG は起動時に `GET /api/v1/config` を 1 回呼び、巡回するアンカーの ID・設置座標・PAN ID・バイアス補正値・テレメトリ送信先を受け取る。

- 取得した構成は NVS に保存する。次回以降の起動ではキャッシュの `rev` を `If-None-Match` で送るため、変更が無ければサーバーは `304` を返す。
- サーバーへ届かないときはキャッシュで動作する。キャッシュも無い場合は巡回先が決まらないため測距を開始せず、LED を赤 (画面付きでは `CFG:NONE`) にして 10 秒ごとに取得をやり直す。
- キャッシュにはどのサーバーから取得したかを一緒に記録する。宛先を別のサーバーへ変更した場合、前のサーバーの構成は使わずに取得をやり直す。
- アンカーの座標はサーバー側の CLI (`tools/anchor_cli.py`) または管理 API で登録する。
- 構成を取得したあとに `POST /api/v1/hello` を 1 回呼び、起動ごとの乱数 `boot_id` とファームウェアバージョンをサーバーへ通知する。失敗しても測距は続ける。

### 6. テレメトリの送信 (TAG のみ)

構成の `telemetry` に送信先が入っていれば、1 周期ごとの測距結果をリングバッファへ積み、
`batch_cycles` 周期ぶんたまったら 1 パケットにまとめて UDP で送る
(形式は [doc/server-design.md](doc/server-design.md) の 6.2)。

- 送信は巡回の先頭スロットの直前だけで行う。再送はせず、Wi-Fi が切れている間は送らずに古い周期から捨てる。
- 送信先の `port` が `0` のとき、または送信先が無いときは送信しない。
- 送信の統計は 10 秒ごとに `TELEMETRY_STAT,...` としてシリアルへ出る。`max_send_us` は `sendto()` にかかった時間の最大値。
- 周期の統計は Wi-Fi の有無に関係なく 10 秒ごとに `CYCLE_STAT,...` としてシリアルへ出る。周期の最小・最大・平均と、予定より遅れて始まったスロットの数 (`slot_late`) で、測距ループが止められていないかを確認できる。
- パケットの測位欄には、下の「8. 2D 測位」で求めた最小二乗の解と、「9. カルマンフィルタ」で求めた
  フィルタ後の位置を別々に入れる。解けなかった (無効な) ほうは「測位なし」で送る。パケット形式は
  フィルタ後の位置を足したときに version 2 へ上げたので、サーバーも同時に更新する。
- 受信側の確認にはサーバー側の `tools/dump_udp.py listen` が使える。

### 7. 測距の周期 (TAG のみ)

TAG は既定で 100ms (10Hz) ごとに全アンカーを 1 回ずつ測距する。1 周期をアンカーの台数で等分したスロットに
1 台ずつ割り当てる。周期はビルドフラグで変えられる。

```sh
PLATFORMIO_BUILD_FLAGS="-D UWB_RANGE_CYCLE_MS=200" pio run -e atoms3-tag -t upload
```

アンカー 4 台で 30Hz (`UWB_RANGE_CYCLE_MS=33`) でも測距を続けられることを確かめた。失敗は 4 台 × 1740 回で 0 回、
4 台 × 1980 回で 1 回 (`RX_TIMEOUT`) だった。スロットはミリ秒単位に切り捨てるので、周期は 32ms になる
([doc/multi-anchor-positioning-design.md](doc/multi-anchor-positioning-design.md) の 5 章)。

TAG と ANCHOR の DS-TWR のタイミングは組で決めてある。ANCHOR だけを新しいファームウェアにすると、古い TAG は
Response を受信窓の外で受けることになって測距できない。更新するときは TAG を先に書き換える。

### 8. 2D 測位 (TAG のみ)

TAG は 1 周期ごとに、応答したアンカーの測距とサーバーから受け取った設置座標から、2D 三辺測量 (最小二乗) で
自己位置を求める。解に渡せる測距 (応答したアンカーのうち、有り得ない負の値を除いたもの) が 3 本未満の周期は解かない
([doc/multi-anchor-positioning-design.md](doc/multi-anchor-positioning-design.md) の 3.6)。

- 高さは解かず、TAG のアンテナの高さを固定値として与える。既定は 0 で、アンカーと高さが違うときは
  サーバーの座標表と同じ座標系で測った値をビルドフラグで与える。

  ```sh
  PLATFORMIO_BUILD_FLAGS="-D UWB_TAG_Z_MM=150" pio run -e atoms3-tag -t upload
  ```

- 結果は 2 秒ごとに `POS,...` としてシリアルへ出る。配置の判定 (`GEOMETRY`) か NaN (`NON_FINITE`) で
  解けなかった周期は `POS_FAIL,...`、有り得ない負の値のため解から外した測距は `POS_REJECT,...` としてその場で出す。
  どちらも種類ごとに 1 秒に 1 行までで、出さなかった分は `POS` 行の件数で分かる。測距の不足 (`TOO_FEW`) は
  件数だけを数える。
- AtomS3 / AtomS3R では LCD の下端 3 行に x, y, z の座標 (メートル) を出す。x と y は「9. カルマンフィルタ」の
  フィルタ後の位置で、z は固定値なので白で出す。Wi-Fi の状態は 1 行目の右端のアイコンの色で表す (緑 = 接続中、
  黄 = 未接続、灰色に赤の斜線 = 未設定)。
- 構成は起動時にしか取得しないので、サーバーの座標を変えたら TAG を再起動する。

### 9. カルマンフィルタ (TAG のみ)

TAG は最小二乗とは別に、測距 1 本ずつを観測にする拡張カルマンフィルタ (等速度モデル) で位置を推定する
([doc/multi-anchor-positioning-design.md](doc/multi-anchor-positioning-design.md) の 3.7)。

- 初期値は最小二乗の解から作る。その後は最小二乗が解けない周期 (`TOO_FEW` など) でも、残った測距で更新するか、
  測距が無ければ予測だけでつなぐ。
- 予測からのずれが大きすぎる測距は取り込まずに棄却する。
- 最後に測距を取り込んでから 1 秒を超えるか、位置の標準偏差が 0.5 m を超えたら、フィルタ後の位置を無効にし、
  次に最小二乗が解けて残差 RMS が 150 mm 以下だった周期で初期化し直す。残差がそれより大きい間は、外れた測距を
  含む疑いがあるので初期値に使わず、無効のままにする (LCD には `RESID` と出る)。
- 結果は 2 秒ごとに `POS_KF,...` としてシリアルへ出る。`state` は直近の周期の状態 (`UPDATE` = 観測で更新、
  `PREDICT` = 予測だけ、`NONE` = 無効) で、続けて観測で更新した周期・予測だけの周期・無効だった周期・初期化・
  棄却した測距の件数を出す。無効にした周期は `POS_KF_RESET,...` としてその場で出す (1 秒に 1 行まで)。
- パラメータは起動時に `POS_KF_CONFIG,...` として出る。値は `src/common/position_filter.h` の定数で決まる。
  プロセスノイズ (加速度のパワースペクトル密度、既定 1.0 m²/s³) だけはビルドフラグで変えられる。小さくすると
  静止時は滑らかになるが、走行中の加減速や旋回への追従が遅れる。ほかのビルドフラグ (`UWB_TAG_Z_MM` など) と
  一緒に指定する。

  ```sh
  PLATFORMIO_BUILD_FLAGS="-D UWB_TAG_Z_MM=20 -D UWB_KF_ACCEL_PSD=0.1" pio run -e atoms3-tag -t upload
  ```

- LCD の座標は、観測で更新した周期は緑、予測だけでつないだ周期は黄で出す。無効な周期は `POS:NG` と、
  最小二乗が解けなかった理由 (残差が大きくて初期化に使わなかったときは `RESID`) を出す。

## 状態表示

AtomS3 / AtomS3R は LCD に状態を表示する。画面のない AtomS3 Lite / Atom Lite は RGB LED 1 個の色で状態を表す。

- 緑: 測距できている
- 黄: 無通信で待機中
- 赤: UWB トランシーバーが使えない、または測距を開始できない (TAG でサーバー構成が無い場合を含む)
- マゼンタ: シリアルコンソールでの設定入力待ち

Atom Matrix は同じ色で、5x5 の LED アレイに自機の ID (TAG ID またはアンカー ID) を 10 進で表示する。

- ID が 2 桁以上のときは 1 桁ずつ約 1 秒ごとに切り替える。桁の間は一度消灯し、最後の桁のあとは長めに消灯してから先頭の桁に戻る
- 設定入力待ちの間は ID を表示せず、25 個すべてをマゼンタで点灯する
- 表示の向きはビルドフラグ `-D LED_MATRIX_ROTATION=<0-3>` で 90 度単位に回せる

横スクロールにしないのは、LED を書き換えている間は測距ループが止まるため。25 個ぶんの送信だけで
約 0.75 ms (25 個 × 24 bit × 1.25 µs) かかる。スクロールは 1 秒に 10 回ほどの書き換えが要るが、
切り替え表示なら 1 秒に 2 回程度で済む。

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
