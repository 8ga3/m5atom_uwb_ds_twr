# AGENTS.md

## 文章の書き方

**ドキュメント (`doc/*.md`、README など) とソースコード中のコメントは、すべて通常の日本語で書く。**

- 会話側で圧縮した口調 (原始人モードなど) を使っている場合でも、ファイルに書き込む文章には適用しない
- 敬語である必要はないが、助詞を省略した圧縮表現や体言止めの多用は避け、後から読んで意味が一意に取れる文にする
- コミットメッセージと PR 本文も同様に通常の日本語または英語で書く
- 既存ファイルの文体に合わせる。英語で書かれているコメントを日本語へ書き換えることはしない

## リポジトリ構成

ファームウェアと測位サーバーを同じリポジトリで管理する。

- リポジトリ直下の `platformio.ini` と `src/` はファームウェアである。
  VS Code ではルートを開き、pioarduino を使う。統合作業を理由にビルド基盤のバージョンを変更しない
- `server/` は独立した Python プロジェクトである。開発規約は
  [server/AGENTS.md](server/AGENTS.md)、実行手順は [server/README.md](server/README.md) に従う。
  `uv` のコマンドは `server/` で実行する
- DS-TWR が現在の実装対象であり、DL-TDoA は検討段階である。実行時の方式切り替えを前提にしない
- 統合後は旧リポジトリとの subtree の pull/push や文書の同期は行わない

## ファームウェアのバージョン管理

ファームウェアのバージョンは [SemVer](https://semver.org/lang/ja/) に従い `MAJOR.MINOR.PATCH` 形式で管理する。開発中のバージョンには `-dev` サフィックスを付ける (例: `0.1.0-dev`)。

- バージョン番号の単一の情報源は `platformio.ini` の `[env]` セクションにある `FW_VERSION` ビルドフラグ。
  `src/common/version.h` がこの値をファームウェアに取り込み、起動時にシリアルへ `FW_VERSION,version=...` として
  出力する
- README.md 冒頭の記載バージョンは `platformio.ini` の値と一致させる。バージョンを変更するときは同じコミットで
  両方を更新する
- 初期バージョンは `0.1.0-dev`
- リリース (`-dev` サフィックスを外す) するコミットには `firmware-vMAJOR.MINOR.PATCH` の git tag を打つ。
  サーバーは独立してバージョンを管理し、`server-vMAJOR.MINOR.PATCH` を使う
- 最初のリリースまでは `main` ブランチで直接開発する (現状)。最初のリリース以降は `develop` ブランチで開発し、
  バージョンは次にリリース予定のバージョンに `-dev` を付けたものにする (例: `firmware-v0.1.0` リリース後、次が `0.2.0`
  予定なら `0.2.0-dev`)
- リリース手順: `develop` の変更を `main` にマージ → バージョンから `-dev` を外す → `firmware-vMAJOR.MINOR.PATCH` の tag
  を打つ → `develop` 側のバージョンを次の開発バージョン (`{次のバージョン}-dev`) に上げる

## 設計文書

`doc/` 以下の設計メモと `doc/images/` を両者で共有し、`server/` 以下には複製しない。
サーバーの設計の正本は `doc/server-design.md` である。

- 設計と実装を変更するときは、同じコミットで整合させる
- 文書からファームウェアへのリンクは `../src/...`、サーバーへのリンクは `../server/...` で書く
- 既存の設計メモ中のサーバーの `tools/...` は、サーバープロジェクト (`server/`) を起点とする
- Markdown の lint はリポジトリのルートで `npx markdownlint-cli2` を実行する

## Markdown

`doc/*.md`、ルートとサーバーの README および AGENTS.md は markdownlint を通す。
規則は `.markdownlint.jsonc`、対象ファイルは `.markdownlint-cli2.jsonc` に定義する。

- 見出しの代わりに太字 (`**...**`) だけの行を置かない (MD036)。見出しにするなら `####` などを使う
- 1 行は 120 文字まで。コードブロックとテーブルは対象外
