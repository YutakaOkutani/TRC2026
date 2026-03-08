# CanSat Mission Architecture Summary

このファイルは `csmn/` の設計責務だけを把握するための資料です。

- セットアップ手順・運用手順: `README.md`
- カメラ中継試験の実行手順: `runs/cam/cam_relay_readme.md`

## 全体像

- `main.py`
  - 本番エントリポイント。
  - `run_full_mission()` を呼ぶだけの最小司令塔。
- `csmn/`
  - ミッション実装本体（定数・状態・司令塔・フェーズ・I/O管理）。
- `runs/orch/orch_*.py`
  - フェーズ限定デバッグ用の入口。

## ディレクトリごとの責務

- `csmn/const.py`
  - 定数の一元管理（フェーズ番号、閾値、GPIO、タイムアウトなど）。
- `csmn/st.py`
  - `CanSatState`（スレッドセーフな共有状態）の管理。
- `csmn/nav.py`
  - 純関数ユーティリティ（距離・方位など）。
- `csmn/gps_util.py`
  - GPS入出力・NMEA処理ユーティリティ。
- `csmn/ctrl.py`
  - `CanSatController` 本体。
  - 初期化、フェーズディスパッチ、デバッグ用の実行範囲制限を担当。
- `csmn/run.py`
  - 実行モード入口（`run_full_mission()`, `run_phase_sequence()`, `run_single_phase()`）。

- `csmn/mgr/hw_mgr.py`
  - ハード初期化、スレッド起動、ログヘッダ作成。
- `csmn/mgr/sns_mgr.py`
  - BNO/BMP/SONAR/GPS/カメラ取得、再初期化、ログ追記。
- `csmn/mgr/mtr_mgr.py`
  - モータ制御、障害物回避、フェーズ別駆動ロジック。
- `csmn/mgr/led_mgr.py`
  - LED点滅・シグナル処理。

- `csmn/phs/base.py`
  - フェーズ共通インターフェース。
- `csmn/phs/p0.py` 〜 `p7.py`
  - フェーズごとの状態遷移ロジック。

## フェーズ遷移

- 通常遷移: `0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7`
- 一部フェーズはフォールバックあり（例: `4 -> 3` 戻り）。

## 変更ルール

- マジックナンバー禁止。値変更は `csmn/const.py` のみで実施。
- フェーズ仕様の変更は対象 `csmn/phs/pX.py` を優先修正。
- ハード不具合調査は `csmn/mgr/sns_mgr.py` と `csmn/mgr/hw_mgr.py` を優先確認。
