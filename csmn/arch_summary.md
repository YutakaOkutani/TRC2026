# CanSat Mission Architecture Summary

このファイルは、`csmn` の役割分担を短時間で把握するための引継ぎメモです。

## 全体像

- `main.py`: 本番用の最小司令塔エントリ。`run_full_mission()` を呼ぶだけ。
- `csmn/`: 実ロジック本体（定数・状態・司令塔・フェーズ・I/O管理）。
- `runs/orch/orch_*.py`: フェーズ限定デバッグ用の司令塔。

## ディレクトリごとの役割

- `csmn/const.py`
  - すべての定数を一元管理。
  - フェーズ番号、タイムアウト、閾値、GPIOピン、速度、ログ設定などを定義。

- `csmn/st.py`
  - `CanSatState`（スレッドセーフな共有状態）を管理。
  - IMU/GPS/BMP/カメラ結果、ナビ値、現在フェーズなどを保持。

- `csmn/nav.py`
  - 純関数系ユーティリティ。
  - 距離・方位計算、ミリ秒時刻生成。

- `csmn/ctrl.py`
  - 実行の司令塔 `CanSatController`。
  - 初期化、フェーズディスパッチ、許可フェーズ制限実行（デバッグ用）を担当。

- `csmn/run.py`
  - 実行モードの入口を提供。
  - `run_full_mission()`, `run_phase_sequence()`, `run_single_phase()` を公開。

- `csmn/mgr/`
  - `hw_mgr.py`: ハード初期化、スレッド起動、ログヘッダ作成。
  - `sns_mgr.py`: BNO/BMP/SONAR/GPS/カメラの取得・再初期化・ログ追記。
  - `mtr_mgr.py`: モータ制御と障害物回避、フェーズ別駆動ロジック。
  - `led_mgr.py`: LED点滅・シグナル処理。

- `csmn/phs/`
  - `p0.py`〜`p6.py`: フェーズごとの状態遷移ロジックを分離。
  - 各ファイルは単体実行エントリ（`run_standalone()`）を保持。
  - `base.py`: フェーズ共通インターフェース。

## フェーズ遷移の基本

- 通常系: `0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6`
- タイムアウト/検知失敗時はフェーズ内フォールバックあり（例: 4/5で3へ戻る）。

## デバッグ時の主な入口

- 本番相当フル実行: `main.py`
- フェーズ2→3限定: `runs/orch/orch_p2_p3.py`
- フェーズ4→7限定: `runs/orch/orch_p4_p7.py`

## 重要な運用ルール

- マジックナンバーは禁止。値追加/変更は `const.py` のみで行う。
- フェーズロジック変更は、対象 `pX.py` を優先して修正する。
- ハード依存の不具合調査は `mgr/sns_mgr.py` と `mgr/hw_mgr.py` を先に確認する。
