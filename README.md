# TRC2026

CanSat（Raspberry Pi Zero 2 W）向けのセットアップ手順と実行方法をまとめています。内容は Raspberry Pi OS bookworm を前提にしています。

## 1. 開発環境の準備

### ハードウェア
- Raspberry Pi Zero 2 W
- microSDカード 16GB 以上（Class10/UHS-I 推奨）
- 安定した 5V / 2.5A 以上の電源（USB 給電でも可。電流不足だとカメラ/GPIOが不安定）
- PC（Windows / macOS / Linux）
  - USB ハブ、Mini HDMI ケーブル、モニター・キーボードは初期設定の保険としてあると安心

### ソフトウェア（PC側）
- Raspberry Pi Imager
- SSH クライアント（Windows は標準の PowerShell で可）
- Git, VS Code（任意）

---

## 2. Raspberry Pi OS の書き込み
1. Raspberry Pi Imager をインストールし起動する（https://www.raspberrypi.com/software/）。
2. 以下を選択  
   - Device: **Raspberry Pi Zero 2 W**  
   - OS: **Raspberry Pi OS Lite (64-bit)**（GUI 不要で軽量）  
   - Storage: 対象の microSD カード
3. 「歯車」アイコンで事前設定を有効化  
   - Hostname  
   - Username / Password  
   - Locale（国・キーボード・タイムゾーン）  
   - Wi-Fi（SSID / パスワード / 国）  
   - SSH: 有効化（Password 認証で十分）  
   - Raspberry Pi Connect: オフで問題なし
4. Write を実行し、完了したら microSD を取り出す。

---

## 3. 初回起動と接続
1. microSD を Pi に挿入し、電源を入れる。
2. 同一ネットワーク上の PC から SSH 接続する。  
   ```bash
   ssh <ユーザー名>@<ホスト名>.local
   ```
3. 接続できない場合の確認ポイント  
   - 同一ネットワークにいるか確認。  
   - `.local` が解決できないときは Pi の IP アドレスを調べ、`ssh <ユーザー名>@<IPアドレス>` で接続（Windows/Android は mDNS が不安定な場合あり）。  
   - パスワード認証が弾かれるときの一時的な回避例：  
     ```bash
     ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no <ユーザー名>@<IPアドレス>
     ```

---

## 4. OS アップデート
```bash
sudo apt update
sudo apt full-upgrade -y
sudo reboot
```

---

## 5. インターフェースの有効化
```bash
sudo raspi-config
```
- Interface Options → **I2C**: Enable  
- Interface Options → **Serial Port**: Login shell = No / Serial hardware = Yes（GPS 用）  
- Interface Options → **SPI**: センサ等で必要なら Enable  
- Performance → **GPU Memory**: 128MB 程度（Picamera2 利用時の余裕確保）

変更後に再起動:
```bash
sudo reboot
```

---

## 6. Python 実行環境
### 6.1 必要パッケージ（apt）
```bash
sudo apt update
sudo apt install -y \
  python3-venv python3-pip python3-opencv python3-picamera2 \
  python3-gpiozero python3-lgpio python3-numpy python3-serial \
  python3-smbus i2c-tools git screen
```

### 6.2 仮想環境（任意）
Picamera2 や OpenCV を apt から使うため `--system-site-packages` を付けるのが簡単です。
```bash
python3 -m venv --system-site-packages ~/cansat-venv
source ~/cansat-venv/bin/activate
pip install --upgrade pip
```

---

## 7. リポジトリの取得
```bash
git clone https://github.com/YutakaOkutani/TRC2026
cd TRC2026
```

---

## 8. プロジェクト構成
```
.
├── README.md
├── main.py
├── library/
│   ├── __init__.py
│   ├── bmp180.py
│   ├── bno055.py
│   ├── capture_roi_img.py
│   ├── detect_corn.py
│   └── micropyGPS.py
└── test/
    ├── LED.py
    └── motor_test.py
```

主なファイル:
- `main.py`: 本体の制御コード。BNO055/BMP180、GPS、超音波センサー、カメラ（Picamera2 + OpenCV）を使ったフェーズ制御。
- `library/`: センサー・画像処理の自前ライブラリ。
- `test/`: モーターや LED の簡易動作確認スクリプト。

---

## 9. 実行方法
1. （仮想環境を作成した場合）有効化  
   ```bash
   source ~/cansat-venv/bin/activate
   ```
2. GPIO/LGPIO とカメラを使うため root 実行を推奨（環境変数を保持するには `-E` を付与）  
   ```bash
   sudo -E python3 main.py
   ```
3. ログは `./log/robust_log_*.csv` として自動作成されます（存在しない場合は作成）。

---

## 10. 動作確認コマンド
- I2C デバイス確認（BNO055: 0x28/0x29, BMP180: 0x77）  
  ```bash
  sudo i2cdetect -y 1
  ```
- GPS の NMEA 確認（`main.py` は 115200bps を使用）  
  ```bash
  sudo screen /dev/serial0 115200
  ```  
  退出: `Ctrl-A` → `K` → `Y`
- カメラ確認（bookworm の rpicam 系コマンド）  
  ```bash
  rpicam-hello -t 0
  ```

---

## 11. トラブルシューティング
- SSH 接続できない  
  - Imager で SSH 有効化を忘れていないか再確認。  
  - Wi-Fi 設定（SSID/パスワード/国コード）を見直す。  
  - `.local` が解決しないときは IP 直打ちで接続。  
- センサーが見つからない  
  - `sudo i2cdetect -y 1` でアドレスが見えるか確認。  
  - 配線と電源を再確認（電流不足で I2C が不安定になることあり）。  
- カメラが動かない  
  - `rpicam-hello` が動くか確認。  
  - raspi-config の「Camera」を有効化（bookworm ではデフォルト有効）。  
  - GPU メモリが不足していないか確認。

---

## 12. Raspberry Pi Zero 2 W の固定 IP 設定（任意）
1. インターフェース名を確認  
   ```bash
   ip a
   ```  
   例: Wi-Fi は `wlan0`
2. `dhcpcd.conf` を編集  
   ```bash
   sudo nano /etc/dhcpcd.conf
   ```
3. 末尾に追記（値は利用中のネットワークに合わせて変更）  
   ```bash
   interface wlan0
   static ip_address=192.168.1.50/24
   static routers=192.168.1.1
   static domain_name_servers=192.168.1.1 8.8.8.8
   ```
4. 再起動して反映  
   ```bash
   sudo reboot
   ip a
   ```

---

## 13. VS Code でのリモート操作（任意）
1. VS Code で拡張機能「Remote - SSH」をインストール。
2. 左下の「><」アイコンから **Remote-SSH: Connect to Host…** を選択し、以下を入力  
   ```
   ssh <ユーザー名@<Raspberry Pi の IP>
   ```
3. ステータスバーが「SSH: <ホスト名>」となれば接続完了。以後のターミナルは Pi 上で動作します。
