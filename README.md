# TRC2026

## ファイル構成

```plaintext
├── main.py
├── cansat_mission/
│   ├── managers/
│       ├── __init__.py
│       ├── hardware_manager.py
│       ├── led_manager.py
│       ├── motor_manager.py
│       ├── sensor_manager.py
│   ├── phases/
│       ├── __init__.py
│       ├── base.py
│       ├── phase0.py
│       ├── phase1.py
│       ├── phase2.py
│       ├── phase3.py
│       ├── phase4.py
│       ├── phase5.py
│       ├── phase6.py
│   ├── __init__.py
│   ├── ARCHITECTURE_SUMMARY.md
│   ├── constants.py
│   ├── controller.py
│   ├── navigation.py
│   ├── runners.py
│   ├── state.py
├── library/
│   ├── __init__.py
│   ├── bno055.py
│   ├── bmp180.py
│   ├── capture_roi_img.py
│   ├── detect_corn.py
├── tests/
│   ├── camera_phase_monitor_pc.py
│   ├── camera_phase_relay_README.md
│   ├── camera_phase_relay_sbc.py
│   ├── gps_test.py
│   ├── landing_impact.py
│   ├── led.py
│   ├── motor_test.py
│   ├── open_parachute.py
│   ├── orchestrator_phase1_to_phase6.py
│   ├── orchestrator_phase2_to_phase3.py
│   ├── orchestrator_phase4_to_phase6.py
└── README.md
```

* 本番用コード：main.py
* センサー用ライブラリ：bno055.py, bmp180.py
* カメラフェーズ用：detect_corn.py
* ゴール画像撮影用：capture_roi_image.py
* テストコード：gps_test.py, landing_impact.py, led.py, motor_test.py, open_parachute.py（landing_impact.py と open_parachute.py の中身は同じ。出力されるログファイルを区別するために分けてある）
* カメラフェーズのリレーコード：camera_phase_relay_sbc.py, camera_phase_monitor_pc.py
* カメラフェーズのリレーコードの説明：camera_phase_relay_README.md

## 1. 環境構築の準備

### ハードウェア

* Raspberry Pi Zero 2 W
* microSDカード（16GB以上あれば良い）
* 電源（5V / 2.5A）（PCからのUSB給電でもよいが、不安定になるときがある）
* PC
* USBハブ
* Mini HDMI ケーブル（モニター接続用）
* モニター・キーボード（あると便利）

### ソフトウェア

* Raspberry Pi Imager
* Git
* ターミナル

---

## 2. Raspberry Pi Zero 2 W のセットアップ

### 2.1 OS イメージの準備

1. Raspberry Pi Imager をインストールする（ <https://www.raspberrypi.com/software/> ）
2. 起動後、

* Device → *Raspberry PI Zero 2 W*
* OS → *Raspberry Pi OS(other)*から*Raspberry Pi OS Lite（64-bit）* （GUIは要らない）
* Storage → MicroSDカード

1. 以下を設定・有効化する

   * Hostname
   * Localisation（国・キーボード設定）
   * User
   * Wi-Fi
   * Remote access（SSHの有効化、パスワード認証で十分）
   * Raspberry Pi Connect（オフでよい）

2. Writing（書き込み）を実行

---

### 2.2 初回起動と接続

1. microSD を ラズパイ に挿入して電源投入
2. PC から次のコマンドで接続

    ```bash
    ssh ユーザー名@ホスト名.local（or IPアドレス）
    ```

3. パスワードは Imager で設定したものを使用

### 2.3 接続できない場合は以下を確認

1. 同一ネットワークにいるか
2. `.local` 解決ができない環境では、ラズパイの IPアドレス を確認して、ホスト名のところを IP に置き換えて接続する（WindowsやAndroid端末では、mDNS（.local）が安定的にサポートされておらず、ホスト名接続は一般に不安定）
3. Permission denied (publickey,password).が出る場合、以下のコマンドで接続

    ```bash
    ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no ユーザー名@IPアドレス
    ```

4. その後、以下のコマンドでラズパイ上の設定を確認

    ```bash
    sudo nano /etc/ssh/sshd_config
    ```

5. その中に、以下の行があれば確認。

    ```plaintext
    PasswordAuthentication yes
    ```

6. PasswordAuthentication が no になっていたら yes に変更。

7. "#" でコメントアウトされている場合は、"#" を外して PasswordAuthentication yes にする

8. 設定を変更したら、SSH サーバーを再起動。

    ```bash
    sudo systemctl restart ssh
    ```

---

### 2.3 初期アップデート

```bash
sudo apt update
sudo apt full-upgrade -y
sudo reboot
```

---

## 3. Python 実行環境の構築

### 3.1 依存パッケージのインストール

```bash
# パッケージリストの更新 + システムアップグレード
sudo apt update && sudo apt full-upgrade -y

# ===== 基本ツール =====
sudo apt install -y git screen tmux i2c-tools

# ===== 実行に必要なPythonライブラリ（apt）=====
# GPIO / シリアル / I2C
sudo apt install -y python3-gpiozero python3-rpi-lgpio liblgpio1 python3-serial python3-smbus

# pipでライブラリを追加インストールするためのツール
sudo apt install -y python3-pip python3-setuptools

# カメラ / 画像処理（Picamera2 + OpenCV + NumPy）
sudo apt install -y python3-picamera2 python3-libcamera libcamera-apps python3-opencv python3-numpy

# OpenCVでGUI表示（imshow等）する場合に必要。ヘッドレス運用なら不要なことが多い
sudo apt install -y libgl1

# ===== ここから下は自前ビルドをするなら必要=====

# C拡張やライブラリをソースからビルドする場合に必要
# sudo apt install -y build-essential python3-dev swig

# lgpio を C で開発/コンパイルする場合のヘッダ（Pythonで動かすだけなら通常不要）
# sudo apt install -y liblgpio-dev

```

### 3.2 Python 仮想環境の作成

```bash
# 仮想環境の作成（システムパッケージを引き継ぐ --system-site-packages が重要）
# これにより、aptで入れた OpenCV や Picamera2 などがそのまま仮想環境内でも使用可能になる
python3 -m venv --system-site-packages venv

# 仮想環境の有効化
source venv/bin/activate

# pip自体の更新
pip install --upgrade pip

# GPS解析用ライブラリ
pip install pynmea2

# その他、main.py等で必要なライブラリがあればここで追加
# pip install smbus2
```

---

## 4. シリアル通信 / GPIO / I2C の有効化

### 4.1 raspi-config での設定

```bash
sudo raspi-config
```

### 4.2 以下を有効化

* Interface Options → I2C

* Interface Options → Serial

完了後、再起動。

```bash
sudo reboot
```

---

## 5. リポジトリのクローン

```bash
git clone https://github.com/YutakaOkutani/TRC2026
cd TRC2026
```

---

## 6. カメラの設定

### カメラの初期設定

```bash
# バージョン確認
lsb_release -a 
```

```bash
# 設定ファイル編集
sudo nano /boot/firmware/config.txt
# 以下の内容を追加または修正
dtoverlay=OV5647 # OV5647 カメラを明示指定
camera_auto_detect=0 # カメラ自動検出を無効化
```

```bash
# 編集後、再起動
sudo reboot
```

### カメラ認識の確認

```bash
# 認識デバイスの一覧
libcamera-hello --list-cameras
# dmesg による初期化ログ確認
dmesg | grep -i ov5647
# 初期化時間の確認
time rpicam-hello -t 1
```

### カメラのプロパティ確認

```bash
rpicam-hello *--info-text*
```

### カメラ映像のテスト

```bash
# ライブプレビュー
rpicam-hello -t 0 # モニターに繋げば映像が見えるはず
# 静止画撮影
rpicam-still -o test.jpg
# 動画撮影
rpicam-vid -t 10000 -o test.h264
# 高解像度での静止画撮影テスト
rpicam-still --width 2592 --height 1944 -o maxres.jpg
```

## 7. 実行（テスト時）

### 本番用コード

```bash
tmux 
source venv/bin/activate
python3 main.py
tmux attach
```

### GPSからの生データ取得（テストコードのほうが見やすいが一応）

```bash
sudo screen /dev/ttyAMA0 115200 # ボーレートが違う場合、9600や38400も試す
```

---

## 8. トラブルシューティング

### センサが認識されない

* `sudo i2cdetect -y 1` で確認
* 配線の導通を確認

---

## 9. その他

### SSH接続が切れてもプログラムが実行され続けるように設定する手順

#### 1. サービスファイルの作成

```bash
sudo nano /etc/systemd/system/cansat.service
```

#### 2.設定内容の書き込み

```
[Unit]
Description=CanSat Main Mission Script
After=multi-user.target

[Service]
# プログラムがあるディレクトリを指定
# パスは適応書き換え 
WorkingDirectory=/home/pi/cansat
# 実行コマンド（pythonのフルパスとスクリプトのフルパスを書く）
ExecStart=/usr/bin/python3 /home/pi/TRC2026/main.py
# venvを使う場合はこちらに置き換え
# ExecStart=/home/pi/cansat/venv/bin/python /home/pi/cansat/main.py
# 落ちても5秒後に自動再起動する設定（ミッション継続に重要）
Restart=always
RestartSec=5
# ログを出力する場合
StandardOutput=inherit
StandardError=inherit
User=pi

[Install]
WantedBy=multi-user.target
```

#### 3. サービスの有効化と開始

```bash
# 設定の反映
sudo systemctl daemon-reload

# ラズパイ起動時に自動実行されるように設定
sudo systemctl enable cansat.service

# 今すぐプログラムを開始する場合
sudo systemctl start cansat.service
```

#### 4. 状態の確認

```bash
sudo systemctl status cansat.service
```

```bash
# cansat.service のログを最新のものから表示する
journalctl -u cansat.service -e
```

---

### 基本的なgit操作コマンド

```bash
# ファイルをステージングに追加
git add .
# コミットを作成
git commit -m "Initial commit"
# GitHub へ初回プッシュ
git push -u origin main
# 2回目以降
git push
```

``` bash
# ローカルを GitHub の最新版で完全に上書きするコマンド
git fetch origin
git reset --hard origin/main
```

```bash
# ローカルの変更を残しつつ、GitHub の更新を取り込むコマンド（pull.ver）
git pull origin main
```

```bash
# ローカルの変更を残しつつ、GitHub の更新を取り込むコマンド（rebase.ver）
git pull --rebase origin main
```

---

### VPNサービスを使って、ラズパイのIPアドレスを固定化する方法（Tailscaleを使う方法）

#### 0. そもそも

前述のとおり、WindowsPCやAndroid端末は、mDNSが不安定なので、ラズパイとのSSH接続にはIPアドレスが必要

##### 仮想VPNサービス（ここではTailscale）を使えば

Tailscaleに登録された各デバイスは：

* 固定の仮想IPアドレスを持つ（100.x.y.z 形式）

* デバイスがオンラインの間、そのIPは常に同じ

* 管理画面に表示される

* そのIPで直接SSH接続が可能になる

```powershell
ssh pi@100.x.y.z）
```

#### 1. 構成手順

##### 0. 前提

PC: Windows（Macならそもそもこの問題は起きないので設定不要）
スマホ: Android（iPhoneの人も、スマホでターミナル操作をするなら、多分やったほうがいい。）

##### 1. アカウント作成（PCで）

[https://tailscale.com/](https://tailscale.com/)

* Google / GitHub / Microsoft などでログイン
* これが 仮想LAN になる

##### 2. Windows にインストール

[https://tailscale.com/download](https://tailscale.com/download)

* Windows版をDL
* インストール
* ログイン
* Tailscale はタスクトレイ常駐アプリとしてふるまう。

##### 3. スマホ にも入れる

* デスクトップで表示されるQRコードか Playストア で検索してインストール
* ログイン
* デスクトップに端末が追加されたか確認
* 案内されるテストコマンドをPCで実行して接続を確認できる

```powershell
ping 100.x.y.z
```

---

##### 4.  Raspberry Pi にもインストール

ラズパイで：

```bash
curl -fsSL https://tailscale.com/install.sh | sh
```

終わったら：

```bash
sudo tailscale up
```

すると、URLが出るので、**PCで開いてログイン**。

##### 5. ここまでで何が起きているか

この時点で：

* Windows
* Android
* Raspberry Pi

が **同じ仮想LAN** に入る

---

##### 6. ラズパイの固定IPを確認する

ラズパイで：

```bash
tailscale ip -4
```

例：

```
100.64.12.34
```

これがTaliscaleで表示される内容と一致するか確認する

---

##### 7. SSH接続

Windowsから：

```powershell
ssh pi@100.64.12.34
```

---

##### さらに便利なところ

###### ホスト名でSSHできるようになる

Tailscale管理画面に行くと：

```
raspberrypi.tailnet-name.ts.net
```

みたいな名前が付くので、

これで

```powershell
ssh pi@raspberrypi
```

も可能になる（Windows PowerShellでOK）。

---

###### 再起動時に自動接続

通常は自動で再接続されるが、念のため

```bash
sudo tailscale set --auto-update
```

---

### VSCodeでSSH接続したラズパイのターミナルを操作する方法

#### 1. VS Codeで拡張機能「Remote - SSH」をインストールする

#### 2. 接続

* 左下の「><」アイコン（リモート接続）からRemote-SSH: Connect to Host… を選択
* 以下を入力

```powershell
ssh ユーザー名@IPアドレス
```

* 接続後、VS Code 下部のステータスバーが「SSH: Raspberry Pi」表示になる

* Terminal → New Terminal を開くと、Pi のターミナルが利用可能

#### 3. 注意点

* 初回接続時は Pi 側に VS Code サーバが自動インストールされる。
* ターミナルは Pi のユーザ権限で動く（root操作は sudo）。

---

### Raspberry Pi Zero 2 W の起動時にIPアドレスを任意のDiscordサーバーに送信させるようにする方法（Pythonファイルの方法）

#### 0. Discordでウェブフックのリンクを取得

* Discordにログインし、ウェブフックを作成したいサーバーを選択

* チャンネルの"Server Settings"を開き、"Integrations"タブを選択し、"Webhooks"をクリック

* "New Webhook"をクリックし、ウェブフックの名前やアイコンを設定。

* ウェブフックURLをコピー

#### 1. 必要なライブラリをインストール

```bash
sudo apt update
sudo apt install python3-requests
```

#### 2. スクリプトファイルを作成

```bash
nano ~/send_ip.py
```

#### 3. 以下のコードを貼り付け

```python
import requests
import socket
import time

# DiscordのウェブフックURL
WEBHOOK_URL = "ここにコピーしたURLを貼り付ける"

def get_ip():
    # 外部（GoogleのDNSなど）に接続しに行って自分のIPを特定する
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # 実際に接続はしないが、経路を確認することでIPを取得できる
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "取得失敗"
    finally:
        s.close()
    return ip

def send_discord(message):
    payload = {"content": message}
    try:
        requests.post(WEBHOOK_URL, json=payload)
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # 起動直後はネットワークが不安定な場合があるため少し待つ
    time.sleep(30)
    
    ip_addr = get_ip()
    msg = f"🚀 ラズパイが起動しました！\nIPアドレス: `{ip_addr}`"
    send_discord(msg)
```

* 失敗したら再試行するバージョン

```python
import requests
import socket
import time

WEBHOOK_URL = "ここにコピーしたURLを貼り付ける"

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = None
    finally:
        s.close()
    return ip

# 最大10回、30秒おきにリトライする
for i in range(10):
    ip_addr = get_ip()
    if ip_addr:
        try:
            payload = {"content": f"🚀 起動成功！\nIPアドレス: `{ip_addr}`"}
            response = requests.post(WEBHOOK_URL, json=payload)
            if response.status_code == 204:
               print("Successfully sent to Discord")
               break # 成功したら終了
        except Exception as e:
            print(f"Post failed: {e}")
    
    print(f"Retry {i+1}...")
    time.sleep(30) # 次のリトライまで待機
```

#### 4. 自動起動の設定をする

ラズパイが電源ONになったとき、このスクリプトを自動で実行するように設定。ここでは crontab を使う。

##### 1. ラズパイのターミナルで以下を実行

```bash
crontab -e
```

（初めて使う場合は、1番の nano を選択）

##### 2. 一番下の行に、以下の内容を追記（起動時と5分おきに実行するように設定）

```plaintext
@reboot python3 /home/pi/send_ip.py
*/5 * * * * /usr/bin/python3 /home/pi/send_ip.py
```

ファイルパスは適応書き換え

##### 3. 保存して終了

##### 4. 再起動

```bash
sudo reboot
```

---

### Raspberry Pi Zero 2 W の起動時にIPアドレスを任意のDiscordサーバーに送信させるようにする方法（シェルスクリプトの方法）

#### 0. Discordでウェブフックのリンクを取得

* Discordにログインし、ウェブフックを作成したいサーバーを選択

* チャンネルの"Server Settings"を開き、"Integrations"タブを選択し、"Webhooks"をクリック

* "New Webhook"をクリックし、ウェブフックの名前やアイコンを設定。

* ウェブフックURLをコピー

#### 1. スクリプトファイルを作成

```bash
nano ~/discord_ip.sh
```

#### 2. 以下のコードを貼り付け

```bash
#!/bin/bash

WEBHOOK_URL="ここにコピーしたURLを貼り付ける"

# IPアドレス取得関数 (Google DNSへのルート情報から取得)
get_ip() {
    ip route get 8.8.8.8 | grep -oP 'src \K\S+'
}

# 最大10回リトライ
for i in {1..10}; do
    IP_ADDR=$(get_ip)
    
    if [ -n "$IP_ADDR" ]; then
        # JSONペイロードの作成
        PAYLOAD="{\"content\": \"🚀 起動成功！\\nIPアドレス: \`$IP_ADDR\`\"}"
        
        # Discordへ送信 (-s: 静かに, -o: 出力なし, -w: ステータスコード表示)
        STATUS=$(curl -H "Content-Type: application/json" -X POST -d "$PAYLOAD" -s -o /dev/null -w "%{http_code}" "$WEBHOOK_URL")
        
        if [ "$STATUS" -eq 204 ]; then
            echo "Successfully sent to Discord"
            exit 0
        else
            echo "Post failed with status: $STATUS"
        fi
    fi
    
    echo "Retry $i..."
    sleep 30
done
```

#### 3. スクリプトに実行権限を付与

```bash
chmod +x ~/discord_ip.sh
```

#### 4. 試しに実行してみる

```bash
./discord_ip.sh
```

#### 5. 自動起動の設定をする

ラズパイが電源ONになったとき、このスクリプトを自動で実行するように設定。ここでは crontab を使う。

##### 1. ラズパイのターミナルで以下を実行

```bash
crontab -e
```

（初めて使う場合は、1番の nano を選択）

##### 2. 一番下の行に、以下の内容を追記（起動時と5分おきに実行するように設定）

```plaintext
@reboot ~/discord_ip.sh
*/5 * * * * ~/discord_ip.sh
```

ファイルパスは適応書き換え

##### 3. 保存して終了

##### 4. 再起動

```bash
sudo reboot
```

---

## 10. 参考資料

* Raspberry Pi公式ドキュメント: <https://www.raspberrypi.com/documentation/>
* Tailscale公式サイト: <https://tailscale.com/>
* Discordウェブフックドキュメント: <https://discord.com/developers/docs/resources/webhook>
* 設計メモ_TRC2026基板: <https://docs.google.com/document/d/1BoxN7ev75-qyxDMl1QDe3Ul_IqAFg0KLx-Su-Op7N-4/edit?tab=t.0>
* TRC2026 電子部品表: <https://docs.google.com/spreadsheets/d/1rFDZrWUXG1Hqm-SPN9i2vzo1shoo_CkjGAdZZAB9toc/edit?gid=1327550036#gid=1327550036>
