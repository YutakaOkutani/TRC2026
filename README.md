# TRC2026

## 主な搭載部品一覧

|分類|型番 or 商品名|提供元 or 購入先|備考|
| ---- | ---- |---|---|
|SBC |Raspberry Pi Zero 2 W|[マルツ（協賛）](https://www.marutsu.co.jp/pc/i/2792770/?srsltid=AfmBOoqj3GkGFJWZ4-w-ZxoEDAaTFkF61yXlAAaS3B1-CzCxX8cdQ3y1)|OSは、Raspberry Pi OS Lite(64bit)|
|マイクロSD |LMEX1L032GG2/LMEX1L032GG4|[秋月](https://akizukidenshi.com/catalog/g/g115844/)|16GBでも足りる|
|9軸センサ|BNO055|[秋月](https://akizukidenshi.com/catalog/g/g116996/)|
|気圧センサ|BMP180|[電子工作ステーション](https://electronicwork.shop/items/633a5766c92c5d60c0317985?srsltid=AfmBOoo89WFWoQuIXLSAinuzEanNzGPgrkFaXNkkVxKlmQEBduGWzTZU)|別に必要ないかも
|モーター|99:1 Metal Gearmotor 25Dx54L mm HP 12V |[POLOLU](https://www.pololu.com/product/3207)|
|モータードライバ|POLOLU-4038|[マルツ](https://www.marutsu.co.jp/pc/i/2350058/?srsltid=AfmBOoosyseXhsd3kJ2jmKrsmHL4Ohx6i0ej_aL8HmGz6hRows6TN5E6)|DRV8256E搭載モジュール|
|超音波センサ|HC-SR04|[秋月](https://akizukidenshi.com/catalog/g/g111009/)|別に必要ないかも|
|GNSS受信機|GT-502MGG-N|[秋月](https://akizukidenshi.com/catalog/g/g117980/)|
|カメラ|KEYESTUDIO 5MP |[アマゾン](https://amzn.asia/d/6HFwEA6)|純正品ではなく互換品。であるが性能は十分|
|バッテリー|KT850/35-3S|[アマゾン](https://amzn.asia/d/fa5oPCb)|360mAhでも完走可能だが、充電切れになりやすいため、850mAhのものを今回は選定。持続時間と重量はトレードオフ。|
|DC-DCコンバータ|AE-OKL-T/6-W12N-C|[秋月](https://akizukidenshi.com/catalog/g/g107728/)|変格ではあるが（可変抵抗を置き換えることで固定にもできる）、出力電流が多く（Max6A）、ラズパイの定格電流（2.5A）を出せるので選定|

* 上記の電子部品は在庫切れになることがよくあるので注意。その場合は、型番・商品名検索でほかのベンダーを探す。それでもない場合は代替を探す。

## その他搭載部品一覧

|分類|型番 or 商品名|提供元 or 購入先|備考|
| ---- | ---- |---|---|
| 電解コンデンサ | 35ZLH100MEFC6.3X11 |[秋月](https://akizukidenshi.com/catalog/g/g102724/)|---|
| 積層セラミックコンデンサ | RDER71H104K0P1H03B |[秋月](https://akizukidenshi.com/catalog/g/g113582/)|---|
| 金属皮膜抵抗 | ---- |---|---|
| ショットキーバリアダイオード | MA10EB045 |[秋月](https://akizukidenshi.com/catalog/g/g117496/)|---|
| LED | ---- |---|---|
| XTコネクタ | ---- |---|---|
| XHコネクタ | ---- |---|---|

## ファイル構成

```
├── main.py
├── venv
├── library/
│   ├── __init__.py
│   ├── bno055.py
│   ├── bmp180.py
│   ├── detect_corn.py
│   ├── capture_roi_img.py
│   ├── micropyGPS.py
├── tests/
│   ├── library/
│       ├── __init__.py
│       ├── bno055.py
│       ├── bmp180.py
│       ├── micropyGPS.py
│   ├── gps_test.py
│   ├── gps_test_new.py
│   ├── landing_impact.py
│   ├── led.py
│   ├── motor_test.py
│   ├── open_parachute.py
└── README.md
```

* 本番用コード：main.py
* センサー用ライブラリ：bno055.py, bmp180.py, mycropyGPS.py
* テストコード：motor_test.py, led.py, gps_test.py, landing_impact.py（着地衝撃試験用）, open_parachute.py（パラシュート投下試験用）
* カメラフェーズ用：detect_corn.py
* ゴール画像撮影用：capture_roi_image.py

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
* VSCode
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

1. その後、以下のコマンドでラズパイ上の設定を確認

```bash
sudo nano /etc/ssh/sshd_config
```

1. その中に、以下の行があれば確認。

```
PasswordAuthentication yes
```

1. PasswordAuthentication が no になっていたら yes に変更。

2. "#" でコメントアウトされている場合は、"#" を外して PasswordAuthentication yes にする

3. 設定を変更したら、SSH サーバーを再起動。

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
sudo apt update
sudo apt install -y python3-smbus i2c-tools python3-gpiozero python3-rpi-lgpio git python3 screen libgl1 

sudo apt update
sudo apt install tmux -y

# GPS処理専用のPythonライブラリ pynmea2 のインストール（serialでも処理できるから導入は任意）
# pynmea2（仮想環境必要）
sudo apt install -y python3-pip
python3 -m pip install --upgrade pip
python3 -m pip install pynmea2

# pynmea2（仮想環境不要）
sudo apt update
sudo apt install python3-pynmea2

# pynmea2（pipxバージョン）
sudo apt install pipx
pipx install pynmea2


sudo apt install swig python3-dev python3-setuptools build-essential
sudo apt install liblgpio1 liblgpio-de

sudo apt install -y python3-picamera2 python3-libcamera libcamera-apps

sudo apt install python3-opencv python3-numpy python3-smbus python3-serial
```

```bash
sudo apt update
sudo apt install -y python3-smbus i2c-tools python3-gpiozero python3-rpi-lgpio git python3 screen libgl1
sudo apt install swig python3-dev python3-setuptools build-essential
sudo apt install liblgpio1 liblgpio-de
sudo apt install -y python3-picamera2 python3-libcamera libcamera-apps
sudo apt install python3-numpy python3-smbus python3-serial
sudo apt install python3-opencv
```

```bash
# パッケージリストの更新
sudo apt update && sudo apt full-upgrade -y

# 基本ツールと通信関連
sudo apt install -y git screen i2c-tools python3-smbus python3-serial

# GPIO制御ライブラリ (lgpio)
sudo apt install -y python3-gpiozero python3-rpi-lgpio

# OpenCV / 画像処理関連（ビルド時間を短縮するため apt でインストール）
sudo apt install -y python3-opencv python3-numpy libgl1

# カメラ制御（libcamera/Picamera2関連）
sudo apt install -y python3-picamera2 python3-libcamera libcamera-apps
# リポジトリのクローン
git clone https://github.com/YutakaOkutani/TRC2026
cd TRC2026
```

### 3.1 Python 仮想環境の作成

```bash
# 仮想環境の作成（システムパッケージを引き継ぐ --system-site-packages が重要）
# これにより、aptで入れた OpenCV や Picamera2 を仮想環境内でも使用できます
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

* Performance → GPU Memory（必要に応じて）

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
rpicam-hello -t 0
# 静止画撮影
rpicam-still -o test.jpg
# 動画撮影
rpicam-vid -t 10000 -o test.h264
# 高解像度での静止画撮影テスト
rpicam-still --width 2592 --height 1944 -o maxres.jpg
```

## 6. 実行（テスト時）

### 本番用コード

```bash
tmux 
source venv/bin/activate # 必要なければ省略可
python3 main.py
tmux attach
```

### GPSからの生データ取得（テストコードのほうが見やすいが一応）

```bash
sudo screen /dev/ttyAMA0 115200 # ボーレートが違う場合、9600も試す
```

---

## 7. 実行（本番）

---

## 7. トラブルシューティング

### センサが認識されない

* `sudo i2cdetect -y 1` で確認
* 配線の導通を確認

---

## 8. その他

### SSH接続が切れてもプログラムが実行され続けるように設定する手順

#### 1. サービスファイルの作成

```bash
sudo nano /etc/systemd/system/cansat.service
```

#### 2.設定内容の書き込み

```bash
[Unit]
Description=CanSat Main Mission Script
After=multi-user.target

[Service]
# プログラムがあるディレクトリを指定
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

### VSCodeでSSH接続したラズパイのターミナルを操作する方法

#### 1. VS Codeで拡張機能「Remote - SSH」をインストールする

#### 2. 接続

* 左下の「><」アイコン（リモート接続）からRemote-SSH: Connect to Host… を選択
* 以下を入力

```
ssh ユーザー名@IPアドレス
```

* 接続後、VS Code 下部のステータスバーが「SSH: Raspberry Pi」表示になる

* Terminal → New Terminal を開くと、Pi のターミナルが利用可能

#### 3. 注意点

* 初回接続時は Pi 側に VS Code サーバが自動インストールされる。
* ターミナルは Pi のユーザ権限で動く（root操作は sudo）。

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

### Raspberry Pi Zero 2 W の起動時にIPアドレスを任意のDiscordサーバーに送信させるようにする方法

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

```bash
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

```bash
import requests
import socket
import time

WEBHOOK_URL = "あなたのURL"

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

##### 2. 一番下の行に、以下の内容を追記

```plaintext
@reboot python3 /home/pi/send_ip.py
```

ファイルパスは適応書き換え

##### 3. 保存して終了

##### 4. 再起動

```bash
sudo reboot
```

---

### Raspberry Pi Zero 2 W で IP アドレスを固定する手順

#### 1. ネットワーク情報の確認

```bash
ip a
```

#### 2. 設定ファイルの編集

dhcpcd.conf を編集。

```bash
sudo nano /etc/dhcpcd.conf
```

#### 3. 静的アドレス設定の追加

ファイル末尾に以下を追加。IP アドレスやルーター情報は使用中のネットワーク環境に合わせて変更。

```bash
interface wlan0
static ip_address=192.168.1.50/24
static routers=192.168.1.1
static domain_name_servers=192.168.1.1 8.8.8.8
```

* static ip_address: 割り当てたい固定 IP とサブネット
* static routers: デフォルトゲートウェイ（通常はルーターのアドレス）
* static domain_name_servers: DNS サーバー（static routersに入力したものと同じでよい）

#### 4. 再起動と確認

Raspberry Pi を再起動。

```bash
sudo reboot
ip a
```

---
