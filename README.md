# DifferentialDriveRobomaster

Raspberry Pi側で動かすファイルだけを管理するリポジトリです。
PiはTCPで受け取ったDualSense形式の8バイトフレームをSTM32のUSB VCPへ送り、
STM32側の速度PIDでC610 ID1/ID2を制御します。

## Piで最初に設定する

```bash
sudo apt update
sudo apt install -y git python3-serial
cd ~
git clone https://github.com/daihachi10/DifferentialDriveRobomaster.git
cd ~/DifferentialDriveRobomaster
python3 -m pip install --user -r requirements.txt
```

`daihachi`ユーザーを`dialout`グループへ追加していない場合は、次も一度だけ実行します。

```bash
sudo usermod -aG dialout "$USER"
```

ログインし直すか、Piを再起動してください。

## 起動

```bash
cd ~/DifferentialDriveRobomaster
python3 controller_motor_receiver.py --max-rpm 3000 --max-current 4000
```

初回は車輪を浮かせて確認してください。

- `--max-rpm`: フルスティック時のモーター軸目標RPM（1～12000）
- `--max-current`: PID出力の電流上限（1～5000）
- `--port`: PCまたはスマホから受けるTCPポート（既定5001）
- `--usb-device`: STM32のUSB VCPを指定する場合のデバイス名。既定は自動検出
- `--invert-id1` / `--invert-id2`: 指定したC610の回転方向だけ反転

例:

```bash
python3 controller_motor_receiver.py \
  --usb-device /dev/ttyACM0 \
  --max-rpm 3000 \
  --max-current 4000
```

送信側は左スティック上下を前後、右スティック左右を旋回として、
`0xAA + 7バイト`のフレームを20 ms周期で送ります。受信側はデータが250 ms途絶えると停止します。

## Windowsで更新してPiで反映する

Windowsでこのリポジトリを編集し、変更をGitHubへ送ります。

```powershell
cd C:\Users\daiha\Documents\DifferentialDriveRobomaster
git add controller_motor_receiver.py README.md requirements.txt
git commit -m "Update Raspberry Pi receiver"
git push origin main
```

Pi側では次を実行します。

```bash
cd ~/DifferentialDriveRobomaster
git pull --ff-only
```

実行中のPythonプロセスは読み込み済みの古いコードを使い続けるため、反映後は一度停止して同じ起動コマンドを実行してください。

## Androidアプリから起動する場合

Androidアプリの「Pi側プログラムを自動起動」をONにすると、SSHで次を実行します。

```bash
python3 -u "$HOME/DifferentialDriveRobomaster/controller_motor_receiver.py"
```

手動で受信プログラムを起動している場合は、アプリ側の自動起動をOFFにしてください。

このリポジトリにはSTM32CubeIDEプロジェクト、ビルド成果物、Windows用送信プログラム、
Androidアプリ、SSHパスワードは含めません。STM32ファームウェアは別途書き込んでください。
