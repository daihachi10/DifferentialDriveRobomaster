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

- `--max-rpm`: 前後フルスティック時のモーター軸目標RPM（0～12000）。旋回はこの値と無関係に最大4000 RPM
- `--max-current`: PID出力の電流上限（1～5000）
- `--port`: PCまたはスマホから受けるTCPポート（既定5001）
- `--usb-device`: STM32のUSB VCPを指定する場合のデバイス名。既定は自動検出
- `--invert-id1`: C610 ID1の回転方向だけ反転
- ID2は実機の取付方向に合わせて既定で符号反転。元に戻す場合だけ`--normal-id2`

例:

```bash
python3 controller_motor_receiver.py \
  --usb-device /dev/ttyACM0 \
  --max-rpm 3000 \
  --max-current 4000
```

左スティック上下で前後の速度を指定します。左右に倒すと、その側の車輪だけを
倒した割合に応じて減速し、反対側は前後の速度を維持します。最大まで倒すと内側輪は停止します。
前進・後退のどちらでも同じ減速を適用し、上下入力がないと左スティック左右だけでは動きません。
右スティック左右はその場旋回です（最大4000 RPM）。
両スティックを同時に使うと減速後の左右輪に旋回入力を加算し、車輪の上限を超える場合は比率を保って縮小します。
実機の配置はID1が右輪、ID2が左輪で、ID2の符号は反転します。
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
python3 -u "$HOME/DifferentialDriveRobomaster/controller_motor_receiver.py" \
  --max-rpm 12000 --max-current 4000
```

アプリの「速度上限」はこの`--max-rpm`へ変換されます（100% = 12000 RPM）。
旋回は常に最大4000 RPMです。アプリから起動せず手動で起動する場合も、前後の上限だけを
`--max-rpm`で指定し、旋回上限は変更されません。

手動で受信プログラムを起動している場合は、アプリ側の自動起動をOFFにしてください。

このリポジトリにはSTM32CubeIDEプロジェクト、ビルド成果物、Windows用送信プログラム、
Androidアプリ、SSHパスワードは含めません。STM32ファームウェアは別途書き込んでください。
