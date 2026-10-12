#!/usr/bin/env python3
"""既存DualSense送信機のTCPデータを受け、STM32へモーター指令を送る。

推奨送信側は ``controller_sender.py``（SDLの標準軸名を使用）。
NUCLEOのST-LINK USB Virtual COM Portを自動検出する。

通信経路:
    DualSense -> controller_sender.py -> TCP -> このプログラム -> USB VCP -> STM32 -> CAN -> C610
"""

from __future__ import annotations

import argparse
import glob
import importlib
import math
import os
import socket
import sys
import time
from dataclasses import dataclass


# RobotCtrl.pyが送るTCPフレームは 0xAA + 7バイト。
TCP_HEADER = 0xAA
CONTROLLER_PAYLOAD_SIZE = 7
CONTROLLER_FRAME_SIZE = 1 + CONTROLLER_PAYLOAD_SIZE

# RobotCtrl.pyではOptionsボタンが1バイト目のbit6。
OPTIONS_BUTTON_MASK = 0x40

# STM32のwatchdogは250 msなので、それより十分短い50 ms周期でUSB送信する。
USB_SEND_INTERVAL_SECONDS = 0.05

# 旋回は走行速度の設定から独立させる。左右スティックを最大まで倒した
# ときの旋回入力は常にこの値になる。
TURN_MAX_RPM = 4000


@dataclass(frozen=True)
class ControllerPacket:
    """RobotCtrl.pyから届く7バイトを、名前付きで保持する。"""

    buttons1: int
    buttons2: int
    buttons3: int
    left_x: int
    left_y: int
    right_x: int
    right_y: int

    @classmethod
    def from_payload(cls, payload: bytes) -> "ControllerPacket":
        if len(payload) != CONTROLLER_PAYLOAD_SIZE:
            raise ValueError("コントローラーペイロードは7バイト必要です")
        return cls(*payload)


class ControllerStreamParser:
    """TCPを任意の位置で分割受信しても、0xAAからフレームを復元する。"""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[ControllerPacket]:
        self._buffer.extend(data)
        packets: list[ControllerPacket] = []

        while True:
            try:
                header_index = self._buffer.index(TCP_HEADER)
            except ValueError:
                # "Hello Raspi!\n"など、0xAAを含まないデータは破棄する。
                self._buffer.clear()
                break

            if header_index > 0:
                del self._buffer[:header_index]

            if len(self._buffer) < CONTROLLER_FRAME_SIZE:
                break

            payload = bytes(self._buffer[1:CONTROLLER_FRAME_SIZE])
            del self._buffer[:CONTROLLER_FRAME_SIZE]
            packets.append(ControllerPacket.from_payload(payload))

        return packets


def stick_y_to_rpm(axis_byte: int, max_rpm: int, deadzone: float) -> int:
    """スティック上下(0～255)をC610目標回転数へ変換する。

    上へ倒すと正転、下へ倒すと逆転、中央付近は0。
    デッドゾーン外を再スケーリングし、最大まで倒すとmax_rpmになる。
    """

    value = max(0, min(255, int(axis_byte)))
    normalized = max(-1.0, min(1.0, (128 - value) / 127.0))

    if abs(normalized) <= deadzone:
        return 0

    scaled = (abs(normalized) - deadzone) / (1.0 - deadzone)
    return int(round(math.copysign(scaled * max_rpm, normalized)))


def stick_x_to_rpm(axis_byte: int, max_rpm: int, deadzone: float) -> int:
    """スティック左右(0～255)を旋回用の目標回転数へ変換する。

    右へ倒すと正、左へ倒すと負、中央付近は0。
    """

    value = max(0, min(255, int(axis_byte)))
    normalized = max(-1.0, min(1.0, (value - 128) / 127.0))

    if abs(normalized) <= deadzone:
        return 0

    scaled = (abs(normalized) - deadzone) / (1.0 - deadzone)
    return int(round(math.copysign(scaled * max_rpm, normalized)))


def mix_arcade_rpms(
    forward_rpm: int,
    turn_rpm: int,
    wheel_limit_rpm: int,
) -> tuple[int, int]:
    """前後入力と旋回入力を、左輪ID1・右輪ID2へ混合する。

    左輪 = 前後 + 旋回、右輪 = 前後 - 旋回。
    合成後に最大値を超えた場合は、左右の比率を保ったまま縮小する。
    """

    rpm_id1 = forward_rpm + turn_rpm
    rpm_id2 = forward_rpm - turn_rpm
    largest = max(abs(rpm_id1), abs(rpm_id2))

    if largest > wheel_limit_rpm:
        scale = wheel_limit_rpm / largest
        rpm_id1 = int(round(rpm_id1 * scale))
        rpm_id2 = int(round(rpm_id2 * scale))

    return rpm_id1, rpm_id2


class MotorCommandState:
    """アーケード操作の2輪指令とOptions非常停止を管理する。"""

    def __init__(
        self,
        max_rpm: int,
        deadzone: float,
        invert_id1: bool = False,
        # 実機のID2は回転方向が逆なので、既定で符号を反転する。
        invert_id2: bool = True,
    ) -> None:
        self.max_rpm = max_rpm
        self.deadzone = deadzone
        self.invert_id1 = invert_id1
        self.invert_id2 = invert_id2
        self.emergency_stopped = False
        self._previous_options_pressed = False
        self.rpm_id1 = 0
        self.rpm_id2 = 0

    def update(self, packet: ControllerPacket) -> None:
        options_pressed = bool(packet.buttons1 & OPTIONS_BUTTON_MASK)

        # 押している間に何度も切り替わらないよう、押した瞬間だけ反応する。
        if options_pressed and not self._previous_options_pressed:
            self.emergency_stopped = not self.emergency_stopped
            state = "ON（停止）" if self.emergency_stopped else "OFF（操作可能）"
            print(f"[非常停止ロック] {state}")
        self._previous_options_pressed = options_pressed

        if self.emergency_stopped:
            self.stop()
        else:
            # 左スティック上下で前後、右スティック左右で旋回します。
            forward_rpm = stick_y_to_rpm(
                packet.left_y, self.max_rpm, self.deadzone
            )
            turn_rpm = stick_x_to_rpm(
                packet.right_x, TURN_MAX_RPM, self.deadzone
            )
            mixed_id1, mixed_id2 = mix_arcade_rpms(
                forward_rpm, turn_rpm, max(self.max_rpm, TURN_MAX_RPM)
            )

            # 実機のCAN ID配置が左右逆なので、ID1/ID2を入れ替えて送る。
            self.rpm_id1, self.rpm_id2 = mixed_id2, mixed_id1

            # 対向二輪は取付方向によって片側だけ符号反転が必要になります。
            if self.invert_id1:
                self.rpm_id1 = -self.rpm_id1
            if self.invert_id2:
                self.rpm_id2 = -self.rpm_id2

    def stop(self) -> None:
        self.rpm_id1 = 0
        self.rpm_id2 = 0


def load_serial_module():
    """pyserialを遅延importし、通信なしの単体テストを可能にする。"""

    try:
        return importlib.import_module("serial")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "pyserialが必要です: sudo apt install python3-serial"
        ) from exc


def read_raspberry_pi_model(
    model_path: str = "/proc/device-tree/model",
) -> str:
    """LinuxのDevice TreeからRaspberry Piの機種名を取得する。"""

    try:
        with open(model_path, "r", encoding="ascii", errors="replace") as model_file:
            return model_file.read().replace("\x00", "").strip()
    except OSError:
        return ""


def resolve_usb_serial_device(
    requested_device: str,
    *,
    usb_candidates: list[str] | None = None,
    path_exists=None,
) -> str:
    """NUCLEO ST-LINKのUSB Virtual COM Portを選ぶ。

    安定した名前の ``/dev/serial/by-id`` を最優先し、なければ
    LinuxのUSB CDC ACMデバイス ``/dev/ttyACM*`` を使用する。
    ``--usb-device``で明示された場合は、その指定をそのまま使う。
    """

    if requested_device != "auto":
        return requested_device

    if path_exists is None:
        path_exists = os.path.exists

    if usb_candidates is None:
        by_id = sorted(glob.glob("/dev/serial/by-id/*"))
        stlink_by_id = [
            path
            for path in by_id
            if "stlink" in path.lower() or "stmicroelectronics" in path.lower()
        ]
        usb_candidates = stlink_by_id + sorted(glob.glob("/dev/ttyACM*"))

    for candidate in usb_candidates:
        if path_exists(candidate):
            return candidate

    raise RuntimeError(
        "NUCLEOのUSB Virtual COM Portが見つかりません。"
        "データ通信対応USBケーブル、NUCLEOのST-LINK側USB端子、"
        "および /dev/ttyACM* を確認してください。"
    )


class Stm32UsbVcp:
    """STM32とのUSB Virtual COM Port通信をまとめたクラス。"""

    def __init__(self, device: str, baudrate: int) -> None:
        serial = load_serial_module()
        self._serial = serial.Serial(
            device,
            baudrate=baudrate,
            timeout=0.02,
            write_timeout=0.1,
        )
        self._receive_buffer = bytearray()

    def close(self) -> None:
        self._serial.close()

    def _write_line(self, line: str) -> None:
        self._serial.write((line + "\n").encode("ascii"))
        self._serial.flush()

    def send_rpms(self, rpm_id1: int, rpm_id2: int) -> None:
        """ID 1とID 2の目標回転数を1行で同時に送る。"""

        self._write_line(f"V {rpm_id1} {rpm_id2}")

    def _request_reply(self, command: str, expected: str) -> bool:
        self._write_line(command)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if expected in self.read_lines():
                return True
            time.sleep(0.01)
        return False

    def configure_speed_control(self, max_current: int) -> bool:
        """古い電流制御ファームを誤って使わないよう、機能と設定を確認する。"""
        if not self._request_reply("CAPS", "CAPS SPEED_PID_V1"):
            return False
        return self._request_reply(f"I {max_current}", f"OK LIMIT {max_current}")

    def request_status(self) -> None:
        self._write_line("STATUS")

    def stop_motor(self) -> None:
        self._write_line("STOP")

    def read_lines(self) -> list[str]:
        waiting = self._serial.in_waiting
        if waiting > 0:
            self._receive_buffer.extend(self._serial.read(waiting))

        lines: list[str] = []
        while b"\n" in self._receive_buffer:
            raw_line, _, remainder = self._receive_buffer.partition(b"\n")
            self._receive_buffer = bytearray(remainder)
            line = raw_line.decode("ascii", errors="replace").strip()
            if line:
                lines.append(line)
        return lines

    def verify_connection(self, timeout_seconds: float = 1.0) -> bool:
        """PING/PONGでUSB VCPの双方向通信を確認する。

        STOPとPINGを連続送信すると、STM32がSTOPへ返信している最中にPINGが届き、
        受信オーバーランになる可能性があるため、間隔を空けて送信する。
        """

        self._serial.reset_input_buffer()
        self._receive_buffer.clear()
        self.stop_motor()
        time.sleep(0.12)

        # STOPへの返信を捨ててからPINGを送る。
        self._serial.reset_input_buffer()
        self._receive_buffer.clear()

        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            self._write_line("PING")
            retry_deadline = min(deadline, time.monotonic() + 0.25)

            while time.monotonic() < retry_deadline:
                for line in self.read_lines():
                    if line == "PONG":
                        return True
                time.sleep(0.01)

        return False

    def send_stop_burst(self) -> None:
        """終了時の取りこぼしに備え、停止指令を3回送る。"""

        for _ in range(3):
            self.stop_motor()
            time.sleep(0.02)


def print_important_stm32_lines(uart: Stm32UsbVcp) -> None:
    """STATUS・異常・イベントを表示する。旧M指令のACKは省略する。"""

    for line in uart.read_lines():
        if not line.startswith("OK M "):
            print(f"[STM32] {line}")


def run_connection(
    connection: socket.socket,
    address,
    uart: Stm32UsbVcp,
    max_rpm: int,
    deadzone: float,
    controller_timeout: float,
    invert_id1: bool,
    invert_id2: bool,
) -> None:
    parser = ControllerStreamParser()
    state = MotorCommandState(
        max_rpm=max_rpm,
        deadzone=deadzone,
        invert_id1=invert_id1,
        invert_id2=invert_id2,
    )
    last_packet_time: float | None = None
    next_uart_send = time.monotonic()
    next_status = next_uart_send + 1.0
    timeout_reported = False
    last_reported_rpms: tuple[int, int] | None = None

    connection.settimeout(0.02)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"[TCP] コントローラー接続: {address[0]}:{address[1]}")

    try:
        while True:
            try:
                received = connection.recv(4096)
                if not received:
                    print("[TCP] コントローラーが切断されました")
                    return

                for packet in parser.feed(received):
                    state.update(packet)
                    last_packet_time = time.monotonic()
                    timeout_reported = False
            except socket.timeout:
                pass

            now = time.monotonic()
            input_is_fresh = (
                last_packet_time is not None
                and now - last_packet_time <= controller_timeout
            )
            if not input_is_fresh:
                state.stop()
                if last_packet_time is not None and not timeout_reported:
                    print("[安全停止] コントローラーデータが途絶えました")
                    timeout_reported = True

            if now >= next_uart_send:
                uart.send_rpms(state.rpm_id1, state.rpm_id2)
                # Avoid back-to-back commands while the polling UART is busy.
                if now >= next_status:
                    next_status = now + 1.0
                    time.sleep(0.005)
                    uart.request_status()
                print_important_stm32_lines(uart)
                next_uart_send = now + USB_SEND_INTERVAL_SECONDS

                # 値が大きく変わったときだけ表示し、ログの流れすぎを防ぐ。
                rpms = (state.rpm_id1, state.rpm_id2)
                if (
                    last_reported_rpms is None
                    or abs(rpms[0] - last_reported_rpms[0]) >= 100
                    or abs(rpms[1] - last_reported_rpms[1]) >= 100
                    or (rpms == (0, 0) and last_reported_rpms != (0, 0))
                ):
                    print(
                        "[モーター目標回転数] "
                        f"ID1(左)={rpms[0]} ID2(右)={rpms[1]}"
                    )
                    last_reported_rpms = rpms
    finally:
        state.stop()
        uart.send_stop_burst()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="DualSenseのTCP入力から、USB接続のSTM32/C610へ目標RPMを送る"
    )
    parser.add_argument("--host", default="0.0.0.0", help="待受IP")
    parser.add_argument("--port", type=int, default=5001, help="TCP待受ポート")
    parser.add_argument(
        "--usb-device",
        default="auto",
        help="NUCLEO USB VCP（既定:auto。/dev/ttyACM*を自動検出）",
    )
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--max-rpm",
        type=int,
        default=1500,
        help="前後フルスティック時のモーター軸RPM（0～12000。旋回は常に最大4000）",
    )
    parser.add_argument(
        "--max-current", type=int, default=3000,
        help="PIDが使用できる電流指令の上限（1～5000、既定3000）",
    )
    parser.add_argument(
        "--deadzone", type=float, default=0.08, help="スティック中央の無反応範囲"
    )
    parser.add_argument(
        "--controller-timeout",
        type=float,
        default=0.25,
        help="TCPデータ途絶時に停止するまでの秒数",
    )
    parser.add_argument(
        "--invert-id1",
        action="store_true",
        help="C610 ID 1（左輪）の回転方向だけ反転する",
    )
    parser.add_argument(
        "--invert-id2",
        action="store_true",
        default=True,
        help="C610 ID 2（右輪）の回転方向を反転する（既定で有効）",
    )
    parser.add_argument(
        "--normal-id2",
        dest="invert_id2",
        action="store_false",
        help="ID2の符号反転を無効にする",
    )
    args = parser.parse_args()

    if not 0 <= args.max_rpm <= 12000:
        parser.error("--max-rpmは0～12000にしてください")
    if not 1 <= args.max_current <= 5000:
        parser.error("--max-currentは1～5000にしてください")
    if not 0.0 <= args.deadzone < 1.0:
        parser.error("--deadzoneは0以上1未満にしてください")
    if not 0.05 <= args.controller_timeout <= 2.0:
        parser.error("--controller-timeoutは0.05～2.0秒にしてください")
    return args


def main() -> int:
    args = parse_arguments()
    uart: Stm32UsbVcp | None = None

    try:
        pi_model = read_raspberry_pi_model()
        uart_device = resolve_usb_serial_device(args.usb_device)
        print(f"[機種] {pi_model or '取得できませんでした'}")
        uart = Stm32UsbVcp(uart_device, args.baudrate)
        print(f"[USB VCP] {uart_device} を開きました（{args.baudrate} bps）")

        if not uart.verify_connection():
            print(
                "[エラー] STM32からPONGが返りません。USBケーブル、RESET、ファームウェアを確認してください。",
                file=sys.stderr,
            )
            return 2
        print("[USB VCP] STM32 PING/PONG 成功")
        if not uart.configure_speed_control(args.max_current):
            print("[エラー] 速度PIDファームの確認・電流上限設定に失敗しました。"
                  "RoboMaster_Controller_Pi5を書き直してRESETしてください。", file=sys.stderr)
            return 2
        print(f"[速度PID] 最大 {args.max_rpm} RPM / 電流上限 {args.max_current}")

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((args.host, args.port))
            server.listen(1)
            server.settimeout(0.5)
            print(f"[TCP] {args.host}:{args.port} で送信機を待っています")

            while True:
                try:
                    connection, address = server.accept()
                except socket.timeout:
                    print_important_stm32_lines(uart)
                    continue

                with connection:
                    run_connection(
                        connection,
                        address,
                        uart,
                        max_rpm=args.max_rpm,
                        deadzone=args.deadzone,
                        controller_timeout=args.controller_timeout,
                        invert_id1=args.invert_id1,
                        invert_id2=args.invert_id2,
                    )
                print("[TCP] 再接続を待っています")

    except KeyboardInterrupt:
        print("\n[終了] Ctrl+Cを受け取りました")
    except (OSError, RuntimeError) as exc:
        print(f"[エラー] {exc}", file=sys.stderr)
        return 1
    finally:
        if uart is not None:
            try:
                uart.send_stop_burst()
            except Exception as exc:
                print(f"[警告] 停止指令を送れませんでした: {exc}", file=sys.stderr)
            uart.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
