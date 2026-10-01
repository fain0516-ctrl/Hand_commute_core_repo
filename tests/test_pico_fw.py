"""Pico 펌웨어 검증 (하드웨어 없이).

1. tools/gen_config.py: 두 보드 설정이 생성되고, 잘못된 핀/키는 거부되는지
2. 펌웨어의 proto.c + session.c 를 호스트용으로 컴파일한 시뮬레이터(pico_fw/host/sim_main.c)에
   Pi 쪽 comm_core.PicoLink 를 붙여 실제 프로토콜로 주고받는지

gcc 가 없으면 2 는 건너뛴다.
"""

import copy
import dataclasses
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import yaml

from comm_core.config import load_config
from comm_core.pico_link import PicoLink
from comm_core.protocol import CmdMode, Frame, FrameDecoder, Hello, HelloAck, MsgType, encode_frame

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW = os.path.join(ROOT, "pico_fw")
GEN = os.path.join(FW, "tools", "gen_config.py")
CONTROLLER = os.path.join(FW, "config", "controller.yaml")
BOARDS = os.path.join(FW, "config", "boards")

ST_TORQUE_ENABLED, ST_WATCHDOG, ST_ESTOP = 0x1, 0x2, 0x4
ERR_UNSUPPORTED_MODE, ERR_VALUE_CLAMPED = 3, 4


def wait_for(cond, timeout=2.0, step=0.005):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(step)
    return False


def gen(config, board, out):
    return subprocess.run([sys.executable, GEN, "--config", config, "--board", board, "--out", out],
                          capture_output=True, text=True)


class GenConfigTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def board(self, name):
        with open(os.path.join(BOARDS, name + ".yaml"), encoding="utf-8") as f:
            return yaml.safe_load(f)

    def write_board(self, data):
        path = os.path.join(self.tmp, "board.yaml")
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True)
        return path

    def test_all_boards_generate(self):
        for name in sorted(os.listdir(BOARDS)):
            r = gen(CONTROLLER, os.path.join(BOARDS, name), os.path.join(self.tmp, name))
            self.assertEqual(r.returncode, 0, r.stderr)
            with open(os.path.join(self.tmp, name, "fw_config.h"), encoding="utf-8") as f:
                h = f.read()
            self.assertIn("#define FW_N_ACT                10", h)
            self.assertIn("#define FW_NET_IP               {192, 168, 10, 20}", h)  # comm_core.yaml pico.host

    def test_pcb_board_reports_inferred_pins(self):
        r = gen(CONTROLLER, os.path.join(BOARDS, "middleware_pcb_rev0.yaml"), self.tmp)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("w5500.cs = GPIO33", r.stderr)
        with open(os.path.join(self.tmp, "fw_config.h"), encoding="utf-8") as f:
            h = f.read()
        self.assertIn("#define FW_W5500_PIN_SCK        6", h)
        self.assertIn("#define FW_W5500_PIN_MOSI       39", h)
        self.assertIn("#define FW_W5500_PIN_MISO       32", h)

    def test_wrong_spi_pin_rejected(self):
        b = self.board("pico2_w5500")
        b["w5500"]["mosi"] = 18  # SCK 기능 핀
        b["w5500"]["sck"] = 14
        r = gen(CONTROLLER, self.write_board(b), self.tmp)
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("SPI0", r.stderr)

    def test_gpio_out_of_range_for_chip(self):
        b = self.board("pico2_w5500")
        b["status_led"]["pin"] = 40  # RP2350A 에는 GPIO40 없음
        r = gen(CONTROLLER, self.write_board(b), self.tmp)
        self.assertNotEqual(r.returncode, 0)

    def test_duplicate_pin_rejected(self):
        b = self.board("pico2_w5500")
        b["pwm_pins"][0] = 25  # LED 핀과 중복
        r = gen(CONTROLLER, self.write_board(b), self.tmp)
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("GPIO25", r.stderr)

    def test_unknown_and_missing_keys_rejected(self):
        b = self.board("pico2_w5500")
        b2 = copy.deepcopy(b)
        b2["servo_bus"]["baudrate"] = 1  # 오타
        self.assertNotEqual(gen(CONTROLLER, self.write_board(b2), self.tmp).returncode, 0)
        del b["servo_bus"]["echo"]
        self.assertNotEqual(gen(CONTROLLER, self.write_board(b), self.tmp).returncode, 0)


@unittest.skipUnless(shutil.which("gcc"), "gcc 없음")
class FirmwareSimTest(unittest.TestCase):
    """펌웨어 세션 코드 + Pi PicoLink 실제 연동."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        r = gen(CONTROLLER, os.path.join(BOARDS, "pico2_w5500.yaml"), cls.tmp)
        assert r.returncode == 0, r.stderr
        cls.bin = os.path.join(cls.tmp, "fw_sim")
        src = [os.path.join(FW, "src", "proto.c"), os.path.join(FW, "src", "session.c"),
               os.path.join(FW, "host", "sim_main.c")]
        subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O1", "-I", os.path.join(FW, "src"),
                        "-I", cls.tmp, *src, "-o", cls.bin], check=True)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp)

    def setUp(self):
        self.proc = subprocess.Popen([self.bin, "0"], stdout=subprocess.PIPE, text=True)
        self.port = int(self.proc.stdout.readline().split()[1])
        self.sim_lines = []
        threading.Thread(target=self._read_sim, daemon=True).start()
        cfg = dataclasses.replace(load_config("config/comm_core.yaml").pico, host="127.0.0.1", port=self.port,
                                  reconnect_min_s=0.02, reconnect_max_s=0.1)
        self.cfg = cfg
        self.link = PicoLink(cfg)

    def tearDown(self):
        self.link.stop()
        self.proc.kill()
        self.proc.wait()
        self.proc.stdout.close()

    def _read_sim(self):
        for line in self.proc.stdout:
            self.sim_lines.append(line.split())

    def sim_enabled(self):
        en = [int(l[1], 16) for l in self.sim_lines if l and l[0] == "enabled"]
        return en[-1] if en else 0

    def start_link(self):
        self.link.start()
        self.assertTrue(wait_for(lambda: self.link.hello_ack is not None))

    def command(self, mode, values, flags=0):
        before = self.link.stats["rx_frames"]
        self.assertTrue(self.link.send_command(mode, values, flags))
        self.assertTrue(wait_for(lambda: self.link.stats["rx_frames"] > before))
        return self.link.latest_state

    def test_hello_ack(self):
        self.start_link()
        ack = self.link.hello_ack
        self.assertEqual((ack.fw_version, ack.n_sts, ack.n_pwm), (0x0100, 6, 4))

    def test_position_command_returns_state(self):
        self.start_link()
        st = self.command(CmdMode.POSITION, [100, 200, 300, 400, 500, 600, 1000, 1200, 1400, 1600])
        self.assertEqual([a.position for a in st.actuators], [100, 200, 300, 400, 500, 600, 1000, 1200, 1400, 1600])
        self.assertTrue(st.status & ST_TORQUE_ENABLED)
        self.assertEqual(st.actuators[0].temperature_c10, 300)
        self.assertTrue(wait_for(lambda: self.sim_enabled() == 0x3FF))

    def test_out_of_range_is_clamped(self):
        self.start_link()
        st = self.command(CmdMode.POSITION, [9999] + [2048] * 5 + [100] + [1500] * 3)
        self.assertEqual(st.actuators[0].position, 4095)
        self.assertEqual(st.actuators[6].position, 500)
        self.assertEqual(st.error, ERR_VALUE_CLAMPED)
        self.assertTrue(st.actuators[0].flags & 0x10)

    def test_torque_zero_releases(self):
        self.start_link()
        self.command(CmdMode.POSITION, [2048] * 6 + [1500] * 4)
        st = self.command(CmdMode.TORQUE, [0] * 10)
        self.assertFalse(st.status & ST_TORQUE_ENABLED)
        self.assertTrue(wait_for(lambda: self.sim_enabled() == 0))

    def test_nonzero_torque_reports_error(self):
        self.start_link()
        st = self.command(CmdMode.TORQUE, [500] + [0] * 9)
        self.assertEqual(st.error, ERR_UNSUPPORTED_MODE)
        self.assertTrue(wait_for(lambda: self.link.stats["errors"] >= 1))

    def test_pico_watchdog_releases_without_commands(self):
        self.start_link()
        self.command(CmdMode.POSITION, [2048] * 6 + [1500] * 4)
        self.assertTrue(wait_for(lambda: self.sim_enabled() == 0x3FF))
        # Pi 는 하트비트만 보내고 지령은 멈춤 -> cmd_timeout_ms(100) 뒤 Pico 가 스스로 해제
        t0 = time.monotonic()
        self.assertTrue(wait_for(lambda: self.sim_enabled() == 0, timeout=1.0))
        self.assertLess(time.monotonic() - t0, 0.5)
        self.assertTrue(self.link.connected)
        st = self.command(CmdMode.TORQUE, [0] * 10)
        self.assertTrue(st.status & ST_WATCHDOG)
        st = self.command(CmdMode.POSITION, [2048] * 6 + [1500] * 4)  # 다음 위치 지령으로 복귀
        self.assertFalse(st.status & ST_WATCHDOG)

    def test_estop(self):
        self.start_link()
        self.command(CmdMode.POSITION, [2048] * 6 + [1500] * 4)
        self.assertTrue(self.link.send_estop())
        self.assertTrue(wait_for(lambda: self.sim_enabled() == 0))

    def test_disconnect_releases_and_new_client_wins(self):
        self.start_link()
        self.command(CmdMode.POSITION, [2048] * 6 + [1500] * 4)
        # 두 번째 클라이언트가 붙으면 이전 연결은 끊기고 토크는 해제된다
        s = socket.create_connection(("127.0.0.1", self.port))
        s.sendall(encode_frame(Frame(MsgType.HELLO, Hello(10, 100).pack(), 1)))
        dec, frames = FrameDecoder(), []
        s.settimeout(1.0)
        while not frames:
            frames += dec.feed(s.recv(4096))
        self.assertEqual(frames[0].msg_type, MsgType.HELLO_ACK)
        self.assertTrue(wait_for(lambda: self.sim_enabled() == 0))
        s.close()
        # Pi 링크는 재연결한다
        self.assertTrue(wait_for(lambda: self.link.connected and self.link.stats["reconnects"] >= 1, timeout=3.0))

    def test_resync_after_garbage(self):
        s = socket.create_connection(("127.0.0.1", self.port))
        good = encode_frame(Frame(MsgType.HELLO, Hello(10, 100).pack(), 7))
        bad_crc = bytearray(good)
        bad_crc[-1] ^= 0xFF
        s.sendall(b"\x00\xAA\x13garbage\xAA" + bytes(bad_crc) + good[:5])
        time.sleep(0.02)
        s.sendall(good[5:])  # 프레임이 두 조각으로 나뉘어 도착
        dec, frames = FrameDecoder(), []
        s.settimeout(1.0)
        while not frames:
            frames += dec.feed(s.recv(4096))
        self.assertEqual(len(frames), 1)
        self.assertEqual(HelloAck.unpack(frames[0].payload).n_sts, 6)
        s.close()


if __name__ == "__main__":
    unittest.main()


@unittest.skipUnless(shutil.which("gcc"), "gcc 없음")
class PipelineCheckTest(unittest.TestCase):
    """가상 팀 입력 -> 실제 CommCore -> 펌웨어(서보 루프 포함, SDK 대체) 출력 확인 스크립트가 기대대로 나오는지."""

    def test_pipeline_outputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, "report.md")
            subprocess.run([sys.executable, os.path.join(FW, "host", "pipeline_check.py"), "--out", out],
                           check=True, capture_output=True, timeout=60, cwd=ROOT)
            with open(out, encoding="utf-8") as f:
                report = f.read()
        sections = {s.split("\n", 1)[0]: s for s in report.split("\n## ")}
        pos = sections["2 position"]
        self.assertIn("SYNC_WRITE GOAL_POSITION+TIME+SPEED: `{1:2374, 2:2309, 3:2244, 4:2178, 5:2113, 6:1983}`", pos)
        self.assertIn("GPIO6=1500us, GPIO7=1627us, GPIO8=1755us, GPIO9=1373us", pos)
        self.assertIn("ERROR x1", sections["4 torque != 0"])
        self.assertIn("GPIO6=0us", sections["6 torque = 0"])
        lat = sections["9 position 후 Pi 루프 정지"].split("쓰기 후 ")[1].split(" ms")[0]
        self.assertTrue(80 <= int(lat) <= 160, lat)   # Pico 자체 워치독 (cmd_timeout 100 ms)
        self.assertIn("토크 해제", sections["10 position 중 Pi 종료"])
