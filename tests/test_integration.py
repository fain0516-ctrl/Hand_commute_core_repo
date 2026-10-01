import dataclasses
import json
import math
import socket
import time
import unittest

from comm_core.config import config_from_dict, load_config, read_file
from comm_core.core import STATUS_LINK_DOWN, STATUS_NORMAL, STATUS_WATCHDOG, CommCore
from comm_core.fake_pico import FakePico
from comm_core.pico_link import PicoLink
from comm_core.protocol import CmdFlag, CmdMode


CONFIG = "config/comm_core.yaml"


def wait_for(cond, timeout=2.0, step=0.005):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(step)
    return False


def udp_listener():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    s.settimeout(1.0)
    return s


def recv_json(sock):
    data, _ = sock.recvfrom(65535)
    return json.loads(data)


def drain(sock):
    sock.setblocking(False)
    last = None
    try:
        while True:
            last = json.loads(sock.recvfrom(65535)[0])
    except BlockingIOError:
        pass
    sock.settimeout(1.0)
    return last


class PicoLinkTest(unittest.TestCase):
    def setUp(self):
        self.pico = FakePico().start()
        self.link_events = []
        pico_cfg = dataclasses.replace(load_config(CONFIG).pico, host="127.0.0.1", port=self.pico.port,
                                       reconnect_min_s=0.02, reconnect_max_s=0.1)
        self.link = PicoLink(pico_cfg,
                             on_link_change=self.link_events.append)

    def tearDown(self):
        self.link.stop()
        self.pico.stop()

    def test_hello_command_state(self):
        self.link.start()
        self.assertTrue(wait_for(lambda: self.link.hello_ack is not None))
        self.assertEqual((self.link.hello_ack.n_sts, self.link.hello_ack.n_pwm), (6, 4))
        self.assertTrue(self.link.send_command(CmdMode.POSITION, [100] * 6 + [1600] * 4))
        self.assertTrue(wait_for(lambda: self.link.latest_state is not None))
        self.assertEqual(self.link.latest_state.actuators[0].position, 100)
        self.assertEqual(self.link.latest_state.actuators[9].position, 1600)

    def test_reconnect_after_drop(self):
        self.link.start()
        self.assertTrue(wait_for(lambda: self.link.connected))
        self.pico.drop_connection()
        self.assertTrue(wait_for(lambda: self.pico.connections >= 2))
        self.assertTrue(wait_for(lambda: self.link.connected))
        self.assertTrue(self.link.send_command(CmdMode.TORQUE, [0] * 10))

    def test_link_timeout_when_pico_silent(self):
        self.link.start()
        self.assertTrue(wait_for(lambda: self.link.connected))
        self.pico.mute = True
        # 재연결이 빨라 connected 는 곧 다시 True 가 되므로 끊김 이벤트로 확인
        self.assertTrue(wait_for(lambda: False in self.link_events, timeout=1.5))

    def test_stop_sends_estop(self):
        self.link.start()
        self.assertTrue(wait_for(lambda: self.link.connected))
        self.link.stop()
        self.assertTrue(wait_for(lambda: self.pico.estops == 1))


class CommCoreTest(unittest.TestCase):
    def setUp(self):
        self.pico = FakePico().start()
        self.telemetry = udp_listener()
        self.slip_sink = udp_listener()
        self.vla_sink = udp_listener()
        self.proprio_sink = udp_listener()
        base = load_config(CONFIG)
        cfg = dataclasses.replace(
            base,
            pico=dataclasses.replace(base.pico, host="127.0.0.1", port=self.pico.port, reconnect_min_s=0.02),
            channels=dataclasses.replace(
                base.channels,
                bind_host="127.0.0.1", telemetry_port=0, command_port=0, slip_port=0, vla_port=0,
                telemetry_dest=self.telemetry.getsockname(),
                proprio_dest=self.proprio_sink.getsockname(),
                slip_relay_dest=self.slip_sink.getsockname(),
                vla_action_relay_dest=self.vla_sink.getsockname(),
                include_fingertips=True,
            ),
        )
        self.core = CommCore(cfg)
        self.core.start()
        self.assertTrue(wait_for(lambda: self.core.link.connected))
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.core.close()
        self.pico.stop()
        for s in (self.telemetry, self.slip_sink, self.vla_sink, self.proprio_sink, self.sender):
            s.close()

    def send(self, endpoint, msg):
        self.sender.sendto(json.dumps(msg).encode(), endpoint.address)

    def tick(self, n=1, dt=0.01):
        for _ in range(n):
            self.core.step(time.monotonic())
            time.sleep(dt)

    def test_no_command_means_zero_torque_and_watchdog_status(self):
        self.tick(3)
        self.assertTrue(wait_for(lambda: self.pico.last_cmd is not None))
        self.assertEqual(self.pico.last_cmd.mode, CmdMode.TORQUE)
        self.assertEqual(self.pico.last_cmd.values, [0] * 10)
        self.assertEqual(self.pico.last_cmd_flags & CmdFlag.WATCHDOG_TRIPPED, CmdFlag.WATCHDOG_TRIPPED)
        self.assertEqual(recv_json(self.telemetry)["status"], STATUS_WATCHDOG)

    def test_torque_command_is_clamped_and_forwarded(self):
        self.send(self.core.cmd_rx, {"mode": "torque", "torques": [5.0, -5.0] + [1.0] * 7 + [-9.0]})
        self.tick(2)
        self.assertTrue(wait_for(lambda: self.pico.last_cmd is not None and self.pico.last_cmd.values[2] == 1000))
        v = self.pico.last_cmd.values
        self.assertEqual(v[0], 1800)    # 엄지 +-1.8 Nm
        self.assertEqual(v[1], -1800)
        self.assertEqual(v[9], -2500)   # 나머지 +-2.5 Nm
        self.assertEqual(self.pico.last_cmd_flags, 0)
        self.assertEqual(drain(self.telemetry)["status"], STATUS_NORMAL)

    def test_watchdog_trips_after_100ms(self):
        self.send(self.core.cmd_rx, {"mode": "torque", "torques": [1.0] * 10})
        self.tick(2)
        self.assertEqual(self.core.status(), STATUS_NORMAL)
        time.sleep(0.15)
        self.tick(2)
        self.assertEqual(self.core.status(), STATUS_WATCHDOG)
        self.assertTrue(wait_for(lambda: self.pico.last_cmd.values == [0] * 10))
        self.assertEqual(self.core.stats["watchdog_trips"], 1)

    def test_invalid_commands_rejected(self):
        for bad in ({"mode": "torque", "torques": [1.0] * 9},
                    {"mode": "torque", "torques": [float("nan")] * 10},
                    {"mode": "dance"}):
            self.sender.sendto(json.dumps(bad).encode(), self.core.cmd_rx.address)
        self.sender.sendto(b"not json", self.core.cmd_rx.address)
        time.sleep(0.02)
        for _ in range(4):
            self.tick(1)
        self.assertEqual(self.core.status(), STATUS_WATCHDOG)
        self.assertGreaterEqual(self.core.stats["cmd_rejected"] + self.core.cmd_rx.rx_bad, 2)

    def test_position_command_and_telemetry(self):
        q = [0.5] * 6 + [0.0] * 4
        self.send(self.core.cmd_rx, {"mode": "position", "positions": q})
        self.tick(3)
        self.assertTrue(wait_for(lambda: self.pico.positions[0] != 2048))
        self.assertEqual(self.pico.last_cmd.mode, CmdMode.POSITION)
        self.assertEqual(self.pico.positions[0], 2048 + round(0.5 / (2 * 3.141592653589793 / 4096)))
        self.tick(2)
        t = drain(self.telemetry)
        self.assertEqual(len(t["q"]), 14)
        self.assertEqual(len(t["actuator_pos"]), 10)
        self.assertAlmostEqual(t["q"][0], 0.5, places=2)       # thumb_opp <- thumb_base
        self.assertAlmostEqual(t["q"][2], 0.5, places=2)       # index_mcp <- index_mcp
        self.assertAlmostEqual(t["q"][3], 0.0, places=6)       # index_pip <- 텐던 (0 rad 지령)
        self.assertEqual(set(t["fingertips"]), {"thumb", "index", "middle", "ring", "little"})
        p = drain(self.proprio_sink)
        self.assertEqual(set(p), {"seq", "timestamp", "q", "dq"})

    def test_slip_and_vla_relay(self):
        slip = {"timestamp": 1.0, "slip_detected": [False, True, False, False, False],
                "normal_forces": [1.2, 0.4, 0, 0, 0], "reflex_action": "boost_grip_force"}
        action = {"task": "pick_and_lift", "synergy_mode": "precision_pinch",
                  "target_waypoints": [[0.05, -0.02, 0.03]], "max_force_limit_N": 3.0}
        self.send(self.core.slip_rx, slip)
        self.send(self.core.vla, action)
        time.sleep(0.02)
        self.tick(1)
        self.assertEqual(recv_json(self.slip_sink), slip)
        self.assertEqual(recv_json(self.vla_sink), action)

    def test_status_link_down(self):
        self.pico.stop()
        self.assertTrue(wait_for(lambda: not self.core.link.connected))
        self.tick(1)
        self.assertEqual(drain(self.telemetry)["status"], STATUS_LINK_DOWN)

    def test_run_loop_rate(self):
        self.send(self.core.cmd_rx, {"mode": "torque", "torques": [0.1] * 10})
        before = self.pico.cmd_count
        self.core.run(duration_s=0.5)
        sent = self.pico.cmd_count - before
        self.assertTrue(40 <= sent <= 55, sent)


class ConfigTest(unittest.TestCase):
    def raw(self):
        data = read_file(CONFIG)
        data["hand_model"] = read_file("config/hand_model.yaml")
        return data

    def test_repo_config_loads(self):
        cfg = load_config(CONFIG)
        self.assertEqual(cfg.channels.command_port, 5556)
        self.assertEqual(cfg.channels.telemetry_dest, ("127.0.0.1", 15555))
        self.assertEqual(len(cfg.actuators), 10)
        self.assertEqual(cfg.n_joints, 14)
        self.assertEqual([a.torque_limit_nm for a in cfg.actuators[:3]], [1.8, 1.8, 2.5])

    def test_missing_and_unknown_keys_rejected(self):
        data = self.raw()
        del data["pico"]["port"]
        with self.assertRaisesRegex(ValueError, "pico.port: missing"):
            config_from_dict(data)
        data = self.raw()
        data["channels"]["comand_port"] = 1
        with self.assertRaisesRegex(ValueError, "unknown keys"):
            config_from_dict(data)
        data = self.raw()
        data["loop_rate_hz"] = "fast"
        with self.assertRaisesRegex(ValueError, "loop_rate_hz: expected a number"):
            config_from_dict(data)

    def test_bad_joint_sources_rejected(self):
        data = self.raw()
        data["hand_model"]["joints"][0]["source"] = [{"actuator": "nope", "scale": 1.0}]
        with self.assertRaisesRegex(ValueError, "unknown actuator"):
            config_from_dict(data)
        data = self.raw()
        data["hand_model"]["joints"][3]["source"] = [{"joint": "index_dip", "scale": 1.0}]
        with self.assertRaisesRegex(ValueError, "defined earlier"):
            config_from_dict(data)

    def test_joint_mapping_terms(self):
        data = self.raw()
        # index_pip = 0.5 * index_tendon - 0.2 * index_mcp + 0.1, index_dip = 0.88 * index_pip
        data["hand_model"]["joints"][3]["source"] = [
            {"actuator": "index_tendon", "scale": 0.5}, {"joint": "index_mcp", "scale": -0.2}]
        data["hand_model"]["joints"][3]["offset"] = 0.1
        data["hand_model"]["joints"][1]["source"] = None
        cfg = config_from_dict(data)
        from comm_core.core import TelemetryBuilder
        pos = [0.3, 0.7, 1.0, 0, 0, 0, 2.0, 0, 0, 0]
        q, _ = TelemetryBuilder(cfg).joints(pos, [0.0] * 10)
        self.assertAlmostEqual(q[0], 0.3)
        self.assertEqual(q[1], 0.0)                     # source null -> 0
        self.assertAlmostEqual(q[3], 0.5 * 2.0 - 0.2 * 1.0 + 0.1)
        self.assertAlmostEqual(q[4], 0.88 * q[3])


class KinematicsTest(unittest.TestCase):
    def test_planar_finger_fk(self):
        from comm_core.kinematics import HandKinematics
        hand = load_config(CONFIG).hand
        kin = HandKinematics(hand)
        q = [0.0] * 14
        tip0 = kin.fingertips(q)["index"]
        # 펴진 상태: 기저 + 링크 길이 합만큼 x 방향
        links = sum(hand.joints[hand.joint_index(n)].dh.a for n in ("index_mcp", "index_pip", "index_dip"))
        self.assertAlmostEqual(tip0[0], 0.080 + links, places=6)
        self.assertAlmostEqual(tip0[2], 0.0, places=6)
        # MCP 90도 굴곡: 손가락이 손바닥 쪽(-z)으로 접힘
        q[hand.joint_index("index_mcp")] = math.pi / 2
        tip = kin.fingertips(q)["index"]
        self.assertAlmostEqual(tip[0], 0.080, places=6)
        self.assertAlmostEqual(tip[2], -links, places=6)


if __name__ == "__main__":
    unittest.main()
