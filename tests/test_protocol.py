import struct
import unittest

from comm_core.protocol import (
    MAGIC,
    ActuatorCmd,
    ActuatorState,
    CmdMode,
    Frame,
    FrameDecoder,
    MsgType,
    State,
    crc16_ccitt,
    encode_frame,
)


class ProtocolTest(unittest.TestCase):
    def test_crc_reference_vector(self):
        # CRC-16/CCITT-FALSE 표준 검증값
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_roundtrip(self):
        cmd = ActuatorCmd(CmdMode.POSITION, [0, 4095, -5, 1500])
        data = encode_frame(Frame(MsgType.ACTUATOR_CMD, cmd.pack(), seq=7, timestamp_us=123, flags=1))
        self.assertTrue(data.startswith(MAGIC))
        frames = FrameDecoder().feed(data)
        self.assertEqual(len(frames), 1)
        f = frames[0]
        self.assertEqual((f.msg_type, f.seq, f.timestamp_us, f.flags), (MsgType.ACTUATOR_CMD, 7, 123, 1))
        back = ActuatorCmd.unpack(f.payload)
        self.assertEqual(back.mode, CmdMode.POSITION)
        self.assertEqual(back.values, [0, 4095, -5, 1500])

    def test_byte_by_byte_and_multiple(self):
        a = encode_frame(Frame(MsgType.HEARTBEAT, b"", seq=1))
        b = encode_frame(Frame(MsgType.STATE, State(1, 0, [ActuatorState(2048, 3, -4, 300, 0)]).pack(), seq=2))
        dec = FrameDecoder()
        out = []
        for byte in a + b:
            out += dec.feed(bytes([byte]))
        self.assertEqual([f.seq for f in out], [1, 2])
        st = State.unpack(out[1].payload)
        self.assertEqual(st.actuators[0].position, 2048)
        self.assertEqual(st.actuators[0].effort, -4)

    def test_resync_after_garbage_and_corruption(self):
        good = encode_frame(Frame(MsgType.HEARTBEAT, b"", seq=9))
        bad = bytearray(encode_frame(Frame(MsgType.HEARTBEAT, b"xx", seq=8)))
        bad[-3] ^= 0xFF  # payload 손상 -> CRC 불일치
        dec = FrameDecoder()
        frames = dec.feed(b"\x00\xAA\x13garbage" + bytes(bad) + good)
        self.assertEqual([f.seq for f in frames], [9])
        self.assertEqual(dec.crc_errors, 1)

    def test_oversized_length_is_rejected(self):
        hdr = struct.pack("<2sBBBBHIH", MAGIC, 1, 0x10, 0, 0, 1, 0, 60000)
        good = encode_frame(Frame(MsgType.HEARTBEAT, b"", seq=3))
        frames = FrameDecoder().feed(hdr + good)
        self.assertEqual([f.seq for f in frames], [3])

    def test_unknown_type_passes_through(self):
        frames = FrameDecoder().feed(encode_frame(Frame(0x55, b"future")))
        self.assertEqual(frames[0].msg_type, 0x55)


if __name__ == "__main__":
    unittest.main()
