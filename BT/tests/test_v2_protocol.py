import unittest

from battle_protocol import (
    Frame,
    FrameDecoder,
    ProtocolError,
    crc16_ccitt_false,
    decode_frame,
    encode_frame,
    is_newer_seq,
)


class ProtocolCodecTests(unittest.TestCase):
    def test_crc_ccitt_false_check_value(self):
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_player_document_golden_frames(self):
        frames = (
            (
                Frame(
                    "0000000000000000", "A", "HUB", 0, "HELLO",
                    ("2.0.0", "ABCDEF01", "DRIVE|POSE|CLAIM", "8"),
                ),
                b"@2,0000000000000000,A,HUB,00000000,HELLO,2.0.0,ABCDEF01,DRIVE|POSE|CLAIM,8*4CD9\n",
            ),
            (
                Frame("7B42E19A03D50211", "HUB", "A", 0x11, "CMD", ("STOP",)),
                b"@2,7B42E19A03D50211,HUB,A,00000011,CMD,STOP*A807\n",
            ),
            (
                Frame("7B42E19A03D50211", "A", "HUB", 1, "ACK", ("00000011", "OK")),
                b"@2,7B42E19A03D50211,A,HUB,00000001,ACK,00000011,OK*2DAE\n",
            ),
        )
        for frame, expected in frames:
            with self.subTest(frame=frame):
                self.assertEqual(encode_frame(frame), expected)
                self.assertEqual(decode_frame(expected), frame)

    def test_crlf_is_accepted_and_normalized_on_encode(self):
        raw = b"@2,7B42E19A03D50211,HUB,A,00000011,CMD,STOP*A807\r\n"
        self.assertEqual(decode_frame(raw).fields, ("STOP",))
        self.assertTrue(encode_frame(decode_frame(raw)).endswith(b"\n"))

    def test_rejects_bad_crc_schema_and_unterminated_frame(self):
        with self.assertRaises(ProtocolError):
            decode_frame(b"@2,7B42E19A03D50211,HUB,A,00000011,CMD,STOP*0000\n")
        with self.assertRaises(ProtocolError):
            encode_frame(Frame("7B42E19A03D50211", "HUB", "A", 0, "CMD", ("DRIVE", "100", "0", "401")))
        with self.assertRaises(ProtocolError):
            decode_frame(b"@2,7B42E19A03D50211,HUB,A,00000011,CMD,STOP*A807")

    def test_incremental_stream_handles_split_and_concatenated_frames(self):
        first = b"@2,7B42E19A03D50211,HUB,A,00000011,CMD,STOP*A807\n"
        second = b"@2,7B42E19A03D50211,A,HUB,00000001,ACK,00000011,OK*2DAE\n"
        decoder = FrameDecoder()
        self.assertEqual(decoder.feed(first[:12]), [])
        decoded = decoder.feed(first[12:] + second)
        self.assertEqual([frame.type for frame in decoded], ["CMD", "ACK"])
        self.assertEqual(decoder.error_count, 0)

    def test_stream_resynchronizes_after_corrupt_frame(self):
        valid = b"@2,7B42E19A03D50211,HUB,A,00000011,CMD,STOP*A807\n"
        decoder = FrameDecoder()
        decoded = decoder.feed(b"@bad*0000\n" + valid)
        self.assertEqual(len(decoded), 1)
        self.assertEqual(decoded[0].type, "CMD")
        self.assertEqual(decoder.error_count, 1)

    def test_stream_drops_overlong_partial_frame(self):
        decoder = FrameDecoder()
        self.assertEqual(decoder.feed(b"@" + b"x" * 127), [])
        self.assertEqual(decoder.buffered_bytes, 0)
        self.assertEqual(decoder.error_count, 1)

    def test_sequence_wrap_order(self):
        self.assertTrue(is_newer_seq(0, 0xFFFFFFFF))
        self.assertFalse(is_newer_seq(0xFFFFFFFF, 0))
        self.assertFalse(is_newer_seq(10, 10))
        self.assertFalse(is_newer_seq(0x80000000, 0))


if __name__ == "__main__":
    unittest.main()
