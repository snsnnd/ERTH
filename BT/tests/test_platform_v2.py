import unittest

from battle_protocol import Frame, PlatformProtocolV2, decode_frame, encode_frame
from battle_protocol.v2 import ZERO_SESSION_ID


class FakeClock:
    def __init__(self, value=10.0):
        self.value = value

    def __call__(self):
        return self.value


class PlatformProtocolTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.sent = []
        self.platform = PlatformProtocolV2(
            self._send,
            mode="OFFICIAL",
            map_version="A1B2C3D4",
            clock=self.clock,
        )

    def _send(self, car_id, raw):
        self.sent.append((car_id, raw))
        return True

    def _connect_car(self, car_id):
        hello = Frame(
            ZERO_SESSION_ID,
            car_id,
            "HUB",
            0,
            "HELLO",
            ("2.0.0", "ABCDEF01", "DRIVE|POSE|CLAIM", "8"),
        )
        self.assertTrue(self.platform.receive_frame(car_id, hello, now=self.clock.value))
        welcome = decode_frame(self.sent[-1][1])
        self.assertEqual(welcome.type, "WELCOME")
        ready = Frame(welcome.sid, car_id, "HUB", 0, "READY", ("OK",))
        self.assertTrue(self.platform.receive_frame(car_id, ready, now=self.clock.value))
        self.assertTrue(self.platform.sessions[car_id].ready)
        return welcome.sid

    def _ack_last_command(self, car_id, car_seq):
        command = next(raw for target, raw in reversed(self.sent) if target == car_id and decode_frame(raw).type == "MATCH")
        command_frame = decode_frame(command)
        ack = Frame(command_frame.sid, car_id, "HUB", car_seq, "ACK", (f"{command_frame.seq:08X}", "OK"))
        self.assertTrue(self.platform.receive_frame(car_id, ack, now=self.clock.value))

    def test_handshake_and_duplicate_ready_replays_same_ack(self):
        sid = self._connect_car("A")
        self.assertNotEqual(sid, ZERO_SESSION_ID)
        ack = self.sent[-1][1]
        self.assertEqual(decode_frame(ack).type, "ACK")
        duplicate_ready = Frame(sid, "A", "HUB", 0, "READY", ("OK",))
        self.assertTrue(self.platform.receive_frame("A", duplicate_ready, now=self.clock.value))
        self.assertEqual(self.sent[-1][1], ack)

    def test_duplicate_hello_replays_welcome_but_offline_hello_starts_new_session(self):
        sid = self._connect_car("A")
        welcome = next(raw for car, raw in self.sent if car == "A" and decode_frame(raw).type == "WELCOME")
        hello = Frame(
            ZERO_SESSION_ID,
            "A",
            "HUB",
            0,
            "HELLO",
            ("2.0.0", "ABCDEF01", "DRIVE|POSE|CLAIM", "8"),
        )
        self.assertTrue(self.platform.receive_frame("A", hello, now=self.clock.value))
        self.assertEqual(self.sent[-1][1], welcome)

        self.clock.value += 1.1
        self.platform.tick(now=self.clock.value)
        self.assertTrue(self.platform.receive_frame("A", hello, now=self.clock.value))
        new_welcome = decode_frame(self.sent[-1][1])
        self.assertEqual(new_welcome.type, "WELCOME")
        self.assertNotEqual(new_welcome.sid, sid)
        self.assertFalse(self.platform.sessions["A"].ready)

    def test_new_boot_id_during_official_match_faults_and_rehandshakes(self):
        old_sid_a = self._connect_car("A")
        self._connect_car("B")
        self.assertTrue(self.platform.prepare_official())
        self._ack_last_command("A", 1)
        self._ack_last_command("B", 1)
        self.assertTrue(self.platform.start_match())
        self._ack_last_command("A", 2)
        self._ack_last_command("B", 2)
        self.assertEqual(self.platform.match_state, "RUNNING")

        rebooted_hello = Frame(
            ZERO_SESSION_ID,
            "A",
            "HUB",
            0,
            "HELLO",
            ("2.0.0", "ABCDEF02", "DRIVE|POSE|CLAIM", "8"),
        )
        self.assertTrue(self.platform.receive_frame("A", rebooted_hello, now=self.clock.value))
        self.assertEqual(self.platform.match_state, "FAULT")
        self.assertFalse(self.platform.sessions["A"].ready)
        self.assertNotEqual(self.platform.sessions["A"].sid, old_sid_a)

    def test_official_match_requires_both_acknowledgements(self):
        self._connect_car("A")
        self._connect_car("B")
        self.assertTrue(self.platform.prepare_official())
        self._ack_last_command("A", 1)
        self.assertEqual(self.platform.match_state, "LOBBY")
        self._ack_last_command("B", 1)
        self.assertEqual(self.platform.match_state, "PREPARING")

        self.assertTrue(self.platform.start_match())
        self._ack_last_command("A", 2)
        self.assertEqual(self.platform.match_state, "PREPARING")
        self._ack_last_command("B", 2)
        self.assertEqual(self.platform.match_state, "RUNNING")

    def test_official_manual_drive_is_rejected_before_match_start(self):
        self._connect_car("A")
        sent_before = len(self.sent)
        self.assertIsNone(self.platform.send_command("A", "DRIVE", "100", "100", "400"))
        self.assertEqual(len(self.sent), sent_before)

    def test_late_ack_is_accepted_inside_reordering_window(self):
        sid = self._connect_car("A")
        command_seq = self.platform.send_message("A", "CMD", ("STOP",))
        self.assertIsNotNone(command_seq)

        # The state report overtakes a lost ACK; the retry ACK keeps its old seq.
        state = Frame(sid, "A", "HUB", 2, "STATE", ("PARKED", "MANUAL", "A", "00"))
        self.assertTrue(self.platform.receive_frame("A", state, now=self.clock.value))
        ack = Frame(sid, "A", "HUB", 1, "ACK", (f"{command_seq:08X}", "OK"))
        self.assertTrue(self.platform.receive_frame("A", ack, now=self.clock.value))
        self.assertNotIn(("A", command_seq), self.platform.pending)

    def test_transition_timeout_fails_closed_and_sends_stop(self):
        self._connect_car("A")
        self._connect_car("B")
        self.assertTrue(self.platform.prepare_official())
        for _ in range(4):
            self.clock.value += 0.31
            self.platform.tick(now=self.clock.value)
        self.assertEqual(self.platform.match_state, "FAULT")
        sent_types = [decode_frame(raw).fields for _, raw in self.sent]
        self.assertGreaterEqual(sent_types.count(("STOP",)), 2)


if __name__ == "__main__":
    unittest.main()
