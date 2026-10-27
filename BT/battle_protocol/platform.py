"""Session and match-control layer for V2 vehicle links."""

from __future__ import annotations

import time
import re
from dataclasses import dataclass, field
from typing import Callable

from .v2 import (
    Frame,
    ProtocolError,
    ZERO_SESSION_ID,
    encode_frame,
    is_newer_seq,
    new_session_id,
)

SendBytes = Callable[[str, bytes], bool]
EventHandler = Callable[[str, str, str], None]
FrameHandler = Callable[[str, Frame], None]

OFFLINE_TIMEOUT_S = 1.0
RETRY_INTERVAL_S = 0.3
MAX_RETRIES = 3
HEARTBEAT_MS = 250
DRIVE_LEASE_MS = 400


@dataclass(slots=True)
class CarSession:
    car_id: str
    sid: str = ZERO_SESSION_ID
    tx_seq: int = 0
    last_rx_seq: int | None = None
    last_seen: float = 0.0
    ready: bool = False
    online: bool = False
    match_mode: str = "OFFICIAL"
    control_mode: str = "MANUAL"
    map_version: str = "NONE"
    car_state: str = "PARKED"
    estop_latched: bool = False
    last_state_at: float = 0.0
    boot_id: str | None = None
    welcome_frame: bytes | None = None
    recent_frames: dict[int, Frame] = field(default_factory=dict)
    replay_responses: dict[int, bytes] = field(default_factory=dict)


@dataclass(slots=True)
class PendingRequest:
    car_id: str
    seq: int
    raw: bytes
    deadline: float
    retries: int = 0
    context: tuple[str, str] | None = None


class PlatformProtocolV2:
    """V2 protocol endpoint; hardware writes are supplied by the BT transport."""

    def __init__(
        self,
        send_bytes: SendBytes,
        *,
        mode: str = "BENCH",
        map_version: str = "NONE",
        on_event: EventHandler | None = None,
        on_frame: FrameHandler | None = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        if mode not in {"OFFICIAL", "BENCH"}:
            raise ValueError("mode must be OFFICIAL or BENCH")
        self._send_bytes = send_bytes
        self.mode = mode
        self.map_version = map_version
        self.on_event = on_event
        self.on_frame = on_frame
        self.clock = clock
        self.sessions = {car_id: CarSession(car_id) for car_id in ("A", "B")}
        self.match_state = "LOBBY"
        self._fault_at: float | None = None
        self.pending: dict[tuple[str, int], PendingRequest] = {}
        self._transition: dict[str, object] | None = None
        self._last_heartbeat = 0.0

    def _event(self, name: str, car_id: str = "SYS", detail: str = "") -> None:
        if self.on_event:
            self.on_event(name, car_id, detail)

    def set_mode(self, mode: str) -> None:
        if mode not in {"OFFICIAL", "BENCH"}:
            raise ValueError("mode must be OFFICIAL or BENCH")
        if self.match_state not in {"LOBBY", "FINISHED"} or self._transition is not None:
            raise RuntimeError("match mode can only change in LOBBY or FINISHED")
        if any(session.sid != ZERO_SESSION_ID for session in self.sessions.values()):
            raise RuntimeError("disconnect V2 cars before changing match mode")
        self.mode = mode

    def set_map_version(self, map_version: str) -> None:
        if self.match_state in {"PREPARING", "RUNNING", "PAUSED"}:
            raise RuntimeError("map cannot change during an active match")
        if not re.fullmatch(r"[A-Za-z0-9_.|:-]+", map_version):
            raise ValueError("map_version must be a nonempty protocol token")
        self.map_version = map_version

    def disconnect(self, car_id: str) -> None:
        session = self._get_session(car_id)
        was_ready = session.ready
        self.sessions[car_id] = CarSession(car_id)
        for key in [key for key in self.pending if key[0] == car_id]:
            del self.pending[key]
        if was_ready:
            self._event("DISCONNECTED", car_id, "link closed")
            if self.mode == "OFFICIAL" and self.match_state in {"PREPARING", "RUNNING", "PAUSED"}:
                self._fail_match("LINK_LOST")

    def receive_line(self, car_id: str, line: str | bytes) -> bool:
        """Handle a complete V2 line, as delivered by a transport adapter."""
        try:
            raw = line.encode("ascii") if isinstance(line, str) else bytes(line)
            if not raw.endswith(b"\n"):
                raw += b"\n"
            from .v2 import decode_frame

            frame = decode_frame(raw)
        except (UnicodeEncodeError, ProtocolError, TypeError, ValueError) as exc:
            self._event("FRAME_REJECTED", car_id, str(exc))
            return False
        return self.receive_frame(car_id, frame)

    def receive_frame(self, car_id: str, frame: Frame, *, now: float | None = None) -> bool:
        session = self._get_session(car_id)
        now = self.clock() if now is None else now
        if frame.src != car_id or frame.dst != "HUB":
            self._event("FRAME_REJECTED", car_id, "source/destination does not match physical link")
            return False

        if frame.type == "HELLO":
            if frame.sid != ZERO_SESSION_ID:
                self._event("FRAME_REJECTED", car_id, "HELLO must use the zero session")
                return False
            if frame.seq != 0:
                self._event("FRAME_REJECTED", car_id, "HELLO must start at sequence zero")
                return False
            return self._handle_hello(session, frame, now)

        if session.sid == ZERO_SESSION_ID or frame.sid != session.sid:
            self._event("STALE_SESSION", car_id, "re-handshake required")
            return False
        if not session.ready and (
            frame.type != "READY" or (session.last_rx_seq is None and frame.seq != 0)
        ):
            self._event("FRAME_REJECTED", car_id, "READY must complete the handshake")
            return False

        out_of_order_ack = False
        if session.last_rx_seq is not None:
            if frame.seq in session.recent_frames:
                if session.recent_frames[frame.seq] != frame:
                    self._event("SEQUENCE_COLLISION", car_id, f"{frame.seq:08X}")
                    return False
                response = session.replay_responses.get(frame.seq)
                if response is not None:
                    self._send_bytes(car_id, response)
                return True
            if is_newer_seq(frame.seq, session.last_rx_seq):
                session.last_rx_seq = frame.seq
            else:
                age = (session.last_rx_seq - frame.seq) & 0xFFFFFFFF
                out_of_order_ack = frame.type in {"ACK", "NACK"} and 0 < age <= 16
                if not out_of_order_ack:
                    self._event("STALE_SEQUENCE", car_id, f"{frame.seq:08X}")
                    return False
        else:
            session.last_rx_seq = frame.seq

        session.recent_frames[frame.seq] = frame
        while len(session.recent_frames) > 16:
            oldest_seq = next(iter(session.recent_frames))
            del session.recent_frames[oldest_seq]
            session.replay_responses.pop(oldest_seq, None)
        session.last_seen = now
        session.online = True

        if out_of_order_ack:
            self._handle_ack(session, frame)
            return True

        if frame.type == "READY":
            if frame.fields[0] == "OK":
                session.ready = True
                response = self._send(session, "ACK", (f"{frame.seq:08X}", "OK"))
                if response is not None:
                    self._cache_response(session, frame.seq, response)
                self._event("READY", car_id, "vehicle ready")
                return True
            response = self._send(
                session,
                "NACK",
                (f"{frame.seq:08X}", frame.fields[0].replace("NOT_READY_", "")),
            )
            if response is not None:
                self._cache_response(session, frame.seq, response)
            self._event("NOT_READY", car_id, frame.fields[0])
            return True

        if frame.type in {"ACK", "NACK"}:
            self._handle_ack(session, frame)
        elif frame.type == "HEARTBEAT":
            if frame.fields[0] == "PING":
                response = self._send(
                    session,
                    "HEARTBEAT",
                    ("PONG", str(int(now * 1000) & 0x7FFFFFFF), session.car_state),
                )
                if response is not None:
                    self._cache_response(session, frame.seq, response)
        elif frame.type == "STATE":
            session.car_state = frame.fields[0]
            session.control_mode = frame.fields[1]
            session.estop_latched = bool(int(frame.fields[3], 16) & 0x01)
            session.last_state_at = now
            self._event("STATE", car_id, f"{session.car_state}/{session.control_mode}")
            if session.estop_latched and self.mode == "OFFICIAL" and self.match_state == "RUNNING":
                self.pause_match("ESTOP")
        elif self.on_frame:
            self.on_frame(car_id, frame)
        return True

    def _handle_hello(self, session: CarSession, frame: Frame, now: float) -> bool:
        boot_id = frame.fields[1]
        if (
            session.sid != ZERO_SESSION_ID
            and session.boot_id == boot_id
            and session.welcome_frame
            and session.online
            and now - session.last_seen <= OFFLINE_TIMEOUT_S
        ):
            self._send_bytes(session.car_id, session.welcome_frame)
            session.last_seen = now
            session.online = True
            return True

        if (
            session.ready
            and session.boot_id != boot_id
            and self.mode == "OFFICIAL"
            and self.match_state in {"PREPARING", "RUNNING", "PAUSED"}
        ):
            self._fail_match("VEHICLE_REBOOT")

        sid = new_session_id()
        for key in [key for key in self.pending if key[0] == session.car_id]:
            del self.pending[key]
        session.sid = sid
        session.tx_seq = 0
        session.last_rx_seq = None
        session.last_seen = now
        session.last_state_at = 0.0
        session.online = True
        session.ready = False
        session.match_mode = self.mode
        session.control_mode = "MANUAL"
        session.map_version = self.map_version
        session.car_state = "PARKED"
        session.estop_latched = False
        session.boot_id = boot_id
        session.welcome_frame = None
        session.recent_frames.clear()
        session.replay_responses.clear()
        welcome = Frame(
            sid,
            "HUB",
            session.car_id,
            0,
            "WELCOME",
            (session.car_id, self.map_version, str(HEARTBEAT_MS), str(DRIVE_LEASE_MS), self.mode),
        )
        try:
            raw = encode_frame(welcome)
        except ProtocolError as exc:
            session.sid = ZERO_SESSION_ID
            session.online = False
            self._event("WELCOME_FAILED", session.car_id, str(exc))
            return False
        if not self._send_bytes(session.car_id, raw):
            session.sid = ZERO_SESSION_ID
            session.online = False
            self._event("WELCOME_FAILED", session.car_id, "transport write failed")
            return False
        session.tx_seq = 1
        session.welcome_frame = raw
        self._event("SESSION_CREATED", session.car_id, sid)
        return True

    def _get_session(self, car_id: str) -> CarSession:
        if car_id not in self.sessions:
            raise ValueError("car_id must be A or B")
        return self.sessions[car_id]

    @staticmethod
    def _cache_response(session: CarSession, seq: int, response: bytes) -> None:
        session.replay_responses[seq] = response
        while len(session.replay_responses) > 16:
            del session.replay_responses[next(iter(session.replay_responses))]

    def _send(
        self,
        session: CarSession,
        message_type: str,
        fields: tuple[str, ...],
        *,
        reliable: bool = False,
        context: tuple[str, str] | None = None,
        seq: int | None = None,
    ) -> bytes | None:
        if session.sid == ZERO_SESSION_ID:
            return None
        use_seq = session.tx_seq if seq is None else seq
        frame = Frame(session.sid, "HUB", session.car_id, use_seq, message_type, fields)
        try:
            raw = encode_frame(frame)
        except ProtocolError as exc:
            self._event("SEND_REJECTED", session.car_id, str(exc))
            return None
        if not self._send_bytes(session.car_id, raw):
            self._event("SEND_FAILED", session.car_id, message_type)
            return None
        if seq is None:
            session.tx_seq = (session.tx_seq + 1) & 0xFFFFFFFF
        if reliable:
            self.pending[(session.car_id, use_seq)] = PendingRequest(
                session.car_id,
                use_seq,
                raw,
                self.clock() + RETRY_INTERVAL_S,
                context=context,
            )
        return raw

    def send_message(
        self,
        car_id: str,
        message_type: str,
        fields: tuple[str, ...],
        *,
        reliable: bool = True,
        context: tuple[str, str] | None = None,
    ) -> int | None:
        session = self._get_session(car_id)
        if not session.ready:
            self._event("COMMAND_REJECTED", car_id, "vehicle is not READY")
            return None
        seq = session.tx_seq
        if self._send(session, message_type, fields, reliable=reliable, context=context) is None:
            return None
        return seq

    def send_command(self, car_id: str, *fields: str) -> int | None:
        session = self._get_session(car_id)
        if not fields:
            return None
        command = fields[0]
        if self._transition is not None and command not in {"STOP", "ESTOP"}:
            self._event("COMMAND_REJECTED", car_id, "match transition is pending")
            return None
        if command in {"ARM", "START", "DRIVE", "MODE", "TARGET"} and not session.online:
            self._event("COMMAND_REJECTED", car_id, "vehicle is offline")
            return None
        if command == "ESTOP":
            result = self.send_message(car_id, "CMD", tuple(fields), reliable=True, context=("command", "ESTOP"))
            if result is not None:
                session.estop_latched = True
            return result
        if command == "STOP":
            return self.send_message(car_id, "CMD", tuple(fields), reliable=True, context=("command", "STOP"))
        if command == "ARM":
            if self.match_state == "RUNNING" or session.car_state not in {"PARKED", "STOPPED", "ESTOP"}:
                self._event("COMMAND_REJECTED", car_id, "ARM requires a stopped car outside RUNNING")
                return None
            return self.send_message(car_id, "CMD", tuple(fields), reliable=True, context=("command", "ARM"))
        if command == "START":
            if self.mode != "BENCH" or session.estop_latched:
                self._event("COMMAND_REJECTED", car_id, "single-car START is BENCH-only and requires ARM")
                return None
            return self.send_message(car_id, "CMD", tuple(fields), reliable=True, context=("command", "START"))
        if command == "DRIVE":
            if self.mode == "OFFICIAL" and self.match_state != "RUNNING":
                self._event("COMMAND_REJECTED", car_id, "official match is not RUNNING")
                return None
            if self.mode == "BENCH" and session.car_state not in {"RUNNING", "MOVING"}:
                self._event("COMMAND_REJECTED", car_id, "BENCH vehicle must be started first")
                return None
            if session.estop_latched:
                self._event("COMMAND_REJECTED", car_id, "emergency stop is latched")
                return None
            if session.control_mode != "MANUAL":
                self._event("COMMAND_REJECTED", car_id, "manual DRIVE requires MANUAL control mode")
                return None
            return self.send_message(car_id, "CMD", tuple(fields), reliable=False)
        if command in {"MODE", "TARGET"}:
            if self.mode == "OFFICIAL" and self.match_state not in {
                "LOBBY", "PREPARING", "RUNNING", "PAUSED",
            }:
                self._event("COMMAND_REJECTED", car_id, "control configuration is locked in this match state")
                return None
            context = ("command", f"MODE:{fields[1]}") if command == "MODE" and len(fields) == 2 else None
            return self.send_message(car_id, "CMD", tuple(fields), reliable=True, context=context)
        self._event("COMMAND_REJECTED", car_id, f"unsupported command {command}")
        return None

    def _handle_ack(self, session: CarSession, frame: Frame) -> None:
        ref_seq = int(frame.fields[0], 16)
        pending = self.pending.pop((session.car_id, ref_seq), None)
        if pending is None:
            self._event("UNMATCHED_ACK", session.car_id, f"{ref_seq:08X}")
            return
        ok = frame.type == "ACK"
        detail = "OK" if ok else frame.fields[1]
        self._event("ACK" if ok else "NACK", session.car_id, detail)
        if pending.context and pending.context[0] == "command":
            if pending.context[1] == "ARM" and ok:
                session.estop_latched = False
            elif pending.context[1] == "START" and ok:
                session.car_state = "RUNNING"
            elif pending.context[1] == "ESTOP" and ok:
                session.estop_latched = True
            elif pending.context[1] == "STOP" and ok:
                session.car_state = "PARKED"
            elif pending.context[1] == "STOP" and not ok:
                session.car_state = "UNKNOWN"
            elif pending.context[1].startswith("MODE:") and ok:
                session.control_mode = pending.context[1].split(":", 1)[1]
        if pending.context and pending.context[0] == "transition":
            self._transition_result(session.car_id, pending.context[1], ok, detail)

    def _send_transition(self, target: str, action: str, reason: str = "NONE") -> bool:
        if self._transition is not None:
            self._event("MATCH_REJECTED", "SYS", "another transition is pending")
            return False
        required = {car for car, session in self.sessions.items() if session.ready}
        online_ready = {car for car, session in self.sessions.items() if session.ready and session.online}
        if self.mode == "OFFICIAL" and required != {"A", "B"}:
            self._event("MATCH_REJECTED", "SYS", "official match requires both cars READY")
            return False
        if self.mode == "OFFICIAL" and online_ready != {"A", "B"}:
            self._fail_match("LINK_OFFLINE")
            return False
        if self.mode == "OFFICIAL":
            required = online_ready
        if not required:
            self._event("MATCH_REJECTED", "SYS", "no READY vehicles")
            return False
        transition_id = f"{action}:{target}"
        self._transition = {"id": transition_id, "target": target, "awaiting": set(required)}
        if target == "PAUSED":
            self.match_state = "PAUSED"
        for car in sorted(required):
            seq = self.send_message(
                car,
                "MATCH",
                (action, reason),
                reliable=True,
                context=("transition", transition_id),
            )
            if seq is None:
                self._fail_match("COMMAND_SEND_FAILED")
                return False
        return True

    def _transition_result(self, car_id: str, transition_id: str, ok: bool, detail: str) -> None:
        transition = self._transition
        if transition is None or transition["id"] != transition_id:
            return
        if not ok:
            self._fail_match(f"{car_id}_{detail}")
            return
        awaiting = transition["awaiting"]
        awaiting.discard(car_id)
        if not awaiting:
            self.match_state = transition["target"]
            self._event("MATCH_STATE", "SYS", str(self.match_state))
            self._transition = None

    def prepare_official(self) -> bool:
        if self.mode != "OFFICIAL" or self.match_state not in {"LOBBY", "FINISHED"}:
            return False
        if self.map_version == "NONE":
            self._event("MATCH_REJECTED", "SYS", "official match requires a loaded map")
            return False
        if any(self.sessions[car].map_version != self.map_version for car in ("A", "B")):
            self._event("MATCH_REJECTED", "SYS", "reconnect V2 cars after loading or changing the map")
            return False
        return self._send_transition("PREPARING", "PREPARE")

    def start_match(self) -> bool:
        if self.mode != "OFFICIAL" or self.match_state != "PREPARING":
            return False
        if any(self.sessions[car].estop_latched for car in ("A", "B")):
            self._event("MATCH_REJECTED", "SYS", "ESTOP is latched")
            return False
        return self._send_transition("RUNNING", "START")

    def pause_match(self, reason: str = "OPERATOR") -> bool:
        if self.mode != "OFFICIAL" or self.match_state != "RUNNING":
            return False
        return self._send_transition("PAUSED", "PAUSE", reason)

    def resume_match(self) -> bool:
        if self.mode != "OFFICIAL" or self.match_state != "PAUSED":
            return False
        if any(not self.sessions[car].ready or not self.sessions[car].online for car in ("A", "B")):
            self._event("MATCH_REJECTED", "SYS", "both cars must be online and READY")
            return False
        if any(self.sessions[car].estop_latched for car in ("A", "B")):
            self._event("MATCH_REJECTED", "SYS", "ESTOP is latched; ARM required")
            return False
        return self._send_transition("RUNNING", "RESUME")

    def finish_match(self, *, abort: bool = False) -> bool:
        if self.mode != "OFFICIAL" or self.match_state not in {"PREPARING", "RUNNING", "PAUSED"}:
            return False
        action = "ABORT" if abort else "FINISH"
        return self._send_transition("FINISHED", action)

    def reset_fault(self) -> bool:
        if self.match_state != "FAULT" or self._transition is not None:
            return False
        if self._fault_at is None:
            return False
        if self.mode == "OFFICIAL" and any(
            not self.sessions[car].ready or not self.sessions[car].online
            for car in ("A", "B")
        ):
            self._event("RESET_REJECTED", "SYS", "both cars must reconnect and become READY")
            return False
        if any(
            session.ready and session.car_state not in {"PARKED", "STOPPED", "ESTOP"}
            for session in self.sessions.values()
        ):
            self._event("RESET_REJECTED", "SYS", "all connected cars must report stopped")
            return False
        if any(
            session.ready and session.last_state_at < self._fault_at
            for session in self.sessions.values()
        ):
            self._event("RESET_REJECTED", "SYS", "fresh stopped STATE report required after fault")
            return False
        if any(
            pending.context == ("command", "STOP")
            for pending in self.pending.values()
        ):
            self._event("RESET_REJECTED", "SYS", "waiting for STOP confirmation")
            return False
        self.pending.clear()
        self.match_state = "LOBBY"
        self._fault_at = None
        self._event("MATCH_STATE", "SYS", "LOBBY (operator reset)")
        return True

    def stop_vehicle(self, car_id: str) -> int | None:
        if self.mode == "OFFICIAL" and self.match_state == "RUNNING":
            self.pause_match("SINGLE_CAR_STOP")
            return None
        return self.send_command(car_id, "STOP")

    def emergency_stop(self, car_id: str | None = None) -> bool:
        targets = (car_id,) if car_id in {"A", "B"} else ("A", "B")
        sent = False
        for target in targets:
            if self.sessions[target].ready:
                sent = self.send_command(target, "ESTOP") is not None or sent
        if self.mode == "OFFICIAL" and self.match_state == "RUNNING":
            self.pause_match("ESTOP")
        return sent

    def arm_vehicle(self, car_id: str) -> int | None:
        return self.send_command(car_id, "ARM")

    def start_bench_vehicle(self, car_id: str) -> int | None:
        if self.mode != "BENCH":
            return None
        return self.send_command(car_id, "START")

    def _fail_match(self, reason: str) -> None:
        self.match_state = "FAULT"
        self._fault_at = self.clock()
        self._transition = None
        for key, pending in list(self.pending.items()):
            if pending.context and pending.context[0] == "transition":
                del self.pending[key]
        self._event("MATCH_FAULT", "SYS", reason)
        for session in self.sessions.values():
            if session.ready:
                if self._send(
                    session,
                    "CMD",
                    ("STOP",),
                    reliable=True,
                    context=("command", "STOP"),
                ) is None:
                    session.car_state = "UNKNOWN"

    def tick(self, *, now: float | None = None) -> None:
        now = self.clock() if now is None else now
        for key, pending in list(self.pending.items()):
            if key not in self.pending:
                continue
            if now < pending.deadline:
                continue
            if pending.retries < MAX_RETRIES:
                if self._send_bytes(pending.car_id, pending.raw):
                    pending.retries += 1
                    pending.deadline = now + RETRY_INTERVAL_S
                    self._event("RETRY", pending.car_id, f"{pending.seq:08X}")
                    continue
            del self.pending[key]
            self._event("ACK_TIMEOUT", pending.car_id, f"{pending.seq:08X}")
            if pending.context and pending.context[0] == "transition":
                self._fail_match("ACK_TIMEOUT")

        if now - self._last_heartbeat >= HEARTBEAT_MS / 1000:
            self._last_heartbeat = now
            for session in self.sessions.values():
                if session.ready and session.sid != ZERO_SESSION_ID:
                    self._send(
                        session,
                        "HEARTBEAT",
                        ("PING", str(int(now * 1000) & 0x7FFFFFFF), self.match_state),
                        reliable=False,
                    )

        for session in self.sessions.values():
            if session.ready and now - session.last_seen > OFFLINE_TIMEOUT_S:
                if session.online:
                    session.online = False
                    self._event("OFFLINE", session.car_id, "heartbeat timeout")
                    if self.mode == "OFFICIAL" and self.match_state == "RUNNING":
                        self.pause_match("LINK_OFFLINE")
                    elif self.mode == "OFFICIAL" and self.match_state == "PREPARING":
                        self._fail_match("LINK_OFFLINE")
