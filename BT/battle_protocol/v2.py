"""Strict V2 ASCII frame codec and byte-stream decoder."""

from __future__ import annotations

import re
import secrets
from dataclasses import dataclass
from typing import Iterable

PROTOCOL_VERSION = "2"
ZERO_SESSION_ID = "0000000000000000"
MAX_FRAME_BYTES = 128
MAX_U32 = 0xFFFFFFFF

_HEX16_RE = re.compile(r"^[0-9A-F]{16}$")
_HEX8_RE = re.compile(r"^[0-9A-F]{8}$")
_TOKEN_RE = re.compile(r"^[A-Za-z0-9_.|:-]+$")
_TYPE_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")
_KNOWN_IDS = {"HUB", "A", "B"}


class ProtocolError(ValueError):
    """Raised when a V2 frame is malformed or violates the wire schema."""


@dataclass(frozen=True, slots=True)
class Frame:
    sid: str
    src: str
    dst: str
    seq: int
    type: str
    fields: tuple[str, ...] = ()


def crc16_ccitt_false(data: bytes | str) -> int:
    """Return CRC-16/CCITT-FALSE for bytes or ASCII text."""
    if isinstance(data, str):
        try:
            data = data.encode("ascii")
        except UnicodeEncodeError as exc:
            raise ProtocolError("CRC input must be ASCII") from exc
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def new_session_id() -> str:
    """Generate a nonzero 64-bit session identifier."""
    while True:
        sid = secrets.token_hex(8).upper()
        if sid != ZERO_SESSION_ID:
            return sid


def is_newer_seq(candidate: int, previous: int) -> bool:
    """Compare 32-bit sequence numbers using RFC-style half-range ordering."""
    delta = (candidate - previous) & MAX_U32
    return 0 < delta < 0x80000000


def _validate_number(value: str, *, minimum: int, maximum: int, name: str) -> None:
    if not re.fullmatch(r"-?(0|[1-9][0-9]*)", value):
        raise ProtocolError(f"{name} must be a decimal integer")
    number = int(value)
    if not minimum <= number <= maximum:
        raise ProtocolError(f"{name} is outside its allowed range")


def _validate_fields(frame: Frame) -> None:
    fields = frame.fields
    kind = frame.type

    def count(expected: int) -> None:
        if len(fields) != expected:
            raise ProtocolError(f"{kind} requires {expected} payload field(s)")

    def number(index: int, minimum: int, maximum: int, name: str) -> None:
        _validate_number(fields[index], minimum=minimum, maximum=maximum, name=name)

    if kind == "HELLO":
        count(4)
        if not _TOKEN_RE.fullmatch(fields[0]) or not _HEX8_RE.fullmatch(fields[1]):
            raise ProtocolError("HELLO firmware/boot_id format is invalid")
        caps = fields[2]
        if caps != "-" and (not _TOKEN_RE.fullmatch(caps) or any(not cap for cap in caps.split("|"))):
            raise ProtocolError("HELLO caps format is invalid")
        number(3, 0, 255, "sensor_count")
    elif kind == "WELCOME":
        count(5)
        if any(not _TOKEN_RE.fullmatch(value) for value in fields[:2]):
            raise ProtocolError("WELCOME role/map_version format is invalid")
        if fields[1] != "NONE" and not _HEX8_RE.fullmatch(fields[1]):
            raise ProtocolError("WELCOME map_version must be NONE or 8-digit uppercase hex")
        number(2, 1, 60000, "hb_ms")
        number(3, 1, 400, "lease_ms")
        if fields[4] not in {"OFFICIAL", "BENCH"}:
            raise ProtocolError("WELCOME mode must be OFFICIAL or BENCH")
    elif kind == "READY":
        count(1)
        if not re.fullmatch(r"(?:OK|NOT_READY_[A-Z0-9_]+)", fields[0]):
            raise ProtocolError("READY status is invalid")
    elif kind == "MATCH":
        count(2)
        if fields[0] not in {"PREPARE", "START", "PAUSE", "RESUME", "ABORT", "FINISH"}:
            raise ProtocolError("MATCH action is unknown")
        if not _TOKEN_RE.fullmatch(fields[1]):
            raise ProtocolError("MATCH reason is invalid")
    elif kind == "CMD":
        if not fields:
            raise ProtocolError("CMD requires a subcommand")
        command = fields[0]
        if command in {"START", "STOP", "ESTOP", "ARM"}:
            count(1)
        elif command == "DRIVE":
            count(4)
            number(1, -999, 999, "left_pwm")
            number(2, -999, 999, "right_pwm")
            number(3, 1, 400, "lease_ms")
        elif command == "MODE":
            count(2)
            if fields[1] not in {"AUTO", "MANUAL"}:
                raise ProtocolError("CMD MODE must be AUTO or MANUAL")
        elif command == "TARGET":
            count(3)
            number(1, -2147483648, 2147483647, "x10")
            number(2, -2147483648, 2147483647, "y10")
        else:
            raise ProtocolError("CMD subcommand is unknown")
    elif kind == "ACK":
        count(2)
        if not _HEX8_RE.fullmatch(fields[0]) or fields[1] != "OK":
            raise ProtocolError("ACK fields are invalid")
    elif kind == "NACK":
        count(2)
        if not _HEX8_RE.fullmatch(fields[0]) or not _TOKEN_RE.fullmatch(fields[1]):
            raise ProtocolError("NACK fields are invalid")
    elif kind == "HEARTBEAT":
        count(3)
        if fields[0] not in {"PING", "PONG"} or not _TOKEN_RE.fullmatch(fields[2]):
            raise ProtocolError("HEARTBEAT fields are invalid")
        number(1, 0, 0x7FFFFFFF, "uptime_ms")
    elif kind == "STATE":
        count(4)
        if (
            not _TOKEN_RE.fullmatch(fields[0])
            or fields[1] not in {"AUTO", "MANUAL"}
            or not _TOKEN_RE.fullmatch(fields[2])
        ):
            raise ProtocolError("STATE fields are invalid")
        if not re.fullmatch(r"[0-9A-F]{2}", fields[3]):
            raise ProtocolError("STATE flags must be 2-digit uppercase hex")
        if int(fields[3], 16) & 0xFE:
            raise ProtocolError("STATE reserved flags must be zero")
    elif kind == "POSE":
        count(4)
        number(0, -2147483648, 2147483647, "x10")
        number(1, -2147483648, 2147483647, "y10")
        number(2, -180000, 180000, "yaw_mdeg")
        number(3, 0, 0x7FFFFFFF, "uptime_ms")
    elif kind == "CLAIM":
        count(3)
        number(0, -2147483648, 2147483647, "x10")
        number(1, -2147483648, 2147483647, "y10")
        number(2, 0, 0x7FFFFFFF, "uptime_ms")
    elif kind == "VERDICT":
        count(3)
        if not _HEX8_RE.fullmatch(fields[0]) or fields[1] not in {"ACCEPT", "REJECT"}:
            raise ProtocolError("VERDICT fields are invalid")
        if not _TOKEN_RE.fullmatch(fields[2]):
            raise ProtocolError("VERDICT reason is invalid")
    elif kind == "OPPONENT":
        count(5)
        if not _TOKEN_RE.fullmatch(fields[0]) or not _HEX8_RE.fullmatch(fields[3]):
            raise ProtocolError("OPPONENT identity/sequence is invalid")
        number(1, -2147483648, 2147483647, "x10")
        number(2, -2147483648, 2147483647, "y10")
        number(4, 0, 0x7FFFFFFF, "age_ms")
    elif kind == "SENSOR":
        count(1)
        if not re.fullmatch(r"[0-9A-F]{2}", fields[0]):
            raise ProtocolError("SENSOR line_mask must be 2-digit uppercase hex")
    elif kind == "SPEED":
        count(2)
        number(0, -32768, 32767, "left_mmps")
        number(1, -32768, 32767, "right_mmps")
    elif kind == "EVENT":
        count(2)
        if not _TOKEN_RE.fullmatch(fields[0]):
            raise ProtocolError("EVENT event_code is invalid")
        number(1, -2147483648, 2147483647, "event_value")
    elif kind == "SCORE":
        count(3)
        number(0, 0, 0x7FFFFFFF, "team_a")
        number(1, 0, 0x7FFFFFFF, "team_b")
        if not _TOKEN_RE.fullmatch(fields[2]):
            raise ProtocolError("SCORE reason is invalid")
    else:
        raise ProtocolError(f"unknown message type: {kind}")


def _validate_frame(frame: Frame) -> None:
    if not isinstance(frame, Frame):
        raise ProtocolError("frame must be a Frame")
    if not _HEX16_RE.fullmatch(frame.sid):
        raise ProtocolError("sid must be 16 uppercase hexadecimal characters")
    if frame.src not in _KNOWN_IDS or frame.dst not in _KNOWN_IDS or frame.src == frame.dst:
        raise ProtocolError("src/dst must be distinct HUB/A/B identifiers")
    if not isinstance(frame.seq, int) or isinstance(frame.seq, bool) or not 0 <= frame.seq <= MAX_U32:
        raise ProtocolError("seq must be an unsigned 32-bit integer")
    if not _TYPE_RE.fullmatch(frame.type):
        raise ProtocolError("message type must be an uppercase token")
    if not isinstance(frame.fields, tuple):
        raise ProtocolError("fields must be a tuple of strings")
    for field in frame.fields:
        if not isinstance(field, str) or not field or not _TOKEN_RE.fullmatch(field):
            raise ProtocolError("payload fields must be nonempty ASCII tokens")
    _validate_fields(frame)


def encode_frame(frame: Frame) -> bytes:
    """Encode a validated frame, returning ASCII bytes including LF."""
    _validate_frame(frame)
    prefix = ",".join((
        PROTOCOL_VERSION,
        frame.sid,
        frame.src,
        frame.dst,
        f"{frame.seq:08X}",
        frame.type,
        *frame.fields,
    ))
    crc = crc16_ccitt_false(prefix.encode("ascii"))
    encoded = f"@{prefix}*{crc:04X}\n".encode("ascii")
    if len(encoded) > MAX_FRAME_BYTES:
        raise ProtocolError(f"frame exceeds {MAX_FRAME_BYTES} bytes")
    return encoded


def decode_frame(data: bytes | bytearray | memoryview) -> Frame:
    """Decode one LF-terminated frame (CRLF is also accepted)."""
    raw = bytes(data)
    if len(raw) > MAX_FRAME_BYTES:
        raise ProtocolError(f"frame exceeds {MAX_FRAME_BYTES} bytes")
    if not raw.endswith(b"\n"):
        raise ProtocolError("frame is not LF-terminated")
    raw = raw[:-1]
    if raw.endswith(b"\r"):
        raw = raw[:-1]
    if not raw or b"\n" in raw or b"\r" in raw:
        raise ProtocolError("input must contain exactly one frame")
    try:
        line = raw.decode("ascii")
    except UnicodeDecodeError as exc:
        raise ProtocolError("frame must be ASCII") from exc
    if not line.startswith("@") or line.count("*") != 1:
        raise ProtocolError("frame delimiters are invalid")
    payload, checksum = line[1:].split("*", 1)
    if not re.fullmatch(r"[0-9A-F]{4}", checksum):
        raise ProtocolError("CRC must be 4 uppercase hexadecimal characters")
    if crc16_ccitt_false(payload) != int(checksum, 16):
        raise ProtocolError("CRC mismatch")
    parts = payload.split(",")
    if len(parts) < 6 or parts[0] != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol version or missing fields")
    if not _HEX8_RE.fullmatch(parts[4]):
        raise ProtocolError("seq must be 8 uppercase hexadecimal characters")
    frame = Frame(
        sid=parts[1],
        src=parts[2],
        dst=parts[3],
        seq=int(parts[4], 16),
        type=parts[5],
        fields=tuple(parts[6:]),
    )
    _validate_frame(frame)
    return frame


class FrameDecoder:
    """Incremental byte-stream decoder with bounded memory and resync."""

    def __init__(self, max_frame_bytes: int = MAX_FRAME_BYTES):
        if not 16 <= max_frame_bytes <= MAX_FRAME_BYTES:
            raise ValueError(f"max_frame_bytes must be in 16..{MAX_FRAME_BYTES}")
        self.max_frame_bytes = max_frame_bytes
        self._buffer = bytearray()
        self.error_count = 0

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes | bytearray | memoryview) -> list[Frame]:
        frames: list[Frame] = []
        for byte in bytes(data):
            if byte == ord("\n"):
                if self._buffer:
                    self._buffer.append(byte)
                    try:
                        frames.append(decode_frame(self._buffer))
                    except ProtocolError:
                        self.error_count += 1
                    self._buffer.clear()
                continue

            if byte == ord("@"):
                if self._buffer:
                    self.error_count += 1
                self._buffer.clear()
                self._buffer.append(byte)
                continue

            if not self._buffer:
                continue

            self._buffer.append(byte)
            if len(self._buffer) > self.max_frame_bytes - 1:
                self.error_count += 1
                self._buffer.clear()
        return frames


def make_frame(
    sid: str,
    src: str,
    dst: str,
    seq: int,
    message_type: str,
    fields: Iterable[str] = (),
) -> Frame:
    """Convenience constructor that normalizes a payload iterable to a tuple."""
    return Frame(sid, src, dst, seq, message_type, tuple(fields))
