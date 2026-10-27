"""ERTH Battle Protocol V2 helpers used by the BT platform."""

from .v2 import (
    Frame,
    FrameDecoder,
    ProtocolError,
    crc16_ccitt_false,
    decode_frame,
    encode_frame,
    is_newer_seq,
    make_frame,
    new_session_id,
)
from .platform import PlatformProtocolV2

__all__ = [
    "Frame",
    "FrameDecoder",
    "ProtocolError",
    "PlatformProtocolV2",
    "crc16_ccitt_false",
    "decode_frame",
    "encode_frame",
    "is_newer_seq",
    "make_frame",
    "new_session_id",
]
