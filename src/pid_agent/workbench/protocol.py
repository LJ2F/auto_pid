from __future__ import annotations

from dataclasses import dataclass
import struct
import time

COMMAND = 0x7D50
FRAME_MAX = 1024
HEADER = struct.Struct(">BBHIIIII")
SAMPLE = struct.Struct(">I13fI")
SAMPLE_NAMES = ("index", "time", "reference", "measurement", "input", "actuator", "perturbation",
                "inner_reference", "inner_measurement", "outer_measurement", "controller_output", "p", "i", "d", "flags")
REASONS = ("ok", "bad_request", "busy", "session", "revision", "limit", "timeout", "cancelled",
           "nonfinite", "overflow", "blocked", "open_denied", "sequence", "slew")


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def encode(body: bytes, command: int = COMMAND) -> bytes:
    if len(body) > FRAME_MAX - 10 or not 0 <= command <= 65535:
        raise ValueError("帧超出固定缓冲区")
    frame = bytearray(struct.pack(">HHHH", 0xAA55, len(body)+10, command, 0) + body + b"\x55\xaa")
    struct.pack_into(">H", frame, 6, crc16(frame[:6]+frame[8:]))
    return bytes(frame)


@dataclass(frozen=True)
class Frame:
    command: int
    body: bytes


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.last_byte = 0.0
        self.errors = 0

    def feed(self, data: bytes, now: float | None = None) -> list[Frame]:
        now = time.monotonic() if now is None else now
        if self.buffer and now-self.last_byte > 0.25:
            self.buffer.clear()
        self.last_byte = now
        frames = []
        # Byte-wise processing bounds storage even for a large hostile TCP chunk.
        for byte in data:
            self.buffer.append(byte)
            while len(self.buffer) >= 2:
                if self.buffer[:2] != b"\xaa\x55":
                    del self.buffer[0]
                    continue
                if len(self.buffer) < 4:
                    break
                size = int.from_bytes(self.buffer[2:4], "big")
                if not 10 <= size <= FRAME_MAX:
                    del self.buffer[0]
                    self.errors += 1
                    continue
                if len(self.buffer) < size:
                    break
                packet = bytes(self.buffer[:size])
                if packet[-2:] != b"\x55\xaa" or crc16(packet[:6]+packet[8:]) != int.from_bytes(packet[6:8], "big"):
                    del self.buffer[0]
                    self.errors += 1
                    continue
                frames.append(Frame(int.from_bytes(packet[4:6], "big"), packet[8:-2]))
                del self.buffer[:size]
        return frames
