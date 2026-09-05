"""Binary protocol shared by the Jetson inspection service and test tools.

Frame (8 bytes):
    SOF1 SOF2 VERSION TYPE SEQ CODE AUX XOR

The checksum is the XOR of VERSION..AUX.  SEQ makes a trigger transaction
idempotent: a retried trigger gets the cached result instead of a second
inference.
"""

from collections import OrderedDict
import time


SOF1 = 0xAA
SOF2 = 0x55
VERSION = 0x01
FRAME_SIZE = 8

MSG_TRIGGER = 0x10
MSG_RESULT = 0x20
MSG_ACK = 0x30
MSG_HEARTBEAT = 0x40

RESULT_NORMAL = 0x00
RESULT_NOTCH = 0x01
RESULT_DEFORMATION = 0x02
RESULT_VISION_ERROR = 0x7E
RESULT_PROTOCOL_ERROR = 0x7F


def xor_checksum(values):
    checksum = 0
    for value in values:
        checksum ^= int(value) & 0xFF
    return checksum


def build_frame(message_type, sequence, code=0, auxiliary=0):
    body = [
        VERSION,
        int(message_type) & 0xFF,
        int(sequence) & 0xFF,
        int(code) & 0xFF,
        int(auxiliary) & 0xFF,
    ]
    return bytes([SOF1, SOF2] + body + [xor_checksum(body)])


class Frame(object):
    __slots__ = ("message_type", "sequence", "code", "auxiliary")

    def __init__(self, message_type, sequence, code, auxiliary):
        self.message_type = message_type
        self.sequence = sequence
        self.code = code
        self.auxiliary = auxiliary

    def __repr__(self):
        return "Frame(type=0x%02X, seq=%d, code=0x%02X, aux=%d)" % (
            self.message_type,
            self.sequence,
            self.code,
            self.auxiliary,
        )


class StreamParser(object):
    """Incremental parser with byte-wise resynchronisation after line noise."""

    def __init__(self):
        self._buffer = bytearray()
        self.checksum_errors = 0
        self.version_errors = 0
        self.discarded_bytes = 0

    def feed(self, data):
        self._buffer.extend(data)
        frames = []

        while len(self._buffer) >= 2:
            if self._buffer[0] != SOF1 or self._buffer[1] != SOF2:
                del self._buffer[0]
                self.discarded_bytes += 1
                continue
            if len(self._buffer) < FRAME_SIZE:
                break

            raw = bytes(self._buffer[:FRAME_SIZE])
            if raw[2] != VERSION:
                del self._buffer[0]
                self.version_errors += 1
                continue
            if raw[7] != xor_checksum(raw[2:7]):
                del self._buffer[0]
                self.checksum_errors += 1
                continue

            del self._buffer[:FRAME_SIZE]
            frames.append(Frame(raw[3], raw[4], raw[5], raw[6]))

        return frames


class ResultCache(object):
    """Small TTL/LRU cache used to answer duplicate trigger frames."""

    def __init__(self, capacity=32, ttl_s=5.0):
        self.capacity = capacity
        self.ttl_s = ttl_s
        self._items = OrderedDict()

    def put(self, sequence, result_frame):
        now = time.monotonic()
        self._purge(now)
        self._items.pop(sequence, None)
        self._items[sequence] = (now, result_frame)
        while len(self._items) > self.capacity:
            self._items.popitem(last=False)

    def get(self, sequence):
        now = time.monotonic()
        self._purge(now)
        item = self._items.get(sequence)
        if item is None:
            return None
        self._items.move_to_end(sequence)
        return item[1]

    def acknowledge(self, sequence):
        self._items.pop(sequence, None)

    def _purge(self, now):
        expired = [
            sequence
            for sequence, (created_at, _) in self._items.items()
            if now - created_at > self.ttl_s
        ]
        for sequence in expired:
            self._items.pop(sequence, None)
