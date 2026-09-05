import unittest

from inspection_protocol import (
    FRAME_SIZE,
    MSG_RESULT,
    MSG_TRIGGER,
    RESULT_NOTCH,
    ResultCache,
    StreamParser,
    build_frame,
)


class ProtocolTests(unittest.TestCase):
    def test_round_trip(self):
        raw = build_frame(MSG_TRIGGER, 42, code=3, auxiliary=9)
        self.assertEqual(FRAME_SIZE, len(raw))
        frames = StreamParser().feed(raw)
        self.assertEqual(1, len(frames))
        self.assertEqual((MSG_TRIGGER, 42, 3, 9), (
            frames[0].message_type,
            frames[0].sequence,
            frames[0].code,
            frames[0].auxiliary,
        ))

    def test_fragmented_and_concatenated_frames(self):
        parser = StreamParser()
        first = build_frame(MSG_TRIGGER, 1)
        second = build_frame(MSG_TRIGGER, 2)
        self.assertEqual([], parser.feed(first[:3]))
        frames = parser.feed(first[3:] + second)
        self.assertEqual([1, 2], [frame.sequence for frame in frames])

    def test_resynchronises_after_noise_and_bad_checksum(self):
        parser = StreamParser()
        bad = bytearray(build_frame(MSG_TRIGGER, 7))
        bad[-1] ^= 0x80
        good = build_frame(MSG_TRIGGER, 8)
        frames = parser.feed(b"\x00\xAA" + bytes(bad) + good)
        self.assertEqual([8], [frame.sequence for frame in frames])
        self.assertGreaterEqual(parser.checksum_errors, 1)

    def test_result_cache_supports_idempotent_retry(self):
        cache = ResultCache(capacity=2, ttl_s=5.0)
        response = build_frame(MSG_RESULT, 9, RESULT_NOTCH, 87)
        cache.put(9, response)
        self.assertEqual(response, cache.get(9))
        cache.acknowledge(9)
        self.assertIsNone(cache.get(9))


if __name__ == "__main__":
    unittest.main()
