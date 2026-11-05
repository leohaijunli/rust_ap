import unittest

from check_stack_frames import entry_frames


class EntryFrameTests(unittest.TestCase):
    def test_actual_large_thumb_poll_frame_is_not_hidden_by_sparse_stack_paint(self):
        text = """08049390 <estimator_poll>:
 8049390: b5f0 push {r4, r5, r6, r7, lr}
 8049394: e92d 0f00 push.w {r8, r9, r10, r11}
 8049398: b081 sub sp, #0x4
 804939a: ed2d 8b10 vpush {d8, d9, d10, d11, d12, d13, d14, d15}
 804939e: f5ad 3d11 sub.w sp, sp, #0x24400
 80493a2: f5ad 7d58 sub.w sp, sp, #0x360
 80493a6: f000 f800 bl 0x809175c <initialize>
 80493aa: b081 sub sp, #0x4
"""
        result = entry_frames(text)
        self.assertEqual(result[0]["bytes"], 149448)
        self.assertGreater(result[0]["bytes"], 65536)

    def test_multiple_symbols_and_register_ranges_do_not_merge_frames(self):
        result = entry_frames("""00000001 <first>:
 01: e92d push.w {r4-r7, lr}
 05: 0000 sub.w sp, sp, #100
 09: 0000 bx lr
00000010 <second>:
 10: 0000 vpush {d8-d15}
 14: 0000 sub sp, #0x10
""")
        self.assertEqual([frame["bytes"] for frame in result], [120, 80])


if __name__ == "__main__":
    unittest.main()
