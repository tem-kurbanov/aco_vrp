import tempfile
import unittest
from pathlib import Path

from analyze_ilp_outputs import parse_block, parse_file


COMPLETE = """Test 1
Goals: 1, 2,
Depot: 0
K: 2; Vertices: 3; Edges: 4; Duration: 0.1
K: 4; Vertices: 3; Edges: 5; Duration: 0.2
Shrinked: Vertices: 3 Edges: 8 Duration: 0.3
Original: (10,20); Duration: 1.0
Cover 2: (10,20); Duration: 0.5
Cover 4:  Duration: 3600.0
Shrinked: (11,21); Duration: 0.4
"""


class IlpParserTest(unittest.TestCase):
    def test_complete_and_timeout_block(self):
        block = parse_block(COMPLETE, 1)
        self.assertTrue(block.complete)
        self.assertEqual(block.complete_graph[1], 8)
        self.assertEqual(block.solutions["Original"].front, [(10.0, 20.0)])
        self.assertEqual(block.solutions["Cover 4"].front, [])

    def test_truncated_block(self):
        block = parse_block(COMPLETE.rsplit("Shrinked:", 1)[0], 1)
        self.assertFalse(block.complete)

    def test_appended_restart_keeps_last_complete_block(self):
        first = COMPLETE.replace("Original: (10,20)", "Original: (99,99)")
        second = COMPLETE.replace("Original: (10,20)", "Original: (7,8)")
        truncated = COMPLETE.replace("Test 1", "Test 2").rsplit("Shrinked:", 1)[0]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "restart.stdout"
            path.write_text(first + second + truncated)
            blocks, audit = parse_file(path)
        self.assertEqual(len(blocks), 1)
        self.assertEqual(blocks[0].solutions["Original"].front, [(7.0, 8.0)])
        self.assertEqual(audit["duplicate_complete_blocks"], 1)
        self.assertEqual(audit["rejected_blocks"], 1)


if __name__ == "__main__":
    unittest.main()
