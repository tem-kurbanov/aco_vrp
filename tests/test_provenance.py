import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from verify_instances import verify_one

ROOT = Path(__file__).resolve().parents[1]


class ProvenanceTest(unittest.TestCase):
    def test_tampered_input_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "graph.csv"
            data = b"2,2\nfromId,toId,d,c1\n0,1,2,3\n1,0,2,3\n"
            path.write_bytes(data)
            entry = dict(path="graph.csv", nodes=2, edges=2,
                         header=["fromId", "toId", "d", "c1"],
                         sha256=hashlib.sha256(data).hexdigest())
            self.assertEqual(verify_one((entry, directory, None)), "graph.csv")
            path.write_bytes(data.replace(b"0,1,2,3", b"0,1,9,3"))
            with self.assertRaisesRegex(ValueError, "Checksum mismatch"):
                verify_one((entry, directory, None))

    def test_capture_identifies_binary_bytes_and_unknown_build_information(self):
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "fixture_binary"
            binary.write_bytes(b"test binary bytes")
            output = Path(directory) / "provenance.json"
            subprocess.run([sys.executable, str(ROOT / "tools/capture_provenance.py"),
                            "--binary", str(binary), "--output", str(output)], check=True)
            report = json.loads(output.read_text())
            self.assertEqual(report["kind"], "current_run_snapshot")
            self.assertEqual(report["binary"]["sha256"], hashlib.sha256(binary.read_bytes()).hexdigest())
            self.assertEqual(report["source_sha256"]["Graph.cpp"],
                             hashlib.sha256((ROOT / "Graph.cpp").read_bytes()).hexdigest())
            self.assertEqual(report["build_cache"], {})
            self.assertIsNone(report["compiler_version"])


if __name__ == "__main__":
    unittest.main()
