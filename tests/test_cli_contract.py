import json
import subprocess
import sys
import unittest
from pathlib import Path


class CliContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = Path(sys.argv.pop(1)).resolve()
        cls.graph = Path(sys.argv.pop(1)).resolve()
        cls.common = [
            str(cls.binary), "--graph", str(cls.graph), "--goal-ids", "1,2,3",
            "--depot", "0", "--tests", "1", "--seed", "123", "--aco-seed", "456",
            "--ants", "2", "--iterations", "2", "--threads", "1",
            "--output-format", "jsonl",
        ]

    def run_solver(self, *extra):
        completed = subprocess.run(
            [*self.common, *extra], check=True, text=True, capture_output=True, timeout=30
        )
        return [json.loads(line) for line in completed.stdout.splitlines() if line.strip()]

    def test_paired_solver_identity_and_validation(self):
        records = self.run_solver("--solver", "both", "--replicates", "2")
        self.assertEqual(len(records), 4)
        for replicate in range(2):
            pair = [record for record in records if record["replicate"] == replicate]
            self.assertEqual({record["solver"] for record in pair}, {"vrp", "svrp"})
            self.assertEqual(len({record["scenario_id"] for record in pair}), 1)
            self.assertEqual(len({record["scenario_seed"] for record in pair}), 1)
            self.assertEqual(len({record["aco_seed"] for record in pair}), 1)
            for record in pair:
                self.assertEqual(record["status"], "ok")
                self.assertEqual(record["invalid_candidates"], 0)
                self.assertTrue(record["pareto"])

    def test_vrp_ignores_svrp_only_options(self):
        baseline = self.run_solver("--solver", "vrp")[0]
        flagged = self.run_solver(
            "--solver", "vrp", "--svrp-pass-through-candidates", "20",
            "--svrp-compare-planned", "--svrp-count-pass-through-goals",
        )[0]
        for record in (baseline, flagged):
            record.pop("timing_seconds")
        self.assertEqual(baseline, flagged)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
