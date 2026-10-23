"""Exercise generated jobs using local stand-ins; never contact Slurm."""

import json
import copy
import hashlib
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class SubmissionScriptsTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="aco-slurm-")
        self.addCleanup(self.directory.cleanup)
        self.work = Path(self.directory.name)
        self.capture = self.work / "jobs"
        self.capture.mkdir()
        commands = self.work / "bin"
        commands.mkdir()
        self.write_command(commands / "sbatch", """
import os, shutil
from pathlib import Path
capture = Path(os.environ['JOB_CAPTURE'])
target = capture / ('job_%03d.sh' % len(list(capture.glob('*.sh'))))
shutil.copyfile(sys.argv[1], target)
print('Submitted batch job 123')
""")
        self.write_command(commands / "srun", """
import json, os
args = sys.argv[1:]
assert args.pop(0) == '--cpu-bind=cores'
with open(os.environ['RUN_CAPTURE'], 'a') as output:
    output.write(json.dumps(args) + '\\n')
solver = args[args.index('--solver') + 1]
sys.exit(7 if solver == os.environ.get('FAIL_SOLVER') else 0)
""")
        for size in (250, 500):
            graphs = self.work / f"u{size}"
            graphs.mkdir()
            (graphs / f"graph_000_n{size}_c0_u1_seed1.csv").write_text(
                f"{size},0\nfromId,toId,d,c1\n"
            )
        self.env = {
            **os.environ,
            "PATH": str(commands) + os.pathsep + os.environ["PATH"],
            "BINARY": "/usr/bin/true",
            "PYTHON": sys.executable,
            "U250_GRAPH_DIR": str(self.work / "u250"),
            "U500_GRAPH_DIR": str(self.work / "u500"),
            "U250_GOALS": "2",
            "U500_GOALS": "2",
            "GRAPH_DIR": str(self.work / "u500"),
            "SCENARIOS": "2",
            "REPLICATES": "2",
            "RESULT_ROOT": str(self.work / "outputs"),
            "LOG_ROOT": str(self.work / "logs"),
            "ERROR_ROOT": str(self.work / "errors"),
            "SCENARIO_ROOT": str(self.work / "scenarios"),
            "TMP_ROOT": str(self.work / "scratch"),
            "JOB_CAPTURE": str(self.capture),
            "RUN_CAPTURE": str(self.work / "runs.jsonl"),
            "DRY_RUN": "false",
            "SKIP_COMPLETE": "true",
        }

    def write_command(self, path, body):
        path.write_text(f"#!{sys.executable}\nimport sys\n" + body)
        path.chmod(0o755)

    def submit(self, script, **overrides):
        result = subprocess.run(
            ["bash", str(ROOT / script)], cwd=self.work,
            env={**self.env, **overrides}, text=True, capture_output=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return sorted(self.capture.glob("*.sh"))

    def run_job(self, job, **overrides):
        subprocess.run(["bash", "-n", str(job)], check=True)
        result = subprocess.run(
            ["bash", str(job)], env={**self.env, "SLURM_CPUS_PER_TASK": "2",
                                   "SLURM_JOB_ID": "123", **overrides},
            text=True, capture_output=True, timeout=30,
        )
        capture = self.work / "runs.jsonl"
        calls = [json.loads(line) for line in capture.read_text().splitlines()] if capture.exists() else []
        return result, calls

    def test_journal_matrix_and_generated_job(self):
        jobs = self.submit("submit_journal_experiments.sh")
        self.assertEqual(len(jobs), 2)
        result, calls = self.run_job(jobs[0])
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(len(calls), 17)
        configurations = set()
        for args in calls:
            value = lambda name: args[args.index(name) + 1]
            configurations.add((value("--solver"), int(value("--ants")),
                                int(value("--svrp-pass-through-candidates")),
                                "--svrp-compare-planned" in args))
            self.assertEqual(value("--output-format"), "jsonl")
            self.assertEqual(value("--aco-seed"), "20260622")
            self.assertEqual(value("--replicates"), "2")
            self.assertTrue(Path(value("--scenario-manifest")).is_file())
        expected = {("vrp", ants, 0, False) for ants in (5, 10, 20)}
        expected |= {("svrp", ants, candidates, False)
                     for ants in (5, 10, 20) for candidates in (0, 5, 10, 20)}
        expected |= {("svrp", 10, candidates, True) for candidates in (0, 20)}
        self.assertEqual(configurations, expected)
        snapshots = list((self.work / "logs").rglob("provenance_123.json"))
        self.assertEqual(len(snapshots), 1)
        snapshot = json.loads(snapshots[0].read_text())
        self.assertEqual(snapshot["kind"], "current_run_snapshot")
        self.assertEqual(snapshot["binary"]["sha256"],
                         hashlib.sha256(Path(self.env["BINARY"]).read_bytes()).hexdigest())
        self.assertFalse((self.work / "scratch" / "job_123").exists())

    def test_journal_failure_is_reported_after_remaining_configs_run(self):
        jobs = self.submit("submit_journal_experiments.sh")
        result, calls = self.run_job(jobs[0], FAIL_SOLVER="vrp")
        self.assertEqual(result.returncode, 1)
        self.assertEqual(len(calls), 17)
        self.assertIn("failures: 3", result.stdout)

    def test_legacy_generated_job(self):
        jobs = self.submit("submit_vrp_experiments.sh")
        self.assertEqual(len(jobs), 27)
        result, calls = self.run_job(jobs[0])
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(len(calls), 1)
        self.assertIn("--graph", calls[0])
        self.assertIn("--output", calls[0])
        result, _ = self.run_job(jobs[0], FAIL_SOLVER="both")
        self.assertNotEqual(result.returncode, 0)

    def test_dry_runs_never_submit_jobs(self):
        for script in ("submit_journal_experiments.sh", "submit_vrp_experiments.sh"):
            with self.subTest(script=script):
                jobs = self.submit(script, DRY_RUN="true")
                self.assertEqual(jobs, [])

    def test_resume_rejects_malformed_duplicate_and_stale_records(self):
        script = (ROOT / "submit_journal_experiments.sh").read_text()
        validator = "is_complete() {" + script.split("is_complete() {", 1)[1].split(
            "\nsubmit_graph_goal_bundle()", 1
        )[0]
        graph = self.work / "u250/graph_000_n250_c0_u1_seed1.csv"
        manifest = self.work / "manifest.csv"
        # Seed taken from the saved cluster run for this opaque scenario ID.
        identifier = "graph_005_n250_c0_u1_seed41344-g100-s01"
        manifest.write_text("scenario_id,depot,goals,scenario_seed\n"
                            + identifier + ",0,1|2,123\n")
        record = {
            "schema_version": 1, "scenario_id": identifier, "replicate": 0,
            "solver": "vrp", "graph_path": str(graph), "depot": 0,
            "goals": [1, 2], "scenario_seed": 123, "aco_seed": 2827309998,
            "parameters": {"ants": 5, "pass_through_candidates": 0,
                           "compare_planned": False, "threads": 2,
                           "stagnation_limit": 20, "battery": 50000,
                           "k_cover": 24, "max_vehicles": 2,
                           "count_pass_through_goals": False},
            "status": "ok", "pareto": [{"objectives": [10, 20], "route_count": 1}],
        }
        result_file = self.work / "results.jsonl"

        def check(content, replicates="1"):
            result_file.write_text(content)
            completed = subprocess.run(
                ["bash", "-c", validator + '\nis_complete "$@"', "validator",
                 str(result_file), str(manifest), str(graph), "2", "vrp", "5", "0", "false"],
                env={**self.env, "REPLICATES": replicates, "THREADS": "2",
                     "ITERATIONS": "20", "BATTERY": "50000", "K_COVER": "24",
                     "ACO_SEED": "20260622"},
                text=True, capture_output=True, timeout=10,
            )
            return completed.returncode

        valid = json.dumps(record) + "\n"
        self.assertEqual(check(valid), 0)
        self.assertNotEqual(check("{}\n"), 0)
        self.assertNotEqual(check("not-json\n"), 0)
        self.assertNotEqual(check(valid * 2, replicates="2"), 0)
        self.assertNotEqual(check(valid, replicates="2"), 0)
        for field in ("aco_seed", "scenario_seed"):
            stale = copy.deepcopy(record)
            stale[field] += 1
            self.assertNotEqual(check(json.dumps(stale) + "\n"), 0)
        stale = copy.deepcopy(record)
        stale["parameters"]["battery"] += 1
        self.assertNotEqual(check(json.dumps(stale) + "\n"), 0)


if __name__ == "__main__":
    unittest.main()
