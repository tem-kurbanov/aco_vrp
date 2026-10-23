import json
import tempfile
import unittest
from pathlib import Path

from analyze_journal_aco import enrich_records, load_records, pair_rows, validate_record


def fixture(solver="vrp", edges=100, points=((10, 20),), planned=False):
    return {
        "schema_version": 1,
        "scenario_id": "scenario-001",
        "graph_path": "graph.csv",
        "graph_size": 250,
        "scenario_seed": 123,
        "replicate": 0,
        "aco_seed": 456,
        "depot": 0,
        "goals": [1, 2],
        "solver": solver,
        "status": "ok",
        "message": "",
        "parameters": {
            "ants": 10,
            "pass_through_candidates": 20 if solver == "svrp" else 0,
            "compare_planned": planned,
        },
        "graph": {"nodes": 10, "edges": edges},
        "timing_seconds": {"preprocessing": 1.0, "optimization": 2.0, "total": 3.0},
        "iterations": 20,
        "pareto": [{"objectives": list(point), "route_count": 1} for point in points],
    }


class JournalAcoAnalysisTest(unittest.TestCase):
    def test_validation_and_pairing(self):
        records = [fixture(), fixture("svrp", 25, ((9, 19),))]
        for record in records:
            validate_record(record)
        rows = enrich_records(records)
        pairs = pair_rows(rows)
        self.assertEqual(len(pairs), 1)
        self.assertAlmostEqual(pairs[0]["edge_ratio"], 0.25)
        self.assertGreater(pairs[0]["hypervolume_difference"], 0.0)
        self.assertEqual(pairs[0]["coverage_svrp_to_vrp"], 1.0)

    def test_loader_audits_malformed_and_duplicate_records(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "results.jsonl"
            record = fixture()
            path.write_text(json.dumps(record) + "\nnot-json\n" + json.dumps(record) + "\n")
            records, audit = load_records(Path(directory), expected_lines=3)
            self.assertEqual(len(records), 1)
            self.assertEqual(audit[0]["malformed_records"], 1)
            self.assertEqual(audit[0]["duplicate_records"], 1)
            self.assertEqual(audit[0]["complete"], 0)


if __name__ == "__main__":
    unittest.main()
