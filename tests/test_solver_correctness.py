"""Small road graphs with objectives and feasibility derived by hand."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

BIN = None


class SolverCorrectnessTest(unittest.TestCase):
    def solve(self, nodes, edges, goals, battery, vehicles=1, depot=0):
        with tempfile.TemporaryDirectory(prefix="aco-correctness-") as directory:
            graph = Path(directory) / "graph.csv"
            graph.write_text(
                f"{nodes},{len(edges)}\nfromId,toId,d,c1\n"
                + "".join(",".join(map(str, edge)) + "\n" for edge in edges)
            )
            completed = subprocess.run(
                [str(BIN), "--graph", str(graph), "--goal-ids", ",".join(map(str, goals)),
                 "--depot", str(depot), "--battery", str(battery), "--vehicles", str(vehicles),
                 "--tests", "1", "--seed", "123", "--aco-seed", "456", "--ants", "4",
                 "--iterations", "2", "--threads", "1", "--k-cover", "2",
                 "--solver", "both", "--output-format", "jsonl"],
                check=True, text=True, capture_output=True, timeout=15,
            )
        records = [json.loads(line) for line in completed.stdout.splitlines()]
        self.assertEqual({r["solver"] for r in records}, {"vrp", "svrp"})
        self.assertEqual(len(records), 2)
        return records

    def assert_front(self, records, expected):
        for record in records:
            with self.subTest(solver=record["solver"]):
                self.assertEqual(record["status"], "ok")
                self.assertEqual(record["invalid_candidates"], 0)
                self.assertEqual(
                    {(tuple(p["objectives"]), p["route_count"]) for p in record["pareto"]},
                    expected,
                )

    def test_asymmetric_round_trip_at_battery_limit(self):
        # The only tour is 0 -> 1 -> 0: costs (7+11, 4+6).
        self.assert_front(self.solve(2, [(0, 1, 7, 4), (1, 0, 11, 6)], [1], 10),
                          {((18, 10), 1)})

    def test_insufficient_battery_produces_no_solution(self):
        records = self.solve(2, [(0, 1, 7, 4), (1, 0, 11, 6)], [1], 9)
        for record in records:
            self.assertEqual(record["status"], "no_solution")
            self.assertEqual(record["pareto"], [])

    def test_each_goal_is_served_with_a_battery_reset_at_depot(self):
        # Separate tours cost (4,4) and (6,6); a combined tour exceeds B=6.
        edges = [(0, 1, 2, 2), (1, 0, 2, 2), (0, 2, 3, 3), (2, 0, 3, 3)]
        self.assert_front(self.solve(3, edges, [1, 2], 6, vehicles=2), {((10, 10), 2)})

    def test_vehicle_limit_makes_two_required_tours_infeasible(self):
        edges = [(0, 1, 2, 2), (1, 0, 2, 2), (0, 2, 3, 3), (2, 0, 3, 3)]
        for record in self.solve(3, edges, [1, 2], 6, vehicles=1):
            self.assertEqual(record["status"], "no_solution")
            self.assertEqual(record["pareto"], [])

    def test_nonzero_depot_and_noncustomer_transit(self):
        # Depot 2 reaches customer 0 through noncustomer 1, then returns.
        edges = [(2, 1, 2, 3), (1, 2, 2, 3), (1, 0, 5, 4), (0, 1, 5, 4)]
        self.assert_front(self.solve(3, edges, [0], 14, depot=2), {((14, 14), 1)})

    def test_customer_without_a_return_path_is_disconnected(self):
        for record in self.solve(2, [(0, 1, 7, 4)], [1], 10):
            self.assertEqual(record["status"], "disconnected")
            self.assertEqual(record["pareto"], [])

    def test_conflicting_parallel_arcs_preserve_both_objective_extremes(self):
        # Two possible tours: (1+1,5+1) and (5+1,1+1), neither dominates.
        edges = [(0, 1, 1, 5), (0, 1, 5, 1), (1, 0, 1, 1)]
        self.assert_front(self.solve(2, edges, [1], 6), {((2, 6), 1), ((6, 2), 1)})


if __name__ == "__main__":
    BIN = Path(sys.argv.pop(1)).resolve()
    unittest.main()
