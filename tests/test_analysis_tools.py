import math
import unittest

from analysis_tools import (
    additive_epsilon,
    coverage,
    hierarchical_bootstrap_ci,
    holm_adjust,
    hypervolume_2d,
    nondominated,
    normalize,
    objective_bounds,
)


class AnalysisToolsTest(unittest.TestCase):
    def test_pareto_metrics(self):
        front = [(0.0, 1.0), (1.0, 0.0), (1.0, 1.0)]
        self.assertEqual(nondominated(front), [(0.0, 1.0), (1.0, 0.0)])
        self.assertAlmostEqual(hypervolume_2d(front), 0.21)
        self.assertEqual(coverage([(0.0, 0.0)], [(1.0, 1.0)]), 1.0)
        self.assertEqual(additive_epsilon([(0.0, 0.0)], [(1.0, 1.0)]), -1.0)

    def test_normalization(self):
        ideal, nadir = objective_bounds([[(10, 30)], [(20, 20)]])
        self.assertEqual(normalize([(15, 25)], ideal, nadir), [(0.5, 0.5)])

    def test_statistics(self):
        low, high = hierarchical_bootstrap_ci({"a": [1, 1], "b": [3, 3]}, samples=200)
        self.assertLessEqual(low, 2.0)
        self.assertGreaterEqual(high, 2.0)
        adjusted = holm_adjust([0.01, 0.04, math.nan])
        self.assertAlmostEqual(adjusted[0], 0.02)
        self.assertAlmostEqual(adjusted[1], 0.04)
        self.assertTrue(math.isnan(adjusted[2]))


if __name__ == "__main__":
    unittest.main()
