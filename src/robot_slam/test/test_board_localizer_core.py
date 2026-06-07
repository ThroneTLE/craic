#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCRIPTS = os.path.join(ROOT, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from board_localizer_core import (  # noqa: E402
    FieldGrid,
    estimate_translation_correction,
    snap_points_to_grid,
)


class BoardLocalizerCoreTest(unittest.TestCase):
    def test_snap_points_to_grid_keeps_repeated_internal_lines(self):
        grid = FieldGrid(origin_x=0.0, origin_y=0.0, cell_size=0.39,
                         cols=9, rows=9, internal_only=True)
        points = []
        for y in [0.42, 0.58, 0.77, 0.95, 1.11]:
            points.append((1.17 + 0.018, y))
        for x in [0.48, 0.70, 0.92, 1.08, 1.31]:
            points.append((x, 0.78 - 0.021))

        boards = snap_points_to_grid(points, grid, max_snap_dist=0.06,
                                     min_points_per_line=4)

        self.assertEqual([1.17], [round(b.position, 2) for b in boards["vertical"]])
        self.assertEqual([0.78], [round(b.position, 2) for b in boards["horizontal"]])

    def test_estimate_translation_correction_uses_median_residual(self):
        grid = FieldGrid(origin_x=0.0, origin_y=0.0, cell_size=0.39,
                         cols=9, rows=9, internal_only=True)
        survey_points = [(1.17, 0.5), (1.17, 0.8), (1.17, 1.1),
                         (0.5, 0.78), (0.8, 0.78), (1.1, 0.78)]
        boards = snap_points_to_grid(survey_points, grid, max_snap_dist=0.03,
                                     min_points_per_line=3)

        current_points = [(1.21, 0.5), (1.21, 0.8), (1.21, 1.1),
                          (0.5, 0.73), (0.8, 0.73), (1.1, 0.73)]
        correction = estimate_translation_correction(
            current_points, boards, max_match_dist=0.10, min_points=3)

        self.assertTrue(correction.valid)
        self.assertAlmostEqual(-0.04, correction.dx, places=3)
        self.assertAlmostEqual(0.05, correction.dy, places=3)
        self.assertGreaterEqual(correction.confidence, 0.5)


if __name__ == "__main__":
    unittest.main()
