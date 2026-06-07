#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

import math
from collections import defaultdict, namedtuple


BoardLine = namedtuple("BoardLine", ["orientation", "position", "count", "spread_min", "spread_max"])
PoseCorrection = namedtuple("PoseCorrection", ["valid", "dx", "dy", "yaw", "confidence", "match_count"])


class FieldGrid(object):
    def __init__(self, origin_x, origin_y, cell_size, cols, rows, internal_only=True):
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.cell_size = float(cell_size)
        self.cols = int(cols)
        self.rows = int(rows)
        self.internal_only = bool(internal_only)

    def vertical_lines(self):
        start = 1 if self.internal_only else 0
        end = self.cols if self.internal_only else self.cols + 1
        return [self.origin_x + i * self.cell_size for i in range(start, end)]

    def horizontal_lines(self):
        start = 1 if self.internal_only else 0
        end = self.rows if self.internal_only else self.rows + 1
        return [self.origin_y + i * self.cell_size for i in range(start, end)]


def _nearest(value, candidates):
    if not candidates:
        return None, float("inf")
    best = min(candidates, key=lambda c: abs(value - c))
    return best, abs(value - best)


def _median(values):
    if not values:
        return 0.0
    ordered = sorted(values)
    n = len(ordered)
    mid = n // 2
    if n % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def snap_points_to_grid(points, grid, max_snap_dist=0.06, min_points_per_line=4):
    vertical_bins = defaultdict(list)
    horizontal_bins = defaultdict(list)
    vertical_lines = grid.vertical_lines()
    horizontal_lines = grid.horizontal_lines()

    for x, y in points:
        line_x, dist_x = _nearest(x, vertical_lines)
        if dist_x <= max_snap_dist:
            vertical_bins[line_x].append((x, y))

        line_y, dist_y = _nearest(y, horizontal_lines)
        if dist_y <= max_snap_dist:
            horizontal_bins[line_y].append((x, y))

    vertical = []
    for line_x, ps in vertical_bins.items():
        if len(ps) < min_points_per_line:
            continue
        ys = [p[1] for p in ps]
        vertical.append(BoardLine("vertical", line_x, len(ps), min(ys), max(ys)))

    horizontal = []
    for line_y, ps in horizontal_bins.items():
        if len(ps) < min_points_per_line:
            continue
        xs = [p[0] for p in ps]
        horizontal.append(BoardLine("horizontal", line_y, len(ps), min(xs), max(xs)))

    vertical.sort(key=lambda b: b.position)
    horizontal.sort(key=lambda b: b.position)
    return {"vertical": vertical, "horizontal": horizontal}


def _point_overlaps_line(point_coord, line):
    return line.spread_min - 0.08 <= point_coord <= line.spread_max + 0.08


def estimate_translation_correction(points, boards, max_match_dist=0.10, min_points=5):
    x_residuals = []
    y_residuals = []

    vertical = boards.get("vertical", [])
    horizontal = boards.get("horizontal", [])

    for x, y in points:
        candidates = [b for b in vertical if _point_overlaps_line(y, b)]
        if candidates:
            line = min(candidates, key=lambda b: abs(x - b.position))
            residual = x - line.position
            if abs(residual) <= max_match_dist:
                x_residuals.append(residual)

        candidates = [b for b in horizontal if _point_overlaps_line(x, b)]
        if candidates:
            line = min(candidates, key=lambda b: abs(y - b.position))
            residual = y - line.position
            if abs(residual) <= max_match_dist:
                y_residuals.append(residual)

    match_count = len(x_residuals) + len(y_residuals)
    if match_count < min_points:
        return PoseCorrection(False, 0.0, 0.0, 0.0, 0.0, match_count)

    dx = -_median(x_residuals) if x_residuals else 0.0
    dy = -_median(y_residuals) if y_residuals else 0.0
    confidence = min(1.0, match_count / float(max(min_points, 1) * 2))
    return PoseCorrection(True, dx, dy, 0.0, confidence, match_count)


def transform_points(points, x, y, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    out = []
    for px, py in points:
        out.append((x + cos_yaw * px - sin_yaw * py,
                    y + sin_yaw * px + cos_yaw * py))
    return out
