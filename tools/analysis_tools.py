#!/usr/bin/env python3
"""Shared Pareto metrics and statistical helpers for journal analyses."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence

import numpy as np

Point = tuple[float, float]


def nondominated(points: Iterable[Point]) -> list[Point]:
    unique = sorted(set(points))
    result: list[Point] = []
    for index, point in enumerate(unique):
        if not any(
            other[0] <= point[0]
            and other[1] <= point[1]
            and other != point
            for other_index, other in enumerate(unique)
            if other_index != index
        ):
            result.append(point)
    return result


def objective_bounds(fronts: Iterable[Iterable[Point]]) -> tuple[Point, Point]:
    points = [point for front in fronts for point in front]
    if not points:
        raise ValueError("cannot compute bounds without Pareto points")
    return (
        (min(point[0] for point in points), min(point[1] for point in points)),
        (max(point[0] for point in points), max(point[1] for point in points)),
    )


def normalize(points: Iterable[Point], ideal: Point, nadir: Point) -> list[Point]:
    ranges = (nadir[0] - ideal[0], nadir[1] - ideal[1])
    return [
        (
            (point[0] - ideal[0]) / ranges[0] if ranges[0] > 0 else 0.0,
            (point[1] - ideal[1]) / ranges[1] if ranges[1] > 0 else 0.0,
        )
        for point in points
    ]


def hypervolume_2d(points: Iterable[Point], reference: Point = (1.1, 1.1)) -> float:
    front = nondominated(points)
    volume = 0.0
    for index, point in enumerate(front):
        next_first = front[index + 1][0] if index + 1 < len(front) else reference[0]
        width = next_first - point[0]
        height = reference[1] - point[1]
        if width > 0 and height > 0:
            volume += width * height
    return volume


def coverage(source: Iterable[Point], target: Iterable[Point]) -> float:
    source_front = list(source)
    target_front = list(target)
    if not target_front:
        return math.nan
    covered = sum(
        any(left[0] <= right[0] and left[1] <= right[1] for left in source_front)
        for right in target_front
    )
    return covered / len(target_front)


def additive_epsilon(source: Iterable[Point], target: Iterable[Point]) -> float:
    source_front = list(source)
    target_front = list(target)
    if not source_front or not target_front:
        return math.nan
    return max(
        min(max(left[0] - right[0], left[1] - right[1]) for left in source_front)
        for right in target_front
    )


def mean(values: Iterable[float]) -> float:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    return float(np.mean(finite)) if finite else math.nan


def hierarchical_bootstrap_ci(
    values_by_cluster: Mapping[str, Sequence[float]],
    samples: int = 10_000,
    seed: int = 20260622,
) -> tuple[float, float]:
    clusters = sorted(values_by_cluster)
    if not clusters:
        return math.nan, math.nan
    cluster_values = {
        cluster: np.asarray(
            [value for value in values_by_cluster[cluster] if math.isfinite(value)],
            dtype=float,
        )
        for cluster in clusters
    }
    cluster_values = {key: value for key, value in cluster_values.items() if value.size}
    clusters = sorted(cluster_values)
    if not clusters:
        return math.nan, math.nan

    rng = np.random.default_rng(seed)
    estimates = np.empty(samples, dtype=float)
    for index in range(samples):
        selected = rng.choice(clusters, size=len(clusters), replace=True)
        cluster_means = []
        for cluster in selected:
            values = cluster_values[str(cluster)]
            sampled = rng.choice(values, size=values.size, replace=True)
            cluster_means.append(float(np.mean(sampled)))
        estimates[index] = float(np.mean(cluster_means))
    lower, upper = np.quantile(estimates, (0.025, 0.975))
    return float(lower), float(upper)


def holm_adjust(p_values: Sequence[float]) -> list[float]:
    adjusted = [math.nan] * len(p_values)
    finite_indices = [index for index, value in enumerate(p_values) if math.isfinite(value)]
    ordered = sorted(finite_indices, key=lambda index: p_values[index])
    running = 0.0
    count = len(ordered)
    for rank, index in enumerate(ordered):
        candidate = min(1.0, (count - rank) * p_values[index])
        running = max(running, candidate)
        adjusted[index] = running
    return adjusted
