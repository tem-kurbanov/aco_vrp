#!/usr/bin/env python3
"""Aggregate ACO VRP experiment output into CSV and Markdown tables."""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from collections import defaultdict
from pathlib import Path
from statistics import fmean


METRIC_NAMES = (
    "hv_svrp",
    "hv_vrp",
    "coverage_vrp_to_svrp",
    "coverage_svrp_to_vrp",
    "epsilon_svrp_to_vrp",
    "epsilon_vrp_to_svrp",
)

BASE_COLUMNS = (
    "graph_size",
    "goals",
    "ants",
    "pass_through",
    "threads",
    "graph_configs",
    "result_files",
    "runs",
    "expected_runs",
    "completion_percent",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Aggregate outputs produced by submit_vrp_experiments.sh"
    )
    parser.add_argument(
        "input_dir",
        nargs="?",
        type=Path,
        default=Path("outputs/vrp"),
        help="experiment output directory (default: outputs/vrp)",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=Path("analysis/vrp"),
        help="directory for generated tables (default: analysis/vrp)",
    )
    return parser.parse_args()


def prefixed_value(parts: tuple[str, ...], prefix: str) -> str:
    matches = [part[len(prefix) :] for part in parts if part.startswith(prefix)]
    if not matches:
        raise ValueError(f"missing '{prefix}*' path component")
    return matches[-1]


def parse_file_metadata(path: Path) -> dict[str, object]:
    parts = path.parts
    graph_config = prefixed_value(parts, "graph_")
    size_match = re.search(r"(?:^|_)n(\d+)(?:_|$)", graph_config)
    if not size_match:
        raise ValueError(f"cannot find graph size in '{graph_config}'")

    filename_match = re.fullmatch(r"tests_(\d+)_threads_(\d+)\.txt", path.name)
    if not filename_match:
        raise ValueError("filename must match tests_<n>_threads_<n>.txt")

    return {
        "graph_config": graph_config,
        "graph_size": int(size_match.group(1)),
        "goals": int(prefixed_value(parts, "goals_")),
        "ants": int(prefixed_value(parts, "ants_")),
        "pass_through": int(prefixed_value(parts, "pass_")),
        "expected_tests": int(filename_match.group(1)),
        "threads": int(filename_match.group(2)),
    }


def parse_solver_line(line: str) -> dict[str, float | int]:
    header = line.split(maxsplit=1)[0]
    fields = header.split(",")
    if len(fields) != 3:
        raise ValueError("solver line must start with edges,iterations,duration")
    return {
        "edges": int(fields[0]),
        "iterations": int(fields[1]),
        "time_seconds": float(fields[2]),
    }


def parse_metrics_line(line: str) -> dict[str, float]:
    if not line.startswith("metrics "):
        raise ValueError("expected metrics line")
    values: dict[str, float] = {}
    for field in line[len("metrics ") :].split(","):
        name, separator, value = field.partition("=")
        if not separator:
            raise ValueError(f"invalid metric field '{field}'")
        values[name] = float(value)
    missing = set(METRIC_NAMES) - values.keys()
    if missing:
        raise ValueError(f"missing metrics: {', '.join(sorted(missing))}")
    return values


def parse_result_file(path: Path) -> tuple[dict[str, object], list[dict[str, object]]]:
    metadata = parse_file_metadata(path)
    lines = [line.strip() for line in path.read_text().splitlines() if line.strip()]
    if len(lines) % 3 != 0:
        raise ValueError(f"expected complete three-line blocks, found {len(lines)} lines")

    records: list[dict[str, object]] = []
    for offset in range(0, len(lines), 3):
        svrp = parse_solver_line(lines[offset])
        vrp = parse_solver_line(lines[offset + 1])
        metrics = parse_metrics_line(lines[offset + 2])
        record = dict(metadata)
        record.update(
            {
                "source_file": str(path),
                "svrp_edges": svrp["edges"],
                "vrp_edges": vrp["edges"],
                "svrp_iterations": svrp["iterations"],
                "vrp_iterations": vrp["iterations"],
                "svrp_time_seconds": svrp["time_seconds"],
                "vrp_time_seconds": vrp["time_seconds"],
            }
        )
        record.update(metrics)
        records.append(record)
    return metadata, records


def finite_mean(records: list[dict[str, object]], name: str) -> float:
    values = [float(record[name]) for record in records]
    finite_values = [value for value in values if math.isfinite(value)]
    return fmean(finite_values) if finite_values else math.nan


def aggregate(
    records: list[dict[str, object]], file_metadata: list[dict[str, object]]
) -> list[dict[str, object]]:
    key_names = ("graph_size", "goals", "ants", "pass_through", "threads")
    record_groups: dict[tuple[int, ...], list[dict[str, object]]] = defaultdict(list)
    file_groups: dict[tuple[int, ...], list[dict[str, object]]] = defaultdict(list)

    for record in records:
        key = tuple(int(record[name]) for name in key_names)
        record_groups[key].append(record)
    for metadata in file_metadata:
        key = tuple(int(metadata[name]) for name in key_names)
        file_groups[key].append(metadata)

    rows: list[dict[str, object]] = []
    for key in sorted(file_groups):
        group_records = record_groups.get(key, [])
        group_files = file_groups[key]
        expected_runs = sum(int(item["expected_tests"]) for item in group_files)
        row: dict[str, object] = dict(zip(key_names, key))
        row.update(
            {
                "graph_configs": len(
                    {str(item["graph_config"]) for item in group_files}
                ),
                "result_files": len(group_files),
                "runs": len(group_records),
                "expected_runs": expected_runs,
                "completion_percent": (
                    100.0 * len(group_records) / expected_runs if expected_runs else 0.0
                ),
            }
        )

        for name in (
            "svrp_edges",
            "vrp_edges",
            "svrp_iterations",
            "vrp_iterations",
            "svrp_time_seconds",
            "vrp_time_seconds",
            *METRIC_NAMES,
        ):
            row[f"avg_{name}"] = finite_mean(group_records, name)

        vrp_edges = float(row["avg_vrp_edges"])
        svrp_time = float(row["avg_svrp_time_seconds"])
        row["svrp_vrp_edge_ratio"] = (
            float(row["avg_svrp_edges"]) / vrp_edges if vrp_edges else math.nan
        )
        row["vrp_svrp_time_ratio"] = (
            float(row["avg_vrp_time_seconds"]) / svrp_time
            if svrp_time
            else math.nan
        )
        rows.append(row)
    return rows


def write_csv(path: Path, rows: list[dict[str, object]], columns: tuple[str, ...]) -> None:
    with path.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def display(value: object, digits: int = 4) -> str:
    if isinstance(value, float):
        return f"{value:.{digits}f}" if math.isfinite(value) else "NA"
    return str(value)


def markdown_table(headers: tuple[str, ...], rows: list[tuple[object, ...]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend("| " + " | ".join(display(value) for value in row) + " |" for row in rows)
    return "\n".join(lines)


def write_markdown(path: Path, rows: list[dict[str, object]]) -> None:
    grouped: dict[tuple[int, int], list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[(int(row["graph_size"]), int(row["goals"]))].append(row)

    sections = [
        "# VRP Experiment Summary",
        "",
        "Averages are across all graph configurations and test runs in each group.",
        "The completion column is parsed runs divided by the tests expected from filenames.",
    ]
    for (graph_size, goals), group_rows in sorted(grouped.items()):
        sections.extend(
            [
                "",
                f"## Graph size {graph_size}, goals {goals}",
                "",
                "### Graph Size And Runtime",
                "",
                markdown_table(
                    (
                        "Ants",
                        "Pass",
                        "Threads",
                        "Graphs",
                        "Runs",
                        "Complete %",
                        "SVRP edges",
                        "VRP edges",
                        "SVRP s",
                        "VRP s",
                    ),
                    [
                        (
                            row["ants"],
                            row["pass_through"],
                            row["threads"],
                            row["graph_configs"],
                            row["runs"],
                            row["completion_percent"],
                            row["avg_svrp_edges"],
                            row["avg_vrp_edges"],
                            row["avg_svrp_time_seconds"],
                            row["avg_vrp_time_seconds"],
                        )
                        for row in group_rows
                    ],
                ),
                "",
                "### Quality",
                "",
                markdown_table(
                    (
                        "Ants",
                        "Pass",
                        "HV SVRP",
                        "HV VRP",
                        "Cov V->S",
                        "Cov S->V",
                        "Eps S->V",
                        "Eps V->S",
                    ),
                    [
                        (
                            row["ants"],
                            row["pass_through"],
                            row["avg_hv_svrp"],
                            row["avg_hv_vrp"],
                            row["avg_coverage_vrp_to_svrp"],
                            row["avg_coverage_svrp_to_vrp"],
                            row["avg_epsilon_svrp_to_vrp"],
                            row["avg_epsilon_vrp_to_svrp"],
                        )
                        for row in group_rows
                    ],
                ),
            ]
        )
    path.write_text("\n".join(sections) + "\n")


def main() -> int:
    args = parse_args()
    if not args.input_dir.is_dir():
        print(f"Input directory does not exist: {args.input_dir}", file=sys.stderr)
        return 1

    paths = sorted(args.input_dir.rglob("tests_*_threads_*.txt"))
    if not paths:
        print(f"No result files found under: {args.input_dir}", file=sys.stderr)
        return 1

    records: list[dict[str, object]] = []
    file_metadata: list[dict[str, object]] = []
    errors: list[str] = []
    for path in paths:
        try:
            metadata, file_records = parse_result_file(path)
            file_metadata.append(metadata)
            records.extend(file_records)
            if len(file_records) != int(metadata["expected_tests"]):
                errors.append(
                    f"{path}: parsed {len(file_records)} of "
                    f"{metadata['expected_tests']} expected tests"
                )
        except (OSError, ValueError) as error:
            errors.append(f"{path}: {error}")

    if not records:
        print("No valid experiment runs could be parsed.", file=sys.stderr)
        for error in errors:
            print(f"warning: {error}", file=sys.stderr)
        return 1

    rows = aggregate(records, file_metadata)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    graph_columns = BASE_COLUMNS + (
        "avg_svrp_edges",
        "avg_vrp_edges",
        "svrp_vrp_edge_ratio",
    )
    timing_columns = BASE_COLUMNS + (
        "avg_svrp_time_seconds",
        "avg_vrp_time_seconds",
        "vrp_svrp_time_ratio",
        "avg_svrp_iterations",
        "avg_vrp_iterations",
    )
    quality_columns = BASE_COLUMNS + tuple(f"avg_{name}" for name in METRIC_NAMES)
    all_columns = graph_columns + tuple(
        column for column in timing_columns + quality_columns if column not in graph_columns
    )

    write_csv(args.output_dir / "graph_sizes.csv", rows, graph_columns)
    write_csv(args.output_dir / "solution_times.csv", rows, timing_columns)
    write_csv(args.output_dir / "quality_metrics.csv", rows, quality_columns)
    write_csv(args.output_dir / "summary.csv", rows, all_columns)
    write_markdown(args.output_dir / "summary.md", rows)

    print(
        f"Parsed {len(records)} runs from {len(file_metadata)} files into "
        f"{len(rows)} aggregate rows."
    )
    print(f"Tables written to: {args.output_dir}")
    for error in errors:
        print(f"warning: {error}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
