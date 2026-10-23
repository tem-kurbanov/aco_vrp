#!/usr/bin/env python3
"""Parse, audit and summarize the original ILP experiment logs."""

from __future__ import annotations

import argparse
import csv
import math
import re
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from statistics import fmean

from analysis_tools import coverage, hypervolume_2d, mean as finite_mean, normalize, objective_bounds

POINT_RE = re.compile(r"\((-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)\)")
K_GRAPH_RE = re.compile(
    r"^K: (\d+); Vertices: (\d+); Edges: (\d+); Duration: ([0-9.eE+-]+)$",
    re.MULTILINE,
)
COMPLETE_GRAPH_RE = re.compile(
    r"^Shrinked: Vertices: (\d+) Edges: (\d+) Duration: ([0-9.eE+-]+)$",
    re.MULTILINE,
)
SOLUTION_RE = re.compile(
    r"^(Original|Cover \d+|Shrinked(?!: Vertices)):\s*(.*?)\s*Duration: ([0-9.eE+-]+)$",
    re.MULTILINE,
)


@dataclass
class Solution:
    name: str
    duration: float
    front: list[tuple[float, float]]


@dataclass
class TestBlock:
    test_id: int
    depot: int
    goals: list[int]
    cover_graphs: dict[int, tuple[int, int, float]] = field(default_factory=dict)
    complete_graph: tuple[int, int, float] | None = None
    solutions: dict[str, Solution] = field(default_factory=dict)
    complete: bool = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_dir", nargs="?", type=Path, default=Path("outs"))
    parser.add_argument("-o", "--output-dir", type=Path, default=Path("analysis/journal_ilp"))
    parser.add_argument(
        "--families", nargs="+", default=("u100", "u250"),
        help="subdirectories to include (default: u100 u250)",
    )
    return parser.parse_args()


def parse_points(text: str) -> list[tuple[float, float]]:
    return [(float(first), float(second)) for first, second in POINT_RE.findall(text)]


def parse_block(text: str, test_id: int) -> TestBlock:
    depot_match = re.search(r"^Depot: (\d+)\s*$", text, re.MULTILINE)
    goals_match = re.search(r"^Goals:\s*(.*?)\s*$", text, re.MULTILINE)
    goals = [int(value) for value in re.findall(r"\d+", goals_match.group(1))] if goals_match else []
    block = TestBlock(test_id, int(depot_match.group(1)) if depot_match else -1, goals)

    for k, vertices, edges, duration in K_GRAPH_RE.findall(text):
        block.cover_graphs[int(k)] = (int(vertices), int(edges), float(duration))
    graph_match = COMPLETE_GRAPH_RE.search(text)
    if graph_match:
        block.complete_graph = (
            int(graph_match.group(1)), int(graph_match.group(2)), float(graph_match.group(3))
        )
    for name, front_text, duration in SOLUTION_RE.findall(text):
        block.solutions[name] = Solution(name, float(duration), parse_points(front_text))

    expected_covers = {f"Cover {k}" for k in block.cover_graphs}
    block.complete = bool(
        depot_match
        and goals
        and block.complete_graph
        and block.cover_graphs
        and "Original" in block.solutions
        and "Shrinked" in block.solutions
        and expected_covers.issubset(block.solutions)
    )
    return block


def parse_file(path: Path) -> tuple[list[TestBlock], dict[str, int]]:
    text = path.read_text(errors="replace")
    matches = list(re.finditer(r"^Test (\d+)\s*$", text, re.MULTILINE))
    parsed: list[TestBlock] = []
    for index, match in enumerate(matches):
        end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
        parsed.append(parse_block(text[match.start():end], int(match.group(1))))

    retained: dict[int, TestBlock] = {}
    for block in parsed:
        if block.complete:
            retained[block.test_id] = block
    audit = {
        "blocks": len(parsed),
        "complete_blocks": sum(block.complete for block in parsed),
        "retained_tests": len(retained),
        "duplicate_complete_blocks": max(0, sum(block.complete for block in parsed) - len(retained)),
        "rejected_blocks": sum(not block.complete for block in parsed),
    }
    return [retained[key] for key in sorted(retained)], audit


def file_metadata(path: Path) -> tuple[str, str, int, int]:
    match = re.fullmatch(r"(.+)_n(\d+)_c\d+_u\d+_seed\d+_(\d+)\.stdout", path.name)
    if not match:
        raise ValueError(f"unexpected ILP filename: {path}")
    graph_config = path.name[: path.name.rfind(f"_{match.group(3)}.stdout")]
    return path.parent.name, graph_config, int(match.group(2)), int(match.group(3))


def write_csv(path: Path, rows: list[dict[str, object]], columns: list[str]) -> None:
    with path.open("w", newline="") as destination:
        writer = csv.DictWriter(destination, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def build_rows(input_dir: Path, families: list[str]):
    audit_rows: list[dict[str, object]] = []
    graph_rows: list[dict[str, object]] = []
    solution_rows: list[dict[str, object]] = []
    blocks_by_group: dict[tuple[str, int, int], list[TestBlock]] = defaultdict(list)

    for family in families:
        for path in sorted((input_dir / family).glob("*.stdout")):
            family_name, graph_config, size, goal_count = file_metadata(path)
            blocks, audit = parse_file(path)
            audit_rows.append({"file": str(path), "family": family_name, "graph_size": size,
                               "goals": goal_count, **audit})
            blocks_by_group[(family_name, size, goal_count)].extend(blocks)
            for block in blocks:
                base = {"family": family_name, "graph_config": graph_config,
                        "graph_size": size, "goals": goal_count, "test_id": block.test_id,
                        "depot": block.depot}
                assert block.complete_graph is not None
                graph_rows.append({**base, "variant": "complete", "k": "",
                                   "vertices": block.complete_graph[0],
                                   "edges": block.complete_graph[1],
                                   "preprocessing_seconds": block.complete_graph[2]})
                for k, (vertices, edges, duration) in sorted(block.cover_graphs.items()):
                    graph_rows.append({**base, "variant": "cover", "k": k,
                                       "vertices": vertices, "edges": edges,
                                       "preprocessing_seconds": duration})

                comparison_solutions = {
                    name: solution for name, solution in block.solutions.items()
                    if name == "Shrinked" or name.startswith("Cover ")
                }
                fronts = [solution.front for solution in comparison_solutions.values() if solution.front]
                bounds = objective_bounds(fronts) if fronts else None
                normalized = {
                    name: normalize(solution.front, *bounds) if bounds and solution.front else []
                    for name, solution in block.solutions.items()
                }
                baseline = block.solutions["Shrinked"]
                baseline_hv = hypervolume_2d(normalized["Shrinked"]) if normalized["Shrinked"] else 0.0
                for name, solution in block.solutions.items():
                    variant = "complete" if name == "Shrinked" else (
                        "original_road" if name == "Original" else "cover"
                    )
                    k = int(name.split()[1]) if name.startswith("Cover ") else ""
                    hv = hypervolume_2d(normalized[name]) if normalized[name] else 0.0
                    solution_rows.append({
                        **base, "variant": variant, "k": k, "duration_seconds": solution.duration,
                        "timeout": int(solution.duration >= 3599.0), "front_size": len(solution.front),
                        "front_equal_original": int(set(solution.front) == set(baseline.front)),
                        "hypervolume": hv,
                        "hypervolume_ratio": hv / baseline_hv if baseline_hv else math.nan,
                        "coverage_to_original": coverage(normalized[name], normalized["Shrinked"]),
                        "coverage_from_original": coverage(normalized["Shrinked"], normalized[name]),
                    })
    return audit_rows, graph_rows, solution_rows, blocks_by_group


def summarize_graphs(graph_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, int, int], list[dict[str, object]]] = defaultdict(list)
    for row in graph_rows:
        grouped[(str(row["family"]), int(row["graph_size"]), int(row["goals"]))].append(row)
    summaries = []
    for (family, size, goals), rows in sorted(grouped.items()):
        complete = [float(row["edges"]) for row in rows if row["variant"] == "complete"]
        by_k: dict[int, list[float]] = defaultdict(list)
        for row in rows:
            if row["variant"] == "cover":
                by_k[int(row["k"])].append(float(row["edges"]))
        means = {k: fmean(values) for k, values in by_k.items()}
        k_min = min(means, key=means.get)
        summaries.append({
            "family": family, "graph_size": size, "goals": goals,
            "retained_tests": len(complete), "avg_complete_edges": fmean(complete),
            "k_min": k_min, "avg_min_cover_edges": means[k_min],
            "reduction_factor": fmean(complete) / means[k_min],
            "avg_edges_k32": means.get(32, math.nan),
        })
    return summaries


def summarize_solutions(solution_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, int, int], list[dict[str, object]]] = defaultdict(list)
    for row in solution_rows:
        grouped[(str(row["family"]), int(row["graph_size"]), int(row["goals"]))].append(row)
    summaries = []
    for (family, size, goals), rows in sorted(grouped.items()):
        original = [row for row in rows if row["variant"] == "complete"]
        covers: dict[int, list[dict[str, object]]] = defaultdict(list)
        for row in rows:
            if row["variant"] == "cover":
                covers[int(row["k"])].append(row)
        k_star = min(covers, key=lambda k: fmean(float(row["duration_seconds"]) for row in covers[k]))
        selected = covers[k_star]
        original_time = fmean(float(row["duration_seconds"]) for row in original)
        selected_time = fmean(float(row["duration_seconds"]) for row in selected)
        summaries.append({
            "family": family, "graph_size": size, "goals": goals,
            "retained_tests": len(original), "avg_complete_seconds": original_time,
            "complete_timeout_percent": 100 * fmean(float(row["timeout"]) for row in original),
            "k_star": k_star, "avg_cover_seconds": selected_time,
            "cover_timeout_percent": 100 * fmean(float(row["timeout"]) for row in selected),
            "speedup": original_time / selected_time if selected_time else math.nan,
            "suboptimal_percent": 100 * (1 - fmean(float(row["front_equal_original"]) for row in selected)),
            "avg_hypervolume_ratio": finite_mean(float(row["hypervolume_ratio"])
                                                   for row in selected),
        })
    return summaries


def latex_table(path: Path, rows: list[dict[str, object]], kind: str) -> None:
    lines = ["% Generated by analyze_ilp_outputs.py; do not edit manually."]
    if kind == "graphs":
        lines.extend([r"\begin{tabular}{lrrrrrr}", r"\toprule",
                      r"Configuration & $n$ & $|\mathcal E|$ & $\min |E_k|$ & RF & $k_{\min}$ & $|E_{32}|$ \\", r"\midrule"])
        for row in rows:
            lines.append(f"{row['graph_size']}-{row['goals']} & {row['retained_tests']} & "
                         f"{row['avg_complete_edges']:.1f} & {row['avg_min_cover_edges']:.1f} & "
                         f"{row['reduction_factor']:.1f} & {row['k_min']} & {row['avg_edges_k32']:.1f} \\\\")
    else:
        lines.extend([r"\begin{tabular}{lrrrrrrrr}", r"\toprule",
                      r"Configuration & $n$ & $T_{std}$ & TO$_{std}$ & $k^*$ & $T_{k^*}$ & TO$_{k^*}$ & Speedup & HV ratio \\", r"\midrule"])
        for row in rows:
            lines.append(f"{row['graph_size']}-{row['goals']} & {row['retained_tests']} & "
                         f"{row['avg_complete_seconds']:.1f} & {row['complete_timeout_percent']:.1f} & "
                         f"{row['k_star']} & {row['avg_cover_seconds']:.1f} & "
                         f"{row['cover_timeout_percent']:.1f} & {row['speedup']:.2f} & "
                         f"{row['avg_hypervolume_ratio']:.3f} \\\\")
    lines.extend([r"\bottomrule", r"\end{tabular}"])
    path.write_text("\n".join(lines) + "\n")


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    audit, graph_rows, solution_rows, _ = build_rows(args.input_dir, list(args.families))
    graph_summary = summarize_graphs(graph_rows)
    solution_summary = summarize_solutions(solution_rows)

    write_csv(args.output_dir / "audit.csv", audit,
              ["file", "family", "graph_size", "goals", "blocks", "complete_blocks",
               "retained_tests", "duplicate_complete_blocks", "rejected_blocks"])
    write_csv(args.output_dir / "graph_runs.csv", graph_rows,
              ["family", "graph_config", "graph_size", "goals", "test_id", "depot",
               "variant", "k", "vertices", "edges", "preprocessing_seconds"])
    write_csv(args.output_dir / "solution_runs.csv", solution_rows,
              ["family", "graph_config", "graph_size", "goals", "test_id", "depot",
               "variant", "k", "duration_seconds", "timeout", "front_size",
               "front_equal_original", "hypervolume", "hypervolume_ratio",
               "coverage_to_original", "coverage_from_original"])
    write_csv(args.output_dir / "preprocessing_summary.csv", graph_summary, list(graph_summary[0]))
    write_csv(args.output_dir / "solution_summary.csv", solution_summary, list(solution_summary[0]))
    latex_table(args.output_dir / "preprocessing_table.tex", graph_summary, "graphs")
    latex_table(args.output_dir / "solution_table.tex", solution_summary, "solutions")
    total_blocks = sum(int(row["blocks"]) for row in audit)
    total_retained = sum(int(row["retained_tests"]) for row in audit)
    total_rejected = sum(int(row["rejected_blocks"]) for row in audit)
    total_duplicates = sum(int(row["duplicate_complete_blocks"]) for row in audit)
    summary_lines = [
        "# Journal ILP log audit", "",
        f"- Source files: {len(audit)}",
        f"- Parsed blocks: {total_blocks}",
        f"- Retained last complete tests: {total_retained}",
        f"- Rejected incomplete blocks: {total_rejected}",
        f"- Superseded complete restart blocks: {total_duplicates}", "",
        "The recalculation retains only the final syntactically complete block for each test ID. "
        "In particular, the fastest mean cover level for configuration 100-50 is k=8; "
        "the conference table reported k=32. The generated journal tables preserve the "
        "recalculated value and actual sample counts.",
    ]
    (args.output_dir / "summary.md").write_text("\n".join(summary_lines) + "\n")
    print(f"Retained {sum(row['retained_tests'] for row in audit)} tests from {len(audit)} files")
    print(f"Wrote ILP analysis to {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
