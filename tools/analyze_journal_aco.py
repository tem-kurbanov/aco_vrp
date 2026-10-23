#!/usr/bin/env python3
"""Audit and analyze structured ACO journal experiment results."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from scipy.stats import wilcoxon

from analysis_tools import (
    additive_epsilon,
    coverage,
    hierarchical_bootstrap_ci,
    holm_adjust,
    hypervolume_2d,
    mean,
    nondominated,
    normalize,
    objective_bounds,
)

PAIR_METRICS = (
    "edge_ratio",
    "preprocessing_difference",
    "optimization_speedup",
    "total_speedup",
    "front_size_difference",
    "hypervolume_difference",
    "coverage_svrp_to_vrp",
    "coverage_vrp_to_svrp",
    "epsilon_svrp_to_vrp",
    "epsilon_vrp_to_svrp",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_dir", nargs="?", type=Path, default=Path("outputs/journal_aco"))
    parser.add_argument("-o", "--output-dir", type=Path, default=Path("analysis/journal_aco"))
    parser.add_argument("--expected-scenarios", type=int, default=10)
    parser.add_argument("--expected-replicates", type=int, default=3)
    parser.add_argument("--bootstrap-samples", type=int, default=10_000)
    parser.add_argument("--seed", type=int, default=20260622)
    parser.add_argument(
        "--allow-incomplete", action="store_true",
        help="generate publication tables and figures even when jobs are incomplete",
    )
    return parser.parse_args()


def write_csv(path: Path, rows: list[dict[str, Any]], columns: Iterable[str]) -> None:
    with path.open("w", newline="") as destination:
        writer = csv.DictWriter(destination, fieldnames=list(columns), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def record_key(record: dict[str, Any]) -> tuple[Any, ...]:
    parameters = record["parameters"]
    return (
        record["graph_path"], record["scenario_id"], record["replicate"],
        record["solver"], parameters["ants"], parameters["pass_through_candidates"],
        bool(parameters["compare_planned"]),
    )


def validate_record(record: dict[str, Any]) -> None:
    required = {
        "schema_version", "scenario_id", "graph_path", "graph_size", "scenario_seed",
        "replicate", "aco_seed", "goals", "solver", "status", "parameters", "graph",
        "timing_seconds", "iterations", "pareto",
    }
    missing = required - record.keys()
    if missing:
        raise ValueError(f"missing fields: {', '.join(sorted(missing))}")
    if record["schema_version"] != 1:
        raise ValueError(f"unsupported schema version {record['schema_version']}")
    if record["solver"] not in {"vrp", "svrp"}:
        raise ValueError(f"unknown solver {record['solver']!r}")
    if not isinstance(record["goals"], list) or not record["goals"]:
        raise ValueError("goals must be a nonempty array")
    for point in record["pareto"]:
        objectives = point.get("objectives", [])
        if len(objectives) != 2 or not all(isinstance(value, (int, float)) for value in objectives):
            raise ValueError("each Pareto point must contain two numeric objectives")


def load_records(input_dir: Path, expected_lines: int):
    records: list[dict[str, Any]] = []
    audit: list[dict[str, Any]] = []
    seen: set[tuple[Any, ...]] = set()
    for path in sorted(input_dir.rglob("*.jsonl")) if input_dir.exists() else []:
        valid = malformed = duplicates = 0
        with path.open(errors="replace") as source:
            lines = source.readlines()
        for line_number, line in enumerate(lines, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
                validate_record(record)
                key = record_key(record)
                if key in seen:
                    duplicates += 1
                    continue
                seen.add(key)
                record["_source"] = str(path)
                records.append(record)
                valid += 1
            except (json.JSONDecodeError, TypeError, ValueError, KeyError):
                malformed += 1
        audit.append({
            "file": str(path), "nonempty_lines": sum(bool(line.strip()) for line in lines),
            "valid_records": valid, "malformed_records": malformed,
            "duplicate_records": duplicates, "expected_records": expected_lines,
            "complete": int(valid == expected_lines and malformed == 0 and duplicates == 0),
        })
    return records, audit


def front(record: dict[str, Any]) -> list[tuple[float, float]]:
    return nondominated(tuple(map(float, point["objectives"])) for point in record["pareto"])


def scenario_bounds(records: list[dict[str, Any]]):
    pooled: dict[tuple[str, str], list[list[tuple[float, float]]]] = defaultdict(list)
    for record in records:
        current = front(record)
        if current:
            pooled[(record["graph_path"], record["scenario_id"])].append(current)
    return {key: objective_bounds(fronts) for key, fronts in pooled.items()}


def enrich_records(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    bounds = scenario_bounds(records)
    rows = []
    for record in records:
        parameters = record["parameters"]
        timing = record["timing_seconds"]
        current = front(record)
        bound = bounds.get((record["graph_path"], record["scenario_id"]))
        normalized = normalize(current, *bound) if current and bound else []
        feasible = record["status"] == "ok" and bool(current)
        rows.append({
            "graph_path": record["graph_path"], "graph_size": record["graph_size"],
            "goals": len(record["goals"]), "scenario_id": record["scenario_id"],
            "scenario_seed": record["scenario_seed"], "replicate": record["replicate"],
            "aco_seed": record["aco_seed"], "solver": record["solver"],
            "ants": parameters["ants"],
            "pass_through_candidates": parameters["pass_through_candidates"],
            "compare_planned": int(bool(parameters["compare_planned"])),
            "status": record["status"], "feasible": int(feasible),
            "nodes": record["graph"]["nodes"], "edges": record["graph"]["edges"],
            "preprocessing_seconds": timing["preprocessing"],
            "optimization_seconds": timing["optimization"], "total_seconds": timing["total"],
            "iterations": record["iterations"], "front_size": len(current),
            "hypervolume": hypervolume_2d(normalized) if normalized else math.nan,
            "_front": current, "_normalized": normalized,
        })
    return rows


def pair_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    baselines = {
        (row["graph_path"], row["scenario_id"], row["replicate"], row["ants"]): row
        for row in rows if row["solver"] == "vrp"
    }
    pairs = []
    for sparse in rows:
        if sparse["solver"] != "svrp":
            continue
        key = (sparse["graph_path"], sparse["scenario_id"], sparse["replicate"], sparse["ants"])
        baseline = baselines.get(key)
        if baseline is None:
            continue
        sf, bf = sparse["_normalized"], baseline["_normalized"]
        both_feasible = bool(sparse["feasible"] and baseline["feasible"])
        pairs.append({
            "graph_path": sparse["graph_path"], "graph_size": sparse["graph_size"],
            "goals": sparse["goals"], "scenario_id": sparse["scenario_id"],
            "replicate": sparse["replicate"], "aco_seed": sparse["aco_seed"],
            "ants": sparse["ants"], "pass_through_candidates": sparse["pass_through_candidates"],
            "compare_planned": sparse["compare_planned"], "both_feasible": int(both_feasible),
            "edge_ratio": sparse["edges"] / baseline["edges"] if baseline["edges"] else math.nan,
            "preprocessing_difference": sparse["preprocessing_seconds"] - baseline["preprocessing_seconds"],
            "optimization_speedup": baseline["optimization_seconds"] / sparse["optimization_seconds"]
            if sparse["optimization_seconds"] else math.nan,
            "total_speedup": baseline["total_seconds"] / sparse["total_seconds"]
            if sparse["total_seconds"] else math.nan,
            "front_size_difference": sparse["front_size"] - baseline["front_size"],
            "hypervolume_difference": sparse["hypervolume"] - baseline["hypervolume"]
            if both_feasible else math.nan,
            "coverage_svrp_to_vrp": coverage(sf, bf) if both_feasible else math.nan,
            "coverage_vrp_to_svrp": coverage(bf, sf) if both_feasible else math.nan,
            "epsilon_svrp_to_vrp": additive_epsilon(sf, bf) if both_feasible else math.nan,
            "epsilon_vrp_to_svrp": additive_epsilon(bf, sf) if both_feasible else math.nan,
        })
    return pairs


def aggregate_runs(rows: list[dict[str, Any]], samples: int, seed: int):
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        key = (row["graph_size"], row["goals"], row["solver"], row["ants"],
               row["pass_through_candidates"], row["compare_planned"])
        groups[key].append(row)
    result = []
    metrics = ("nodes", "edges", "preprocessing_seconds", "optimization_seconds",
               "total_seconds", "iterations", "front_size", "hypervolume")
    for key, group in sorted(groups.items()):
        output = dict(zip(("graph_size", "goals", "solver", "ants",
                           "pass_through_candidates", "compare_planned"), key))
        output["runs"] = len(group)
        output["graphs"] = len({row["graph_path"] for row in group})
        output["feasible_rate"] = mean(row["feasible"] for row in group)
        for metric in metrics:
            output[f"mean_{metric}"] = mean(row[metric] for row in group)
            clusters: dict[str, list[float]] = defaultdict(list)
            for row in group:
                clusters[row["graph_path"]].append(float(row[metric]))
            low, high = hierarchical_bootstrap_ci(clusters, samples, seed)
            output[f"ci_low_{metric}"] = low
            output[f"ci_high_{metric}"] = high
        result.append(output)
    return result


def aggregate_pairs(pairs: list[dict[str, Any]], samples: int, seed: int):
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in pairs:
        key = (row["graph_size"], row["goals"], row["ants"],
               row["pass_through_candidates"], row["compare_planned"])
        groups[key].append(row)
    result = []
    for key, group in sorted(groups.items()):
        output = dict(zip(("graph_size", "goals", "ants", "pass_through_candidates",
                           "compare_planned"), key))
        output["pairs"] = len(group)
        output["graphs"] = len({row["graph_path"] for row in group})
        output["paired_feasible_rate"] = mean(row["both_feasible"] for row in group)
        for metric in PAIR_METRICS:
            output[f"mean_{metric}"] = mean(row[metric] for row in group)
            clusters: dict[str, list[float]] = defaultdict(list)
            for row in group:
                clusters[row["graph_path"]].append(float(row[metric]))
            low, high = hierarchical_bootstrap_ci(clusters, samples, seed)
            output[f"ci_low_{metric}"] = low
            output[f"ci_high_{metric}"] = high
        result.append(output)
    return result


def ablation_pairs(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    index = {
        (row["graph_path"], row["scenario_id"], row["replicate"], row["ants"],
         row["pass_through_candidates"], row["compare_planned"]): row
        for row in rows if row["solver"] == "svrp"
    }
    output = []
    for key, planned in index.items():
        graph, scenario, replicate, ants, candidates, enabled = key
        if ants != 10 or candidates not in {0, 20} or not enabled:
            continue
        unplanned = index.get((graph, scenario, replicate, ants, candidates, 0))
        if unplanned is None:
            continue
        feasible = planned["feasible"] and unplanned["feasible"]
        output.append({
            "graph_path": graph, "graph_size": planned["graph_size"], "goals": planned["goals"],
            "scenario_id": scenario, "replicate": replicate,
            "pass_through_candidates": candidates, "both_feasible": int(feasible),
            "edge_difference": planned["edges"] - unplanned["edges"],
            "total_time_difference": planned["total_seconds"] - unplanned["total_seconds"],
            "hypervolume_difference": planned["hypervolume"] - unplanned["hypervolume"]
            if feasible else math.nan,
            "front_size_difference": planned["front_size"] - unplanned["front_size"],
        })
    return output


def aggregate_ablations(rows: list[dict[str, Any]], samples: int, seed: int):
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        groups[(row["graph_size"], row["goals"], row["pass_through_candidates"])].append(row)
    metrics = ("edge_difference", "total_time_difference", "hypervolume_difference",
               "front_size_difference")
    result = []
    for key, group in sorted(groups.items()):
        output = dict(zip(("graph_size", "goals", "pass_through_candidates"), key))
        output["pairs"] = len(group)
        output["graphs"] = len({row["graph_path"] for row in group})
        output["paired_feasible_rate"] = mean(row["both_feasible"] for row in group)
        for metric in metrics:
            output[f"mean_{metric}"] = mean(row[metric] for row in group)
            clusters: dict[str, list[float]] = defaultdict(list)
            for row in group:
                clusters[row["graph_path"]].append(float(row[metric]))
            low, high = hierarchical_bootstrap_ci(clusters, samples, seed)
            output[f"ci_low_{metric}"] = low
            output[f"ci_high_{metric}"] = high
        result.append(output)
    return result


def graph_wilcoxon(rows: list[dict[str, Any]], comparison: str,
                   group_fields: tuple[str, ...], metrics: tuple[str, ...]):
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[tuple(row[field] for field in group_fields)].append(row)
    tests = []
    for group, values in sorted(grouped.items()):
        for metric in metrics:
            graph_values: dict[str, list[float]] = defaultdict(list)
            for row in values:
                value = float(row[metric])
                if math.isfinite(value):
                    graph_values[row["graph_path"]].append(value)
            differences = [mean(items) for items in graph_values.values() if items]
            if len(differences) < 2 or all(math.isclose(value, 0.0) for value in differences):
                statistic, p_value = math.nan, 1.0 if differences else math.nan
            else:
                result = wilcoxon(differences, alternative="two-sided", zero_method="wilcox")
                statistic, p_value = float(result.statistic), float(result.pvalue)
            tests.append({
                "comparison": comparison,
                **dict(zip(group_fields, group)), "metric": metric,
                "graphs": len(differences), "mean_graph_difference": mean(differences),
                "statistic": statistic, "p_value": p_value,
            })
    adjusted = holm_adjust([row["p_value"] for row in tests])
    for row, value in zip(tests, adjusted):
        row["holm_p_value"] = value
    return tests


def significance_tests(pairs: list[dict[str, Any]], ablations: list[dict[str, Any]]):
    primary = [row for row in pairs if row["ants"] == 10
               and row["pass_through_candidates"] == 20 and not row["compare_planned"]]
    tests = graph_wilcoxon(
        primary, "primary_svrp_minus_vrp", ("graph_size", "goals"),
        ("hypervolume_difference", "front_size_difference", "preprocessing_difference"),
    )
    tests += graph_wilcoxon(
        ablations, "planning_on_minus_off", ("graph_size", "goals", "pass_through_candidates"),
        ("hypervolume_difference", "front_size_difference", "total_time_difference"),
    )
    adjusted = holm_adjust([row["p_value"] for row in tests])
    for row, value in zip(tests, adjusted):
        row["holm_p_value"] = value
    return tests


def publication_outputs(output_dir: Path, pair_summary: list[dict[str, Any]],
                        ablation_summary: list[dict[str, Any]],
                        tests: list[dict[str, Any]]) -> None:
    primary = [row for row in pair_summary if row["ants"] == 10
               and row["pass_through_candidates"] == 20 and not row["compare_planned"]]
    table = ["% Generated by analyze_journal_aco.py; do not edit manually.",
             r"\begin{tabular}{rrrrrrrr}", r"\toprule",
             r"$|V|$ & $|C|$ & pairs & edge ratio & total speedup & $\Delta$HV & $C(S,V)$ & $I_\epsilon(S,V)$ \\",
             r"\midrule"]
    for row in primary:
        table.append(
            f"{row['graph_size']} & {row['goals']} & {row['pairs']} & "
            f"{row['mean_edge_ratio']:.3f} & {row['mean_total_speedup']:.2f} & "
            f"{row['mean_hypervolume_difference']:.3f} & "
            f"{row['mean_coverage_svrp_to_vrp']:.3f} & "
            f"{row['mean_epsilon_svrp_to_vrp']:.3f} \\\\"
        )
    table.extend([r"\bottomrule", r"\end{tabular}"])
    (output_dir / "primary_table.tex").write_text("\n".join(table) + "\n")

    test_table = ["% Generated by analyze_journal_aco.py; do not edit manually.",
                  r"\begin{tabular}{llrrr}", r"\toprule",
                  r"Comparison & Metric & Graphs & Mean difference & Holm $p$ \\", r"\midrule"]
    for row in tests:
        label = str(row["comparison"]).replace("_", r"\_")
        metric = str(row["metric"]).replace("_", r"\_")
        test_table.append(f"{label} & {metric} & {row['graphs']} & "
                          f"{row['mean_graph_difference']:.3g} & {row['holm_p_value']:.3g} \\\\"
                          )
    test_table.extend([r"\bottomrule", r"\end{tabular}"])
    (output_dir / "statistical_tests_table.tex").write_text("\n".join(test_table) + "\n")

    if ablation_summary:
        ablation_table = ["% Generated by analyze_journal_aco.py; do not edit manually.",
                          r"\begin{tabular}{rrrrrr}", r"\toprule",
                          r"$|V|$ & $|C|$ & $p$ & pairs & $\Delta$ total time & $\Delta$HV \\",
                          r"\midrule"]
        for row in ablation_summary:
            ablation_table.append(
                f"{row['graph_size']} & {row['goals']} & {row['pass_through_candidates']} & "
                f"{row['pairs']} & {row['mean_total_time_difference']:.3f} & "
                f"{row['mean_hypervolume_difference']:.3f} \\\\"
            )
        ablation_table.extend([r"\bottomrule", r"\end{tabular}"])
        (output_dir / "planning_ablation_table.tex").write_text(
            "\n".join(ablation_table) + "\n"
        )

    os.environ.setdefault("MPLCONFIGDIR", str(output_dir / ".matplotlib"))
    import matplotlib.pyplot as plt

    if primary:
        labels = [f"{row['graph_size']}-{row['goals']}" for row in primary]
        x = np.arange(len(primary))
        figure, axes = plt.subplots(1, 3, figsize=(10.8, 3.1))
        specifications = (
            ("edge_ratio", "SVRP/VRP edges", 1.0),
            ("total_speedup", "VRP/SVRP total time", 1.0),
            ("hypervolume_difference", "SVRP - VRP hypervolume", 0.0),
        )
        for axis, (metric, ylabel, reference) in zip(axes, specifications):
            means = np.asarray([row[f"mean_{metric}"] for row in primary])
            low = np.asarray([row[f"ci_low_{metric}"] for row in primary])
            high = np.asarray([row[f"ci_high_{metric}"] for row in primary])
            axis.errorbar(x, means, yerr=np.vstack((means - low, high - means)), fmt="o",
                          color="#1b6ca8", ecolor="#555555", capsize=3)
            axis.axhline(reference, color="#a33a2b", linewidth=0.8, linestyle="--")
            axis.set_ylabel(ylabel)
            axis.set_xticks(x, labels, rotation=35, ha="right")
            axis.grid(axis="y", linewidth=0.4, alpha=0.4)
        figure.tight_layout()
        figure.savefig(output_dir / "primary_comparison.pdf", bbox_inches="tight")
        plt.close(figure)

    if ablation_summary:
        labels = [f"{row['graph_size']}-{row['goals']}, p={row['pass_through_candidates']}"
                  for row in ablation_summary]
        x = np.arange(len(ablation_summary))
        figure, axes = plt.subplots(1, 2, figsize=(8.0, 3.2))
        for axis, metric, ylabel, reference in (
            (axes[0], "total_time_difference", "Planning on - off time (s)", 0.0),
            (axes[1], "hypervolume_difference", "Planning on - off hypervolume", 0.0),
        ):
            values = np.asarray([row[f"mean_{metric}"] for row in ablation_summary])
            low = np.asarray([row[f"ci_low_{metric}"] for row in ablation_summary])
            high = np.asarray([row[f"ci_high_{metric}"] for row in ablation_summary])
            axis.errorbar(x, values, yerr=np.vstack((values - low, high - values)), fmt="o",
                          color="#28784f", ecolor="#555555", capsize=3)
            axis.axhline(reference, color="#a33a2b", linewidth=0.8, linestyle="--")
            axis.set_ylabel(ylabel)
            axis.set_xticks(x, labels, rotation=40, ha="right")
            axis.grid(axis="y", linewidth=0.4, alpha=0.4)
        figure.tight_layout()
        figure.savefig(output_dir / "planning_ablation.pdf", bbox_inches="tight")
        plt.close(figure)


def markdown_summary(output_dir: Path, records: list[dict[str, Any]], audit: list[dict[str, Any]],
                     pair_summary: list[dict[str, Any]], publication_ready: bool) -> None:
    complete = sum(row["complete"] for row in audit)
    primary = [row for row in pair_summary if row["ants"] == 10
               and row["pass_through_candidates"] == 20 and not row["compare_planned"]]
    lines = ["# Journal ACO experiment audit", "",
             f"- JSONL files: {len(audit)} ({complete} complete)",
             f"- Valid records: {len(records)}",
             f"- Publication artifacts generated: {'yes' if publication_ready else 'no'}", ""]
    if primary:
        lines.extend(["## Primary SVRP configuration", "",
                      "Ants 10, pass-through candidates 20, planned comparison off.", "",
                      "| Graph-goals | Edge ratio | Total speedup | Hypervolume difference |",
                      "|---|---:|---:|---:|"])
        for row in primary:
            lines.append(f"| {row['graph_size']}-{row['goals']} | "
                         f"{row['mean_edge_ratio']:.3f} | {row['mean_total_speedup']:.2f} | "
                         f"{row['mean_hypervolume_difference']:.3f} |")
    elif not records:
        lines.append("No journal ACO JSONL records were found. Run the journal Slurm matrix first.")
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n")


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    expected = args.expected_scenarios * args.expected_replicates
    records, audit = load_records(args.input_dir, expected)
    write_csv(args.output_dir / "completeness.csv", audit,
              ("file", "nonempty_lines", "valid_records", "malformed_records",
               "duplicate_records", "expected_records", "complete"))
    if not records:
        markdown_summary(args.output_dir, records, audit, [], False)
        print(f"No JSONL records found under {args.input_dir}; wrote completeness report")
        return 0

    rows = enrich_records(records)
    pairs = pair_rows(rows)
    ablations = ablation_pairs(rows)
    ablation_summary = aggregate_ablations(ablations, args.bootstrap_samples, args.seed)
    run_summary = aggregate_runs(rows, args.bootstrap_samples, args.seed)
    pair_summary = aggregate_pairs(pairs, args.bootstrap_samples, args.seed)
    tests = significance_tests(pairs, ablations)

    public_run_columns = [key for key in rows[0] if not key.startswith("_")]
    write_csv(args.output_dir / "runs.csv", rows, public_run_columns)
    write_csv(args.output_dir / "paired_runs.csv", pairs, pairs[0].keys() if pairs else ())
    write_csv(args.output_dir / "planning_ablation_runs.csv", ablations,
              ablations[0].keys() if ablations else ())
    write_csv(args.output_dir / "planning_ablation_summary.csv", ablation_summary,
              ablation_summary[0].keys() if ablation_summary else ())
    write_csv(args.output_dir / "run_summary.csv", run_summary,
              run_summary[0].keys() if run_summary else ())
    write_csv(args.output_dir / "paired_summary.csv", pair_summary,
              pair_summary[0].keys() if pair_summary else ())
    write_csv(args.output_dir / "statistical_tests.csv", tests,
              tests[0].keys() if tests else ())

    all_complete = bool(audit) and all(row["complete"] for row in audit)
    publication_ready = all_complete or args.allow_incomplete
    if publication_ready:
        publication_outputs(args.output_dir, pair_summary, ablation_summary, tests)
    markdown_summary(args.output_dir, records, audit, pair_summary, publication_ready)
    print(f"Analyzed {len(records)} records from {len(audit)} files")
    if not publication_ready:
        print("Publication tables withheld: see completeness.csv (or use --allow-incomplete)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
