#!/usr/bin/env python3
"""Generate deterministic depot/goal scenarios for one graph instance."""

from __future__ import annotations

import argparse
import csv
import hashlib
import random
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--graph", required=True, type=Path)
    parser.add_argument("--goals", required=True, type=int)
    parser.add_argument("--scenarios", type=int, default=10)
    parser.add_argument("--seed", type=int, default=20260622)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def graph_size(path: Path) -> int:
    with path.open() as source:
        first_line = source.readline().strip()
    try:
        return int(first_line.split(",", 1)[0])
    except (ValueError, IndexError) as error:
        raise ValueError(f"Invalid graph metadata in {path}") from error


def scenario_seed(base_seed: int, graph_name: str, goals: int, index: int) -> int:
    material = f"{base_seed}|{graph_name}|{goals}|{index}".encode()
    digest = hashlib.sha256(material).digest()
    return int.from_bytes(digest[:4], "big") & 0x7FFFFFFF


def main() -> int:
    args = parse_args()
    nodes = graph_size(args.graph)
    if args.goals <= 0 or args.goals >= nodes:
        raise SystemExit(f"--goals must be between 1 and {nodes - 1}")
    if args.scenarios <= 0:
        raise SystemExit("--scenarios must be positive")
    if args.seed < 0:
        raise SystemExit("--seed must be non-negative")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as destination:
        writer = csv.writer(destination, lineterminator="\n")
        writer.writerow(("scenario_id", "depot", "goals", "scenario_seed"))
        for index in range(args.scenarios):
            seed = scenario_seed(args.seed, args.graph.stem, args.goals, index)
            selected = random.Random(seed).sample(range(nodes), args.goals + 1)
            depot = selected[0]
            goals = sorted(selected[1:])
            identifier = f"{args.graph.stem}-g{args.goals}-s{index + 1:02d}"
            writer.writerow((identifier, depot, "|".join(map(str, goals)), seed))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
