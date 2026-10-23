#!/usr/bin/env python3
"""Verify bundled input hashes, optionally reconstructing them with art_city."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import io
import json
from concurrent.futures import ProcessPoolExecutor
from functools import lru_cache
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


@lru_cache(maxsize=1)
def load_generator(path: str):
    spec = importlib.util.spec_from_file_location("art_city_generator", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def verify_one(task):
    entry, directory, generator = task
    path = Path(directory) / entry["path"]
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != entry["sha256"]:
        raise ValueError(f"Checksum mismatch: {path}")
    rows = list(csv.reader(io.StringIO(data.decode("utf-8"))))
    if rows[0] != [str(entry["nodes"]), str(entry["edges"])]:
        raise ValueError(f"Metadata mismatch: {path}")
    if rows[1] != entry["header"] or len(rows) != entry["edges"] + 2:
        raise ValueError(f"Invalid graph shape: {path}")
    for row in rows[2:]:
        if len(row) != 4:
            raise ValueError(f"Invalid edge: {path}")
        start, end, first, second = map(int, row)
        if not (0 <= start < entry["nodes"] and 0 <= end < entry["nodes"]
                and first > 0 and second > 0):
            raise ValueError(f"Invalid edge: {path}")
    if generator:
        module = load_generator(generator)
        _, edges, costs = module.generate_graph(
            entry["nodes"], seed=entry["seed"],
            l_correlated=entry["correlated"], m_uncorrelated=entry["uncorrelated"],
        )
        # Bundled CSVs preserve generation order, whereas the upstream exporter
        # sorts by endpoints. Reconstruct the observed export without modifying it.
        output = io.StringIO(newline="")
        writer = csv.writer(output, lineterminator="\n")
        writer.writerow([entry["nodes"], len(edges)])
        writer.writerow(entry["header"])
        writer.writerows((start, end, *costs[(start, end)]) for start, end in edges)
        if output.getvalue().encode("utf-8") != data:
            raise ValueError(f"Generator reconstruction differs: {path}")
    return entry["path"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=ROOT / "art/manifest.json")
    parser.add_argument("--generator", type=Path,
                        help="local main.py from the compatible art_city revision")
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--families", nargs="+", help="limit checks to named families, e.g. u100 u250")
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    manifest = json.loads(args.manifest.read_text())
    if manifest["schema_version"] != 1 or not manifest["instances"]:
        parser.error("expected a nonempty version-1 input manifest")
    entries = manifest["instances"]
    paths = [entry["path"] for entry in entries]
    actual = {str(path.relative_to(args.manifest.parent))
              for path in args.manifest.parent.rglob("*.csv")}
    if len(paths) != len(set(paths)) or set(paths) != actual:
        parser.error("manifest must list each bundled CSV exactly once")
    if args.families:
        known = {Path(entry["path"]).parts[0] for entry in entries}
        if set(args.families) - known:
            parser.error("unknown instance family")
        entries = [entry for entry in entries if Path(entry["path"]).parts[0] in args.families]
    generator = None
    if args.generator:
        data = args.generator.read_bytes()
        if hashlib.sha256(data).hexdigest() != manifest["generator"]["source_sha256"]:
            parser.error("generator source does not match the pinned compatible revision")
        generator = str(args.generator.resolve())
    tasks = [(entry, str(args.manifest.resolve().parent), generator)
             for entry in entries]
    with ProcessPoolExecutor(max_workers=args.jobs) as executor:
        for path in executor.map(verify_one, tasks):
            print(f"Verified {path}", flush=True)
    mode = "checksums and generator reconstruction" if generator else "checksums and structure"
    print(f"Verified {len(tasks)} instances: {mode}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
