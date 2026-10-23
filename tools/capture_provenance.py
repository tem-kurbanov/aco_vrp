#!/usr/bin/env python3
"""Capture the current source, binary and run environment without results."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import platform
import subprocess
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def command(*args) -> str | None:
    try:
        result = subprocess.run(args, cwd=ROOT, text=True, capture_output=True, timeout=10)
        return result.stdout.strip() if result.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        return None


def capture(binary: Path) -> dict:
    binary = binary.resolve(strict=True)
    cache = binary.parent / "CMakeCache.txt"
    build = {}
    if cache.exists():
        for line in cache.read_text().splitlines():
            if not line or line.startswith(("//", "#")) or "=" not in line:
                continue
            key, value = line.split("=", 1)
            name = key.split(":", 1)[0]
            if (name in {"CMAKE_BUILD_TYPE", "CMAKE_CXX_COMPILER"}
                    or name.startswith("CMAKE_CXX_FLAGS") or name.startswith("OpenMP_")):
                build[name] = value
    packages = {}
    for name in ("numpy", "scipy", "matplotlib"):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = None
    status = command("git", "status", "--porcelain")
    return {
        "schema_version": 1,
        "kind": "current_run_snapshot",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_revision": command("git", "rev-parse", "HEAD"),
        "worktree_dirty": bool(status) if status is not None else None,
        "source_sha256": {p.name: sha256(p) for p in sorted(ROOT.iterdir())
                          if p.suffix in {".cpp", ".h"}},
        "binary": {"path": str(binary), "sha256": sha256(binary)},
        "build_cache": build,
        "compiler_version": command(build["CMAKE_CXX_COMPILER"], "--version")
                            if build.get("CMAKE_CXX_COMPILER") else None,
        "platform": platform.platform(),
        "machine": platform.machine(),
        "processor": platform.processor(),
        "cpu_details": command("lscpu") if platform.system() == "Linux"
                       else command("sysctl", "-n", "machdep.cpu.brand_string")
                       if platform.system() == "Darwin" else None,
        "python_version": platform.python_version(),
        "python_packages": packages,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    report = capture(args.binary)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
