#!/usr/bin/env python3
"""Check the mechanical submission requirements for the journal manuscript."""

from __future__ import annotations

import argparse
import difflib
import re
import subprocess
from pathlib import Path


REQUIRED_SECTIONS = (
    "Introduction", "Related Work", "Problem and Planning Representations",
    "Cover Construction and Augmentation", "Exact Methodology",
    "Multicriteria Ant Colony Optimization", "Experimental Methodology",
    "Results", "Discussion and Threats to Validity", "Conclusion",
)


def prose(text: str) -> str:
    text = re.sub(r"%.*", " ", text)
    text = re.sub(r"\\[A-Za-z]+\*?(?:\[[^]]*\])?", " ", text)
    return " ".join(re.findall(r"[A-Za-z]{3,}", text.lower()))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--paper-dir", type=Path, default=Path("LaTeX2e Proceedings"))
    parser.add_argument("--allow-pending-results", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    journal_path = args.paper_dir / "journalpaper.tex"
    conference_path = args.paper_dir / "samplepaper.tex"
    pdf_path = args.paper_dir / "journalpaper.pdf"
    log_path = args.paper_dir / "journalpaper.log"
    journal = journal_path.read_text()
    conference = conference_path.read_text()
    failures: list[str] = []

    if "\\cite{ICAART2026}" not in journal:
        failures.append("ICAART preliminary-work citation is missing")
    if "preliminary conference paper" not in journal.lower():
        failures.append("explicit conference-extension statement is missing")
    for section in REQUIRED_SECTIONS:
        if f"\\section{{{section}}}" not in journal:
            failures.append(f"required section is missing: {section}")

    similarity = difflib.SequenceMatcher(None, prose(conference), prose(journal), autojunk=False).ratio()
    if similarity >= 0.70:
        failures.append(f"conference/journal prose similarity is unexpectedly high ({similarity:.1%})")

    if not pdf_path.exists():
        failures.append("journalpaper.pdf has not been built")
        pages = -1
    else:
        info = subprocess.run(["pdfinfo", str(pdf_path)], check=True, text=True,
                              capture_output=True).stdout
        match = re.search(r"^Pages:\s+(\d+)", info, re.MULTILINE)
        pages = int(match.group(1)) if match else -1
        if pages < 1 or pages >= 30:
            failures.append(f"page count must be between 1 and 29 (found {pages})")

    if log_path.exists() and re.search(r"undefined references|undefined citations|LaTeX Error",
                                       log_path.read_text(errors="replace"), re.IGNORECASE):
        failures.append("LaTeX log contains unresolved references, citations, or errors")

    pending = "Experiment status." in journal and not (
        Path("analysis/journal_aco/primary_table.tex").exists()
        and Path("analysis/journal_aco/planning_ablation_table.tex").exists()
    )
    if pending and not args.allow_pending_results:
        failures.append("ACO result artifacts are pending")

    print(f"Pages: {pages}")
    print(f"Conference/journal prose similarity: {similarity:.1%}")
    print(f"ACO result artifacts: {'pending' if pending else 'present'}")
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
        return 1
    print("Mechanical manuscript audit passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
