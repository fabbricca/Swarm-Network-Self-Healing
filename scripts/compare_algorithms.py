#!/usr/bin/env python3
"""Side-by-side comparison wrapper for the two formation-control algorithms.

Invokes `scripts/run_declaration.py` twice — once with `--algorithm=centroid`
and once with `--algorithm=weighted` — against the same JSON declaration, then
renders a side-by-side table of the per-metric averages.

The wrapper is intentionally thin: it adds zero metric parsing, delegates all
simulation/aggregation work to `run_declaration.py`, and only reads the
resulting `aggregate.json` files to build the comparison.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


REPO = Path(__file__).resolve().parent.parent
RUN_SCRIPT = Path(__file__).resolve().parent / "run_declaration.py"
ALGORITHMS = ("centroid", "weighted")


def default_out_dir() -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return REPO / "output" / f"compare_{stamp}"


def run_single_algorithm(
    declaration: Path,
    algorithm: str,
    sub_out_dir: Path,
    image: str,
    timeout: int,
    continue_on_error: bool,
    only: list[str] | None = None,
) -> int:
    cmd = [
        sys.executable,
        str(RUN_SCRIPT),
        str(declaration),
        f"--algorithm={algorithm}",
        f"--out-dir={sub_out_dir}",
        f"--image={image}",
        f"--timeout={timeout}",
    ]
    if continue_on_error:
        cmd.append("--continue-on-error")
    if only:
        for name in only:
            cmd.append(f"--only={name}")
    print(f"\n▶ running {algorithm} batch → {sub_out_dir}")
    return subprocess.call(cmd)


def load_aggregate(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"expected aggregate.json at {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def build_comparison(aggregates: dict[str, list[dict[str, Any]]]) -> list[dict[str, Any]]:
    labels: list[tuple[str, str]] = []
    seen_keys: set[str] = set()
    for algo in ALGORITHMS:
        for item in aggregates[algo]:
            if item["key"] not in seen_keys:
                labels.append((item["key"], item["label"]))
                seen_keys.add(item["key"])

    index: dict[str, dict[str, dict[str, Any]]] = {
        algo: {item["key"]: item for item in aggregates[algo]} for algo in ALGORITHMS
    }

    rows: list[dict[str, Any]] = []
    for key, label in labels:
        row: dict[str, Any] = {"key": key, "label": label}
        means: dict[str, float | None] = {}
        for algo in ALGORITHMS:
            agg = index[algo].get(key)
            mean = agg["mean"] if agg else None
            row[algo] = {
                "mean": mean,
                "stdev": agg["stdev"] if agg else None,
                "samples": agg["samples"] if agg else 0,
            }
            means[algo] = mean if isinstance(mean, (int, float)) else None

        c, w = means["centroid"], means["weighted"]
        if c is None or w is None:
            row["delta_absolute"] = None
            row["delta_percent"] = None
        else:
            row["delta_absolute"] = w - c
            row["delta_percent"] = ((w - c) / c * 100.0) if c != 0 else None
        rows.append(row)
    return rows


def render_markdown(comparison: list[dict[str, Any]], summaries: dict[str, dict[str, Any]]) -> str:
    out: list[str] = []
    out.append("# Algorithm comparison")
    out.append("")
    out.append(f"- centroid: {summaries['centroid']['successful_runs']}/{summaries['centroid']['scenario_count']} runs ok")
    out.append(f"- weighted: {summaries['weighted']['successful_runs']}/{summaries['weighted']['scenario_count']} runs ok")
    out.append("")
    out.append("| Metric | centroid mean ± stdev | weighted mean ± stdev | Δ (w − c) | Δ% |")
    out.append("|---|---:|---:|---:|---:|")
    for row in comparison:
        def cell(entry: dict[str, Any]) -> str:
            if entry["mean"] is None:
                return "n/a"
            return f"{entry['mean']:.4f} ± {entry['stdev']:.4f}"
        delta = row["delta_absolute"]
        pct = row["delta_percent"]
        delta_str = "n/a" if delta is None else f"{delta:+.4f}"
        pct_str = "n/a" if pct is None else f"{pct:+.2f}%"
        out.append(f"| {row['label']} | {cell(row['centroid'])} | {cell(row['weighted'])} | {delta_str} | {pct_str} |")
    return "\n".join(out) + "\n"


def print_comparison_table(comparison: list[dict[str, Any]]) -> None:
    print("\n================ ALGORITHM COMPARISON ================")
    fmt = "{:<34}{:>22}{:>22}{:>14}{:>10}"
    print(fmt.format("Metric", "centroid mean±std", "weighted mean±std", "Δ abs", "Δ%"))
    print("-" * 102)
    for row in comparison:
        def fmt_cell(entry: dict[str, Any]) -> str:
            if entry["mean"] is None:
                return "n/a"
            return f"{entry['mean']:.4f}±{entry['stdev']:.4f}"
        delta = row["delta_absolute"]
        pct = row["delta_percent"]
        delta_str = "n/a" if delta is None else f"{delta:+.4f}"
        pct_str = "n/a" if pct is None else f"{pct:+.2f}%"
        print(fmt.format(row["label"], fmt_cell(row["centroid"]), fmt_cell(row["weighted"]), delta_str, pct_str))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the declaration twice (centroid + weighted) and print a side-by-side comparison.",
    )
    parser.add_argument("declaration", type=Path, help="Path to the JSON declaration file")
    parser.add_argument("--out-dir", type=Path, default=None, help="Output directory (default: output/compare_<timestamp>)")
    parser.add_argument("--image", default="swarm-sim", help="Docker image tag (default: swarm-sim)")
    parser.add_argument("--timeout", type=int, default=1200, help="Per-run timeout in seconds")
    parser.add_argument("--continue-on-error", action="store_true", help="Do not abort on first failed run in either batch")
    parser.add_argument("--only", action="append", default=None,
                        help="Run only scenarios whose name matches one of these values (repeatable)")
    args = parser.parse_args()

    if not args.declaration.exists():
        print(f"[ERR] declaration file not found: {args.declaration}")
        return 1

    out_dir = args.out_dir or default_out_dir()
    out_dir.mkdir(parents=True, exist_ok=True)

    summaries: dict[str, dict[str, Any]] = {}
    aggregates: dict[str, list[dict[str, Any]]] = {}
    for algo in ALGORITHMS:
        sub_dir = out_dir / algo
        rc = run_single_algorithm(
            declaration=args.declaration,
            algorithm=algo,
            sub_out_dir=sub_dir,
            image=args.image,
            timeout=args.timeout,
            continue_on_error=args.continue_on_error,
            only=args.only,
        )
        if rc != 0 and not args.continue_on_error:
            print(f"[ERR] {algo} batch exited with rc={rc}; aborting comparison")
            return rc
        summary = load_aggregate(sub_dir / "aggregate.json")
        summaries[algo] = summary
        aggregates[algo] = summary.get("aggregates", [])

    comparison = build_comparison(aggregates)
    print_comparison_table(comparison)

    (out_dir / "comparison.json").write_text(
        json.dumps(
            {
                "declaration": str(args.declaration),
                "summaries": summaries,
                "comparison": comparison,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    (out_dir / "comparison.md").write_text(render_markdown(comparison, summaries), encoding="utf-8")

    print(f"\nJSON:     {out_dir / 'comparison.json'}")
    print(f"Markdown: {out_dir / 'comparison.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
