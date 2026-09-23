"""Compare two sets of Google Benchmark JSON results and flag regressions.

Each input directory must contain one JSON file per benchmark executable, as
produced by ``--benchmark_out=<file>.json --benchmark_out_format=json``, ideally
with ``--benchmark_repetitions`` > 1 so that every benchmark has several samples.

For each benchmark, the medians of the per-repetition ``real_time`` values are
compared. A benchmark is flagged as a regression only if its median slows down by
more than the tolerance AND a two-sided Mann-Whitney U test finds the difference
significant (p < alpha). Benchmarks faster than ``--min-time-ns`` are reported but
never fail the comparison, since they are dominated by noise.

Exit code: 1 if at least one regression is found (unless ``--no-fail``), else 0.
"""

import argparse
import json
import math
import os
import re
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

TIME_UNIT_TO_NS = {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}

REGRESSION = "🔴 regression"
IMPROVEMENT = "🟢 improvement"
NOISE = "⚪ within noise"
IGNORED = "⚪ ignored"
NEW = "🆕 new"
REMOVED = "🗑️ removed"


@dataclass
class Comparison:
    """The comparison result of a single benchmark."""

    name: str
    status: str
    base_median: float | None = None
    head_median: float | None = None
    delta: float | None = None
    p_value: float | None = None


def load_samples(directory: Path, warnings: list[str]) -> dict[str, list[float]]:
    """Load the per-repetition real times (in ns) of every benchmark in a directory.

    Files that cannot be parsed (e.g. truncated by a crash) are skipped, and a
    message is appended to ``warnings``.
    """
    samples: dict[str, list[float]] = {}
    for json_file in sorted(directory.glob("*.json")):
        try:
            with json_file.open(encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError) as e:
            warnings.append(f"Could not read `{json_file}`: {e}")
            continue
        for bench in data.get("benchmarks", []):
            if bench.get("run_type") == "aggregate" or bench.get("error_occurred"):
                continue
            name = bench.get("run_name", bench["name"])
            factor = TIME_UNIT_TO_NS[bench.get("time_unit", "ns")]
            samples.setdefault(name, []).append(bench["real_time"] * factor)
    return samples


def mann_whitney_u(x: list[float], y: list[float]) -> float:
    """Two-sided Mann-Whitney U test p-value (normal approximation, tie-corrected)."""
    n1, n2 = len(x), len(y)
    if n1 < 2 or n2 < 2:
        return 1.0
    combined = sorted([(v, 0) for v in x] + [(v, 1) for v in y])
    n = n1 + n2
    ranks = [0.0] * n
    tie_term = 0.0
    i = 0
    while i < n:
        j = i
        while j + 1 < n and combined[j + 1][0] == combined[i][0]:
            j += 1
        # Average rank for tied values (ranks are 1-based)
        avg_rank = (i + j) / 2 + 1
        for k in range(i, j + 1):
            ranks[k] = avg_rank
        t = j - i + 1
        tie_term += t**3 - t
        i = j + 1
    r1 = sum(rank for rank, (_, group) in zip(ranks, combined) if group == 0)
    u1 = r1 - n1 * (n1 + 1) / 2
    mu = n1 * n2 / 2
    sigma = math.sqrt(n1 * n2 / 12 * ((n + 1) - tie_term / (n * (n - 1))))
    if sigma == 0:
        return 1.0
    # Continuity correction
    z = (abs(u1 - mu) - 0.5) / sigma
    return min(1.0, math.erfc(max(z, 0.0) / math.sqrt(2)))


def compare(
    base: dict[str, list[float]],
    head: dict[str, list[float]],
    tolerance: float,
    alpha: float,
    min_time_ns: float,
    ignore: list[re.Pattern],
) -> list[Comparison]:
    """Compare every benchmark found in either the base or the head results."""
    results = []
    for name in sorted(base.keys() | head.keys()):
        if name not in base:
            results.append(
                Comparison(name, NEW, head_median=statistics.median(head[name]))
            )
            continue
        if name not in head:
            results.append(
                Comparison(name, REMOVED, base_median=statistics.median(base[name]))
            )
            continue
        base_median = statistics.median(base[name])
        head_median = statistics.median(head[name])
        delta = head_median / base_median - 1 if base_median > 0 else 0.0
        p_value = mann_whitney_u(base[name], head[name])
        significant = p_value < alpha
        if any(pattern.search(name) for pattern in ignore):
            status = IGNORED
        elif delta > tolerance and significant and base_median >= min_time_ns:
            status = REGRESSION
        elif delta < -tolerance and significant:
            status = IMPROVEMENT
        else:
            status = NOISE
        results.append(
            Comparison(name, status, base_median, head_median, delta, p_value)
        )
    return results


def format_time(ns: float | None) -> str:
    """Format a duration in ns with a human-readable unit."""
    if ns is None:
        return "–"
    for unit, factor in (("s", 1e9), ("ms", 1e6), ("µs", 1e3)):
        if ns >= factor:
            return f"{ns / factor:.3g} {unit}"
    return f"{ns:.3g} ns"


def format_table(rows: list[Comparison]) -> list[str]:
    """Format comparisons as a Markdown table."""
    lines = [
        "| Benchmark | Base | Head | Δ | p-value | Status |",
        "|:--|--:|--:|--:|--:|:--|",
    ]
    for row in rows:
        delta = f"{row.delta:+.1%}" if row.delta is not None else "–"
        p_value = f"{row.p_value:.3f}" if row.p_value is not None else "–"
        lines.append(
            f"| `{row.name}` | {format_time(row.base_median)} | "
            f"{format_time(row.head_median)} | {delta} | {p_value} | {row.status} |"
        )
    return lines


def build_report(
    results: list[Comparison], warnings: list[str], args: argparse.Namespace
) -> str:
    """Build the Markdown report of the comparison."""
    counts = {
        status: sum(r.status == status for r in results)
        for status in (REGRESSION, IMPROVEMENT, NOISE, IGNORED, NEW, REMOVED)
    }
    n_regressions = counts[REGRESSION]
    if args.head_failed:
        verdict = (
            "❌ **At least one benchmark executable failed on the head commit** "
            "(see the warnings below)."
        )
    elif n_regressions == 0:
        verdict = "✅ **No performance regressions detected.**"
    elif args.no_fail:
        verdict = (
            f"⚠️ **{n_regressions} regression(s) detected**, "
            "but failure is disabled for this run."
        )
    else:
        verdict = f"❌ **{n_regressions} regression(s) detected.**"

    lines = ["## Benchmark comparison", "", verdict, ""]
    base_label = f"`{args.base_label}`" if args.base_label else "base"
    head_label = f"`{args.head_label}`" if args.head_label else "head"
    lines.append(
        f"{base_label} → {head_label} · tolerance ±{args.tolerance:.0%} · "
        f"α = {args.alpha} · regressions below {format_time(args.min_time_ns)} "
        "are not flagged"
    )
    lines.append("")
    lines.append(
        " · ".join(f"{status}: {count}" for status, count in counts.items() if count)
    )
    lines.append("")
    for warning in warnings:
        lines.append(f"> [!WARNING]\n> {warning}")
        lines.append("")

    notable = [r for r in results if r.status in (REGRESSION, IMPROVEMENT)]
    if notable:
        notable.sort(key=lambda r: (r.status != REGRESSION, -abs(r.delta or 0)))
        lines += format_table(notable)
        lines.append("")

    lines.append(f"<details><summary>All benchmarks ({len(results)})</summary>")
    lines.append("")
    lines += format_table(results)
    lines.append("")
    lines.append("</details>")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--base", type=Path, required=True, help="Base results dir")
    parser.add_argument("--head", type=Path, required=True, help="Head results dir")
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.10,
        help="Max allowed relative slowdown of the median (default: 0.10)",
    )
    parser.add_argument(
        "--alpha",
        type=float,
        default=0.05,
        help="Significance level of the Mann-Whitney U test (default: 0.05)",
    )
    parser.add_argument(
        "--min-time-ns",
        type=float,
        default=5.0,
        help="Benchmarks with a base median below this are never flagged (default: 5)",
    )
    parser.add_argument(
        "--ignore",
        action="append",
        default=[],
        help="Regex of benchmark names that never fail (can be repeated)",
    )
    parser.add_argument("--base-label", help="Label of the base (e.g. commit SHA)")
    parser.add_argument("--head-label", help="Label of the head (e.g. commit SHA)")
    parser.add_argument(
        "--warning",
        action="append",
        default=[],
        help="Extra warning to show in the report (can be repeated)",
    )
    parser.add_argument(
        "--head-failed",
        action="store_true",
        help="Report that a benchmark executable failed on the head",
    )
    parser.add_argument("--output", type=Path, help="Write the Markdown report here")
    parser.add_argument(
        "--no-fail", action="store_true", help="Always exit 0, even with regressions"
    )
    args = parser.parse_args()

    warnings: list[str] = list(args.warning)
    base = load_samples(args.base, warnings)
    head = load_samples(args.head, warnings)
    for warning in warnings:
        print(f"WARNING: {warning}", file=sys.stderr)
    if not head:
        print(f"No benchmark results found in {args.head}", file=sys.stderr)
        return 2

    results = compare(
        base,
        head,
        args.tolerance,
        args.alpha,
        args.min_time_ns,
        [re.compile(pattern) for pattern in args.ignore],
    )
    report = build_report(results, warnings, args)
    print(report)
    if args.output:
        args.output.write_text(report, encoding="utf-8")
    if summary_file := os.environ.get("GITHUB_STEP_SUMMARY"):
        with open(summary_file, "a", encoding="utf-8") as f:
            f.write(report)

    has_regressions = any(r.status == REGRESSION for r in results)
    return 1 if has_regressions and not args.no_fail else 0


if __name__ == "__main__":
    sys.exit(main())
