#!/usr/bin/env bash
# Run the benchmarks of two builds (base and head) alternately on the same machine.
#
# Usage: run_ab.sh BASE_BENCHMARK_DIR HEAD_BENCHMARK_DIR OUTPUT_DIR
#
# Each benchmark executable of HEAD_BENCHMARK_DIR is run in several rounds, alternating
# the order of the two sides (base, head, head, base, base, head, ...):
# - a linear drift of the machine's performance affects both sides equally;
# - a transient disturbance (e.g. a noisy neighbour on a CI runner) only affects a
#   fraction of the samples of one side, which barely moves the medians.
# Results are written as Google Benchmark JSON files to
# OUTPUT_DIR/{base,head}/<executable>.<round>.json, to be compared with
# compare_benchmarks.py. Failures are listed in OUTPUT_DIR/warnings.txt.
#
# Environment variables:
#   BENCH_REPETITIONS  repetitions per executable and side, split between the rounds
#                      (default: 12)
#   BENCH_ROUNDS       number of rounds (default: 4)
#   BENCH_MIN_TIME     minimum time of each repetition (default: 0.1s)
#   BENCH_FILTER       optional regex passed to --benchmark_filter
#
# Exit code: 1 if an executable failed on head, else 0.

set -uo pipefail

if [ $# -ne 3 ]; then
  echo "Usage: $0 BASE_BENCHMARK_DIR HEAD_BENCHMARK_DIR OUTPUT_DIR" >&2
  exit 2
fi

base_dir=$(realpath "$1")
head_dir=$(realpath "$2")
mkdir -p "$3/base" "$3/head"
out_dir=$(realpath "$3")
rounds=${BENCH_ROUNDS:-4}
repetitions_per_round=$(((${BENCH_REPETITIONS:-12} + rounds - 1) / rounds))
min_time=${BENCH_MIN_TIME:-0.1s}
: >"$out_dir/warnings.txt"
head_failed=false
# Failed rounds, keyed by "<executable> <side>"
declare -A failed_rounds

run_side() {
  local side=$1 name=$2 round=$3
  local dir
  if [ "$side" = base ]; then dir=$base_dir; else dir=$head_dir; fi
  if [ ! -x "$dir/$name.out" ]; then
    echo "$name does not exist on $side, skipping"
    return
  fi
  echo "::group::$name ($side, round $round)"
  local args=(
    --benchmark_repetitions="$repetitions_per_round"
    --benchmark_min_time="$min_time"
    --benchmark_enable_random_interleaving=true
    --benchmark_display_aggregates_only=true
    --benchmark_out="$out_dir/$side/$name.$round.json"
    --benchmark_out_format=json
  )
  if [ -n "${BENCH_FILTER:-}" ]; then
    args+=(--benchmark_filter="$BENCH_FILTER")
  fi
  # Run from the benchmark folder, like a local run would
  if ! (cd "$dir" && "./$name.out" "${args[@]}"); then
    # Drop the (possibly truncated) results of the failed run
    rm -f "$out_dir/$side/$name.$round.json"
    failed_rounds["$name $side"]+=" $round"
    if [ "$side" = head ]; then
      echo "::error::$name failed on head"
      head_failed=true
    else
      echo "::warning::$name failed on base"
    fi
  fi
  echo "::endgroup::"
}

for head_exe in "$head_dir"/Bench_*.out; do
  name=$(basename "$head_exe" .out)
  for ((round = 1; round <= rounds; ++round)); do
    if ((round % 2 == 1)); then
      run_side base "$name" "$round"
      run_side head "$name" "$round"
    else
      run_side head "$name" "$round"
      run_side base "$name" "$round"
    fi
  done
done

for key in "${!failed_rounds[@]}"; do
  read -r name side <<<"$key"
  rounds_list=$(echo ${failed_rounds[$key]} | sed 's/ /, /g')
  echo "\`$name\` failed on $side in round(s) $rounds_list of $rounds: its results are" \
    "missing or incomplete, see the job log." >>"$out_dir/warnings.txt"
done
sort -o "$out_dir/warnings.txt" "$out_dir/warnings.txt"

if [ "$head_failed" = true ]; then
  exit 1
fi
