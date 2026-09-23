# Benchmarks

The benchmarks use [Google Benchmark](https://github.com/google/benchmark). Each `Bench_*.cpp` file becomes an executable `Bench_*.out` in this folder.

## Building and running

```shell
cmake -B build -DCMAKE_BUILD_TYPE=Release -DDSF_BENCHMARKS=ON
cmake --build build -j$(nproc)
cd benchmark
./Bench_Dynamics.out
./Bench_Dynamics.out --benchmark_filter=Evolve   # run a subset
```

The loaded-dynamics benchmarks (`Bench_Dynamics`) build each scenario once per process, on the Forlì network in `test/data`. They warm it up and then time one `evolve()` step per iteration. The first run of each scenario therefore takes a few seconds before any timing starts.

## Comparing two builds

`run_ab.sh` runs the benchmarks of two builds alternately on the same machine. `compare_benchmarks.py` then compares the results:

```shell
# Build the base commit somewhere else, e.g. in a worktree
git worktree add ../dsf-base main
cmake -S ../dsf-base -B ../dsf-base/build -DCMAKE_BUILD_TYPE=Release -DDSF_BENCHMARKS=ON
cmake --build ../dsf-base/build -j$(nproc)

benchmark/run_ab.sh ../dsf-base/benchmark benchmark results
python3 benchmark/compare_benchmarks.py --base results/base --head results/head
```

`run_ab.sh` runs each executable in several rounds per side, alternating the order (base, head, then head, base, and so on). A steady drift in the machine's speed therefore affects both sides equally. A brief disturbance, such as a noisy neighbour on a CI runner, only affects one round, so it moves the medians very little. It reads these environment variables:

- `BENCH_REPETITIONS`: repetitions per side in total, split over the rounds (default 12).
- `BENCH_ROUNDS`: number of rounds (default 4).
- `BENCH_MIN_TIME`: minimum duration of each repetition (default `0.1s`).
- `BENCH_FILTER`: a regular expression that selects which benchmarks run.

### How a regression is decided

For each benchmark, the script compares the medians of the per-repetition wall-clock times. It reports the benchmark as:

- 🔴 **regression** when all of these hold:
  - the head median is slower than the base median by more than `--tolerance` (default 10%);
  - a two-sided Mann–Whitney U test finds the difference significant (p < `--alpha`, default 0.05). A consistent shift passes even with noisy samples, while a single outlier does not;
  - the base median is at least `--min-time-ns` (default 5 ns). Nanosecond-scale benchmarks are dominated by noise, so they are reported but never flagged.
- 🟢 **improvement**: the same conditions in the other direction.
- ⚪ **within noise**: everything else.
- 🆕 **new** / 🗑️ **removed**: the benchmark exists on only one side.

Benchmarks matching an `--ignore REGEX` are reported but never flagged. The script exits with 1 when there is at least one regression, unless `--no-fail` is passed.

## Continuous integration

The `CI: Benchmark` workflow (`.github/workflows/benchmark.yml`) does the following on every non-draft pull request:

1. It builds the PR base and the PR head on the same runner and runs `run_ab.sh`.
2. It posts the comparison as a sticky PR comment and in the job summary.
3. The check fails if any benchmark regresses, or if a benchmark executable crashes on the head commit.

For an intended slowdown, add the `skip-benchmark` label to the PR. The comparison is still reported, but regressions no longer fail the check. A crash on the head commit still fails it.

On pushes to `main`, the workflow compares against the previous commit and only reports the result. The tolerance and the number of repetitions are set in the workflow's `env` section.
