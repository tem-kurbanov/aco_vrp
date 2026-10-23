# Multicriteria vehicle routing with ant colony optimization

C++17 research implementation comparing ant colony optimization on complete
and sparse compressed planning graphs. Both solvers optimize two edge-cost
criteria while enforcing battery and vehicle limits, and return a set of
nondominated solutions.

The associated paper has been accepted for journal publication. The manuscript
is not distributed in this repository.

Implementation and experiment tooling by Temirlan Kurbanov, with research
supervision by Jiří Vokřínek.

## The problem and approach

A road network can offer several useful paths between the same customers:
one path may be shorter while another consumes less of the constrained
resource. Keeping every nondominated alternative can make the planning graph
large. This project compares that complete representation with a sparse
alternative built using a k-path cover and multicriteria path searches.

The implementation combines parallel ant construction, a shared Pareto
archive, battery-feasible route construction, and optional paths that serve
customers encountered between an arc's endpoints. The sparse solver can also
compare planned multi-arc moves with direct moves. These options expose the
tradeoff between the planning alternatives retained and the work needed to
search them; benchmark findings are withheld until they can be released.

See [architecture](docs/architecture.md) for the observed code flow and
[provenance](docs/provenance.md) for the experiment-source snapshot and the
limits of historical reproducibility. This repository contains the ACO
solvers; the separate ILP implementation is not included.

## Build and test

Requirements: a C++17 compiler, CMake 3.20+, and OpenMP. Python 3.9+ and the
analysis dependencies are needed for the full test suite.

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-analysis.txt
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

The test suite includes hand-checkable routes, objective tradeoffs, resource
and vehicle limits, disconnected inputs, CLI contracts, analysis helpers,
submission scripts, and input checksums. The Linux CI workflow runs these
checks without the manuscript or saved experiment outputs.

On macOS with Apple Clang, install `cmake` and `libomp` with Homebrew and use
these OpenMP options when configuring:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$PWD/.venv/bin/python" \
  -DOpenMP_CXX_FLAGS="-Xpreprocessor -fopenmp -I$(brew --prefix libomp)/include" \
  -DOpenMP_CXX_LIB_NAMES=omp \
  -DOpenMP_omp_LIBRARY="$(brew --prefix libomp)/lib/libomp.dylib"
```

Use `-DBUILD_TESTING=OFF` to build only the solver without requiring Python.

## Run a small example

```sh
./build/aco_vrp \
  --graph art/u100/graph_001_n100_c0_u1_seed20063.csv \
  --goal-ids 1,2,3 --depot 0 --tests 1 \
  --seed 123 --aco-seed 456 --solver both \
  --ants 2 --iterations 2 --threads 1 --output-format jsonl
```

`--solver vrp` uses the complete planning graph; `--solver svrp` uses the sparse
variant. Run `./build/aco_vrp --help` for all options. `--iterations` is the
number of consecutive non-improving iterations, rather than a total iteration
cap.

JSONL output records the scenario, derived ACO seed, parameters, preprocessing
and optimization times, status, invalid-candidate count, and Pareto objective
pairs with route counts. Check `status` before interpreting a run: the process
can finish successfully while reporting a disconnected graph or no solution.
The output does not include full routes or a source-revision identifier.

## Inputs and scenarios

`art/` contains 60 synthetic instances: ten per size (100, 250, 500 vertices)
and cost family (`c` for correlated, `u` for uncorrelated). They were generated
using [art_city](https://github.com/tem-kurbanov/art_city). The input manifest
records each seed, cost family, graph size, and SHA-256 checksum, together with
a compatible public generator revision. The original checkout was not
recorded; reconstruction evidence and the CSV export difference are described
in [provenance](docs/provenance.md).

```sh
python3 tools/verify_instances.py
```

Each graph CSV starts with `num_nodes,num_edges`, followed by a header and
directed edges with two integer costs. Headers are `fromId,toId,d,c1` for the
correlated families and `fromId,toId,d,u1` for the uncorrelated families. Node
IDs start at zero.

`scenarios/journal_aco/` contains the fixed depot/goal selections used by the
journal experiments. Each manifest contains ten scenarios and can be passed
to `--scenario-manifest`. The scenario generator is
`tools/generate_scenario_manifest.py`; the submission script uses it with a
fixed seed.

## SLURM experiments

The submission scripts remain Bash scripts. They resolve project files from
their own directory, so they can be invoked from another working directory.
They require `sbatch` and `srun` on the cluster; journal submission also uses
Python for scenario generation. Configuration values are at the top of each
script, with environment overrides for paths, resources, and seeds.

- `submit_journal_experiments.sh`: 60 graph/goal bundles, each with 17 sequential
  configurations: three VRP baselines, twelve SVRP sensitivity configurations,
  and two planned-path ablations. Each configuration uses ten scenarios and
  three replicates, producing 30 JSONL records.
- `submit_vrp_experiments.sh`: the earlier paired-solver sweep, producing the
  legacy text format consumed by `tools/analyze_vrp_outputs.py`.

```sh
# Prepare and check generated jobs without calling sbatch.
DRY_RUN=true bash submit_journal_experiments.sh
DRY_RUN=true bash submit_vrp_experiments.sh

# Adapt the partition to your cluster; an empty value uses its default.
PARTITION=your_partition bash submit_journal_experiments.sh
```

Dry runs create output directories, temporary job files, and journal scenario
manifests, but do not run the solver or submit jobs. Journal manifests are
regenerated from the configured scenario seed. Use separate `SCENARIO_ROOT`
and `RESULT_ROOT` directories for a different experiment configuration.
The journal script skips a result only when its JSON records cover every
expected scenario/replicate and match the graph, parameters, and derived ACO
seeds. Incomplete or incompatible results are rerun. Use `SKIP_COMPLETE=false`
when intentionally rerunning all existing results.

Raw outputs, cluster logs, temporary files, the local `cluster_code/` backup,
manuscript sources, and manuscript archives are ignored by Git. No jobs are submitted by the test
suite: it exercises generated jobs using local SLURM stand-ins.

Journal jobs write a provenance sidecar alongside their bundle logs, recording
the current source and binary hashes, available build settings, and runtime
environment. To capture the same metadata for a manual run:

```sh
python3 tools/capture_provenance.py --binary build/aco_vrp \
  --output build/run-provenance.json
```

This captures the current state; it does not recover the historical compiler
or prove that a binary was built from the source files currently on disk.

## Analyze structured results

```sh
.venv/bin/python tools/analyze_journal_aco.py outputs/journal_aco \
  --output-dir build/analysis/journal_aco
```

The analyzer audits records and produces summaries, paired comparisons,
Pareto metrics, confidence intervals, tables, and figures. These are derived
artifacts; the full raw experiment outputs are kept separately from the
source repository. The ILP and manuscript helpers are historical research
tooling and require additional inputs not included here.

## License

The code, documentation, bundled synthetic input graphs, and scenario manifests
are available under the [MIT License](LICENSE).

The unpublished manuscript and withheld experimental results are excluded
from this release and this license grant.
