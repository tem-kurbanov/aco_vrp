#!/bin/bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

# Journal ACO experiment configuration. Edit values here before submission.
BINARY="${BINARY:-./build/aco_vrp}"
PYTHON="${PYTHON:-python3}"
SCENARIO_GENERATOR="${SCENARIO_GENERATOR:-./tools/generate_scenario_manifest.py}"

U250_GRAPH_DIR="${U250_GRAPH_DIR:-./art/u250}"
read -r -a U250_GOALS <<< "${U250_GOALS:-50 100 150}"
U500_GRAPH_DIR="${U500_GRAPH_DIR:-./art/u500}"
read -r -a U500_GOALS <<< "${U500_GOALS:-100 200 250}"

ANTS_LIST=(5 10 20)
PASS_THROUGH_LIST=(0 5 10 20)
SCENARIOS="${SCENARIOS:-10}"
REPLICATES="${REPLICATES:-3}"
SCENARIO_SEED="${SCENARIO_SEED:-20260622}"
ACO_SEED="${ACO_SEED:-20260622}"

THREADS="${THREADS:-2}"
ITERATIONS="${ITERATIONS:-20}"
BATTERY="${BATTERY:-50000}"
K_COVER="${K_COVER:-24}"

PARTITION="${PARTITION-amdextralong}"
MEM_PER_CPU="${MEM_PER_CPU:-8G}"
# This limit applies to all 17 sequential configurations in one graph-goal bundle.
TIME_LIMIT="${TIME_LIMIT:-}"
DRY_RUN="${DRY_RUN:-false}"

RESULT_ROOT="${RESULT_ROOT:-./outputs/journal_aco}"
LOG_ROOT="${LOG_ROOT:-./logs/journal_aco}"
ERROR_ROOT="${ERROR_ROOT:-./errors/journal_aco}"
SCENARIO_ROOT="${SCENARIO_ROOT:-./scenarios/journal_aco}"
TMP_ROOT="${TMP_ROOT:-./.slurm_tmp}"

# A complete result is not submitted again. Incomplete files are overwritten.
SKIP_COMPLETE="${SKIP_COMPLETE:-true}"

if [[ ! -x "$BINARY" ]]; then
    echo "Binary is not executable: $BINARY" >&2
    exit 1
fi
if [[ ! -f "$SCENARIO_GENERATOR" ]]; then
    echo "Scenario generator does not exist: $SCENARIO_GENERATOR" >&2
    exit 1
fi
if [[ "$THREADS" -lt 1 || "$SCENARIOS" -lt 1 || "$REPLICATES" -lt 1 ]]; then
    echo "THREADS, SCENARIOS and REPLICATES must be positive" >&2
    exit 1
fi
if [[ "$DRY_RUN" != true && "$DRY_RUN" != false ]]; then
    echo "DRY_RUN must be true or false" >&2
    exit 1
fi

mkdir -p "$RESULT_ROOT" "$LOG_ROOT" "$ERROR_ROOT" "$SCENARIO_ROOT" "$TMP_ROOT/submit"
TMP_ROOT="$(cd "$TMP_ROOT" && pwd)"
export TMPDIR="$TMP_ROOT/submit"

submitted=0
bundles=0
queued=0
skipped=0

is_complete() {
    [[ -f "$1" ]] || return 1
    "$PYTHON" - "$@" "$REPLICATES" "$THREADS" "$ITERATIONS" "$BATTERY" "$K_COVER" "$ACO_SEED" << 'PY'
import csv
import json
import sys
from pathlib import Path

def require(condition):
    if not condition:
        raise ValueError("incomplete or incompatible result")

def aco_seed(base, identifier, replicate):
    value = 2166136261 ^ base
    for byte in identifier.encode():
        value = ((value ^ byte) * 16777619) & 0xFFFFFFFF
    return ((value ^ replicate) * 16777619) & 0xFFFFFFFF

try:
    (result, manifest, graph, goals, solver, ants, candidates, planned,
     replicates, threads, iterations, battery, cover, base_seed) = sys.argv[1:]
    replicates = int(replicates)
    with open(manifest) as source:
        scenarios = {row["scenario_id"]: row for row in csv.DictReader(source)}
    expected = {(identifier, replicate) for identifier in scenarios
                for replicate in range(replicates)}
    parameters = dict(ants=int(ants), pass_through_candidates=int(candidates),
                      compare_planned=planned == "true", threads=int(threads),
                      stagnation_limit=int(iterations), battery=int(battery),
                      k_cover=int(cover), max_vehicles=int(goals),
                      count_pass_through_goals=int(candidates) > 0)
    seen = set()
    with open(result) as source:
        for line in source:
            record = json.loads(line)
            key = (record["scenario_id"], record["replicate"])
            require(key in expected and key not in seen)
            scenario = scenarios[key[0]]
            require(record["schema_version"] == 1 and record["solver"] == solver)
            require(Path(record["graph_path"]).resolve() == Path(graph).resolve())
            require(record["depot"] == int(scenario["depot"]))
            require(record["goals"] == [int(goal) for goal in scenario["goals"].split("|")])
            require(record["scenario_seed"] == int(scenario["scenario_seed"]))
            require(record["aco_seed"] == aco_seed(int(base_seed), key[0], key[1]))
            require(all(record["parameters"].get(key) == value for key, value in parameters.items()))
            require(record["status"] in {"ok", "disconnected", "no_solution"})
            require(isinstance(record["pareto"], list))
            require(record["status"] != "ok" or bool(record["pareto"]))
            seen.add(key)
    require(seen == expected and bool(expected))
except (OSError, ValueError, KeyError, TypeError):
    sys.exit(1)
PY
}

submit_graph_goal_bundle() {
    local graph_path="$1"
    local goals="$2"
    local manifest="$3"

    local graph_file graph_config
    graph_file="$(basename "$graph_path")"
    graph_config="${graph_file%.csv}"

    local -a bundle_solvers=()
    local -a bundle_ants=()
    local -a bundle_passes=()
    local -a bundle_planned=()
    local -a bundle_variants=()
    local -a bundle_results=()
    local -a bundle_log_dirs=()
    local -a bundle_error_dirs=()

    queue_config() {
        local solver="$1"
        local ants="$2"
        local pass_through="$3"
        local compare_planned="$4"
        local variant="$5"
        local config_dir="graph_${graph_config}/goals_${goals}/${solver}/${variant}"
        local result_dir="${RESULT_ROOT}/${config_dir}"
        local log_dir="${LOG_ROOT}/${config_dir}"
        local error_dir="${ERROR_ROOT}/${config_dir}"
        local result_file="${result_dir}/results.jsonl"
        mkdir -p "$result_dir" "$log_dir" "$error_dir"

        if [[ "$SKIP_COMPLETE" == true ]] && is_complete "$result_file" "$manifest" "$graph_path" \
                "$goals" "$solver" "$ants" "$pass_through" "$compare_planned"; then
            skipped=$((skipped + 1))
            return
        fi

        bundle_solvers+=("$solver")
        bundle_ants+=("$ants")
        bundle_passes+=("$pass_through")
        bundle_planned+=("$compare_planned")
        bundle_variants+=("$variant")
        bundle_results+=("$result_file")
        bundle_log_dirs+=("$log_dir")
        bundle_error_dirs+=("$error_dir")
    }

    # Complete-graph baseline: one run per ant count, independent of SVRP options.
    local ants pass_through
    for ants in "${ANTS_LIST[@]}"; do
        queue_config vrp "$ants" 0 false "ants_${ants}"
    done

    # Full sparse/pass-through sensitivity sweep with planned comparison disabled.
    for ants in "${ANTS_LIST[@]}"; do
        for pass_through in "${PASS_THROUGH_LIST[@]}"; do
            queue_config svrp "$ants" "$pass_through" false \
                "ants_${ants}_pass_${pass_through}_planned_0"
        done
    done

    # Targeted planning ablation; matching planning-off runs are in the sweep.
    for pass_through in 0 20; do
        queue_config svrp 10 "$pass_through" true \
            "ants_10_pass_${pass_through}_planned_1"
    done

    local configuration_count="${#bundle_solvers[@]}"
    if [[ "$configuration_count" -eq 0 ]]; then
        return
    fi

    local bundle_dir="graph_${graph_config}/goals_${goals}/bundle"
    local bundle_log_dir="${LOG_ROOT}/${bundle_dir}"
    local bundle_error_dir="${ERROR_ROOT}/${bundle_dir}"
    mkdir -p "$bundle_log_dir" "$bundle_error_dir"

    local graph_index graph_size job_name run_log run_err batch_script project_dir
    graph_index="$(sed -n 's/^graph_\([0-9][0-9]*\)_.*/\1/p' <<< "$graph_config")"
    graph_size="$(sed -n 's/.*_n\([0-9][0-9]*\)_.*/\1/p' <<< "$graph_config")"
    job_name="jaco_n${graph_size}_g${goals}_${graph_index}"
    run_log="${bundle_log_dir}/slurm_%j.out"
    run_err="${bundle_error_dir}/slurm_%j.err"
    batch_script="$(mktemp "${TMP_ROOT}/submit/batch.XXXXXX")"
    project_dir="$(pwd)"

    cat > "$batch_script" << EOBATCH
#!/bin/bash
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=${THREADS}
#SBATCH --mem-per-cpu=${MEM_PER_CPU}
#SBATCH --job-name=${job_name}
#SBATCH --output=${run_log}
#SBATCH --error=${run_err}
EOBATCH
    if [[ -n "$PARTITION" ]]; then
        echo "#SBATCH --partition=${PARTITION}" >> "$batch_script"
    fi
    if [[ -n "$TIME_LIMIT" ]]; then
        echo "#SBATCH --time=${TIME_LIMIT}" >> "$batch_script"
    fi

    {
    cat << EOBATCH

set -euo pipefail
cd "${project_dir}"

export OMP_NUM_THREADS="\${SLURM_CPUS_PER_TASK}"
export TMPDIR="${TMP_ROOT}/job_\${SLURM_JOB_ID}"
mkdir -p "\${TMPDIR}"
trap 'rm -rf "\${TMPDIR}"' EXIT

echo "Started: \$(date --iso-8601=seconds)"
echo "Graph: ${graph_path}"
echo "Manifest: ${manifest}"
echo "Queued configurations: ${configuration_count}"
echo "Threads: \${SLURM_CPUS_PER_TASK}"

EOBATCH
    printf 'PYTHON=%q\nREPLICATES=%q\nTHREADS=%q\nITERATIONS=%q\nBATTERY=%q\nK_COVER=%q\nACO_SEED=%q\n' \
        "$PYTHON" "$REPLICATES" "$THREADS" "$ITERATIONS" "$BATTERY" "$K_COVER" "$ACO_SEED"
    declare -f is_complete
    cat << EOBATCH

"\$PYTHON" "${project_dir}/tools/capture_provenance.py" --binary "${BINARY}" \
    --output "${bundle_log_dir}/provenance_\${SLURM_JOB_ID}.json"

run_config() {
    local solver="\$1"
    local ants="\$2"
    local pass_through="\$3"
    local compare_planned="\$4"
    local variant="\$5"
    local result_file="\$6"
    local config_log_dir="\$7"
    local config_error_dir="\$8"

    if [[ "${SKIP_COMPLETE}" == true ]] && is_complete "\$result_file" "${manifest}" "${graph_path}" \
            "${goals}" "\$solver" "\$ants" "\$pass_through" "\$compare_planned"; then
        echo "Skipping complete configuration: \$solver / \$variant"
        return 0
    fi

    local config_log="\${config_log_dir}/run_\${SLURM_JOB_ID}.out"
    local config_error="\${config_error_dir}/run_\${SLURM_JOB_ID}.err"
    local -a extra_args=()
    if [[ "\$compare_planned" == true ]]; then
        extra_args+=(--svrp-compare-planned)
    fi

    echo "Starting configuration: solver=\$solver ants=\$ants pass=\$pass_through planned=\$compare_planned"
    if ! srun --cpu-bind=cores "${BINARY}" \
        --graph "${graph_path}" \
        --scenario-manifest "${manifest}" \
        --replicates "${REPLICATES}" \
        --aco-seed "${ACO_SEED}" \
        --solver "\$solver" \
        --ants "\$ants" \
        --iterations "${ITERATIONS}" \
        --threads "\${SLURM_CPUS_PER_TASK}" \
        --battery "${BATTERY}" \
        --k-cover "${K_COVER}" \
        --vehicles "${goals}" \
        --svrp-pass-through-candidates "\$pass_through" \
        --output-format jsonl \
        --output "\$result_file" \
        \${extra_args[@]+"\${extra_args[@]}"} >"\$config_log" 2>"\$config_error"; then
        echo "FAILED configuration: \$solver / \$variant (see \$config_error)" >&2
        return 1
    fi
    echo "Finished configuration: \$solver / \$variant"
}

failures=0
EOBATCH
    } >> "$batch_script"

    local index
    for ((index = 0; index < configuration_count; ++index)); do
        # Expand the failure counter in the generated job, not during submission.
        # shellcheck disable=SC2016
        printf 'if ! run_config %q %q %q %q %q %q %q %q; then failures=$((failures + 1)); fi\n' \
            "${bundle_solvers[index]}" \
            "${bundle_ants[index]}" \
            "${bundle_passes[index]}" \
            "${bundle_planned[index]}" \
            "${bundle_variants[index]}" \
            "${bundle_results[index]}" \
            "${bundle_log_dirs[index]}" \
            "${bundle_error_dirs[index]}" >> "$batch_script"
    done

    cat >> "$batch_script" << 'EOBATCH'

echo "Finished bundle: $(date --iso-8601=seconds); failures: ${failures}"
if [[ "$failures" -gt 0 ]]; then
    exit 1
fi
EOBATCH

    bash -n "$batch_script"
    if [[ "$DRY_RUN" == true ]]; then
        echo "Would submit ${job_name} with ${configuration_count} configurations"
    else
        sbatch "$batch_script"
        submitted=$((submitted + 1))
    fi
    rm "$batch_script"
    bundles=$((bundles + 1))
    queued=$((queued + configuration_count))
}

submit_graph_family() {
    local graph_dir="$1"
    shift
    local goals_list=("$@")
    local -a graph_files=()
    local graph_path
    if [[ ! -d "$graph_dir" ]]; then
        echo "Graph directory does not exist: $graph_dir" >&2
        exit 1
    fi
    while IFS= read -r graph_path; do
        graph_files+=("$graph_path")
    done < <(find "$graph_dir" -maxdepth 1 -type f -name '*.csv' -print | LC_ALL=C sort)
    if [[ "${#graph_files[@]}" -eq 0 ]]; then
        echo "No graph CSV files found in: $graph_dir" >&2
        exit 1
    fi

    for graph_path in "${graph_files[@]}"; do
        local graph_file graph_config
        graph_file="$(basename "$graph_path")"
        graph_config="${graph_file%.csv}"
        for goals in "${goals_list[@]}"; do
            local manifest_dir manifest
            manifest_dir="${SCENARIO_ROOT}/${graph_config}"
            manifest="${manifest_dir}/goals_${goals}.csv"
            mkdir -p "$manifest_dir"
            "$PYTHON" "$SCENARIO_GENERATOR" --graph "$graph_path" --goals "$goals" \
                --scenarios "$SCENARIOS" --seed "$SCENARIO_SEED" --output "$manifest"

            submit_graph_goal_bundle "$graph_path" "$goals" "$manifest"
        done
    done
}

submit_graph_family "$U250_GRAPH_DIR" "${U250_GOALS[@]}"
submit_graph_family "$U500_GRAPH_DIR" "${U500_GOALS[@]}"

if [[ "$DRY_RUN" == true ]]; then
    echo "Dry run prepared ${bundles} bundled jobs containing ${queued} configurations; " \
         "skipped ${skipped} complete configurations."
else
    echo "Submitted ${submitted} bundled jobs containing ${queued} configurations; " \
         "skipped ${skipped} complete configurations."
fi
