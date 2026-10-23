#!/bin/bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

# Submit one Slurm job for each:
#   graph CSV x goal count x ant count x SVRP pass-through candidate count
#
# Edit this block before running the script.
GRAPH_DIR="${GRAPH_DIR:-./art/u500}"
BINARY="${BINARY:-./build/aco_vrp}"

GOALS_LIST=(50 100 150)
ANTS_LIST=(5 10 20)
PASS_THROUGH_LIST=(5 10 20)

TESTS="${TESTS:-10}"
THREADS="${THREADS:-2}"
ITERATIONS="${ITERATIONS:-50}"
SOLVER="${SOLVER:-both}"
SEED="${SEED:-42}"

BATTERY="${BATTERY:-50000}"
K_COVER="${K_COVER:-24}"
PARTITION="${PARTITION-amdextralong}"
MEM_PER_CPU="${MEM_PER_CPU:-8G}"
TIME_LIMIT="${TIME_LIMIT:-}"
DRY_RUN="${DRY_RUN:-false}"

RESULT_ROOT="${RESULT_ROOT:-./outputs/vrp}"
LOG_ROOT="${LOG_ROOT:-./logs/vrp}"
ERROR_ROOT="${ERROR_ROOT:-./errors/vrp}"

# Optional program flags, for example:
# EXTRA_ARGS=(--svrp-compare-planned)
EXTRA_ARGS=()

if [[ ! -d "$GRAPH_DIR" ]]; then
    echo "Graph directory does not exist: $GRAPH_DIR" >&2
    exit 1
fi

if [[ ! -x "$BINARY" ]]; then
    echo "Binary is not executable: $BINARY" >&2
    echo "Build it first, or set BINARY=/path/to/aco_vrp" >&2
    exit 1
fi

if [[ "$THREADS" -lt 1 ]]; then
    echo "THREADS must be positive" >&2
    exit 1
fi
if [[ "$DRY_RUN" != true && "$DRY_RUN" != false ]]; then
    echo "DRY_RUN must be true or false" >&2
    exit 1
fi

mkdir -p "$RESULT_ROOT" "$LOG_ROOT" "$ERROR_ROOT"

GRAPH_FILES=()
while IFS= read -r graph_path; do
    GRAPH_FILES+=("$graph_path")
done < <(find "$GRAPH_DIR" -maxdepth 1 -type f -name '*.csv' -print | LC_ALL=C sort)

if [[ "${#GRAPH_FILES[@]}" -eq 0 ]]; then
    echo "No .csv graph files found in: $GRAPH_DIR" >&2
    exit 1
fi

submit_count=0

for graph_path in "${GRAPH_FILES[@]}"; do
    graph_file="$(basename "$graph_path")"
    graph_config="${graph_file%.csv}"

    for goals in "${GOALS_LIST[@]}"; do
        for ants in "${ANTS_LIST[@]}"; do
            for pass_through in "${PASS_THROUGH_LIST[@]}"; do
                config_dir="graph_${graph_config}/goals_${goals}/ants_${ants}/pass_${pass_through}"
                result_dir="${RESULT_ROOT}/${config_dir}"
                log_dir="${LOG_ROOT}/${config_dir}"
                error_dir="${ERROR_ROOT}/${config_dir}"
                mkdir -p "$result_dir" "$log_dir" "$error_dir"

                result_file="${result_dir}/tests_${TESTS}_threads_${THREADS}.txt"
                run_log="${log_dir}/slurm_%j.out"
                run_err="${error_dir}/slurm_%j.err"
                job_name="vrp_g${goals}_a${ants}_p${pass_through}"

                tmp_batch_script="$(mktemp)"
                cat > "$tmp_batch_script" << EOBATCH
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
                    echo "#SBATCH --partition=${PARTITION}" >> "$tmp_batch_script"
                fi
                if [[ -n "$TIME_LIMIT" ]]; then
                    echo "#SBATCH --time=${TIME_LIMIT}" >> "$tmp_batch_script"
                fi

                cat >> "$tmp_batch_script" << EOBATCH

set -euo pipefail

cd "$(pwd)"

export OMP_NUM_THREADS="\${SLURM_CPUS_PER_TASK}"

echo "Started: \$(date)"
echo "Host: \$(hostname)"
echo "Job id: \${SLURM_JOB_ID}"
echo "Graph: ${graph_path}"
echo "Goals: ${goals}"
echo "Ants: ${ants}"
echo "Pass-through candidates: ${pass_through}"
echo "Tests: ${TESTS}"
echo "Threads: \${SLURM_CPUS_PER_TASK}"
echo "Result file: ${result_file}"

EXTRA_ARGS=(
EOBATCH

                for extra_arg in ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}; do
                    printf '    %q\n' "$extra_arg" >> "$tmp_batch_script"
                done

                cat >> "$tmp_batch_script" << EOBATCH
)

srun --cpu-bind=cores "${BINARY}" \\
    --graph "${graph_path}" \\
    --goals "${goals}" \\
    --tests "${TESTS}" \\
    --seed "${SEED}" \\
    --solver "${SOLVER}" \\
    --ants "${ants}" \\
    --iterations "${ITERATIONS}" \\
    --threads "\${SLURM_CPUS_PER_TASK}" \\
    --battery "${BATTERY}" \\
    --k-cover "${K_COVER}" \\
    --svrp-pass-through-candidates "${pass_through}" \\
    --output "${result_file}" \\
    \${EXTRA_ARGS[@]+"\${EXTRA_ARGS[@]}"}

echo "Finished: \$(date)"
EOBATCH

                bash -n "$tmp_batch_script"
                if [[ "$DRY_RUN" == true ]]; then
                    echo "Would submit ${job_name}: ${graph_config}"
                else
                    sbatch "$tmp_batch_script"
                fi
                rm "$tmp_batch_script"
                submit_count=$((submit_count + 1))
            done
        done
    done
done

if [[ "$DRY_RUN" == true ]]; then
    echo "Dry run prepared ${submit_count} jobs."
else
    echo "Submitted ${submit_count} jobs."
fi
