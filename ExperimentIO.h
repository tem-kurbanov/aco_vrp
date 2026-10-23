#ifndef ACO_VRP_EXPERIMENT_IO_H
#define ACO_VRP_EXPERIMENT_IO_H

#include "SolverResult.h"

#include <cstdint>
#include <string>
#include <vector>

struct ScenarioDefinition {
    // Depot and goal IDs refer to the original CSV, before Graph remaps them.
    std::string id;
    int depot = -1;
    std::vector<int> goals;
    std::uint32_t scenario_seed = 0;
};

struct ExperimentRecordContext {
    std::string scenario_id;
    std::string graph_path;
    int graph_size = 0;
    std::uint32_t scenario_seed = 0;
    int replicate = 0;
    std::uint32_t aco_seed = 0;
    int depot = -1;
    std::vector<int> goals;
    int ants = 0;
    int stagnation_limit = 0;
    int threads = 0;
    int battery = 0;
    int k_cover = 0;
    int max_vehicles = 0;
    int pass_through_candidates = 0;
    bool compare_planned = false;
    bool count_pass_through_goals = false;
};

std::vector<ScenarioDefinition> read_scenario_manifest(const std::string& path);
std::string solver_result_json(const ExperimentRecordContext& context,
                               const SolverRunResult& result);

#endif
