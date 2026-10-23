#include "ExperimentIO.h"

#include <cassert>
#include <fstream>
#include <string>

int main() {
    const std::string fixture = std::string(TEST_SOURCE_DIR) + "/tests/fixtures/scenarios.csv";
    std::vector<ScenarioDefinition> scenarios = read_scenario_manifest(fixture);
    assert(scenarios.size() == 2);
    assert(scenarios[0].id == "fixture-1");
    assert(scenarios[0].depot == 0);
    assert((scenarios[0].goals == std::vector<int>{1, 2}));
    assert(scenarios[0].scenario_seed == 12345);

    SolverRunResult result;
    result.solver = "svrp";
    result.status = "ok";
    result.graph_nodes = 3;
    result.graph_edges = 4;
    result.iterations = 7;
    result.preprocessing_seconds = 0.1;
    result.optimization_seconds = 0.2;
    result.total_seconds = 0.3;
    result.pareto.push_back(SolverParetoPoint{{10, 20}, 1});

    ExperimentRecordContext context;
    context.scenario_id = "fixture-1";
    context.graph_path = "graph.csv";
    context.graph_size = 6;
    context.scenario_seed = 12345;
    context.aco_seed = 67890;
    context.goals = {1, 2};
    context.depot = 0;
    context.ants = 5;
    std::string json = solver_result_json(context, result);
    assert(json.find("\"schema_version\":1") != std::string::npos);
    assert(json.find("\"scenario_id\":\"fixture-1\"") != std::string::npos);
    assert(json.find("\"objectives\":[10,20]") != std::string::npos);

    const std::string invalid_path = std::string(TEST_BINARY_DIR) + "/invalid_scenarios.csv";
    std::ofstream invalid(invalid_path);
    invalid << "wrong,header\n";
    invalid.close();
    bool rejected = false;
    try {
        (void)read_scenario_manifest(invalid_path);
    } catch (const std::exception&) {
        rejected = true;
    }
    assert(rejected);
    return 0;
}
