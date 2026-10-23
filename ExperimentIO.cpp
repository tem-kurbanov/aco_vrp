#include "ExperimentIO.h"

#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

using namespace std;

namespace {

string trim_copy(const string& value) {
    const size_t first = value.find_first_not_of(" \t\r\n");
    if (first == string::npos) {
        return {};
    }
    const size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

vector<string> split(const string& value, char separator) {
    vector<string> fields;
    string field;
    stringstream stream(value);
    while (getline(stream, field, separator)) {
        fields.push_back(trim_copy(field));
    }
    if (!value.empty() && value.back() == separator) {
        fields.emplace_back();
    }
    return fields;
}

int parse_int(const string& value, const string& label, int line_number) {
    size_t parsed = 0;
    try {
        int result = stoi(value, &parsed);
        if (parsed != value.size()) {
            throw invalid_argument("trailing input");
        }
        return result;
    } catch (const exception&) {
        throw runtime_error("Invalid " + label + " on manifest line " + to_string(line_number));
    }
}

uint32_t parse_seed(const string& value, int line_number) {
    int seed = parse_int(value, "scenario seed", line_number);
    if (seed < 0) {
        throw runtime_error("Scenario seed must be non-negative on manifest line " + to_string(line_number));
    }
    return static_cast<uint32_t>(seed);
}

string json_escape(const string& value) {
    ostringstream output;
    for (unsigned char ch: value) {
        switch (ch) {
            case '"': output << "\\\""; break;
            case '\\': output << "\\\\"; break;
            case '\b': output << "\\b"; break;
            case '\f': output << "\\f"; break;
            case '\n': output << "\\n"; break;
            case '\r': output << "\\r"; break;
            case '\t': output << "\\t"; break;
            default:
                if (ch < 0x20) {
                    output << "\\u" << hex << setw(4) << setfill('0') << static_cast<int>(ch)
                           << dec << setfill(' ');
                } else {
                    output << ch;
                }
        }
    }
    return output.str();
}

void write_int_array(ostringstream& output, const vector<int>& values) {
    output << "[";
    for (size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            output << ",";
        }
        output << values[index];
    }
    output << "]";
}

} // namespace

vector<ScenarioDefinition> read_scenario_manifest(const string& path) {
    ifstream input(path);
    if (!input) {
        throw runtime_error("Cannot open scenario manifest: " + path);
    }

    string line;
    if (!getline(input, line)) {
        throw runtime_error("Scenario manifest is empty: " + path);
    }
    if (trim_copy(line) != "scenario_id,depot,goals,scenario_seed") {
        throw runtime_error(
                "Scenario manifest header must be: scenario_id,depot,goals,scenario_seed");
    }

    vector<ScenarioDefinition> scenarios;
    set<string> ids;
    int line_number = 1;
    while (getline(input, line)) {
        line_number += 1;
        if (trim_copy(line).empty()) {
            continue;
        }
        vector<string> fields = split(line, ',');
        if (fields.size() != 4) {
            throw runtime_error("Expected four fields on manifest line " + to_string(line_number));
        }

        ScenarioDefinition scenario;
        scenario.id = fields[0];
        if (scenario.id.empty() || !ids.insert(scenario.id).second) {
            throw runtime_error("Scenario ids must be non-empty and unique (line "
                                + to_string(line_number) + ")");
        }
        scenario.depot = parse_int(fields[1], "depot", line_number);
        scenario.scenario_seed = parse_seed(fields[3], line_number);

        set<int> unique_goals;
        for (const string& goal: split(fields[2], '|')) {
            int goal_id = parse_int(goal, "goal id", line_number);
            if (!unique_goals.insert(goal_id).second) {
                throw runtime_error("Duplicate goal id on manifest line " + to_string(line_number));
            }
            scenario.goals.push_back(goal_id);
        }
        if (scenario.goals.empty()) {
            throw runtime_error("Scenario has no goals on manifest line " + to_string(line_number));
        }
        scenarios.push_back(std::move(scenario));
    }

    if (scenarios.empty()) {
        throw runtime_error("Scenario manifest contains no scenarios: " + path);
    }
    return scenarios;
}

string solver_result_json(const ExperimentRecordContext& context,
                          const SolverRunResult& result) {
    ostringstream output;
    output << setprecision(numeric_limits<double>::max_digits10);
    output << "{\"schema_version\":1"
           << ",\"scenario_id\":\"" << json_escape(context.scenario_id) << "\""
           << ",\"graph_path\":\"" << json_escape(context.graph_path) << "\""
           << ",\"graph_size\":" << context.graph_size
           << ",\"scenario_seed\":" << context.scenario_seed
           << ",\"replicate\":" << context.replicate
           << ",\"aco_seed\":" << context.aco_seed
           << ",\"depot\":" << context.depot
           << ",\"goals\":";
    write_int_array(output, context.goals);
    output << ",\"solver\":\"" << json_escape(result.solver) << "\""
           << ",\"status\":\"" << json_escape(result.status) << "\""
           << ",\"message\":\"" << json_escape(result.message) << "\""
           << ",\"parameters\":{"
           << "\"ants\":" << context.ants
           << ",\"stagnation_limit\":" << context.stagnation_limit
           << ",\"threads\":" << context.threads
           << ",\"battery\":" << context.battery
           << ",\"k_cover\":" << context.k_cover
           << ",\"max_vehicles\":" << context.max_vehicles
           << ",\"pass_through_candidates\":" << context.pass_through_candidates
           << ",\"compare_planned\":" << (context.compare_planned ? "true" : "false")
           << ",\"count_pass_through_goals\":"
           << (context.count_pass_through_goals ? "true" : "false") << "}"
           << ",\"graph\":{\"nodes\":" << result.graph_nodes
           << ",\"edges\":" << result.graph_edges << "}"
           << ",\"timing_seconds\":{\"preprocessing\":" << result.preprocessing_seconds
           << ",\"optimization\":" << result.optimization_seconds
           << ",\"total\":" << result.total_seconds << "}"
           << ",\"iterations\":" << result.iterations
           << ",\"invalid_candidates\":" << result.invalid_candidates
           << ",\"pareto\":[";

    for (size_t index = 0; index < result.pareto.size(); ++index) {
        if (index > 0) {
            output << ",";
        }
        output << "{\"objectives\":";
        write_int_array(output, result.pareto[index].objectives);
        output << ",\"route_count\":" << result.pareto[index].route_count << "}";
    }
    output << "]}";
    return output.str();
}
