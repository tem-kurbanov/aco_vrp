#include "Graph.h"
#include "ExperimentIO.h"
#include "SVRPAntColony.h"
#include "VRPAntColony.h"
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std;

enum class SolverMode {
    Both,
    Svrp,
    Vrp
};

enum class OutputFormat {
    Legacy,
    Jsonl
};

struct InputGraph {
    int num_nodes = 0;
    vector<pair<int, int>> edge_points;
    vector<pair<int, int>> edge_parameters;
};

using ObjectivePoint = pair<double, double>;

struct ParetoMetrics {
    double svrp_hypervolume = 0.0;
    double vrp_hypervolume = 0.0;
    double coverage_vrp_to_svrp = 0.0;
    double coverage_svrp_to_vrp = 0.0;
    double epsilon_svrp_to_vrp = 0.0;
    double epsilon_vrp_to_svrp = 0.0;
};

struct ObjectiveBounds {
    double min_first = 0.0;
    double max_first = 0.0;
    double min_second = 0.0;
    double max_second = 0.0;
    bool has_points = false;
};

struct ProgramOptions {
    string graph_path;
    int num_goals = -1;
    vector<int> fixed_goals;
    optional<int> depot;
    int num_tests = 10;
    optional<unsigned int> seed;
    optional<unsigned int> aco_seed;
    SolverMode solver = SolverMode::Both;
    int ants = 100;
    int iterations = 20;
    int threads = 5;
    int battery = 50000;
    int k_cover = 24;
    int max_vehicles = 0;
    int replicates = 1;
    bool svrp_compare_planned = false;
    bool svrp_count_pass_through_goals = false;
    int svrp_pass_through_candidates = 0;
    string scenario_manifest_path;
    OutputFormat output_format = OutputFormat::Legacy;
    string output_path;
};

string trim(const string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == string::npos) {
        return "";
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

int parse_int_field(const string& field, int line_number) {
    string cleaned = trim(field);
    size_t parsed = 0;
    int value = 0;
    try {
        value = stoi(cleaned, &parsed);
    } catch (const exception&) {
        throw runtime_error("Invalid integer on line " + to_string(line_number) + ": '" + field + "'");
    }
    if (parsed != cleaned.size()) {
        throw runtime_error("Invalid integer on line " + to_string(line_number) + ": '" + field + "'");
    }
    return value;
}

int parse_int_arg(const string& value, const string& name) {
    try {
        return parse_int_field(value, 0);
    } catch (const exception&) {
        throw runtime_error("Invalid value for " + name + ": '" + value + "'");
    }
}

unsigned int parse_seed_arg(const string& value) {
    int seed = parse_int_arg(value, "--seed");
    if (seed < 0) {
        throw runtime_error("--seed must be non-negative");
    }
    return static_cast<unsigned int>(seed);
}

vector<int> parse_int_csv_line(const string& line, int line_number) {
    vector<int> values;
    string field;
    stringstream stream(line);

    while (getline(stream, field, ',')) {
        values.push_back(parse_int_field(field, line_number));
    }

    if (!line.empty() && line.back() == ',') {
        throw runtime_error("Trailing comma on line " + to_string(line_number));
    }

    return values;
}

vector<int> parse_int_list(const string& text, const string& source_name) {
    string normalized = text;
    for (char& ch: normalized) {
        if (ch == ',' || ch == ';' || isspace(static_cast<unsigned char>(ch))) {
            ch = ' ';
        }
    }

    vector<int> values;
    string token;
    stringstream stream(normalized);
    while (stream >> token) {
        values.push_back(parse_int_arg(token, source_name));
    }

    if (values.empty()) {
        throw runtime_error(source_name + " did not contain any ids");
    }

    return values;
}

vector<int> read_goal_ids_file(const string& path) {
    ifstream file(path);
    if (!file) {
        throw runtime_error("Cannot open goals file: " + path);
    }

    stringstream buffer;
    buffer << file.rdbuf();
    return parse_int_list(buffer.str(), path);
}

InputGraph read_graph(const string& graph_path) {
    ifstream graph_file(graph_path);
    if (!graph_file) {
        throw runtime_error("Cannot open graph file: " + graph_path);
    }

    string line;
    int line_number = 1;
    if (!getline(graph_file, line)) {
        throw runtime_error("Graph file is empty: " + graph_path);
    }

    vector<int> metadata = parse_int_csv_line(line, line_number);
    if (metadata.size() != 2) {
        throw runtime_error("Expected first line to contain '<num_nodes>,<num_edges>'");
    }

    InputGraph graph;
    graph.num_nodes = metadata[0];
    const int expected_edges = metadata[1];
    if (graph.num_nodes <= 0) {
        throw runtime_error("Number of nodes must be positive");
    }
    if (expected_edges < 0) {
        throw runtime_error("Number of edges cannot be negative");
    }

    line_number += 1;
    if (!getline(graph_file, line)) {
        throw runtime_error("Missing CSV header line");
    }

    while (getline(graph_file, line)) {
        line_number += 1;
        if (trim(line).empty()) {
            continue;
        }

        vector<int> row = parse_int_csv_line(line, line_number);
        if (row.size() != 4) {
            throw runtime_error("Expected 4 columns on line " + to_string(line_number));
        }

        int start = row[0];
        int end = row[1];
        int p1 = row[2];
        int p2 = row[3];

        if (start < 0 || start >= graph.num_nodes || end < 0 || end >= graph.num_nodes) {
            throw runtime_error("Edge endpoint outside node range on line " + to_string(line_number));
        }

        graph.edge_points.emplace_back(start, end);
        graph.edge_parameters.emplace_back(p1, p2);
    }

    if ((int)graph.edge_points.size() != expected_edges) {
        throw runtime_error(
                "Expected " + to_string(expected_edges) + " edges, read " + to_string(graph.edge_points.size()));
    }

    return graph;
}

bool is_option(const string& value) {
    return value.rfind("--", 0) == 0;
}

string require_value(int& index, int argc, char* argv[], const string& option) {
    if (index + 1 >= argc || is_option(argv[index + 1])) {
        throw runtime_error("Missing value for " + option);
    }
    index += 1;
    return argv[index];
}

SolverMode parse_solver(const string& value) {
    if (value == "both") {
        return SolverMode::Both;
    }
    if (value == "svrp") {
        return SolverMode::Svrp;
    }
    if (value == "vrp") {
        return SolverMode::Vrp;
    }
    throw runtime_error("--solver must be one of: both, svrp, vrp");
}

OutputFormat parse_output_format(const string& value) {
    if (value == "legacy") {
        return OutputFormat::Legacy;
    }
    if (value == "jsonl") {
        return OutputFormat::Jsonl;
    }
    throw runtime_error("--output-format must be one of: legacy, jsonl");
}

ProgramOptions parse_options(int argc, char* argv[]) {
    ProgramOptions options;

    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            throw runtime_error("");
        } else if (arg == "--graph") {
            options.graph_path = require_value(i, argc, argv, arg);
        } else if (arg == "--goals") {
            string value = require_value(i, argc, argv, arg);
            if (value.find(',') != string::npos || value.find(';') != string::npos) {
                options.fixed_goals = parse_int_list(value, "--goals");
                options.num_goals = (int)options.fixed_goals.size();
            } else {
                options.num_goals = parse_int_arg(value, arg);
            }
        } else if (arg == "--goal-ids") {
            options.fixed_goals = parse_int_list(require_value(i, argc, argv, arg), arg);
            options.num_goals = (int)options.fixed_goals.size();
        } else if (arg == "--goals-file") {
            options.fixed_goals = read_goal_ids_file(require_value(i, argc, argv, arg));
            options.num_goals = (int)options.fixed_goals.size();
        } else if (arg == "--tests") {
            options.num_tests = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--seed") {
            options.seed = parse_seed_arg(require_value(i, argc, argv, arg));
        } else if (arg == "--aco-seed") {
            options.aco_seed = parse_seed_arg(require_value(i, argc, argv, arg));
        } else if (arg == "--depot") {
            options.depot = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--solver") {
            options.solver = parse_solver(require_value(i, argc, argv, arg));
        } else if (arg == "--ants") {
            options.ants = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--iterations") {
            options.iterations = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--threads") {
            options.threads = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--battery") {
            options.battery = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--k-cover") {
            options.k_cover = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--vehicles") {
            options.max_vehicles = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--replicates") {
            options.replicates = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--svrp-compare-planned") {
            options.svrp_compare_planned = true;
        } else if (arg == "--svrp-count-pass-through-goals") {
            options.svrp_count_pass_through_goals = true;
        } else if (arg == "--svrp-pass-through-candidates") {
            options.svrp_pass_through_candidates = parse_int_arg(require_value(i, argc, argv, arg), arg);
        } else if (arg == "--scenario-manifest") {
            options.scenario_manifest_path = require_value(i, argc, argv, arg);
        } else if (arg == "--output-format") {
            options.output_format = parse_output_format(require_value(i, argc, argv, arg));
        } else if (arg == "--output") {
            options.output_path = require_value(i, argc, argv, arg);
        } else if (!is_option(arg) && options.num_goals < 0) {
            options.num_goals = parse_int_arg(arg, "<num_goals>");
        } else if (!is_option(arg) && options.graph_path.empty()) {
            options.graph_path = arg;
        } else {
            throw runtime_error("Unknown or duplicate argument: " + arg);
        }
    }

    return options;
}

void print_usage(const char* program_name) {
    cerr << "Usage:\n";
    cerr << "  " << program_name << " --graph <csv> --goals <n> [options]\n";
    cerr << "  " << program_name << " <num_goals> <graph_csv> [options]\n\n";
    cerr << "Options:\n";
    cerr << "  --tests <n>           Number of random scenarios. Default: 10\n";
    cerr << "  --seed <n>            Seed for depot/goal sampling and VRP ACO. Default: random_device\n";
    cerr << "  --aco-seed <n>        Independent base seed for ACO replicates\n";
    cerr << "  --depot <id>          Fixed depot id. Default: random per test\n";
    cerr << "  --goal-ids <ids>      Fixed comma/space-separated goal ids\n";
    cerr << "  --goals-file <path>   File with comma/space-separated goal ids\n";
    cerr << "  --solver <mode>       both, svrp, or vrp. Default: both\n";
    cerr << "  --ants <n>            Ants per iteration. Default: 100\n";
    cerr << "  --iterations <n>      Stop after this many non-improving iterations. Default: 20\n";
    cerr << "  --threads <n>         OpenMP thread count. Default: 5\n";
    cerr << "  --battery <n>         Battery capacity. Default: 50000\n";
    cerr << "  --k-cover <n>         k for k-path-cover shrink graph. Default: 24\n";
    cerr << "  --vehicles <n>        Maximum routes/vehicles. Default: number of goals\n";
    cerr << "  --replicates <n>      ACO replicates per scenario. Default: 1\n";
    cerr << "  --scenario-manifest <csv>  Fixed scenario manifest\n";
    cerr << "  --svrp-compare-planned  Compare direct SVRP moves with planned paths\n";
    cerr << "  --svrp-count-pass-through-goals  Count goals crossed inside SVRP shrink edges\n";
    cerr << "  --svrp-pass-through-candidates <n>  Add up to n goal-crossing SVRP edges per source\n";
    cerr << "  --output-format <format>  legacy or jsonl. Default: legacy\n";
    cerr << "  --output <path>       Write result lines to file instead of stdout\n\n";

    cerr << "Example:\n";
    cerr << "  " << program_name << " --graph art/u100/graph_000_n100_c0_u1_seed20062.csv --goals 10 --seed 42 --solver both\n";
}

void validate_positive(const ProgramOptions& options) {
    if (options.num_tests <= 0) {
        throw runtime_error("--tests must be positive");
    }
    if (options.ants <= 0) {
        throw runtime_error("--ants must be positive");
    }
    if (options.iterations <= 0) {
        throw runtime_error("--iterations must be positive");
    }
    if (options.threads <= 0) {
        throw runtime_error("--threads must be positive");
    }
    if (options.battery <= 0) {
        throw runtime_error("--battery must be positive");
    }
    if (options.k_cover <= 1) {
        throw runtime_error("--k-cover must be greater than 1");
    }
    if (options.svrp_pass_through_candidates < 0) {
        throw runtime_error("--svrp-pass-through-candidates cannot be negative");
    }
    if (options.max_vehicles < 0) {
        throw runtime_error("--vehicles cannot be negative");
    }
    if (options.replicates <= 0) {
        throw runtime_error("--replicates must be positive");
    }
}

void validate_node_id(int id, int num_nodes, const string& name) {
    if (id < 0 || id >= num_nodes) {
        throw runtime_error(name + " id " + to_string(id) + " is outside graph node range 0.." + to_string(num_nodes - 1));
    }
}

set<int> build_fixed_goals(const vector<int>& goal_ids, int num_nodes) {
    set<int> goals;
    for (int goal: goal_ids) {
        validate_node_id(goal, num_nodes, "Goal");
        if (!goals.insert(goal).second) {
            throw runtime_error("Duplicate goal id: " + to_string(goal));
        }
    }
    return goals;
}

int choose_depot(mt19937& rng, int num_nodes, const set<int>& forbidden_goals) {
    uniform_int_distribution<int> node_distribution(0, num_nodes - 1);
    int depot = node_distribution(rng);
    while (forbidden_goals.find(depot) != forbidden_goals.end()) {
        depot = node_distribution(rng);
    }
    return depot;
}

set<int> choose_random_goals(mt19937& rng, int num_nodes, int num_goals, int depot) {
    uniform_int_distribution<int> node_distribution(0, num_nodes - 1);
    set<int> goals;
    while ((int)goals.size() < num_goals) {
        int new_goal = node_distribution(rng);
        if (new_goal != depot) {
            goals.insert(new_goal);
        }
    }
    return goals;
}

uint32_t derive_replicate_seed(uint32_t base_seed, const string& scenario_id, int replicate) {
    // Stable FNV-style mixing gives both solvers the same seed for a paired run.
    uint32_t hash = 2166136261u ^ base_seed;
    for (unsigned char ch: scenario_id) {
        hash ^= ch;
        hash *= 16777619u;
    }
    hash ^= static_cast<uint32_t>(replicate);
    hash *= 16777619u;
    return hash;
}

void validate_scenario(const ScenarioDefinition& scenario, int num_nodes,
                       optional<int> expected_goals) {
    validate_node_id(scenario.depot, num_nodes, "Depot");
    set<int> unique_goals;
    for (int goal: scenario.goals) {
        validate_node_id(goal, num_nodes, "Goal");
        if (goal == scenario.depot) {
            throw runtime_error("Scenario '" + scenario.id + "' uses its depot as a goal");
        }
        if (!unique_goals.insert(goal).second) {
            throw runtime_error("Scenario '" + scenario.id + "' contains duplicate goals");
        }
    }
    if (expected_goals.has_value() && static_cast<int>(unique_goals.size()) != *expected_goals) {
        throw runtime_error("Scenario '" + scenario.id + "' goal count does not match --goals");
    }
}

vector<ObjectivePoint> parse_pareto_points(const string& answer) {
    vector<ObjectivePoint> points;
    size_t pos = 0;

    while (true) {
        size_t start = answer.find('(', pos);
        if (start == string::npos) {
            break;
        }
        size_t separator = answer.find(';', start + 1);
        size_t end = answer.find(')', start + 1);
        if (separator == string::npos || end == string::npos || separator > end) {
            break;
        }

        double first = stod(answer.substr(start + 1, separator - start - 1));
        double second = stod(answer.substr(separator + 1, end - separator - 1));
        points.emplace_back(first, second);
        pos = end + 1;
    }

    sort(points.begin(), points.end());
    points.erase(unique(points.begin(), points.end()), points.end());
    return points;
}

bool dominates(const ObjectivePoint& lhs, const ObjectivePoint& rhs) {
    return lhs.first <= rhs.first && lhs.second <= rhs.second;
}

vector<ObjectivePoint> nondominated_points(vector<ObjectivePoint> points) {
    sort(points.begin(), points.end());
    points.erase(unique(points.begin(), points.end()), points.end());

    vector<ObjectivePoint> result;
    for (int i = 0; i < points.size(); ++i) {
        bool dominated = false;
        for (int j = 0; j < points.size(); ++j) {
            if (i == j) {
                continue;
            }
            bool weakly_dominated = dominates(points[j], points[i]);
            bool strictly_better = points[j].first < points[i].first || points[j].second < points[i].second;
            if (weakly_dominated && strictly_better) {
                dominated = true;
                break;
            }
        }
        if (!dominated) {
            result.push_back(points[i]);
        }
    }

    sort(result.begin(), result.end());
    return result;
}

ObjectivePoint reference_point(const vector<ObjectivePoint>& first_set, const vector<ObjectivePoint>& second_set) {
    double max_first = 0.0;
    double max_second = 0.0;
    for (const auto& point: first_set) {
        max_first = max(max_first, point.first);
        max_second = max(max_second, point.second);
    }
    for (const auto& point: second_set) {
        max_first = max(max_first, point.first);
        max_second = max(max_second, point.second);
    }

    return make_pair(max_first + max(1.0, max_first * 0.1),
                     max_second + max(1.0, max_second * 0.1));
}

ObjectiveBounds objective_bounds(const vector<ObjectivePoint>& first_set, const vector<ObjectivePoint>& second_set) {
    ObjectiveBounds bounds;
    auto include_point = [&bounds](const ObjectivePoint& point) {
        if (!bounds.has_points) {
            bounds.min_first = point.first;
            bounds.max_first = point.first;
            bounds.min_second = point.second;
            bounds.max_second = point.second;
            bounds.has_points = true;
            return;
        }

        bounds.min_first = min(bounds.min_first, point.first);
        bounds.max_first = max(bounds.max_first, point.first);
        bounds.min_second = min(bounds.min_second, point.second);
        bounds.max_second = max(bounds.max_second, point.second);
    };

    for (const auto& point: first_set) {
        include_point(point);
    }
    for (const auto& point: second_set) {
        include_point(point);
    }

    return bounds;
}

vector<ObjectivePoint> normalize_points(const vector<ObjectivePoint>& points, const ObjectiveBounds& bounds) {
    vector<ObjectivePoint> normalized_points;
    if (!bounds.has_points) {
        return normalized_points;
    }

    double first_range = bounds.max_first - bounds.min_first;
    double second_range = bounds.max_second - bounds.min_second;
    for (const auto& point: points) {
        double first = first_range > 0.0 ? (point.first - bounds.min_first) / first_range : 0.0;
        double second = second_range > 0.0 ? (point.second - bounds.min_second) / second_range : 0.0;
        normalized_points.emplace_back(first, second);
    }

    return normalized_points;
}

double hypervolume_2d(vector<ObjectivePoint> points, const ObjectivePoint& reference) {
    points = nondominated_points(std::move(points));
    if (points.empty()) {
        return 0.0;
    }

    double hypervolume = 0.0;
    for (int i = 0; i < points.size(); ++i) {
        double next_first = (i + 1 < points.size()) ? points[i + 1].first : reference.first;
        double width = next_first - points[i].first;
        double height = reference.second - points[i].second;
        if (width > 0.0 && height > 0.0) {
            hypervolume += width * height;
        }
    }
    return hypervolume;
}

double coverage_indicator(const vector<ObjectivePoint>& source, const vector<ObjectivePoint>& target) {
    if (target.empty()) {
        return 0.0;
    }

    int covered = 0;
    for (const auto& target_point: target) {
        bool is_covered = false;
        for (const auto& source_point: source) {
            if (dominates(source_point, target_point)) {
                is_covered = true;
                break;
            }
        }
        if (is_covered) {
            covered += 1;
        }
    }

    return (double)covered / (double)target.size();
}

double additive_epsilon_indicator(const vector<ObjectivePoint>& source, const vector<ObjectivePoint>& target) {
    if (source.empty() || target.empty()) {
        return numeric_limits<double>::infinity();
    }

    double epsilon = -numeric_limits<double>::infinity();
    for (const auto& target_point: target) {
        double best_for_target = numeric_limits<double>::infinity();
        for (const auto& source_point: source) {
            double candidate = max(source_point.first - target_point.first,
                                   source_point.second - target_point.second);
            best_for_target = min(best_for_target, candidate);
        }
        epsilon = max(epsilon, best_for_target);
    }

    return epsilon;
}

ParetoMetrics compute_pareto_metrics(const string& svrp_answer, const string& vrp_answer) {
    vector<ObjectivePoint> svrp_points = parse_pareto_points(svrp_answer);
    vector<ObjectivePoint> vrp_points = parse_pareto_points(vrp_answer);
    ObjectiveBounds bounds = objective_bounds(svrp_points, vrp_points);
    vector<ObjectivePoint> normalized_svrp_points = normalize_points(svrp_points, bounds);
    vector<ObjectivePoint> normalized_vrp_points = normalize_points(vrp_points, bounds);
    ObjectivePoint normalized_reference = make_pair(1.1, 1.1);

    ParetoMetrics metrics;
    metrics.svrp_hypervolume = hypervolume_2d(normalized_svrp_points, normalized_reference);
    metrics.vrp_hypervolume = hypervolume_2d(normalized_vrp_points, normalized_reference);
    metrics.coverage_vrp_to_svrp = coverage_indicator(vrp_points, svrp_points);
    metrics.coverage_svrp_to_vrp = coverage_indicator(svrp_points, vrp_points);
    metrics.epsilon_svrp_to_vrp = additive_epsilon_indicator(normalized_svrp_points, normalized_vrp_points);
    metrics.epsilon_vrp_to_svrp = additive_epsilon_indicator(normalized_vrp_points, normalized_svrp_points);
    return metrics;
}

string format_pareto_metrics(const ParetoMetrics& metrics) {
    ostringstream stream;
    stream << "metrics "
           << "hv_svrp=" << metrics.svrp_hypervolume << ","
           << "hv_vrp=" << metrics.vrp_hypervolume << ","
           << "coverage_vrp_to_svrp=" << metrics.coverage_vrp_to_svrp << ","
           << "coverage_svrp_to_vrp=" << metrics.coverage_svrp_to_vrp << ","
           << "epsilon_svrp_to_vrp=" << metrics.epsilon_svrp_to_vrp << ","
           << "epsilon_vrp_to_svrp=" << metrics.epsilon_vrp_to_svrp;
    return stream.str();
}

int main(int argc, char *argv[]) {
    ProgramOptions options;
    try {
        options = parse_options(argc, argv);
    } catch (const exception& e) {
        if (string(e.what()).empty()) {
            print_usage(argv[0]);
            return 0;
        }
        cerr << e.what() << endl;
        print_usage(argv[0]);
        return 1;
    }

    if (options.graph_path.empty()
        || (options.num_goals < 0 && options.scenario_manifest_path.empty())) {
        print_usage(argv[0]);
        return 1;
    }

    try {
        validate_positive(options);
    } catch (const exception& e) {
        cerr << e.what() << endl;
        return 1;
    }

    InputGraph input_graph;
    try {
        input_graph = read_graph(options.graph_path);
    } catch (const exception& e) {
        cerr << "Failed to read graph: " << e.what() << endl;
        return 1;
    }

    int num_nodes = input_graph.num_nodes;
    if (options.num_goals >= num_nodes || options.num_goals == 0) {
        cerr << "num_goals must be between 1 and " << num_nodes - 1 << endl;
        return 1;
    }

    uint32_t scenario_base_seed = 0;
    if (options.seed.has_value()) {
        scenario_base_seed = *options.seed;
    } else {
        random_device rd;
        scenario_base_seed = rd();
    }
    mt19937 rng(scenario_base_seed);

    vector<ScenarioDefinition> scenarios;
    try {
        if (!options.scenario_manifest_path.empty()) {
            if (!options.fixed_goals.empty() || options.depot.has_value()) {
                throw runtime_error(
                        "--scenario-manifest cannot be combined with --depot, --goal-ids, or --goals-file");
            }
            scenarios = read_scenario_manifest(options.scenario_manifest_path);
            optional<int> expected_goals = options.num_goals > 0
                                           ? optional<int>(options.num_goals)
                                           : nullopt;
            for (const auto& scenario: scenarios) {
                validate_scenario(scenario, num_nodes, expected_goals);
            }
            if (options.num_goals < 0) {
                options.num_goals = static_cast<int>(scenarios.front().goals.size());
                for (const auto& scenario: scenarios) {
                    if (static_cast<int>(scenario.goals.size()) != options.num_goals) {
                        throw runtime_error("All manifest scenarios must have the same goal count");
                    }
                }
            }
        } else {
            set<int> fixed_goals = build_fixed_goals(options.fixed_goals, num_nodes);
            if (options.depot.has_value()) {
                validate_node_id(*options.depot, num_nodes, "Depot");
                if (fixed_goals.find(*options.depot) != fixed_goals.end()) {
                    throw runtime_error("Depot cannot also be a goal");
                }
            }
            if (!fixed_goals.empty() && static_cast<int>(fixed_goals.size()) != options.num_goals) {
                throw runtime_error("Fixed goal count does not match --goals");
            }

            for (int index = 0; index < options.num_tests; ++index) {
                set<int> goals = fixed_goals;
                int depot = options.depot.value_or(choose_depot(rng, num_nodes, goals));
                if (goals.empty()) {
                    goals = choose_random_goals(rng, num_nodes, options.num_goals, depot);
                }
                ScenarioDefinition scenario;
                scenario.id = "random-" + to_string(index + 1);
                scenario.depot = depot;
                scenario.goals.assign(goals.begin(), goals.end());
                scenario.scenario_seed = scenario_base_seed;
                scenarios.push_back(std::move(scenario));
            }
        }
    } catch (const exception& e) {
        cerr << "Failed to prepare scenarios: " << e.what() << endl;
        return 1;
    }

    uint32_t aco_base_seed = options.aco_seed.value_or(scenario_base_seed);

    ofstream output_file;
    ostream* output = &cout;
    if (!options.output_path.empty()) {
        output_file.open(options.output_path);
        if (!output_file) {
            cerr << "Cannot open output file: " << options.output_path << endl;
            return 1;
        }
        output = &output_file;
    }

    for (const auto& scenario: scenarios) {
        set<int> goals(scenario.goals.begin(), scenario.goals.end());
        int depot = scenario.depot;
        int max_vehicles = options.max_vehicles > 0
                           ? options.max_vehicles
                           : static_cast<int>(goals.size());

        for (int replicate = 0; replicate < options.replicates; ++replicate) {
            uint32_t solver_seed = derive_replicate_seed(aco_base_seed, scenario.id, replicate);
            Graph graph(num_nodes, input_graph.edge_points, input_graph.edge_parameters, depot, goals);

            SolverRunResult svrp_result;
            SolverRunResult vrp_result;
            if (options.solver == SolverMode::Both || options.solver == SolverMode::Svrp) {
                SVRPAntColony sant(graph, static_cast<int>(goals.size()), options.ants,
                                   options.iterations, options.threads, options.battery,
                                   options.k_cover, solver_seed, options.svrp_compare_planned,
                                   options.svrp_count_pass_through_goals,
                                   options.svrp_pass_through_candidates, max_vehicles);
                svrp_result = sant.bicriterion_ant_svrp_result();
            }
            if (options.solver == SolverMode::Both || options.solver == SolverMode::Vrp) {
                VRPAntColony ant(graph, static_cast<int>(goals.size()), options.ants,
                                 options.iterations, options.threads, options.battery,
                                 options.k_cover, solver_seed, max_vehicles);
                vrp_result = ant.bicriterion_ant_vrp_result();
            }

            if (options.output_format == OutputFormat::Jsonl) {
                ExperimentRecordContext context;
                context.scenario_id = scenario.id;
                context.graph_path = options.graph_path;
                context.graph_size = num_nodes;
                context.scenario_seed = scenario.scenario_seed;
                context.replicate = replicate;
                context.aco_seed = solver_seed;
                context.depot = depot;
                context.goals = scenario.goals;
                context.ants = options.ants;
                context.stagnation_limit = options.iterations;
                context.threads = options.threads;
                context.battery = options.battery;
                context.k_cover = options.k_cover;
                context.max_vehicles = max_vehicles;

                if (options.solver == SolverMode::Both || options.solver == SolverMode::Svrp) {
                    context.pass_through_candidates = options.svrp_pass_through_candidates;
                    context.compare_planned = options.svrp_compare_planned;
                    context.count_pass_through_goals = options.svrp_count_pass_through_goals
                                                       || options.svrp_pass_through_candidates > 0;
                    *output << solver_result_json(context, svrp_result) << '\n';
                }
                if (options.solver == SolverMode::Both || options.solver == SolverMode::Vrp) {
                    context.pass_through_candidates = 0;
                    context.compare_planned = false;
                    context.count_pass_through_goals = false;
                    *output << solver_result_json(context, vrp_result) << '\n';
                }
                continue;
            }

            string svrp_answer = svrp_result.legacy_answer();
            string vrp_answer = vrp_result.legacy_answer();
            if (options.solver == SolverMode::Svrp) {
                *output << svrp_answer << endl;
            } else if (options.solver == SolverMode::Vrp) {
                *output << vrp_answer << endl;
            } else {
                ParetoMetrics metrics = compute_pareto_metrics(svrp_answer, vrp_answer);
                *output << svrp_answer << "\n"
                        << vrp_answer << "\n"
                        << format_pareto_metrics(metrics) << "\n" << endl;
            }
        }
    }

    return 0;
}
